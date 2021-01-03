// Note: all temperature values are in 0.1 degF

/* *** Feature list ***
 * Temp sensing via AI
 * SP adjustment via pushbuttons on voltage divider into AI
 * PID control
 * TODO PID loop tuning
 * relay control
 * TODO stepper motor control
 * servo motor control
 * LCD printout
 * serial output
 * serial output graphing
 * multiple zones
 * real-time clock
 * schedule
 * TODO AI calibration
 * ******************** */

//#define TINKERCAD
#define DEBUG
//#define SHOWMEMORY
//#define VERBOSESERIAL

#ifdef TINKERCAD
#define DEBUG
#endif

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__
 
int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

#include <LiquidCrystal.h>
#include <Servo.h>
#ifndef TINKERCAD
#include <FastPID.h>
#endif

// helper functions
bool ready(unsigned long *, unsigned long);
byte readAnalogButton(unsigned long);
void printVal(char*, long);
void updateLCDclock();

// Serial
// http://www.arduino.cc/en/Tutorial/AnalogReadSerial
// https://www.arduino.cc/en/Tutorial/BuiltInExamples/ReadASCIIString
const bool serialplot = true; // false = verbose text output

// LCD
// http://www.arduino.cc/en/Tutorial/LiquidCrystalHelloWorld
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
long lcdold[] = {-1, 0, 0, 0, 0};
byte lcd_tc = 0;

// lcd update timing
const unsigned long KLOOP = 100; // ms
unsigned long kloopold = 0;

// analog input setup
const unsigned long AIRES = 358;
const unsigned long AIRES2 = 1011;

class AI {
	public:
		byte pin;
		unsigned long raw;
		int pv;
		int pvmin;
		int pvmax;
		unsigned long freq; // ms
		unsigned long last=0;

		AI(byte _pin, int _pvmin, int _pvmax, unsigned long _freq) {
			pin = _pin;
			pvmin = _pvmin;
			pvmax = _pvmax;
			freq = _freq;
			forceupdate();
		}

		bool update() {
			if (ready(&last, freq)) {
				forceupdate();
        return true;
			}
			else
				return false;
		}

   private:
   void forceupdate() {
     raw = analogRead(pin);
     pv = map(raw, 0, 1023, pvmin, pvmax);
   }

		~AI() { }
};
const byte n_ti = 2;
AI **ti; // array of pointers to AI objects

// YC-03: heater output
const byte RELAY = 0; // Cv: 0-100%
const byte STEPPER = 1; // TODO
const byte SERVO = 2; // Cv: 0-180 deg
class YC {
	public:
	byte type;
	byte pin;
	int pos = 0;
	int minp;
	int maxp;
	unsigned long freq; // ms
	unsigned long last;
	unsigned long lastf;
	Servo smotor;

	YC(byte _type, byte _pin, int _minp, int _maxp, unsigned long _freq) {
		type = _type;
		pin = _pin;
		pos = _minp;
		minp = _minp;
		maxp = _maxp;
		freq = _freq;
		switch(type) {
			case RELAY:
			pinMode(pin, OUTPUT);
			digitalWrite(pin, LOW);
			break;

			case SERVO:
			smotor.attach(pin);
			smotor.write(pos);
			break;
		}
	}

	bool update() {
		return update(pos);
	}

	bool update(int _cv) {
		if (ready(&last, freq)) {
			if (_cv < minp)
				pos = minp;
			else if (_cv > maxp)
				pos = maxp;
			else
				pos = _cv;
			switch(type) {
				case RELAY:
				if (pos<=0) {
					digitalWrite(pin, LOW);
					lastf = last;
				} else if (pos>=100) {
					digitalWrite(pin, HIGH);
					lastf = last;
				} else {
					digitalWrite(pin, HIGH);
					lastf = pos * freq / 100 + last - freq;
				}
				break;

				case SERVO:
				smotor.write(pos);
				break;
			}
			return true;
		} else if (type==RELAY && pos < 100 && ready(&lastf, freq)) {
			digitalWrite(pin, LOW);
		} else
			return false;
	}

	~YC() { }
};
YC **actuators; // array of pointers to YC objects
#ifdef DEBUG
byte n_yc = 2;
const unsigned long actuator_freq = 500;
#else
byte n_yc = 2;
const unsigned long actuator_freq = 300000; // 300ks=5min
#endif

// HS-01: select current controller
const byte hs01pin = 8;
byte hs01old = HIGH; // with pull-up resistor, HIGH=open

// HS-02: setpoint
const byte NUMSP = 6;
const byte hs02pin=1;
byte hs02idold = 0;
const int hs02lookup[7] = {0, -100, -10, -1, 1, 10, 100};
const unsigned long KHS02 = 50;
unsigned long khs02old = 0;

// *** Real Time Clock ***
const unsigned long MAXSEC = 86400;
unsigned long rtc = 0; // seconds since midnight
const long hs02seclookup[] = {0, -60*60, -60, -1, 1, 60, 60*60};
unsigned long krtcold = 0;

// TC-01: room temperature control
class AC {
	public:
		AI *pv;
		YC *cv;
		int sp;
		float kp, ki, kd;
		unsigned long freq; //ms
		unsigned long last=0;
		byte nsteps=0;
		byte currstep=0, nextstep=0;
		unsigned long *sched_t; // assumed to be sorted
		int *sched_sp;

	private:
		int range2;

	public:
#ifndef TINKERCAD
		FastPID *pid;
#endif

		AC(AI *_pv, int _sp, YC *_cv, float _kp, float _ki, float _kd, unsigned long _freq) {
			pv = _pv;
			sp = _sp;
			cv = _cv;
			kp = _kp;
			ki = _ki;
			kd = _kd;
			freq = _freq;
			range2 = (pv->pvmax - pv->pvmin)/2;
#ifndef TINKERCAD
			pid = new FastPID(_kp, _ki, _kd, 1000.0/(float)_freq, 8, (cv->minp < 0));
			pid->setOutputRange(cv->minp, cv->maxp);
#endif
		}

		// sched_t and sched_sp are created and initialized by caller, deleted by class
		// shed_t is assumed to be sorted
		// must be called after rtc is initialized
		void assign_schedule(byte _nsteps, unsigned long *_sched_t, int *_sched_sp) {
			if (nsteps != 0) { // schedule update?
				delete[] sched_t;
				delete[] sched_sp;
			}
			nsteps = _nsteps;
			sched_t = _sched_t;
			sched_sp = _sched_sp;
			if (nsteps != 0) {
				if (nsteps == 1) {
					currstep = nextstep = 0;
					sp = sched_sp[0];
				} else {
					int i;
					for (i=0; i<nsteps; i++) // will overflow gracefully
						if (rtc < sched_t[i])
							break;
					if (i == 0)
						currstep = nsteps - 1;
					else
						currstep = i - 1; // including for-loop overflow
					sp = sched_sp[currstep];
					if (currstep == nsteps - 1)
						nextstep = 0;
					else
						nextstep = currstep + 1;
				}
			}
		}

		bool update() {
			// update sp per schedule
			if (nsteps != 0 && rtc >= sched_t[nextstep]) { // assume we can catch it between last step and midnight
				currstep = nextstep;
				if (currstep == nsteps - 1)
					nextstep = 0;
				else
					nextstep = currstep + 1;
				sp = sched_sp[currstep];
			}
			// calculate new cv
			if (ready(&last, freq)) {
				pv->update();
				int error = pv->pv - sp;
#ifdef TINKERCAD
				int _cv = map(error*kp, -range2, range2, cv->maxp, cv->minp);
#else
				int _cv = pid->step(sp, pv->pv);
#endif
				cv->update(_cv);
				return true;
			} else
				return false;
		}

		~AC() {
#ifndef TINKERCAD
			delete pid;
#endif
			if (nsteps != 0) {
				delete[] sched_t;
				delete[] sched_sp;
			}
		}
};
AC **tc;
byte n_tc = n_yc;

void printAvailMemory() {
#ifdef SHOWMEMORY
	Serial.print("Available memory: ");
	Serial.println(freeMemory());
#endif
}

void setup() {
	// *** Serial Part 1 ***
	Serial.begin(115200);
	Serial.println();
	Serial.println();
#ifdef VERBOSESERIAL
	Serial.print("Starting setup at ");
	Serial.println(millis());
#endif
	printAvailMemory();
	delay(100);

	// *** Temp input ***
	ti = new AI*[n_ti]; // array of pointers to AI objects
	ti[0] = new AI(0, 300, 900, 200);
	if (n_ti>=2) {
		ti[1] = new AI(2, 300, 900, 200);
	}

	// *** Actuators ***
	actuators = new YC*[n_yc];
	actuators[0] = new YC(RELAY, 13, 0, 100, 500);
	if (n_yc >= 2) {
		actuators[1] = new YC(SERVO, 6, 0, 180, 500);
		if (n_yc >= 3) {
			actuators[2] = new YC(SERVO, 9, 0, 180, 500);
			if (n_yc >= 4) {
				actuators[3] = new YC(RELAY, 7, 0, 100, 500);
			}
		}
	}

	// *** Setpoint ***
	pinMode(hs01pin, INPUT_PULLUP);

	// *** Controllers ***
	float Kp = 0.1, Ki = 0.1, Kd = 0;
	tc = new AC*[n_tc];
	tc[0] = new AC(ti[0], 680, actuators[0], Kp, Ki, Kd, actuators[0]->freq);
	if (n_tc >= 2) {
		tc[1] = new AC(ti[1], 680, actuators[1], Kp, Ki, Kd, actuators[1]->freq);
		if (n_tc >=3) {
			tc[2] = new AC(ti[0], 680, actuators[2], Kp, Ki, Kd, actuators[2]->freq);
			if (n_tc >= 4) {
				tc[3] = new AC(ti[1], 680, actuators[3], Kp, Ki, Kd, actuators[3]->freq);

			}
		}
	}

	// *** LCD(ncol, nrow) ***
	lcd.begin(16, 2);

	// *** Real-Time Clock ***
	//lcdold[0] = -1; //default
	printAvailMemory();
#ifdef VERBOSESERIAL
	Serial.print("Determining RTC...");
#endif
	byte hs01 = digitalRead(hs01pin);
	while (hs01 == HIGH) {
		updateLCDclock();
		while (!ready(&krtcold, KHS02) && hs01 == HIGH && Serial.available() == 0)
			hs01 = digitalRead(hs01pin);
		unsigned long hs02raw = analogRead(hs02pin);
		if (Serial.available() > 0) {
			unsigned long hr = Serial.parseInt();
			unsigned long min = Serial.parseInt();
			unsigned long sec = Serial.parseInt();
			if (Serial.read() == '\n')
				;
			rtc = (hr*60+min)*60+sec;
			break;
		} else {
			byte hs02id = readAnalogButton(hs02raw);
			if (hs02id != hs02idold) {
				hs02idold = hs02id;
				int hs02delta = hs02seclookup[hs02id];
				if (hs02delta != 0) {
					rtc += hs02delta;
					if (rtc >= 2*MAXSEC)
						rtc += MAXSEC;
					else if (rtc >= MAXSEC)
						rtc -= MAXSEC;
				}
			}
		}
	}
	krtcold = millis();
	updateLCDclock();
#ifdef VERBOSESERIAL
	Serial.print("Done at ");
	Serial.println(millis());
	Serial.print(krtcold);
	Serial.print(" ms = ");
	Serial.print(rtc);
	Serial.println(" sec");
#endif
	printAvailMemory();
	while (hs01 == LOW)
		hs01 = digitalRead(hs01pin);
	hs01old = HIGH;

	// *** Controller schedules ***
#ifdef VERBOSESERIAL
	Serial.print("Creating TC[0] schedule...");
#endif
	byte nsteps = 3;
	unsigned long *sched_t = (unsigned long*)malloc(sizeof(unsigned long)*nsteps);
	int *sched_sp = (int*)malloc(sizeof(int)*nsteps);
	// bedroom
	sched_t[0]=(unsigned long) 7*60*60; sched_sp[0]=620;
	sched_t[1]=(unsigned long)19*60*60; sched_sp[1]=700;
	sched_t[2]=(unsigned long)22*60*60; sched_sp[2]=670;
#ifdef VERBOSESERIAL
	Serial.print("Done at ");
	Serial.println(millis());
#endif
	printAvailMemory();
#ifdef VERBOSESERIAL
	Serial.print("Assigning TC[0] schedule...");
#endif
	tc[0]->assign_schedule(nsteps, sched_t, sched_sp);
#ifdef VERBOSESERIAL
	Serial.print("Done at ");
	Serial.println(millis());
#endif
	printAvailMemory();
	if (n_tc >= 2) {
#ifdef VERBOSESERIAL
		Serial.print("Creating TC[1] schedule...");
#endif
		nsteps = 6;
		sched_t = (unsigned long*)malloc(sizeof(unsigned long)*nsteps);
		sched_sp = (int*)malloc(sizeof(int)*nsteps);
		// living room
		sched_t[0]=(unsigned long)(6*60+30)*60; sched_sp[0]=680;
		sched_t[1]=(unsigned long) 8*60*60; sched_sp[1]=650;
		sched_t[2]=(unsigned long)12*60*60; sched_sp[2]=670;
		sched_t[3]=(unsigned long)13*60*60; sched_sp[3]=650;
		sched_t[4]=(unsigned long)(16*60+30)*60; sched_sp[4]=680;
		sched_t[5]=(unsigned long)22*60*60; sched_sp[5]=620;
#ifdef VERBOSESERIAL
		Serial.print("Done at ");
		Serial.println(millis());
#endif
		printAvailMemory();
#ifdef VERBOSESERIAL
		Serial.print("Assigning TC[1] schedule...");
#endif
		tc[1]->assign_schedule(nsteps, sched_t, sched_sp);
#ifdef VERBOSESERIAL
		Serial.print("Done at ");
		Serial.println(millis());
#endif
		printAvailMemory();
		if (n_tc >= 3) {
#ifdef VERBOSESERIAL
			Serial.print("Creating TC[2] schedule...");
#endif
			nsteps = 2;
			sched_t = (unsigned long*)malloc(sizeof(unsigned long)*nsteps);
			sched_sp = (int*)malloc(sizeof(int)*nsteps);
			// office
			sched_t[0]=(unsigned long) 8*60*60; sched_sp[0]=670;
			sched_t[1]=(unsigned long)17*60*60; sched_sp[1]=620;
#ifdef VERBOSESERIAL
			Serial.print("Done at ");
			Serial.println(millis());
#endif
			printAvailMemory();
#ifdef VERBOSESERIAL
			Serial.print("Assigning TC[2] schedule...");
#endif
			tc[2]->assign_schedule(nsteps, sched_t, sched_sp);
#ifdef VERBOSESERIAL
			Serial.print("Done at ");
			Serial.println(millis());
#endif
			printAvailMemory();
			if (n_tc >= 4) {
#ifdef VERBOSESERIAL
				Serial.print("Creating TC[3] schedule...");
#endif
				nsteps = 2;
				sched_t = (unsigned long*)malloc(sizeof(unsigned long)*nsteps);
				sched_sp = (int*)malloc(sizeof(int)*nsteps);
				// office #2
				sched_t[0]=(unsigned long) 8*60*60; sched_sp[0]=670;
				sched_t[1]=(unsigned long)17*60*60; sched_sp[1]=620;
#ifdef VERBOSESERIAL
				Serial.print("Done at ");
				Serial.println(millis());
#endif
				printAvailMemory();
#ifdef VERBOSESERIAL
				Serial.print("Assigning TC[3] schedule...");
#endif
				tc[3]->assign_schedule(nsteps, sched_t, sched_sp);
#ifdef VERBOSESERIAL
				Serial.print("Done at ");
				Serial.println(millis());
#endif
				printAvailMemory();
			}
		}
	}

	// *** Initial LCD text ***
	lcd.setCursor(10, 0);
	lcd.print("CV");
	int _cv = tc[lcd_tc]->cv->pos;
	//lcdold[4] = 0; //default
	lcd.print(_cv);
	lcdold[1] = _cv;
	lcd.setCursor(0, 1);
	lcd.print(lcd_tc);
	lcd.print(" PV");
	int _pv = tc[lcd_tc]->pv->pv;
	lcd.print(_pv);
	lcdold[2] = _pv;
	lcd.setCursor(10, 1);
	lcd.print("SP");
	int _sp = tc[lcd_tc]->sp;
	lcd.print(_sp);
	lcdold[3] = _sp;

	// *** Serial Part 2 ***
	delay(50);
	for (int i=0; i<n_tc; i++) {
		Serial.print("pv");
		Serial.print(i);
		Serial.print("\tsp");
		Serial.print(i);
		Serial.print("\tcv");
		Serial.print(i);
		Serial.print("\t");
		delay(50);
	}
	Serial.println("");
	for (int i=0; i<n_tc; i++) {
		Serial.print(tc[i]->pv->pv);
		Serial.print('\t');
		Serial.print(tc[i]->sp);
		Serial.print('\t');
		Serial.print(tc[i]->cv->pos);
		Serial.print('\t');
		delay(50);
	}
	delay(200);
	Serial.println("");
	delay(50);
#ifdef VERBOSESERIAL
	Serial.print("Setup...Done at ");
	Serial.println(millis());
#endif
	printAvailMemory();
	delay(250);
}

void loop() {
	unsigned long now;

	// *** Input PV ***
	for (int i=0; i<n_ti; i++)
		ti[i]->update();

	// *** Input SP***
	if (ready(&khs02old, KHS02)) {
		byte hs01 = digitalRead(hs01pin);
		if (hs01 != hs01old) {
			if (hs01 == LOW) { // with pull-up resistor, LOW=closed
				lcd_tc++;
				if (lcd_tc >= n_tc)
					lcd_tc = 0;
			}
			hs01old = hs01;
		}
		unsigned long hs02raw = analogRead(hs02pin);
		byte hs02id = readAnalogButton(hs02raw);
		if (hs02id != hs02idold) {
			hs02idold = hs02id;
			int hs02delta = hs02lookup[hs02id];
			if (hs02delta != 0) {
				int *sp = &(tc[lcd_tc]->sp);
				*sp += hs02delta;
				if (*sp > tc[lcd_tc]->pv->pvmax)
					*sp = tc[lcd_tc]->pv->pvmax;
				else if (*sp < tc[lcd_tc]->pv->pvmin)
					*sp = tc[lcd_tc]->pv->pvmin;
			}
		}
	}

	// *** Calculate CV and Output Heater ***
	for (int i=0; i<n_tc; i++)
		tc[i]->update();

	// *** Output Heater ***
	//for (int i=0; i<n_yc; i++)
	//actuators[i]->update();

	// *** Real Time Clock ***
	if (ready(&krtcold, 1000)) {
		rtc++;
		if (rtc >= MAXSEC)
			rtc -= MAXSEC;
		updateLCDclock();
	}

	// *** Output LCD & plot ***
	if (ready(&kloopold, KLOOP)) {
		int _cv = tc[lcd_tc]->cv->pos;
		int _pv = tc[lcd_tc]->pv->pv;
		int _sp = tc[lcd_tc]->sp;
		// lcdold[0] updated in RTC above
		if (_cv != lcdold[1]) {
			lcd.setCursor(10, 0);
			lcd.print("CV   ");
			lcd.setCursor(12, 0);
			lcd.print(_cv);
			lcdold[1] = _cv;
		}
		if (lcd_tc != lcdold[4]) {
			lcd.setCursor(0, 1);
			lcd.print(lcd_tc);
			lcdold[4] = lcd_tc;
		}
		if (_pv != lcdold[2]) {
			lcd.setCursor(2, 1);
			lcd.print("PV   ");
			lcd.setCursor(4, 1);
			lcd.print(_pv);
			lcdold[2] = _pv;
		}
		if (_sp != lcdold[3]) {
			lcd.setCursor(10, 1);
			lcd.print("SP   ");
			lcd.setCursor(12, 1);
			lcd.print(_sp);
			lcdold[3] = _sp;
		}
#ifdef VERBOSESERIAL
		Serial.print(millis());
		Serial.print('\t');
		Serial.print(rtc / 60 / 60);
		Serial.print('\t');
		Serial.print((rtc / 60) % 60);
		Serial.print('\t');
		Serial.print(rtc % 60);
		Serial.print('\t');
#endif
		for (int i=0; i<n_tc; i++) {
			Serial.print(tc[i]->pv->pv);
			Serial.print('\t');
			Serial.print(tc[i]->sp);
			Serial.print('\t');
			Serial.print(tc[i]->cv->pos);
			Serial.print('\t');
		}
		Serial.println("");
		printAvailMemory();
	}
}

// determine if we are ready to execute something with a given period, given the last time we executed it
// if we are ready, update kold to new execution benchmark
// if period is not provided (or 0), ready will always return true and kold updated to millis()
// handle overflows (~50 days)
// try to be fast
bool ready(unsigned long *kold, unsigned long period=0) {
	if (period==0) {
		*kold=millis();
		return true;
	} else {
		unsigned long nextrun = *kold + period;
		bool overflow_reqd = nextrun < *kold;
		unsigned long now = millis(); // millis() may change during execution
		bool overflow = now < *kold;
		bool elapsed = now >= nextrun;

		if (elapsed && (overflow || !overflow_reqd)) {
			*kold = nextrun;
			ready(kold, period); // recursively update kold
			return true;
		} else
			return false;
	}
	return false; // unexpected
}

byte readAnalogButton(unsigned long val) {
	float avg = (float)AIRES2 / (float)(NUMSP+1);

	if (val < 0.5*avg)
		return 0;
	for (int currbutton=0; currbutton<NUMSP; currbutton++)
		if (val < ((float)currbutton + 1.5)*avg)
			return NUMSP-currbutton;

	return 0;
}

void printVal(char* str, long val) {
	Serial.print(str);
	Serial.println(val);
}

void updateLCDclock() {
	byte hr, min, sec;
	sec = rtc % 60;
	min = (rtc / 60) % 60;
	hr = rtc / 60 / 60;
	if (rtc != lcdold[0]) {
		lcd.setCursor(0, 0);
		lcd.print("  :  :  ");
		lcd.setCursor(0, 0);
		if (hr < 10)
			lcd.print("0");
		lcd.print(hr);
		lcd.setCursor(3, 0);
		if (min < 10)
			lcd.print("0");
		lcd.print(min);
		lcd.setCursor(6, 0);
		if (sec < 10)
			lcd.print("0");
		lcd.print(sec);
		lcdold[0] = rtc;
	}
}
