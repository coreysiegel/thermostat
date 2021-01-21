// Note: all temperature values are in 0.1 degF

/* *** Feature list ***
 * Temp sensing via AI
 * Moving low-pass filter for AI
 * AI calibration
 * SP adjustment via pushbuttons on voltage divider into AI
 * TODO use median of multiple AIs per AC
 * TODO PID control
 * TODO PID loop tuning
 * relay control
 * TODO stepper motor control
 * TODO servo motor control
 * LCD printout
 * serial output
 * serial output graphing
 * multiple zones
 * clock
 * TODO real-time clock
 * TODO scheduled temp sensing
 * TODO EEPROM (for cals, SPs)
 * ******************** */

/* *** Physical I/O ***
 * Analog pins
 * 0 TI-01
 * 1 HS-02 (6-button array for setpoint or cal pv adjustment)
 * 2 TI-02
 * 3 TI-03
 * 4 TI-04
 * 5 Not connected
 * Note: Quantity of temperature sensors is specified by "n_ti"
 * 
 * Digital pins
 * 0-1 Reserved (Serial)
 * 2-5 LCD
 * 6 YC-02 (Servo from TC/TI-02)
 * 7 HS-03 (Enter calibration mode, or exit cal mode and save cal info)
 * 8 HS-01 (change current TC or cal point)
 * 9 YC-03 (Servo from TC-03/TI-01)
 * 10 YC-04 (Relay from TC-04/TI-02)
 * 11-12 LCD
 * 13 YC-01 (Relay from TC/TI-01)
 * Note: If HS-01 is held and then HS-03 is pressed, controller will exit 
 * calibration mode without saving current cal info
 * Note: Quantity of actuators (and corresponding software controllers is
 * specified by "n_yc"
 * 
 * Power
 * 3.3V: Not connected
 * 5V: TIs, YC-03, YC-04, LCD, HS-02
 * Gnd: TIs, YCs, LCD, HSs
 * 
 * USB
 * PC USB port for power and data logging (CSV with 1 header row)
 * ******************** */

#define TINKERCAD
//#define SHOWMEMORY

const byte n_ti = 4; // PVs
const unsigned long ti_freq = (unsigned long)5*60*1000;
const byte pv_filt_len = 1; // number of samples in moving low-pass filter
const long cal[4] = {119, 223, 585, 720}; // initial PV cal points {raw1, raw2, pv1, pv2}
const byte n_yc = n_ti; // actuators
const unsigned long actuator_freq = (unsigned long)5000; // also for tc's
const unsigned long KHS02 = 50; // all pushbuttons
const unsigned long KLCD = 1000; // LCD
const unsigned long KSERIAL = ti_freq;
#ifdef TINKERCAD
const float Kp=1, Ki=0, Kd=0;
#else
const float Kp=0.1, Ki=0.3, Kd=0;
#endif

// https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
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
void printClockToLCD(unsigned long, boolean = false);

// Serial
// http://www.arduino.cc/en/Tutorial/AnalogReadSerial
const bool serialplot = true; // false = verbose text output
unsigned long kserialold = 0;

// LCD
// http://www.arduino.cc/en/Tutorial/LiquidCrystalHelloWorld
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
int lcdold[] = {0, 0, 0, 0, 0};
byte lcd_tc = 0;
boolean lcd_cal = false;
unsigned long klcdold = 0;

// analog input setup
//const unsigned long AIRES = 358;
const float ANALOGBUTTONSLOPE = -132;
const float ANALOGBUTTONINTERCEPT = 996;

// analog input calibration
byte ai_cal = false;
const byte hs03pin = 7;
byte hs03old = HIGH; // with pull-up resistor, HIGH=open
byte curr_cal_val = 0;
long curr_cal_raw;
long curr_cal_pv;
// timing: use HS-02

class AI {
	public:
		byte pin;
		unsigned long raw = 0;
		unsigned long raw_acc = 0;
		unsigned long raw_arr[pv_filt_len];
		byte curr_raw_id = 0;

		int pv;
		int pvmin;
		int pvmax;

		unsigned long freq; // ms
		unsigned long last=0;

		long cal[4] = {0, 1023, 300, 900};

		AI(byte _pin, int _pvmin, int _pvmax, unsigned long _freq) {
			this->pin = _pin;
			this->pvmin = _pvmin;
			this->pvmax = _pvmax;
			this->freq = _freq;
			for (int i=0; i<pv_filt_len; i++)
				this->raw_arr[i] = 0;
			for (int i=0; i<pv_filt_len; i++)
				this->forceupdate();
		}

		void updateCal(byte id, long raw, long pv) {
			if (id>=0 && id<=1) {
				this->cal[id] = raw;
				this->cal[id+2] = pv;
				this->forceupdate();
			}
		}

		void updateCal(long raw1, long raw2, long pv1, long pv2) {
			this->cal[0] = raw1;
			this->cal[1] = raw2;
			this->cal[2] = pv1;
			this->cal[3] = pv2;
			this->forceupdate();
		}

		bool update() {
			if (ready(&this->last, this->freq)) {
				this->forceupdate();
				return true;
			}
			else
				return false;
		}

	private:
		void forceupdate() {
			this->curr_raw_id++;
			if (this->curr_raw_id >= pv_filt_len)
				this->curr_raw_id = 0;
			this->raw_acc -= this->raw_arr[this->curr_raw_id];
			this->raw_arr[this->curr_raw_id] = analogRead(this->pin);
			this->raw_acc += this->raw_arr[this->curr_raw_id];
			this->raw = this->raw_acc / pv_filt_len;
			this->pv = map(this->raw, this->cal[0], this->cal[1], this->cal[2], this->cal[3]);
		}

		~AI() { }
};
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
		unsigned long last=0;
		unsigned long lastf=0;
		Servo smotor;

		YC(byte _type, byte _pin, int _minp, int _maxp, unsigned long _freq) {
			this->type = _type;
			this->pin = _pin;
			this->pos = _minp;
			this->minp = _minp;
			this->maxp = _maxp;
			this->freq = _freq;
			switch(this->type) {
				case RELAY:
					pinMode(this->pin, OUTPUT);
					digitalWrite(this->pin, LOW);
					break;

				case SERVO:
					this->smotor.attach(this->pin);
					this->smotor.write(this->pos);
					break;
			}
		}

		bool update(int _cv) {
			this->pos = constrain(_cv, this->minp, this->maxp);
			return this->update();
		}

		bool update() {
			if (ready(&this->last, this->freq)) {
				//Serial.println("Updating actuator");
				switch(this->type) {
					case RELAY:
						if (this->pos<=0) {
							digitalWrite(this->pin, LOW);
							//Serial.println("Relay de-energized");
							this->lastf = this->last+this->freq;
						} else if (this->pos>=100) {
							digitalWrite(this->pin, HIGH);
							//Serial.println("Relay energized");
							this->lastf = this->last+this->freq;
						} else {
							digitalWrite(this->pin, HIGH);
							//Serial.println("Relay energized");
							this->lastf = this->pos * this->freq / 100 + this->last - this->freq;
						}
						break;

					case SERVO:
						this->smotor.write(this->pos);
						break;
				}
				return true;
			} else if (this->type==RELAY && this->pos < 100 && ready(&this->lastf, this->freq)) {
				digitalWrite(this->pin, LOW);
				//Serial.println("Relay de-energized");
				return true;
			} else
				//Serial.println("No update");
				return false;
		}

		~YC() { }
};
YC **actuators; // array of pointers to YC objects

// HS-01: select current controller
const byte hs01pin = 8;
byte hs01old = HIGH; // with pull-up resistor, HIGH=open
// timing: use HS-02

// HS-02: setpoint
const byte NUMSP = 6;
const byte hs02pin=1;
byte hs02idold = 0;
const int hs02lookup[7] = {0, -100, -10, -1, 1, 10, 100};
unsigned long khs02old = 0;

// TC-01: room temperature control
class AC {
	public:
		AI *pv;
		YC *cv;
		int sp;
		float kp, ki, kd;
		unsigned long freq; //ms
		unsigned long last=0;

	private:
		int range2;

	public:
#ifndef TINKERCAD
		FastPID *pid;
#endif

		AC(AI *_pv, int _sp, YC *_cv, float _kp, float _ki, float _kd, unsigned long _freq) {
			this->pv = _pv;
			this->sp = _sp;
			this->cv = _cv;
			this->kp = _kp;
			this->ki = _ki;
			this->kd = _kd;
			this->freq = _freq;
			this->range2 = (this->pv->pvmax - this->pv->pvmin)/2;
			//printVal("Range2: ", range2);
#ifndef TINKERCAD
			this->pid = new FastPID(_kp, _ki, _kd, 1000.0/(float)_freq, 8, (this->cv->minp < 0));
			this->pid->setOutputRange(this->cv->minp, this->cv->maxp);
#endif
		}

		bool update() {
			if (ready(&this->last, this->freq)) {
				this->pv->update();
				int error = this->pv->pv - this->sp;
				//printVal("Error: ", error);
				int _cv;
				if (error <= 0)
					_cv = this->cv->maxp;
				else
					_cv = this->cv->minp;
				//printVal("CV: ", _cv);
				this->cv->update(_cv);
				return true;
			} else
				return false;
		}

		~AC() {
#ifndef TINKERCAD
			delete this->pid;
#endif
		}
};
AC **tc;
const byte n_tc = n_yc;

void setup() {
	Serial.begin(115200);
	Serial.println("");
	Serial.println("");
#ifdef SHOWMEMORY
	Serial.print("Available memory: ");
	Serial.println(freeMemory());
#endif

	// Temp input
	ti = new AI*[n_ti]; // array of pointers to AI objects
	ti[0] = new AI(0, 300, 900, ti_freq);
	ti[0]->updateCal(cal[0], cal[1], cal[2], cal[3]);
	if (n_ti>=2) {
		ti[1] = new AI(2, 300, 900, ti_freq);
		ti[1]->updateCal(cal[0], cal[1], cal[2], cal[3]);
		if (n_ti>=3) {
			ti[2] = new AI(3, 300, 900, ti_freq);
			ti[2]->updateCal(cal[0], cal[1], cal[2], cal[3]);
			if (n_ti>=4) {
				ti[3] = new AI(4, 300, 900, ti_freq);
				ti[3]->updateCal(cal[0], cal[1], cal[2], cal[3]);
			}
		}
	}

	// Actuators
	actuators = new YC*[n_yc];
	actuators[0] = new YC(RELAY, 13, 0, 100, actuator_freq);
	if (n_yc >= 2) {
		actuators[1] = new YC(SERVO, 6, 0, 180, actuator_freq);
		if (n_yc >= 3) {
			actuators[2] = new YC(SERVO, 9, 0, 180, actuator_freq);
			if (n_yc >= 4) {
				actuators[3] = new YC(RELAY, 10, 0, 100, actuator_freq);
			}
		}
	}

	// Setpoint and Calibration
	pinMode(hs01pin, INPUT_PULLUP);
	pinMode(hs03pin, INPUT_PULLUP);

	// Controllers
	tc = new AC*[n_tc];
	tc[0] = new AC(ti[0], 680, actuators[0], Kp, Ki, Kd, actuators[0]->freq);
	if (n_tc >= 2) {
		tc[1] = new AC(ti[1], 680, actuators[1], Kp, Ki, Kd, actuators[1]->freq);
		if (n_tc >=3) {
			if (n_ti >= 3)
				tc[2] = new AC(ti[2], 680, actuators[2], Kp, Ki, Kd, actuators[2]->freq);
			else
				tc[2] = new AC(ti[0], 680, actuators[2], Kp, Ki, Kd, actuators[2]->freq);
			if (n_tc >= 4) {
				if (n_ti >= 4)
					tc[3] = new AC(ti[3], 680, actuators[3], Kp, Ki, Kd, actuators[3]->freq);
				else
					tc[3] = new AC(ti[1], 680, actuators[3], Kp, Ki, Kd, actuators[3]->freq);
			}
		}
	}

	// LCD(ncol, nrow)
	lcd.begin(16, 2);
	// Initial LCD text
	printClockToLCD(0, true);
	lcd.setCursor(10, 0);
	lcd.print("CV");
	int _cv = tc[lcd_tc]->cv->pos;
	lcd.print(_cv);
	lcdold[1] = _cv;
	lcd.setCursor(0, 1);
	lcd.print(lcd_tc);
	//lcdold[4] = 0; //default
	lcd.print(" PV");
	int _pv = tc[lcd_tc]->pv->pv;
	lcd.print(_pv);
	lcdold[2] = _pv;
	lcd.setCursor(10, 1);
	lcd.print("SP");
	int _sp = tc[lcd_tc]->sp;
	lcd.print(_sp);
	lcdold[3] = _sp;

	// Serial
	Serial.print("hr min sec ");
	for (int i=0; i<n_tc; i++) {
		Serial.print("pv");
		Serial.print(i);
		Serial.print(" sp");
		Serial.print(i);
		Serial.print(" cv");
		Serial.print(i);
		Serial.print(' ');
	}
	Serial.println("");
	Serial.print("0 0 0 ");
	for (int i=0; i<n_tc; i++) {
		Serial.print(tc[i]->pv->pv);
		Serial.print(' ');
		Serial.print(tc[i]->sp);
		Serial.print(' ');
		Serial.print(tc[i]->cv->pos);
		Serial.print(' ');
	}
	Serial.println("");
#ifdef SHOWMEMORY
	Serial.print("Available memory: ");
	Serial.println(freeMemory());
#endif

}

unsigned long now = 0;
byte hs01 = 0;
unsigned long hs02raw = 0;
byte hs03 = 0;
byte hs02id = 0;
int hs02delta = 0;
int _cv = 0;
int _pv = 0;
int _sp = 0;

void loop() {

	/* *** Input PV *** */
	if (! ai_cal)
		for (int i=0; i<n_ti; i++)
			ti[i]->update();

	/* *** Input SP or Calibration *** */
	if (ready(&khs02old, KHS02)) {
		hs01 = digitalRead(hs01pin);
		hs02raw = analogRead(hs02pin);
		hs03 = digitalRead(hs03pin);
		hs02id = readAnalogButton(hs02raw);
		if (hs03 == LOW) { // if pushed, don't process other buttons
			if (hs03 != hs03old) {
				ai_cal = !ai_cal;
				if (ai_cal) { // initialize cal info
					curr_cal_val = 0;
					curr_cal_raw = tc[lcd_tc]->pv->raw;
					curr_cal_pv = tc[lcd_tc]->pv->pv;
				} else { // save cal info if HS-01 is not pushed
					if (hs01 == HIGH)
						tc[lcd_tc]->pv->updateCal(curr_cal_val, curr_cal_raw, curr_cal_pv);
				}
			}
		} else {
			if (ai_cal) { // calibrate
				// HS-01 (change current TC) is disabled
				if (hs01 != hs01old && hs01 == LOW) { // advance cal value of interest
					curr_cal_val++;
					if (curr_cal_val >= 2)
						curr_cal_val = 0;
				}
				if (hs02id != hs02idold) {
					hs02delta = hs02lookup[hs02id];
					if (hs02delta != 0) {
						curr_cal_pv += hs02delta;
					}
				}
			} else { // adjust setpoint
				if (hs01 != hs01old && hs01 == LOW) { // advance controller of interest
					lcd_tc++;
					if (lcd_tc >= n_tc)
						lcd_tc = 0;
				}
				if (hs02id != hs02idold) {
					hs02delta = hs02lookup[hs02id];
					if (hs02delta != 0) {
						int *sp = &(tc[lcd_tc]->sp);
						*sp += hs02delta;
						*sp = constrain(*sp, tc[lcd_tc]->pv->pvmin, tc[lcd_tc]->pv->pvmax);
					}
				}
			}
		}
		hs01old = hs01;
		hs02idold = hs02id;
		hs03old = hs03;
	}

	/* *** Calculate CV *** */
	for (int i=0; i<n_tc; i++)
		tc[i]->update();

	/* *** Output Heater *** */
	for (int i=0; i<n_yc; i++)
		actuators[i]->update();

	/* *** Output LCD *** */
	if (ready(&klcdold, KLCD)) {
		now = millis()/1000;
		_cv = tc[lcd_tc]->cv->pos;
		//_cv = hs02raw;
		_pv = tc[lcd_tc]->pv->pv;
		_sp = tc[lcd_tc]->sp;
		if (lcd_cal != ai_cal) {
			lcd.clear();

			lcd.setCursor(0, 0);

			if (ai_cal) {
				lcd.print("RAW");
				lcd.print(tc[lcd_tc]->pv->cal[curr_cal_val]);

				lcd.setCursor(8,0);
				lcd.print("PV");
				lcd.print(tc[lcd_tc]->pv->cal[curr_cal_val+2]);

				lcd.setCursor(0, 1);
				lcd.print(lcd_tc);
				lcd.print(curr_cal_val+1);
				lcdold[4] = curr_cal_val+1;

				lcd.setCursor(3, 1);
				lcd.print(curr_cal_raw);
				lcdold[0] = curr_cal_raw;

				lcd.setCursor(10, 1);
				lcd.print(curr_cal_pv);
				lcdold[1] = curr_cal_pv;
			} else {
				printClockToLCD(now, true);

				lcd.setCursor(10, 0);
				lcd.print("CV");
				lcd.print(_cv);
				lcdold[1] = _cv;

				lcd.setCursor(0, 1);
				lcd.print(lcd_tc);
				lcd.print(" PV");
				_pv = tc[lcd_tc]->pv->pv;
				lcd.print(_pv);
				lcdold[2] = _pv;
				lcd.setCursor(10, 1);
				lcd.print("SP");
				_sp = tc[lcd_tc]->sp;
				lcd.print(_sp);
				lcdold[3] = _sp;
			}
			lcd_cal = ai_cal;
		} else {
			if (ai_cal) {
				if (curr_cal_raw != lcdold[0]) {
					lcd.setCursor(3, 1);
					lcd.print("    ");
					lcd.setCursor(3, 1);
					lcd.print(curr_cal_raw);
					lcdold[0] = curr_cal_raw;
				}
				if (curr_cal_pv != lcdold[1]) {
					lcd.setCursor(10, 1);
					lcd.print("   ");
					lcd.setCursor(10, 1);
					lcd.print(curr_cal_pv);
					lcdold[0] = curr_cal_pv;
				}
				if (curr_cal_val+1 != lcdold[4]) {
					lcd.setCursor(3, 0);
					lcd.print("    ");
					lcd.setCursor(3, 0);
					lcd.print(tc[lcd_tc]->pv->cal[curr_cal_val]);

					lcd.setCursor(10, 0);
					lcd.print("   ");
					lcd.setCursor(10, 0);
					lcd.print(tc[lcd_tc]->pv->cal[curr_cal_val+2]);

					lcd.setCursor(1, 1);
					lcd.print(curr_cal_val+1);
					lcdold[4] = curr_cal_val+1;
				}
			} else {
				if (now != lcdold[0])
					printClockToLCD(now);

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
			}
		}
	}

	/* *** Output Serial *** */
	if (ready(&kserialold, KSERIAL)) {
		now = millis()/1000;
		Serial.print(now/60/60);
		Serial.print(' ');
		Serial.print(now/60%60);
		Serial.print(' ');
		Serial.print(now%60);
		Serial.print(' ');
		for (int i=0; i<n_tc; i++) {
			Serial.print(tc[i]->pv->pv);
			Serial.print(' ');
			Serial.print(tc[i]->sp);
			Serial.print(' ');
			Serial.print(tc[i]->cv->pos);
			Serial.print(' ');
		}
		Serial.println("");
#ifdef SHOWMEMORY
		Serial.print("Available memory: ");
		Serial.println(freeMemory());
#endif
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
	if (val < 6.5*ANALOGBUTTONSLOPE + ANALOGBUTTONINTERCEPT)
		return 0;
	for (int currbutton=1; currbutton<=NUMSP; currbutton++)
		if (val > ((float)currbutton + 0.5)*ANALOGBUTTONSLOPE + ANALOGBUTTONINTERCEPT)
			return currbutton;

	return 0;
}

void printVal(char* str, long val) {
	Serial.print(str);
	Serial.println(val);
}

void printClockToLCD(unsigned long now, boolean forceupdate) {
	byte hr, min, sec, hrold, minold, secold;
	hrold = lcdold[0]/60/60;
	minold = lcdold[0]/60%60;
	secold = lcdold[0]%60;
	hr = now/60/60;
	min = now/60%60;
	sec = now%60;
	if (forceupdate || hr != hrold) {
		lcd.setCursor(0, 0);
		lcd.print("  ");
		lcd.setCursor(0, 0);
		if (hr < 10)
			lcd.print(' ');
		lcd.print(hr);
	}
	if (forceupdate)
		lcd.print(':');
	if (forceupdate || min != minold) {
		lcd.setCursor(3, 0);
		lcd.print("  ");
		lcd.setCursor(3, 0);
		if (min < 10)
			lcd.print('0');
		lcd.print(min);
	}
	if (forceupdate)
		lcd.print(':');
	if (forceupdate || sec != secold) {
		lcd.setCursor(6, 0);
		lcd.print("  ");
		lcd.setCursor(6, 0);
		if (sec < 10)
			lcd.print('0');
		lcd.print(sec);
	}
	lcdold[0] = now;
}
