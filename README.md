# thermostat
Arduino thermostat

Designed to be hardware-agnostic, but tested with Arduino Uno and other hardware from Arduino Start Kit.<br>
Prototyping and hardware design at: https://www.tinkercad.com/things/53NZLudUo1p

Note: all temperature values are intended to be in 0.1 degF, but can be easily adjusted. Integer math used for speed and simplicity, and to utilize the FastPID library: https://github.com/mike-matera/FastPID

Feature list
============
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
 * schedule (untested)
 * TODO AI calibration

Inputs and Outputs
==================
 * AI's: 0-5V temperature sensor(s) based on TMP36
 * YC's: On-off heater connected to relay, LED (with resistor in series), or servo motor based on SM-S2309S

Common Terminology
==================
 * AI = analog input (physical device)
 * AC = analog controller (software device)
 * CV = control value (controller output)
 * PID = proportional, integral, derivative controller
 * PV = process value (controller input)
 * TC = temperature controller (an AC)
 * TI = temperature indicator (a PV)
 * SP = setpoint (controller input)
 * YC = actuator (physical device, e.g. motor, relay)
