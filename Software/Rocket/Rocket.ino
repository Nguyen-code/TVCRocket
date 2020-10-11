/*
By Aaron Becker

rocket notes:

- write page of flash using struct
- USE i2c for 
- kalman filter - really high acceleration events - throw out bad data

Sensor List

IMU: BMI088

ms5611 https://github.com/jarzebski/Arduino-MS5611


update todo notes:
2nd adc support

buzzer enum state
tvc enabled enum state
fix odd bug with lastRadioRecievedTimer and state changed based on telconn state


Thrust curve interpolation
https://github.com/luisllamasbinaburo/Arduino-Interpolation


optimizations:
telem can use different packet struct?
potential for PWM resolution changing? https://www.pjrc.com/teensy/td_pulse.html


things to test:
pyro channels (all 5)
servo movement and centering
PID values (duh)
flash chip!
sd!
max travel constants
reset I in PID before liftoff
IMU use interrupts

self test where rocket checks the following:
- rail voltages ok
- batt v within range
- pyro continuous vs short detect - make sure not shorted

zeroing

ori stuff
1) start in calibrate mode on pad
2) complementary filter while on pad
3) zero roll and PID I value right before launch
4) launch go gyro only
Also use interrupts for sensor

make constant for servo angle to tvc mount angle

*/


//External libs
#include <SPI.h>
#include <SD.h>
#include "libs/ADS1X15/ADS1X15.cpp"
#include <Servo.h>
#include <RH_RF95.h>
#include "libs/PID/pid.cpp"
#include "libs/BMI088/BMI088.cpp"
#include "libs/Orientation/Quaternion.cpp"
#include "libs/Orientation/Orientation.cpp"

//Internal header files
#include "states.h"
#include "pindefs.h"
#include "constants.h"

/*
DEBUG
*/

//#define DEBUG


#ifdef DEBUG
	#define debugPrintln(x)  Serial.println (x)
	#define debugPrint(x)  Serial.print(x)
	#include "printf.h"
	const boolean debug = true;
#else
	#define debugPrint(x)
	#define debugPrintln(x)
	const boolean debug = false;
#endif


/*
SD CARD
*/

Sd2Card card;
SdVolume volume;
SdFile root;

/*
FLIGHT MODE ENUMS
*/
FlightMode flightMode = BOOTING;
PyroStates pyroState = PY_DISARMED;
ChuteStates chuteState = C_DISARMED;
DataLoggingStates dataLoggingState = DL_DISABLED;
TelemSendStates telemetryState = TEL_DISABLED;
TelemConnStates telemetryConnectionState = TEL_DISCONNECTED;
OriMode oriMode = INIT;

/*
RADIO
*/
RH_RF95 radio(RADIO_CS, RADIO_IRQ);

struct RadioPacket {
	byte id;
	byte subID;
	float data1;
	float data2;
	float data3;
};

/*
Radio Command List
0: Ping/pong check (heartbeat)
1: GetRocketState
2: SetRocketState
3: RocketTelem
sub IDs:
	0 - time since startup, MET, loop freq
	1 - flightmode, pyrostates, chutestates
	2 - dataloggingstates, telemsendstates, telemconnstates
	3 - battv, servov, rollmotorv
	4 - boardtemp, gpsfix, gpssats
	5 - gyro x, y, z
	6 - acc x, y, z
	7 - mag x, y, z
	8 - gnss lat, gnss lon, alt
	9 - ori x, y, z
	10 - pos x, y, z
	11 - vel x, y, z
	12 - pyro1 cont, pyro2 cont, pyro3 cont
	13 - pyro4 cont, pyro5 cont, blank
	14 - pyro1 fire, pyro2 fire, pyro3 fire
	15 - pyro4 fire, pyro5 fire, blank	
	16 - tvc x, tvc y, blank
4: RequestRocketTelem
5: Enable/Disable Pyros
6: FirePyro
*/

typedef enum {
	HEARTBEAT = 0,
	GETSTATE = 1,
	SETSTATE = 2,
	GETTELEM = 3,
	REQTELEM = 4,
	PYROARMING = 5,
	FIREPYRO = 6
} RadioCommands;

const int radioTimeoutDelay = 5000;
long lastRadioRecieveTime = (long)-radioTimeoutDelay;

/*
TELEMETRY STRUCT
*/

struct TELEMETRY {
	//Raw vehicle information
	unsigned long timeSinceStartup;
	unsigned long missionElapsedTime;
	double guidanceFrequency;

	//State stuff
	FlightMode fMode;
	PyroStates pState;
	ChuteStates cState;
	DataLoggingStates dState;
	TelemSendStates tSState;
	TelemConnStates tCState;

	//Basic sensor data
	float battV;
	float servoV;
	float rollMotorV;
	float boardTemp;

	//Raw sensor data
	double gyroX;
	double gyroY;
	double gyroZ;
	double accX;
	double accY;
	double accZ;
	double magX;
	double magY;
	double magZ;
	double GNSSLat;
	double GNSSLon;
	double alt;

	//Calculated state vector data
	double oriX;
	double oriY;
	double oriZ;
	double posX;
	double posY;
	double posZ;
	double velX;
	double velY;
	double velZ;

	//TVC Data
	double tvcX;
	double tvcY;
	
	//GNSS locking data
	byte gpsFix;
	int gpsSats;

	//Pyro channel data
	bool pyro1Cont;
	bool pyro2Cont;
	bool pyro3Cont;
	bool pyro4Cont;
	bool pyro5Cont;
	bool pyro1Fire;
	bool pyro2Fire;
	bool pyro3Fire;
	bool pyro4Fire;
	bool pyro5Fire;
};
struct TELEMETRY telem;

unsigned long lastTelem = 0;
unsigned long lastHeartbeat = 0;
const int heartbeatDelay = 200;

/*
ADC CONFIG
*/

ADS1015 ADS1(ADC1_I2C_ADDR);
ADS1015 ADS2(ADC2_I2C_ADDR);

/*
GYRO/ACCELEROMETER SETUP
*/
Bmi088Accel accel(Wire,ACC_I2C_ADDR);
Bmi088Gyro gyro(Wire,GYRO_I2C_ADDR);

/*
BUZZER CONFIG
*/

unsigned long lastBuzzer = 0;
int buzzerDelay = 1000;

const byte toneBufferLength = 20;
int toneFreqQueue[toneBufferLength];
int toneDelayQueue[toneBufferLength];
unsigned long lastToneStart = 0;
byte toneStackPos = 0;

/*
LED CONFIG
*/

unsigned long lastLED = 0;
int LEDDelay = 1000;

const byte ledBufferLength = 20;
int ledRQueue[ledBufferLength];
int ledGQueue[ledBufferLength];
int ledBQueue[ledBufferLength];
int ledDelayQueue[ledBufferLength];
unsigned long lastLEDStart = 0;
byte ledStackPos = 0;

/*
PYRO CHANNEL CONFIG
*/

unsigned long pyroOffTimes[5];
bool pyroStates[5];
int pyrosInUse = 0;

/*
TVC SERVOS CONFIG
*/
Servo TVC_X_CH1;
Servo TVC_Y_CH1;
Servo TVC_X_CH2;
Servo TVC_Y_CH2;

PID zAxis(PID_Z_P, PID_Z_I, PID_Z_D, 0);
PID yAxis(PID_Y_P, PID_Y_I, PID_Y_D, 0);


/*
MISC VARS
*/
int loopCounter = 0;

unsigned long lastSensorUpdate = 0;
const int sensorUpdateDelay = 200;

void setup() {	
	//Setup pin states
	pinMode(IND_R_PIN, OUTPUT);
	pinMode(IND_G_PIN, OUTPUT);
	pinMode(IND_B_PIN, OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);

	pinMode(PYRO1_PIN, OUTPUT);
	pinMode(PYRO2_PIN, OUTPUT);
	pinMode(PYRO3_PIN, OUTPUT);
	pinMode(PYRO4_PIN, OUTPUT);
	pinMode(PYRO5_PIN, OUTPUT);

	pinMode(TVC_X_CH1_PIN, OUTPUT);
	pinMode(TVC_Y_CH1_PIN, OUTPUT);
	pinMode(TVC_X_CH2_PIN, OUTPUT);
	pinMode(TVC_Y_CH2_PIN, OUTPUT);

	pinMode(ROLL_FIN, OUTPUT);
	pinMode(ROLL_RIN, OUTPUT);

	pinMode(SD_INS, INPUT);
	//Note: do not use pinMode on any of the SPI pins otherwise stuff doesn't work!

	//TVC Setup
	TVC_X_CH1.attach(TVC_X_CH1_PIN); //Attach all servos to their respective pins
	TVC_Y_CH1.attach(TVC_Y_CH1_PIN);
	TVC_X_CH2.attach(TVC_X_CH2_PIN);
	TVC_Y_CH2.attach(TVC_Y_CH2_PIN);

	TVC_X_CH1.write(90+TVC_X_CH1_OFFSET); //Write all servos to center
	TVC_Y_CH1.write(90+TVC_Y_CH1_OFFSET);
	TVC_X_CH2.write(90+TVC_X_CH2_OFFSET);
	TVC_Y_CH2.write(90+TVC_Y_CH2_OFFSET);

	//Init serial
	if (debug) {
		Serial.begin(115200);
	}

	//Setup rocket for initial conditions before initialization
	configureInitialConditions(); 

	boolean error = false;

	//Setup radio
	if (!radio.init()) {
		Serial.println("[INIT] Radio failed to initialize");
		error = true;
	} else {
		debugPrintln("[INIT] RADIO OK");
	}
	radio.setTxPower(23, false);
  	radio.setFrequency(868);

	//Setup SD card
	if (!card.init(SPI_HALF_SPEED, SD_CS)) {
		dataLoggingState = DL_DISABLED;
	} else {
		debugPrintln("[INIT] SD OK");
	}

	//Setup BMI088
	if (accel.begin() < 0) {
		Serial.println("[INIT] BMI088 accel failed to initialize");
		error = true;
	} else {
		debugPrintln("[INIT] ACCEL INIT OK");
	}
	if (gyro.begin() < 0) {
		Serial.println("[INIT] BMI088 gyro failed to initialize");
		error = true;
	} else {
		debugPrintln("[INIT] GYRO INIT OK");
	}

	//Setup ODR and range
	gyro.setOdr(Bmi088Gyro::ODR_2000HZ_BW_532HZ);
	gyro.setRange(Bmi088Gyro::RANGE_2000DPS);

	accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_280HZ);
	accel.setRange(Bmi088Accel::RANGE_24G);

	//Setup ADS1015 #1
	if (!ADS1.begin()) {
		error = true;
		Serial.println("[INIT] ADS1 failed to initialize");
	} else {
		debugPrintln("[INIT] ADS1 OK");
	}
  	ADS1.setGain(1);
  	

  	//Setup ADS1015 #2
	if (!ADS2.begin()) {
		error = true;
		Serial.println("[INIT] ADS2 failed to initialize");
	} else {
		debugPrintln("[INIT] ADS2 OK");
	}
  	ADS2.setGain(1);

  	if (error) {
  		analogWrite(IND_G_PIN, 0);
  		analogWrite(IND_R_PIN, 127);
  		analogWrite(IND_B_PIN, 0);
  		while(1);
  	} else {
  		analogWrite(IND_G_PIN, 127);
  		analogWrite(IND_R_PIN, 0);
  		analogWrite(IND_B_PIN, 0);
  		delay(100);
  		analogWrite(IND_G_PIN, 0);
  		analogWrite(IND_R_PIN, 0);
  		analogWrite(IND_B_PIN, 0);
	}
	
}


EulerAngles gyroMeasure;
EulerAngles gyroOut;

Orientation ori;
float locX,locY,locZ;

float gbiasX, gbiasY, gbiasZ;
float abiasX, abiasY;
unsigned long biasStart;
int biasCount = 0;

unsigned long currentMicros;
unsigned long lastOriMicros;
unsigned long lastTVCMicros;

void loop() {
	unsigned long currentMillis = millis();
	/*
	MASTER FLIGHT LOOP
	*/
	switch (flightMode) {
		case CONN_WAIT:
			if (telemetryConnectionState == TEL_CONNECTED) {
				Serial.println("TEL_CONN in wait");
				transitionMode(IDLE);
			}
			break;
		case IDLE:
			if (telemetryConnectionState == TEL_DISCONNECTED) {
				Serial.println("TEL_DISCONN in idle");
				transitionMode(CONN_WAIT);
			}
			break;
		case LAUNCH:
			break;
	}

	/*
	ESSENTIAL ORIENTATION CALCULATIONS
	*/

	currentMicros = micros();
	double dtOri = (double)(currentMicros-lastOriMicros) / 1000000.0;
	double dtTVC = (double)(currentMicros-lastTVCMicros) / 1000000.0;

	if (dtOri > 0.002f) {
		if (oriMode == COMPLEMENTARY) {
			gyro.readSensor();

			gyroMeasure.roll = (gyro.getGyroX_rads() + gbiasX);
			gyroMeasure.pitch = -(gyro.getGyroY_rads() + gbiasY);
			gyroMeasure.yaw = (gyro.getGyroZ_rads() + gbiasZ);

			ori.update(gyroMeasure, dtOri);

			accel.readSensor();
			Quaternion accVec(accel.getAccelX_mss() + abiasX, accel.getAccelY_mss() + abiasY, accel.getAccelZ_mss());

			ori.applyComplementary(accVec,0.02);
		} else if (oriMode == GYRONLY) {
			gyro.readSensor();
			gyroMeasure.roll = (gyro.getGyroX_rads() + gbiasX);
			gyroMeasure.pitch = -(gyro.getGyroY_rads() + gbiasY);
			gyroMeasure.yaw = (gyro.getGyroZ_rads() + gbiasZ);

			ori.update(gyroMeasure, dtOri);
		} else if (oriMode == CALCBIASES) {
			gyro.readSensor();
			gbiasX += (double)gyro.getGyroX_rads();
			gbiasY += (double)gyro.getGyroY_rads();
			gbiasZ += (double)gyro.getGyroZ_rads();
			accel.readSensor();
			abiasX += accel.getAccelX_mss();
			abiasY += accel.getAccelY_mss();
			biasCount++;

			if (currentMicros - biasStart > 3000000) { //3sec
				abiasX /= biasCount; //Find bias in 1 reading
				abiasY /= biasCount;
				gbiasX /= biasCount; //Find bias in 1 reading
				gbiasY /= biasCount;
				gbiasZ /= biasCount;

				Serial.print("gbiasX=");
				Serial.println(gbiasX, 5);
				Serial.print("gbiasY=");
				Serial.println(gbiasY, 5);
				Serial.print("gbiasZ=");
				Serial.println(gbiasZ, 5);
				Serial.print("abiasX=");
				Serial.println(abiasX, 5);
				Serial.print("abiasY=");
				Serial.println(abiasY, 5);
				oriMode = GYRONLY;
			}
		} else {
			biasStart = micros();
			oriMode = CALCBIASES;
		}

		gyroOut = ori.toEuler();

		locX = gyroOut.roll*57.2958*1.7;
		locY = gyroOut.pitch*57.2958*1.7; //pid x
		locZ = gyroOut.yaw*57.2958*1.7; //pid y

		zAxis.update(locZ, dtOri);
	  	yAxis.update(locY, dtOri);
	  	lastOriMicros = micros();
	}

	/*
	PID UPDATES
	*/
	if (dtTVC > 0.02f) {
		float zAng = zAxis.getLast();
		float yAng = yAxis.getLast();

		//Roll correction required
		double cs = cos(-locX*0.0174533);
		double sn = sin(-locX*0.0174533);

		float trueYOut = (yAng*cs) - (zAng*sn);
		float trueZOut = (yAng*sn) + (zAng*cs);

		writeTVCCH1(trueYOut, -trueZOut);
		lastTVCMicros = micros();
	}

	/*
	NON ESSENTIAl SENSOR UPDATES
	*/
	if (currentMillis - lastSensorUpdate > sensorUpdateDelay) {
		Serial.print("pitch="); Serial.println(locY);
		Serial.print("yaw="); Serial.println(locZ);
		Serial.println();


		debugPrintln("[SENSOR] update");
		//First set state variables
		telem.fMode = flightMode;
		telem.pState = pyroState;
		telem.cState = chuteState;
		telem.dState = dataLoggingState;
		telem.tSState = telemetryState;
		telem.tCState = telemetryConnectionState;

		//Pyro channel stuff
		telem.pyro1Fire = pyroStates[0];
		telem.pyro2Fire = pyroStates[1];
		telem.pyro3Fire = pyroStates[2];
		telem.pyro4Fire = pyroStates[3];
		telem.pyro5Fire = pyroStates[4];

		//Now sample ADCs
		int16_t ads1_0 = ADS1.readADC(0);  
		int16_t ads1_1 = ADS1.readADC(1);  
		int16_t ads1_2 = ADS1.readADC(2);  
		int16_t ads1_3 = ADS1.readADC(3);
		int16_t ads2_0 = ADS2.readADC(0);  
		int16_t ads2_1 = ADS2.readADC(1);  
		int16_t ads2_2 = ADS2.readADC(2);  
		int16_t ads2_3 = ADS2.readADC(3);

		//Calculate line voltages using conversion factors
		telem.battV = ads1_0*ADC_DIV_FACTOR_V/ADC_RES_DIV_FACTOR_VBUS; //use conv factor
  		telem.servoV = ads1_1*ADC_DIV_FACTOR_V/ADC_RES_DIV_FACTOR_VSERVO;
  		telem.rollMotorV = ads1_2*ADC_DIV_FACTOR_V/ADC_RES_DIV_FACTOR_VMOTOR;

  		//Calculate status of pyro channels (short, continuity ok,)
  		telem.pyro1Cont = adcPyroContinuity(telem.battV, ads2_0);
  		telem.pyro2Cont = adcPyroContinuity(telem.battV, ads2_1);
  		telem.pyro3Cont = adcPyroContinuity(telem.battV, ads2_2);
  		telem.pyro4Cont = adcPyroContinuity(telem.battV, ads2_3);
  		telem.pyro5Cont = adcPyroContinuity(telem.battV, ads1_3);

  		//Misc vars
		telem.timeSinceStartup = currentMillis;
		telem.guidanceFrequency = (float)loopCounter/((float)(millis()-lastSensorUpdate)/1000.0);
		loopCounter = 0;

		lastSensorUpdate = currentMillis;
	}

	/*
	TELEMETRY
	*/
	if (telemetryState != TEL_DISABLED) {
		//this next line is a meme
		if (currentMillis - lastTelem > ((telemetryState == TEL_ENABLED_30HZ) ? 33.33 : (telemetryState == TEL_ENABLED_15HZ) ? 66.66 : (telemetryState == TEL_ENABLED_5HZ) ? 200 : 500)) {
			debugPrintln("[TELEM] sent");
			sendTelemetry();
			lastTelem = currentMillis;
		}
	}

	/*
	HEARTBEAT
	*/
	if (currentMillis - lastHeartbeat > heartbeatDelay) {
		debugPrintln("[RADIO] hb");
		//sendRadioPacket(HEARTBEAT, 0, 0, 0, 0);
		lastHeartbeat = currentMillis;
	}

	/*
	BUZZER
	*/
	if (toneStackPos > 0) {
		if (currentMillis - lastToneStart > toneDelayQueue[0]) {
			for (int i=1; i<toneBufferLength; i++) { //Left shift all results by 1
				toneFreqQueue[i-1] = toneFreqQueue[i];
				toneDelayQueue[i-1] = toneDelayQueue[i];
			}
			toneStackPos--; //we've removed one from the stack
			if (toneStackPos > 0) { //is there something new to start playing?
				if (toneFreqQueue[0] > 0) {
					tone(BUZZER_PIN, toneFreqQueue[0]); //start new tone
				}
				lastToneStart = currentMillis;
			} else {
				noTone(BUZZER_PIN); //otherwise just stop playing
			}
		}
	}

	if (currentMillis - lastBuzzer > buzzerDelay) {
		switch (flightMode) {
			case BOOTING:
			case CONN_WAIT:
				addToneQueue(2000, 150);
				addToneQueue(1000, 150);
				buzzerDelay = 3000;
				break;
			case IDLE:
				addToneQueue(2000, 150);
				buzzerDelay = 3000;
				break;
			case LAUNCH:
			case DESCEND:
				buzzerDelay = 3000;
				break;
			case COPYINGSD:
				addToneQueue(1500, 150);
				addToneQueue(0, 150);
				addToneQueue(1500, 150);
				buzzerDelay = 1000;
				break;
			case LANDED:
				addToneQueue(1500, 100);
				addToneQueue(1750, 100);
				addToneQueue(2000, 100);
				addToneQueue(2250, 100);
				addToneQueue(0, 100);
				addToneQueue(6000, 50);
				addToneQueue(4000, 50);
				addToneQueue(6000, 50);
				addToneQueue(4000, 50);
				addToneQueue(6000, 50);
				buzzerDelay = 1500;
				break;
		}
		lastBuzzer = currentMillis;
	}

	/*
	LED
	*/
	if (ledStackPos > 0) {
		if (currentMillis - lastLEDStart > ledDelayQueue[0]) {
			for (int i=1; i<ledBufferLength; i++) { //Left shift all results by 1
				ledRQueue[i-1] = ledRQueue[i];
				ledGQueue[i-1] = ledGQueue[i];
				ledBQueue[i-1] = ledBQueue[i];
				ledDelayQueue[i-1] = ledDelayQueue[i];
			}
			ledStackPos--; //we've removed one from the stack
			if (ledStackPos > 0) { //is there something new to start playing?
				analogWrite(IND_R_PIN, ledRQueue[0]);
				analogWrite(IND_G_PIN, ledGQueue[0]);
				analogWrite(IND_B_PIN, ledBQueue[0]);
				lastLEDStart = currentMillis;
			} else { //otherwise nothing left to do, so stop
				analogWrite(IND_R_PIN, 0);
				analogWrite(IND_G_PIN, 0);
				analogWrite(IND_B_PIN, 0);
			}
		}
	}

	if (currentMillis - lastLED > LEDDelay) {
		switch (flightMode) {
			case BOOTING:
			case CONN_WAIT:
				addLEDQueue(0, 0, 127, 250);
				LEDDelay = 1000;
				break;
			case IDLE:
				addLEDQueue(127, 127, 0, 250);
				LEDDelay = 1000;
				break;
			case LAUNCH:
			case DESCEND:
				addLEDQueue(0, 127, 0, 100);
				LEDDelay = 250;
				break;
			case COPYINGSD:
				addLEDQueue(127, 127, 0, 500);
				addLEDQueue(0, 0, 127, 500);
				LEDDelay = 1000;
				break;
			case LANDED:
				addLEDQueue(127, 0, 0, 250);
				addLEDQueue(0, 127, 0, 250);
				addLEDQueue(0, 0, 127, 250);
				LEDDelay = 750;
				break;
		}
		lastLED = currentMillis;
	}

	/*
	RADIO
	*/
	if (radio.available()) {
		struct RadioPacket *rx; //get poInteRiZeD!
		uint8_t buf[RH_RF95_MAX_MESSAGE_LEN]; //no buffer overruns here!
    	uint8_t len = sizeof(buf);
		radio.recv(buf, &len);

		rx = (struct RadioPacket *)buf; //mm yes tasty struct conversions

		switch (rx->id) {
			case GETSTATE:
				sendRadioPacket(GETSTATE, 0, flightMode, pyroState, telemetryState);
				break;
			case SETSTATE:
				switch ((int)rx->data1) {
					case 0:
						flightMode = CONN_WAIT;
						break;
					case 1:
						flightMode = IDLE;
						break;
					case 2:
						flightMode = LAUNCH;
						break;
					case 3:
						flightMode = DESCEND;
						break;
					case 4:
						flightMode = COPYINGSD;
						break;
					case 5:
						flightMode = LANDED;
						break;
				}
				break;
			case REQTELEM:
				sendTelemetry();
				break;
			case PYROARMING:
				if (rx->data1) {
					pyroState = PY_ARMED;
				} else {
					pyroState = PY_DISARMED;
				}
				break;
			case FIREPYRO:
				firePyroChannel(rx->data1, rx->data2);
				break;
		}

		lastRadioRecieveTime = (long)millis();
	}

	/*
	RADIO TIME CHECK
	*/
	if (currentMillis - lastRadioRecieveTime > radioTimeoutDelay) {
		//debugPrintln("[RADIO] disconnected");
		telemetryConnectionState = TEL_DISCONNECTED;
	} else {
		//debugPrintln("[RADIO] connected");
		telemetryConnectionState = TEL_CONNECTED;
	}

	/*
	PYRO CHANNELS
	*/

	if (pyrosInUse > 0) {
		if (pyroState == PY_ARMED) {
			for (int i=0; i<5; i++) {
				if (pyroStates[i]) {
					if (currentMillis > pyroOffTimes[i]) {
						pyroStates[i] = false;
						pyrosInUse--;

						debugPrint("[PYRO] ch ");
						debugPrint(i+1);
						debugPrintln("off");

						switch (i+1) { //i plus one because 1st element zeroooo bb im sorry im really tired
							case 1:
								digitalWrite(PYRO1_PIN, LOW);
								break;
							case 2:
								digitalWrite(PYRO2_PIN, LOW);
								break;
							case 3:
								digitalWrite(PYRO3_PIN, LOW);
								break;
							case 4:
								digitalWrite(PYRO4_PIN, LOW);
								break;
							case 5:
								digitalWrite(PYRO5_PIN, LOW);
								break;

						}
					}
				}
			}
		} else { //uhoh they've been disabled, immediately cut them off
			for (int i=0; i<5; i++) {
				pyroStates[i] = 0;
			}
			pyrosInUse = 0;

			debugPrintln("[PYRO] all pyros off");
			digitalWrite(PYRO1_PIN, LOW);
			digitalWrite(PYRO2_PIN, LOW);
			digitalWrite(PYRO3_PIN, LOW);
			digitalWrite(PYRO4_PIN, LOW);
			digitalWrite(PYRO5_PIN, LOW);
		}

	}

	/*
	MISC
	*/
	loopCounter++;
}

void transitionMode(FlightMode newMode) {
	if (newMode != flightMode) { //Are we not already in the state?
		flightMode = newMode; //Actually switch the state

		//Play a sound to indicate state change
		if (buzzerEnabled) {
			for (int i=0; i<newMode+1; i++) {
				toneFreqQueue[toneStackPos] = 2500;
				toneDelayQueue[toneStackPos] = 100;
				toneStackPos++; //always increase stack pointer
				toneFreqQueue[toneStackPos] = 0;
				toneDelayQueue[toneStackPos] = 100;
				toneStackPos++; //always increase stack pointer
			}
		}
		debugPrint("[STATE] switch to newState: ");
		debugPrintln(newMode);
		switch (newMode) {
			default:
			break;
		}
	}
}

void sendTelemetry() {
	// sub IDs:
	// 0 - time since startup, MET, loop freq
	// 1 - flightmode, pyrostates, chutestates
	// 2 - dataloggingstates, telemsendstates, telemconnstates
	// 3 - battv, servov, rollmotorv
	// 4 - boardtemp, gpsfix, gpssats
	// 5 - gyro x, y, z
	// 6 - acc x, y, z
	// 7 - mag x, y, z
	// 8 - gnss lat, gnss lon, alt
	// 9 - ori x, y, z
	// 10 - pos x, y, z
	// 11 - vel x, y, z
	// 12 - pyro1 status, pyro2 status, pyro3 status
	// 13 - pyro4 status, pyro5 status, blank
	// 14 - pyro1 fire, pyro2 fire, pyro3 fire
	// 15 - pyro4 fire, pyro5 fire, blank	
	// 16 - tvc x, tvc y, blank

	sendRadioPacket(GETTELEM, 0, telem.timeSinceStartup, telem.missionElapsedTime, telem.guidanceFrequency);
	sendRadioPacket(GETTELEM, 1, telem.fMode, telem.pState, telem.cState);
	sendRadioPacket(GETTELEM, 2, telem.dState, telem.tSState, telem.tCState);
	sendRadioPacket(GETTELEM, 3, telem.battV, telem.servoV, telem.rollMotorV);
	sendRadioPacket(GETTELEM, 4, telem.boardTemp, telem.gpsFix, telem.gpsSats);

	sendRadioPacket(GETTELEM, 5, telem.gyroX, telem.gyroY, telem.gyroZ);
	sendRadioPacket(GETTELEM, 6, telem.accX, telem.accY, telem.accZ);
	sendRadioPacket(GETTELEM, 7, telem.magX, telem.magY, telem.magZ);
	sendRadioPacket(GETTELEM, 8, telem.GNSSLat, telem.GNSSLon, telem.alt);

	sendRadioPacket(GETTELEM, 9, telem.oriX, telem.oriY, telem.oriZ);
	sendRadioPacket(GETTELEM, 10, telem.posX, telem.posY, telem.posZ);
	sendRadioPacket(GETTELEM, 11, telem.velX, telem.velY, telem.velZ);

	sendRadioPacket(GETTELEM, 12, telem.pyro1Cont, telem.pyro2Cont, telem.pyro3Cont);
	sendRadioPacket(GETTELEM, 13, telem.pyro4Cont, telem.pyro5Cont, 0);

	sendRadioPacket(GETTELEM, 14, telem.pyro1Fire, telem.pyro2Fire, telem.pyro3Fire);
	sendRadioPacket(GETTELEM, 15, telem.pyro4Fire, telem.pyro5Fire, 0);

	sendRadioPacket(GETTELEM, 16, telem.tvcX, telem.tvcY, 0);
}

boolean adcPyroContinuity(float battV, int16_t pyroReading) {
	double vDrop = battV-((double)pyroReading*(double)ADC_DIV_FACTOR_V*(double)ADC_RES_DIV_FACTOR_PYRO);
	//Serial.print("Drop=");
	//Serial.print(vDrop);
	if (vDrop <= ADC_PYRO_THRESH_CONT) {
		//Serial.println(" Stat=Cont");
		return true; //We got dat continuity lads
	} else {
		//Serial.println(" Stat=Disconn");
		return false; //Disconnected :(
	}
}

void configureInitialConditions() {
	flightMode = CONN_WAIT;
	pyroState = PY_DISARMED;
	chuteState = C_DISARMED;
	dataLoggingState = DL_ENABLED_5HZ;
	telemetryState = TEL_DISABLED;
	telemetryConnectionState = TEL_DISCONNECTED;

	analogWrite(IND_B_PIN, 127);
	analogWrite(IND_G_PIN, 0);
	analogWrite(IND_R_PIN, 0);

	int t = 1400;
	for (int i=0; i<3; i++) {
		tone(BUZZER_PIN, t);
		t+=200;
		delay(100);
	}
	noTone(BUZZER_PIN);

	writeTVCCH1(0, 0);
	delay(1000);
	writeTVCCH1(10, 10); //Up and right
	delay(1000);
	writeTVCCH1(0, 0);

	delay(100);
	transitionMode(CONN_WAIT);
}

bool addToneQueue(int freq, int delay) {
	if (buzzerEnabled) {
		if (toneStackPos < toneBufferLength && delay > 0) {
			toneFreqQueue[toneStackPos] = freq;
			toneDelayQueue[toneStackPos] = delay;
			toneStackPos++; //always increase stack pointer
			if (toneStackPos == 1) { //If it's the first sound, start playing it
				tone(BUZZER_PIN, toneFreqQueue[0]); //start new tone
				lastToneStart = millis();
			}
			return true;
		}
	}
	return false;
}

bool addLEDQueue(byte r, byte g, byte b, int delay) {
	if (indicatorEnabled) {
		if (ledStackPos < ledBufferLength && delay > 0) {
			ledRQueue[ledStackPos] = r;
			ledGQueue[ledStackPos] = g;
			ledBQueue[ledStackPos] = b;
			ledDelayQueue[ledStackPos] = delay;
			ledStackPos++; //always increase stack pointer
			if (ledStackPos == 1) { //If it's the first sound, start playing it
				analogWrite(IND_R_PIN, ledRQueue[0]);
				analogWrite(IND_G_PIN, ledGQueue[0]);
				analogWrite(IND_B_PIN, ledBQueue[0]);
				lastLEDStart = millis();
			}
			return true;
		}
	}
	return false;
}

bool firePyroChannel(byte nChannel, int time) { //We don't really care about continuity when firing channels, may as well turn it on
	if (pyroState == PY_ARMED) {
		if (nChannel <= 5 && nChannel >= 1) {
			if (!pyroStates[nChannel-1]) { //gotta subtract one because of array indexing being weird
				pyroStates[nChannel-1] = true; //set channel to fire
				pyroOffTimes[nChannel-1] = millis()+time;
				pyrosInUse++;

				debugPrint("[PYRO] ch ");
				debugPrint(nChannel);
				debugPrintln("on");

				switch (nChannel) {
					case 1:
						digitalWrite(PYRO1_PIN, HIGH);
						break;
					case 2:
						digitalWrite(PYRO2_PIN, HIGH);
						break;
					case 3:
						digitalWrite(PYRO3_PIN, HIGH);
						break;
					case 4:
						digitalWrite(PYRO4_PIN, HIGH);
						break;
					case 5:
						digitalWrite(PYRO5_PIN, HIGH);
						break;
				}

				return true;
			}
		}
	}
	return false;
}

void writeTVCCH1(float x, float y) {
	x = constrain(x, -20, 20);
	y = constrain(y, -20, 20);
	//x = -x;
	//y = -y;
	//Serial.println(90 + (x * SERVO_MULT) + TVC_X_CH1_OFFSET);
	TVC_X_CH1.write(90 + (x * SERVO_MULT) + TVC_X_CH1_OFFSET);
	TVC_Y_CH1.write(90 + (y * SERVO_MULT) + TVC_Y_CH1_OFFSET);
}

void writeTVCCH2(float x, float y) {
	x = constrain(x, -45, 45);
	y = constrain(y, -45, 45);
	TVC_X_CH2.write(90 + (x * SERVO_MULT) + TVC_X_CH2_OFFSET);
	TVC_Y_CH2.write(90 + (y * SERVO_MULT) + TVC_Y_CH2_OFFSET);
}

void sendRadioPacket(byte id, byte subID, float data1, float data2, float data3) {
	RadioPacket tx;
	tx.id = id;
	tx.subID = subID;
	tx.data1 = data1;
	tx.data2 = data2;
	tx.data3 = data3;

	radio.send((uint8_t *)&tx, sizeof(struct RadioPacket)); //Gotta love this cursed line of C
}
