/*
By Aaron Becker

rocket notes:

- write page of flash using struct
- USE i2c for 
- kalman filter - really high acceleration events - throw out bad data

Sensor List

IMU: BMI088

ms5611 https://github.com/jarzebski/Arduino-MS5611
new antenna https://www.videoaerialsystems.com/products/868-900mhz-longshot-antenna


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

make constant for servo angle to tvc mount angle\

Everything that joe is logging:
ones that make sense:
- avi voltage
- data error flag
- nav error flag
- abort state flag
- py 1-5 continuity
- py 1-5 fire
- vehicle mass (lin approx)
- baro alt (long avg)
- tvc y, z error
- Leg deployment
- landing abort status
- Ejection progress (multi step)
- ejection success
- fin deployment state
- bleed-off percent
- divert altitude error
- divert altitude height
- max height
- target burn alt
- GNSS y, x vel
- Acc z, y
- gyro x, y, z
- predicted ori for landing z, y
- fused ori x, y, z
- ori setpoints
- roll wheel setpoint
- tvc torque z, y
- gnss pdop
- gnss fix type (3d, etc)
- gnss sat count (SIV)
- raw pressure
- fused pos x, y, z
- fused vel x, y, z
- active accel biasing x, y, z
- velocity bias? x, y, z
- ori bias? x, y, z
- tvc setpoint y, z
- board temp
- system state
- flight time
- gain set 1-5: different guidance modes (trying to control position when rising, velocity at the very end)

ones that I'm not sure what they are:
- gyro_yi seems to be a backup gyroscope possibly
- difference between gyro_yi_sp, gyro_yi_s, gyro_yb_sp_s (sp = setpoint?)
1hz outside of flight, 50hz in flight

i = intertial frame (global, up and down is relative to gravity)
b = body frame (body, relative to vehicle)
move from inertial to body to remove effect of roll

Cool things joey b does:
use torque based controller w/  mass, mass moment of inertia, COM to come up with a more accurate TVC scheme
use max alt and simulated motor ignition to come up with burn altitude for landing (monte carlo sim in lookup table probably)
use more simulation to know where the vehicle "should be" based on Cd after landing ignition, calculate divert altitude to start divert movement
use divert altitude error (error from the vehicle's should be calculation to where it actually is) to come up with bleed percent (peak amplitude of sin wave divert)
can calculate bleed percent with 100-motorthrust*cos(mountangle) = bleed percent
landing burn commit: can light landing motor outside of abort limits as long as vehicle is on track to enter abort limits (predict from gyro rate and current ori)

if(micros() < lastMicros) weRolledOver();
Micros rollover count

*/


//TODOS GPS, SD

//External libs
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include "libs/ADS1X15/ADS1X15.cpp"
#include <Servo.h>
#include <RH_RF95.h>
#include "libs/PID/pid.cpp"
#include "libs/BMI088/BMI088.cpp"
#include "libs/Orientation/Quaternion.cpp"
#include "libs/Orientation/Orientation.cpp"
#include "libs/MS5611/MS5611.cpp"
#include "libs/Ublox/SparkFun_Ublox_Arduino_Library.cpp"
#include <SPIMemory.h>
#include "math.h"

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
FLIGHT MODE ENUMS
*/
FlightMode flightMode = BOOTING;
PyroStates pyroState = PY_DISARMED;
ChuteStates chuteState = C_DISARMED;
DataLoggingStates dataLoggingState = DL_ENABLED_5HZ;
TelemSendStates telemetryState = TEL_ENABLED_30HZ;
TelemConnStates telemetryConnectionState = TEL_DISCONNECTED;
TVCEnabledMode tvcEnabled = TVC_DISABLED;
OriMode oriMode = INIT;

/*
RADIO
*/
RH_RF95 radio(RADIO_CS, RADIO_IRQ);

struct RadioPacket {
	uint8_t id;
	int data1;
	int data2;
	int data3;
};

/*
Radio Command List
0: Ping/pong check (heartbeat)
1: SetRocketState
2: RequestRocketTelem
3: Enable/Disable Pyros
4: FirePyro
*/

typedef enum {
	HEARTBEAT = 0,
	SETSTATE = 1,
	REQTELEM = 2,
	PYROARMING = 3,
	FIREPYRO = 4
} RadioCommands;

const int radioTimeoutDelay = 5000;
unsigned long lastRadioRecieveTime = 0;

/*
TELEMETRY STRUCT
*/
/*
	data-raw-gyro -> gyro data
	data-raw-accel => accel data
	data-ori => orientation fused frame data
	data-pos => XYZ position (xPos, yPos, zPos, time) with xPos being vertical position (altitude)
	data-vel => XYZ velocity (xVel, yVel, zVel, time) with xVel being vertical velocity

	v-connect => vehicle has connected, start VOT timer
	v-disconnect => vehicle has disconnected, stop VOT timer
	DONE t-connect => arduino transmitter is connected
	DONE t-disconnect => arduino transitter disconnected
	vot-set => set vehicle on time forcefully in ms (vot)
	met-set => set mission elapsed time forcefully in ms (met)
	m-start => mission started, start MET timer
	v-stop => mission stopped, stop MET timer

	v-tvc => tvc update (y, z, active, rollPercent, rollSetpoint, twr, mass)

	v-state => various state things (battV, rollV, servoV, vehicleState, pyroState, temp, signal, tlmRate, fixType, pDOP, horizAcc, vertAcc, gpsSats)
*/

struct TELEMETRY {
	//Raw vehicle information
	unsigned long timeSinceStartup;
	unsigned long missionElapsedTime;

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
	long GNSSLat;
	long GNSSLon;

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
	double tvcY;
	double tvcZ;
	bool tvcActive;

	//Roll wheel data
	float rollPercent;
	float rollSetpoint;

	//Vehicle estimation
	float twr;
	float mass;
	
	//GNSS locking data
	byte GNSSFix;
	int GNSSpDOP;
	byte GNSSSats;
	int GNSSAccHoriz; //mm
	int GNSSAccVert; //mm
	int GNSSAccVel; //mm

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
MS5611 BAROMETER CONFIG
*/
MS5611 ms5611;

/*
GNSS
*/

SFE_UBLOX_GPS GPS;

/*
FLASH MEMORY
*/
SPIFlash flash(FLASH_CS);

/*
SD CARD
*/
char dataLogFilename[13];

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
unsigned long lastSensorUpdate = 0;
const int sensorUpdateDelay = 200;

bool gyro_ready = false;
bool accel_ready = false;

void gyro_drdy() {
	gyro_ready = true;
}
void accel_drdy() {
	accel_ready = true;
}

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
	TVC_X_CH1.attach(TVC_X_CH1_PIN, SERVO_MIN_US, SERVO_MAX_US); //Attach all servos to their respective pins
	TVC_Y_CH1.attach(TVC_Y_CH1_PIN, SERVO_MIN_US, SERVO_MAX_US);
	TVC_X_CH2.attach(TVC_X_CH2_PIN, SERVO_MIN_US, SERVO_MAX_US);
	TVC_Y_CH2.attach(TVC_Y_CH2_PIN, SERVO_MIN_US, SERVO_MAX_US);

	TVC_X_CH1.write(90+TVC_X_CH1_OFFSET); //Write all servos to center
	TVC_Y_CH1.write(90+TVC_Y_CH1_OFFSET);
	TVC_X_CH2.write(90+TVC_X_CH2_OFFSET);
	TVC_Y_CH2.write(90+TVC_Y_CH2_OFFSET);

	//Init serial
	if (debug) {
		Serial.begin(115200);
		debugPrintln("[INIT] hello!");
	}

	//Setup rocket for initial conditions before initialization
	configureInitialConditions(); 
	debugPrintln("[INIT] initconfig ok");

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

  	//Setup ms5611
  	if (!ms5611.begin()) {
  		Serial.println("[INIT] Baro failed to initialize");
  		error = true;
  	} else {
  		debugPrintln("[INIT] BARO OK");
  	}

  	//Setup flash
  	if (!flash.begin()) {
  		Serial.println("[INIT] Error initializing flash memory");
  		error = true;
  	} else {
  		debugPrint("[INIT] FLASH OK, SIZE=");
  		debugPrintln(flash.getCapacity());
  	}
  	flash.eraseChip();
  	debugPrintln("FLASH ERASE OK");

	//Setup SD card
	if (!SD.begin(SD_CS)) {
		debugPrintln("[INIT] Error initializing SD card");
		dataLoggingState = DL_DISABLED;
		error = true;
	} else {
		Serial.println("[INIT] SD OK");
	}

	char filename[] = "FLIGHT00.CSV";
	for (uint8_t i = 0; i < 100; i++) {
		filename[6] = i/10 + '0';
		filename[7] = i%10 + '0';
		if (!SD.exists(filename)) {
		 break;  // leave the loop!
		}
	}

	for (int i=0; i<sizeof(filename); i++) {
		dataLogFilename[i] = filename[i];
	}

	Serial.print("SDFilename: ");
	Serial.println(dataLogFilename);

	//Setup GPS
	if (GPS.begin(Wire, 0x42) == false) {
		Serial.println("[INIT] SAM-M8Q GPS module failed to initialize");
		error = true;
	} else {
		debugPrintln("[INIT] GPS INIT OK");
	}
	GPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
	GPS.setNavigationFrequency(5); //Produce five solutions per second
	GPS.setAutoPVT(true); //Tell the GPS to "send" each solution
	GPS.saveConfiguration(); //Save the current settings to flash and BBR

	if (GPS.setDynamicModel(DYN_MODEL_AIRBORNE2g) == false) { // Set the dynamic model to airborne
		Serial.println("[INIT] GPS dynamic model set fail");
		error = true;
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
	gyro.pinModeInt3(Bmi088Gyro::PUSH_PULL,Bmi088Gyro::ACTIVE_HIGH);
	gyro.mapDrdyInt3(true);
	attachInterrupt(ACCEL_INT, accel_drdy, RISING);

	accel.setOdr(Bmi088Accel::ODR_1600HZ_BW_280HZ);
	accel.setRange(Bmi088Accel::RANGE_24G);
	accel.pinModeInt1(Bmi088Accel::PUSH_PULL,Bmi088Accel::ACTIVE_HIGH);
	accel.mapDrdyInt1(true);
	attachInterrupt(GYRO_INT, gyro_drdy, RISING);

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
unsigned long biasStart;
int biasCount = 0;

unsigned long lastOriMicros;
unsigned long lastTVCMicros;
unsigned long lastAltMicros;

double currentAlt = 0;

const byte altBufferLen = 50;
double altBuffer[altBufferLen];
byte altBufferPos = 0;

double lastAlt = 0;
double altSlope = 0;
double launchAlt = 0;

unsigned long flightStartTime;

unsigned long lastDataloggingTime;
uint32_t currentFlashAddr = 0;

float fAbs(float n) {
	if (n < 0) {
		return -n;
	} else {
		return n;
	}
}

unsigned long lastNotLandedTime;

bool setState = false;

void loop() {
	/*
	MASTER FLIGHT LOOP
	*/
	unsigned long timeSinceFlightStart = millis() - flightStartTime;

	if (flightMode == CONN_WAIT) {
		if (telemetryConnectionState == TEL_CONNECTED) {
			Serial.println("TEL_CONN in wait");
			transitionMode(IDLE);
		}
	} else if (flightMode == IDLE) {
		if (telemetryConnectionState == TEL_DISCONNECTED) {
			Serial.println("TEL_DISCONN in idle");
			transitionMode(CONN_WAIT);
		}
	} else if (flightMode == LAUNCH) {
		//State transitions!

		/*
		Different ways the chutes can deploy:
		1) abort condition: >30deg over on roll or pitch axes after flight time abort
		2) derivative condition: slope of altitude is within a certain margin after flight begins
		3) force condition: if not already deployed, chutes will be forced to deploy after a certain number of seconds
		*/
		bool chutesAbort = timeSinceFlightStart > flight_minTimeBeforeAbort && (fAbs(locZ*57.2958) >= flight_abortDegrees || fAbs(locY*57.2958) >= flight_abortDegrees);
		bool chutesApogee = timeSinceFlightStart > flight_minTimeBeforeApogee && fAbs(altSlope) < flight_altSlopeApogeeCutoff && altSlope != 0;
		bool chutesForce = timeSinceFlightStart > flight_timeBeforeForceChutesDeploy;

		// Serial.print("conditions: abo=");
		// Serial.print(chutesAbort?"T":"F");
		// Serial.print(" apo=");
		// Serial.print(chutesApogee?"T":"F");
		// Serial.print(" for=");
		// Serial.println(chutesForce?"T":"F");
		if (chutesAbort || chutesApogee || chutesForce) {
			transitionMode(DESCEND);
		}
	} else if (flightMode == DESCEND) {
		if (fAbs(altSlope) < flight_altSlopeLandedCutoff) {
		} else {
			lastNotLandedTime = millis();
		}
		Serial.println(millis() - lastNotLandedTime);
		if (timeSinceFlightStart > flight_timeBeforeForceSDCopy || millis() - lastNotLandedTime > flight_minLandedTime) {
			transitionMode(COPYINGSD);
		}
	} else if (flightMode == COPYINGSD) {
		uint32_t addr = 0;
		TELEMETRY tlmOut;

		File dataFile = SD.open(dataLogFilename, FILE_WRITE);

		// if (dataFile) {
		// 	dataFile.println("timeSinceStartup,missionElapsedTime,guidanceFrequency,flightMode,pyroState,chuteState,dataLoggingState,telemSendState,telemConnState,battV,servoV,rollMotorV,boardTemp,gyroX,gyroY,gyroZ,alt,oriX,oriY,oriZ,tvcX,tvcY,pyro1Cont,pyro2Cont,pyro3Cont,pyro4Cont,pyro5Cont,pyro1Fire,pyro2Fire,pyro3Fire,pyro4Fire,pyro5Fire");
		// 	while (addr < flash.getCapacity()) {
		// 		flash.readAnything(addr, tlmOut);
		// 		if (isnan(tlmOut.battV)) {
		// 			break; //we got end
		// 		} else {
		// 			dataFile.print(tlmOut.timeSinceStartup);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.missionElapsedTime);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.guidanceFrequency);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.fMode);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.pState);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.cState);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.dState);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.tSState);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.tCState);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.battV);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.servoV);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.rollMotorV);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.boardTemp);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.gyroX);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.gyroY);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.gyroZ);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.alt);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.oriX);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.oriY);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.oriZ);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.tvcX);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.tvcY);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.pyro1Cont);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.pyro2Cont);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.pyro3Cont);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.pyro4Cont);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.pyro5Cont);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.pyro1Fire);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.pyro2Fire);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.pyro3Fire);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.pyro4Fire);
		// 			dataFile.print(",");
		// 			dataFile.print(tlmOut.pyro5Fire);
		// 			dataFile.println();
		// 		}
		// 		addr+=sizeof(TELEMETRY);
		// 	}
		// 	dataFile.close();
		// } else {
		// 	Serial.println("ERROR opening SD file");
		// }
		transitionMode(LANDED);
	}

	/*
	ESSENTIAL ORIENTATION CALCULATIONS
	*/
	double dtOri = (double)(micros()-lastOriMicros) / 1000000.0;
	if (dtOri > ORI_CALC_DELTA) {
		if (oriMode == COMPLEMENTARY && gyro_ready && accel_ready) {
			gyro_ready = false;
			accel_ready = false;

			gyro.readSensor();

			gyroMeasure.roll = (gyro.getGyroX_rads() + gbiasX);
			gyroMeasure.pitch = -(gyro.getGyroZ_rads() + gbiasY);
			gyroMeasure.yaw = -(gyro.getGyroY_rads() + gbiasZ);


			ori.update(gyroMeasure, dtOri);

			accel.readSensor();
			Quaternion accVec(accel.getAccelX_mss(), -(accel.getAccelZ_mss()), -(accel.getAccelY_mss()));

			ori.applyComplementary(accVec,0.02);
		} else if (oriMode == GYRONLY && gyro_ready) {
			gyro_ready = false;

			gyro.readSensor();
			gyroMeasure.roll = (gyro.getGyroX_rads() + gbiasX);
			gyroMeasure.pitch = -(gyro.getGyroZ_rads() + gbiasY);
			gyroMeasure.yaw = -(gyro.getGyroY_rads() + gbiasZ);

			ori.update(gyroMeasure, dtOri);
		} else if (oriMode == CALCBIASES && gyro_ready) {
			gyro_ready = false;

			gyro.readSensor();
			gbiasX += (double)gyro.getGyroX_rads();
			gbiasY += (double)gyro.getGyroY_rads();
			gbiasZ += (double)gyro.getGyroZ_rads();
			biasCount++;

			if (micros() - biasStart > 3000000) { //1.5sec
				Serial.println("BIASCOUNT");
				Serial.println(biasCount);
				gbiasX /= biasCount; //Find bias in 1 reading
				gbiasY /= biasCount;
				gbiasZ /= biasCount;

				Serial.print("gbiasX=");
				Serial.println(gbiasX, 5);
				Serial.print("gbiasY=");
				Serial.println(gbiasY, 5);
				Serial.print("gbiasZ=");
				Serial.println(gbiasZ, 5);
				oriMode = GYRONLY;
			}
		} else {
			biasStart = micros();
			oriMode = CALCBIASES;
		}

		if (gyro_ready) {
			gyro.readSensor();
			telem.gyroX = (gyro.getGyroX_rads() + gbiasX);
			telem.gyroY = -(gyro.getGyroY_rads() + gbiasY);
			telem.gyroZ = -(gyro.getGyroZ_rads() + gbiasZ);

			telem.gyroX = gyroMeasure.roll;
			telem.gyroY = gyroMeasure.yaw;
			telem.gyroZ = gyroMeasure.pitch;
		}
		if (accel_ready) {
			accel.readSensor();
			telem.accX = accel.getAccelX_mss();
			telem.accY = accel.getAccelY_mss();
			telem.accZ = accel.getAccelZ_mss();
		}

		gyroOut = ori.toEuler();

		locX = gyroOut.roll*GYRO_MULT;
		locY = gyroOut.pitch*GYRO_MULT; //pid x
		locZ = gyroOut.yaw*GYRO_MULT; //pid y

		zAxis.update(locZ*57.2958, dtOri);
	  	yAxis.update(locY*57.2958, dtOri);
	  	lastOriMicros = micros();
	}

	/*
	PID UPDATES
	*/
	double dtTVC = (double)(micros()-lastTVCMicros) / 1000000.0;
	if (dtTVC > SERVO_WRITE_DELTA && tvcEnabled == TVC_ENABLED) {
		float zAng = zAxis.getLast();
		float yAng = yAxis.getLast();

		//Roll correction required
		double cs = cos(-locX);
		double sn = sin(-locX);

		float trueYOut = (yAng*cs) - (zAng*sn);
		float trueZOut = (yAng*sn) + (zAng*cs);

		writeTVCCH1(trueZOut, trueYOut);
		lastTVCMicros = micros();
	}

	/*
	ALTITUDE UPDATES
	*/
	double dtAlt = (double)(micros()-lastAltMicros) / 1000000.0;
	if (dtAlt > BARO_CALC_DELTA) {
		long pressure = ms5611.readPressure(true);

		currentAlt = ms5611.getAltitude(pressure);
		telem.posX = currentAlt;

		altSlope = (currentAlt-lastAlt)/dtAlt;
		
		lastAlt = currentAlt;
		lastAltMicros = micros();
	}

	/*
	NON ESSENTIAl SENSOR UPDATES
	*/
	if (millis() - lastSensorUpdate > sensorUpdateDelay) {
		// Serial.print("pitch="); Serial.println(locY*57.2958);
		// Serial.print("yaw="); Serial.println(locZ*57.2958);
		// Serial.println();

		debugPrintln("[SENSOR] update");

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

  		//Calculate status of pyro channels (short, continuity ok)
  		telem.pyro1Cont = adcPyroContinuity(telem.battV, ads2_0);
  		telem.pyro2Cont = adcPyroContinuity(telem.battV, ads2_1);
  		telem.pyro3Cont = adcPyroContinuity(telem.battV, ads2_2);
  		telem.pyro4Cont = adcPyroContinuity(telem.battV, ads2_3);
  		telem.pyro5Cont = adcPyroContinuity(telem.battV, ads1_3);

  		//Get GPS stats and updates
  		if (GPS.getPVT() && (GPS.getInvalidLlh() == false)) { //is there an update available
  			telem.GNSSLon = GPS.getLongitude();
	  		telem.GNSSLat = GPS.getLatitude();

	  		telem.GNSSFix = GPS.getFixType();
	  		telem.GNSSSats = GPS.getSIV();
	  		telem.GNSSpDOP = GPS.getPDOP();
	  		telem.GNSSAccVert = GPS.getVerticalAccEst();
	  		telem.GNSSAccHoriz = GPS.getHorizontalAccEst();
	  		telem.GNSSAccVel = GPS.getSpeedAccEst();
  		}

		lastSensorUpdate = millis();
	}

	/*
	DATALOGGING
	*/
	if (dataLoggingState != DL_DISABLED) {
		if (millis() - lastDataloggingTime > (dataLoggingState == DL_ENABLED_5HZ ? 200 : dataLoggingState == DL_ENABLED_10HZ ? 100 : dataLoggingState == DL_ENABLED_20HZ ? 50 : dataLoggingState == DL_ENABLED_40HZ ? 25 : 1000)) {
			//First set state variables
			telem.fMode = flightMode;
			telem.pState = pyroState;
			telem.cState = chuteState;
			telem.dState = dataLoggingState;
			telem.tSState = telemetryState;
			telem.tCState = telemetryConnectionState;

			//Set temp vars (FIXME make this right)
			telem.tvcY = yAxis.getLast();
			telem.tvcZ = zAxis.getLast();
			telem.oriX = locX;
			telem.oriY = locY;
			telem.oriZ = locZ;


			//Misc vars
			telem.timeSinceStartup = millis();
			telem.missionElapsedTime = (flightMode != IDLE && flightMode != CONN_WAIT && flightMode != BOOTING) ? millis() - flightStartTime : 0;

			flash.writeAnything(currentFlashAddr, telem);
			currentFlashAddr+=sizeof(TELEMETRY);
		}
	}

	/*
	TELEMETRY
	*/
	if ((millis() - lastTelem) > ((telemetryState == TEL_ENABLED_30HZ) ? 33.33 : (telemetryState == TEL_ENABLED_15HZ) ? 66.66 : 200)) {
		debugPrintln("[TELEM] sent");
		sendTelemetry();
		lastTelem = millis();
	}

	/*
	BUZZER
	*/
	if (toneStackPos > 0) {
		if (millis() - lastToneStart > toneDelayQueue[0]) {
			for (int i=1; i<toneBufferLength; i++) { //Left shift all results by 1
				toneFreqQueue[i-1] = toneFreqQueue[i];
				toneDelayQueue[i-1] = toneDelayQueue[i];
			}
			toneStackPos--; //we've removed one from the stack
			if (toneStackPos > 0) { //is there something new to start playing?
				if (toneFreqQueue[0] > 0) {
					tone(BUZZER_PIN, toneFreqQueue[0]); //start new tone
				}
				lastToneStart = millis();
			} else {
				noTone(BUZZER_PIN); //otherwise just stop playing
			}
		}
	}

	if (millis() - lastBuzzer > buzzerDelay) {
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
		lastBuzzer = millis();
	}

	/*
	LED
	*/
	if (ledStackPos > 0) {
		if (millis() - lastLEDStart > ledDelayQueue[0]) {
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
				lastLEDStart = millis();
			} else { //otherwise nothing left to do, so stop
				analogWrite(IND_R_PIN, 0);
				analogWrite(IND_G_PIN, 0);
				analogWrite(IND_B_PIN, 0);
			}
		}
	}

	if (millis() - lastLED > LEDDelay) {
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
				addLEDQueue(0, 127, 0, 100);
				LEDDelay = 250;
				break;
			case DESCEND:
				addLEDQueue(0, 127, 0, 50);
				addLEDQueue(127, 127, 0, 50);
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
		lastLED = millis();
	}

	/*
	RADIO CHECK
	*/
	if (radio.available()) {
		RadioPacket rx; //get poInteRiZeD!
		uint8_t datalen = sizeof(rx);
		radio.recv((uint8_t*)&rx, &datalen);

		if (rx.id != 0) {
			Serial.print("GOT ID: ");
			Serial.println(rx.id);
		}
		switch (rx.id) {
			case SETSTATE:
				if (rx.data1 == BOOTING) {
					transitionMode(BOOTING);
				} if (rx.data1 == CONN_WAIT) {
					transitionMode(CONN_WAIT);
				} else if (rx.data1 == IDLE) {
					transitionMode(IDLE);
				} else if (rx.data1 == LAUNCH) {
					Serial.println("LAUNCH INITIATED");
					transitionMode(LAUNCH);
				} else if (rx.data1 == DESCEND) {
					transitionMode(DESCEND);
				} else if (rx.data1 == COPYINGSD) {
					transitionMode(COPYINGSD);
				} else if (rx.data1 == LANDED) {
					transitionMode(LANDED);
				}
				break;
			case REQTELEM:
				sendTelemetry();
				break;
			case PYROARMING:
				if (rx.data1) {
					pyroState = PY_ARMED;
				} else {
					pyroState = PY_DISARMED;
				}
				break;
			case FIREPYRO:
				firePyroChannel(rx.data1, rx.data2);
				break;
		}

		lastRadioRecieveTime = millis();
	}

	/*
	RADIO TIME CHECK
	*/
	if ((millis() - lastRadioRecieveTime) > radioTimeoutDelay) {
		telemetryConnectionState = TEL_DISCONNECTED;
	} else {
		telemetryConnectionState = TEL_CONNECTED;
	}

	/*
	PYRO CHANNELS
	*/
	if (pyrosInUse > 0) {
		if (pyroState == PY_ARMED) {
			for (int i=0; i<5; i++) {
				if (pyroStates[i]) {
					if (millis() > pyroOffTimes[i]) {
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

	if (flightMode != LAUNCH) {
		radio.waitAvailableTimeout(60);
	} else {
		radio.waitAvailableTimeout(1);
	}
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
		Serial.print("[STATE] switch to newState: ");
		Serial.println(newMode);

		if (newMode == BOOTING) {
			configureInitialConditions(); //cfg initial conditions
		} if (newMode == CONN_WAIT || newMode == IDLE) {
			dataLoggingState = DL_ENABLED_5HZ;
			pyroState = PY_DISARMED;
		} else if (newMode == LAUNCH) {
			Serial.println("LAUNCH INITIATED");
			//Reset integrator values
			zAxis.resetIntegrator();
			yAxis.resetIntegrator();

			//Reset orientation
			ori.reset();

			//Set flight started time
			flightStartTime = millis();

			//Enable TVC
			tvcEnabled = TVC_ENABLED;

			//Enable datalogging
			dataLoggingState = DL_ENABLED_40HZ;

			//Light this candle!
			pyroState = PY_ARMED; //Arm pyro channels
			firePyroChannel(2, 3000); //Fire pyro channel to start the motor
		} else if (newMode == DESCEND) {
			//Disable TVC
			tvcEnabled = TVC_DISABLED;

			//Set lastNotLandedTime
			lastNotLandedTime = millis();

			//Ensure chutes armed
			pyroState = PY_ARMED;
			firePyroChannel(4, 3000); //Fire parachute channel 1
			delay(2000);
			firePyroChannel(5, 3000); //Fire parachute channel 2
		} else if (newMode == COPYINGSD) {
			//Disable pyros
			pyroState = PY_DISARMED;

			if (dataLoggingState == DL_DISABLED) { //if we're disabled just go landed
				transitionMode(LANDED);
			} else {
				//Disable datalogging
				dataLoggingState = DL_DISABLED;
			}
		}
	}
}

void sendTelemetry() {
	radio.send((uint8_t*)&telem, sizeof(telem)); //Gotta love this cursed line of C
	radio.waitPacketSent();
}

boolean adcPyroContinuity(float battV, int16_t pyroReading) {
	double vDrop = battV-((double)pyroReading*(double)ADC_DIV_FACTOR_V*(double)ADC_RES_DIV_FACTOR_PYRO);
	//Serial.print("Drop=");
	//Serial.print(vDrop);
	if (vDrop <= ADC_PYRO_THRESH_CONT) {
		return true; //We got dat continuity lads
	} else {
		return false; //Disconnected :(
	}
}

void configureInitialConditions() {
	flightMode = CONN_WAIT;
	pyroState = PY_DISARMED;
	chuteState = C_DISARMED;
	dataLoggingState = DL_ENABLED_40HZ;
	telemetryState = TEL_ENABLED_30HZ;
	telemetryConnectionState = TEL_DISCONNECTED;
	tvcEnabled = TVC_DISABLED;

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

	/*for(int i=0; i<15; i+=0.25) {
		writeTVCCH1(0, i);
		delay(1000);
	}*/
	tvcEnabled = TVC_ENABLED;
	writeTVCCH1(0, 0);
	delay(1000);
	writeTVCCH1(5, 5); //Up and right
	delay(1000);
	writeTVCCH1(0, 0);
	tvcEnabled = TVC_DISABLED;


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
	if (tvcEnabled == TVC_ENABLED) {
		x = constrain(x, -SERVO_RANGE_X, SERVO_RANGE_X);
		y = constrain(y, -SERVO_RANGE_Y, SERVO_RANGE_Y);

		TVC_X_CH1.write(90 + (x * SERVO_MULT) + TVC_X_CH1_OFFSET);
		TVC_Y_CH1.write(90 + (y * SERVO_MULT) + TVC_Y_CH1_OFFSET);
	} else {
		TVC_X_CH1.write(90 + TVC_X_CH1_OFFSET);
		TVC_Y_CH1.write(90+ TVC_Y_CH1_OFFSET);
	}
}

void writeTVCCH2(float x, float y) {
	if (tvcEnabled == TVC_ENABLED) {
		x = constrain(x, -45, 45);
		y = constrain(y, -45, 45);
		TVC_X_CH2.write(90 + (x * SERVO_MULT) + TVC_X_CH2_OFFSET);
		TVC_Y_CH2.write(90 + (y * SERVO_MULT) + TVC_Y_CH2_OFFSET);
	} else {
		TVC_X_CH2.write(90 + TVC_X_CH2_OFFSET);
		TVC_Y_CH2.write(90 + TVC_Y_CH2_OFFSET);
	}
}