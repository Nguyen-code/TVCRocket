/*
This code was designed to run on ZENITH 2.0, because I am waiting for ZENITH 3.0 to arrive in the mail (rip)
By Aaron Becker

rocket notes:

- write page of flash using struct
- USE i2c for 
- kalman filter - really high acceleration events - throw out bad data

Sensor List

IMU: BMI088

ms5611 https://github.com/jarzebski/Arduino-MS5611


update todo notes:
rewrite adc code for full 5 pyro channels
2nd adc support
migrate NRF24l01 to RF95
fix pindefs
ability to fire pyros remotely

buzzer enum state
tvc enabled enum state
fix odd bug with lastRadioRecievedTimer and state changed based on telconn state




optimizations:
pyro checking can remove for loop and use bool flag
telem can use different packet struct?

*/

#include <SPI.h>
#include <printf.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SD.h>
#include <ADS1X15.h>
#include "states.h"

/*
DEBUG
*/

#define DEBUG


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

/*
RADIO
*/
RF24 radio(0,1); // CE, CSN
const byte addresses [][6] = {"00001", "00002"}; //write at addr 00002, read at addr 00001
boolean radioListening = false;

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

ADS1015 ADS1(0x48);
ADS1015 ADS2(0x49);
const int ADC_VBUS_DIVD_RES1 = 10000;
const int ADC_VBUS_DIVC_RES2 = 1800;
const double ADC_RES_DIV_FACTOR_VBUS = (double)ADC_VBUS_DIVC_RES2/((double)ADC_VBUS_DIVD_RES1+(double)ADC_VBUS_DIVC_RES2);

const int ADC_PYRO_DIVD_RES1 = 1500;
const int ADC_PYRO_DIVD_RES2 = 1500;
const double ADC_RES_DIV_FACTOR_PYRO = (double)ADC_PYRO_DIVD_RES2/((double)ADC_PYRO_DIVD_RES1+(double)ADC_PYRO_DIVD_RES2);

const double ADC_MAX_V = 6.144;
const double ADC_DIV_FACTOR_V = ADC_MAX_V/4096;

const float ADC_PYRO_VDROP_THRESH = 0.22;


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
PYRO CHANNEL CONFIG
*/

unsigned long pyroOffTimes[5];
bool pyroStates[5];
int pyrosInUse = 0;

/*
MISC VARS
*/
int loopCounter = 0;

/*
SETTINGS AND PINDEFS
*/

const bool buzzerEnabled = false;

unsigned long lastSensorUpdate = 0;
const int sensorUpdateDelay = 200;


#define INDICATOR_PIN 13
#define FETONE_PIN 23
#define FETTWO_PIN 22
#define BUZZER_PIN 20
#define RADIO_IRQ 2
#define RADIO_CE 0
#define RADIO_CSN 1
#define SD_CS 21

void setup() {
	//Setup pin states
	pinMode(INDICATOR_PIN, OUTPUT);
	pinMode(FETONE_PIN, OUTPUT);
	pinMode(FETTWO_PIN, OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);
	pinMode(RADIO_IRQ, INPUT);
	pinMode(RADIO_CE, OUTPUT);
	pinMode(RADIO_CSN, OUTPUT);
	pinMode(SD_CS, OUTPUT);

	//TVC Setup
	//analogWriteResolution(14);

	//Init serial
	if (debug) {
		Serial.begin(115200);
	}

	//Setup rocket for initial conditions before initialization
	configureInitialConditions(); 

	//Setup radio
	printf_begin();
	radio.begin();
	radio.setPALevel(RF24_PA_MAX); //max because we don't want to lose connection
	radio.setRetries(3,3); // delay, count
	radio.openWritingPipe(addresses[1]);
	radio.openReadingPipe(1, addresses[0]); //set address to recieve data
	radio.maskIRQ(1,1,0);
	attachInterrupt(digitalPinToInterrupt(RADIO_IRQ), recv, FALLING);
	radioTransmitMode();
	radio.stopListening();
	radio.printDetails();

	//Setup SD card
	if (!card.init(SPI_HALF_SPEED, SD_CS)) {
		dataLoggingState = DL_DISABLED;
	} else {
		debugPrintln("[INIT] SD OK");
	}

	//Setup ADS1015
	ADS1.begin();
  	ADS1.setGain(0);
  	debugPrintln("[INIT] ADS OK");
}

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
	NON ESSENTIAl SENSOR UPDATES
	*/
	if (currentMillis - lastSensorUpdate > sensorUpdateDelay) {
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

		//Now sample ADC
		int16_t ads0 = ADS1.readADC(0);  
		int16_t ads1 = ADS1.readADC(1);  
		int16_t ads2 = ADS1.readADC(2);  
		int16_t ads3 = ADS1.readADC(3);
		float f = ADS1.toVoltage(1);

		//Calculate ADC voltages
		telem.battV = (float)ads0*ADC_DIV_FACTOR_V*ADC_RES_DIV_FACTOR_VBUS; //use conv factor
  		telem.servoV = (float)ads3*ADC_DIV_FACTOR_V;

  		//Calculate resistance on pyro channels
  		telem.pyro1Cont = adcPyroContinuity(telem.battV, ads1);
  		telem.pyro2Cont = adcPyroContinuity(telem.battV, ads2);

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
		sendRadioPacket(HEARTBEAT, 0, 0, 0, 0);
		radioRecieveMode();
		lastHeartbeat = currentMillis;
	}

	/*
	BUZZER
	*/
	if (toneStackPos > 0) {
		if (currentMillis - lastToneStart > toneDelayQueue[0]) {
			noTone(BUZZER_PIN);
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
				buzzerDelay = 10000;
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
								digitalWrite(FETONE_PIN, LOW);
								break;
							case 2:
								digitalWrite(FETTWO_PIN, LOW);
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
			digitalWrite(FETONE_PIN, LOW);
			digitalWrite(FETTWO_PIN, LOW);
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
	radioTransmitMode();

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
	// 12 - pyro1 cont, pyro2 cont, pyro3 cont
	// 13 - pyro4 cont, pyro5 cont, blank
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

	radioRecieveMode();
}

bool adcPyroContinuity(float inpVoltage, int16_t pyroVoltage) {
	float vDrop = inpVoltage-((float)pyroVoltage*ADC_DIV_FACTOR_V*ADC_RES_DIV_FACTOR_PYRO);
	if (vDrop > ADC_PYRO_VDROP_THRESH) {
		return true;
	}
	return false;
}

void configureInitialConditions() {
	flightMode = CONN_WAIT;
	pyroState = PY_DISARMED;
	chuteState = C_DISARMED;
	dataLoggingState = DL_ENABLED_5HZ;
	telemetryState = TEL_DISABLED;
	telemetryConnectionState = TEL_DISCONNECTED;

	int t = 1400;
	for (int i=0; i<3; i++) {
		tone(BUZZER_PIN, t);
		t+=200;
		delay(100);
	}
	noTone(BUZZER_PIN);

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
						digitalWrite(FETONE_PIN, HIGH);
						break;
					case 2:
						digitalWrite(FETTWO_PIN, HIGH);
				}

				return true;
			}
		}
	}
	return false;
}

void radioRecieveMode() {
  if (!radioListening) { //if we're not listening
    radio.startListening();
    radioListening = true;
  }
}

void radioTransmitMode() {
  if (radioListening) {
    radio.stopListening();
    radioListening = false;
  }
}

void sendRadioPacket(byte id, byte subID, float data1, float data2, float data3) {
	RadioPacket tx;
	tx.id = id;
	tx.subID = subID;
	tx.data1 = data1;
	tx.data2 = data2;
	tx.data3 = data3;

	radioTransmitMode(); //ok to call this since transmitMode keeps track of radio state and won't run if it's already in transmit mode
	radio.write(&tx, sizeof(tx));
}

void recv() {
	radioRecieveMode();
	if (radio.available()) {
		RadioPacket rx;
		radio.read(&rx, sizeof(rx));

		switch (rx.id) {
			case GETSTATE:
				sendRadioPacket(GETSTATE, 0, flightMode, pyroState, telemetryState);
				break;
			case SETSTATE:
				switch ((int)rx.data1) {
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

		radioRecieveMode();
		lastRadioRecieveTime = (long)millis();
	}
}
