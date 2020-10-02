#include <SPI.h>
#include "printf.h"
#include <nRF24L01.h>
#include <RF24.h>
#include "states.h"


/*
Screen code:
http://www.nhdforum.newhavendisplay.com/index.php?topic=11609.0
*/

/*
RADIO
*/
RF24 radio(7, 8); // CE, CSN
const byte addresses [][6] = {"00002", "00001"}; //write at addr 00001, read at addr 00002
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

unsigned long lastRadioRecieveTime = 0;


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


int rocketHBPacketCount = 0;
unsigned long lastPacketPrint = 0;
const int packetPrintDelay = 1000;

unsigned long lastHeartbeat = 0;
const int heartbeatDelay = 100;

String serialBuffer = "";

void setup() {
	Serial.begin(115200);

	//Setup radio
	printf_begin();
	radio.begin();
	radio.setPALevel(RF24_PA_MAX); //max because we don't want to lose connection
	radio.setRetries(3,3); // delay, count
	radio.openWritingPipe(addresses[1]);
	radio.openReadingPipe(1, addresses[0]); //set address to recieve data
	radioTransmitMode();
	radio.stopListening();
	radio.printDetails();	
}

void loop() {
	unsigned long currentMillis = millis();

	if (currentMillis - lastPacketPrint >= packetPrintDelay) {
		float packetRate = rocketHBPacketCount/(packetPrintDelay/1000);
		Serial.print("Getting HB packets at rate: ");
		Serial.print(packetRate);
		Serial.println("Hz");
		rocketHBPacketCount = 0;
		lastPacketPrint = currentMillis;
	}

	if (currentMillis - lastHeartbeat > heartbeatDelay) {
		sendRadioPacket(HEARTBEAT, 0, 0, 0, 0);
		radioRecieveMode();
		lastHeartbeat = currentMillis;
	}

	if (Serial.available() > 0) {
		char inChar = Serial.read();

		if (inChar == ';') {
			serialBuffer.toLowerCase();

			Serial.print("Got command: ");
			Serial.println(serialBuffer);
			if (serialBuffer == "getstate") {
				sendRadioPacket(GETSTATE, 0, 0, 0, 0);
			} else if (serialBuffer == "pyroarm") {
				sendRadioPacket(PYROARMING, 0, 1, 0, 0);
			} else if (serialBuffer == "pyrodisarm") {
				sendRadioPacket(PYROARMING, 0, 0, 0, 0);
			} else if (serialBuffer == "stateidle") {
				sendRadioPacket(SETSTATE, 0, 1, 0, 0);
			} else if (serialBuffer == "statelaunch") {
				sendRadioPacket(SETSTATE, 0, 2, 0, 0);
			} else if (serialBuffer == "firepyro1") {
				sendRadioPacket(FIREPYRO, 0, 1, 1000, 0);
			} else if (serialBuffer == "firepyro2") {
				sendRadioPacket(FIREPYRO, 0, 2, 1000, 0);
			} else if (serialBuffer == "firepyro3") {
				sendRadioPacket(FIREPYRO, 0, 3, 1000, 0);
			} else if (serialBuffer == "firepyro4") {
				sendRadioPacket(FIREPYRO, 0, 4, 1000, 0);
			} else if (serialBuffer == "firepyro5") {
				sendRadioPacket(FIREPYRO, 0, 5, 1000, 0);
			} else {
				Serial.println("Command not understood");
			}
			radioRecieveMode();
			serialBuffer = "";
		} else {
			serialBuffer += inChar;
		}
	}

	if (radio.available()) {
		RadioPacket rx;
		radio.read(&rx, sizeof(rx));

		if (rx.id != 3) {
			Serial.print("GOT CMD: ");
			Serial.println(rx.id);
		}
		switch (rx.id) {
			case HEARTBEAT:
				rocketHBPacketCount++;
				break;
			case GETTELEM:
				break;
			case GETSTATE:
				Serial.println("Current rocket internal state:");
				Serial.print("FlightMode= ");
				FlightMode fm = (FlightMode)rx.data1;
				if (fm == BOOTING) {
					Serial.println("Computer booting");
				} else if (fm == CONN_WAIT) {
					Serial.println("Waiting for connection to radio");
				} else if (fm == IDLE) {
					Serial.println("Idle");
				} else if (fm == LAUNCH) {
					Serial.println("Launched!");
				} else if (fm == DESCEND) {
					Serial.println("Descending");
				} else if (fm == COPYINGSD) {
					Serial.println("Copying SD data");
				} else if (fm == LANDED) {
					Serial.println("Landed!");
				}

				Serial.print("PyroState=");
				if ((PyroStates)rx.data2 == PY_ARMED) {
					Serial.println("Armed");
				} else {
					Serial.println("Disarmed");
				}

				Serial.print("TelemetryState=");
				TelemSendStates tss = (TelemSendStates)rx.data3;
				if (tss == TEL_ENABLED_5HZ) {
					Serial.println("Enabled@5Hz");
				} else if (tss == TEL_ENABLED_15HZ) {
					Serial.println("Enabled@15Hz");
				} else if (tss == TEL_ENABLED_30HZ) {
					Serial.println("Enabled@30Hz");
				} else {
					Serial.println("Disabled");
				}
				break;
		}

		lastRadioRecieveTime = millis();
	}

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