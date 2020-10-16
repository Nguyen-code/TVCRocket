#include <SPI.h>
#include <SPI.h>
#include <RH_RF95.h>
#include "states.h"
#include "pindefs.h"
#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

/*
DISPLAY
*/

U8G2_ST7565_NHD_C12832_F_4W_HW_SPI u8g2(U8G2_R0, /* cs=*/ 9, /* dc=*/ 5, /* reset=*/ 10);
long lastDisplayUpdateTime = 0;
const int displayUpdateDelay = 500;

/*
RADIO
*/
RH_RF95 radio(RADIO_CS, RADIO_IRQ);

struct RadioPacket {
	uint8_t id;
	uint8_t subID;
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
	//Setup pin states
	pinMode(IND_R_PIN, OUTPUT);
	pinMode(IND_G_PIN, OUTPUT);
	pinMode(IND_B_PIN, OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);

	analogWrite(IND_B_PIN, 55);

	int t = 1400;
	for (int i=0; i<3; i++) {
		tone(BUZZER_PIN, t);
		t+=200;
		delay(100);
	}
	noTone(BUZZER_PIN);

	bool error = false;
	if (!radio.init()) {
		error = true;
	}
	radio.setTxPower(23, false);
  	radio.setFrequency(868);

  	if (!u8g2.begin()) {
  		error = true;
  	}
  	u8g2.setFontMode(1);
  	u8g2.setFlipMode(1);

  	if (error) {
  		analogWrite(IND_G_PIN, 0);
  		analogWrite(IND_R_PIN, 55);
  		analogWrite(IND_B_PIN, 0);
  		while(1);
  	} else {
  		analogWrite(IND_G_PIN, 55);
  		analogWrite(IND_R_PIN, 0);
  		analogWrite(IND_B_PIN, 0);
  	}
  	delay(1000);
}

float packetRate;
int rssi;
bool confirmLaunch = false;

void loop() {
	unsigned long currentMillis = millis();

	if (currentMillis - lastPacketPrint >= packetPrintDelay) {
		packetRate = rocketHBPacketCount/(packetPrintDelay/1000);
		rssi = (int)radio.lastRssi();
		int rValue = map(rssi, -20, -90, 64, 0);
		int gValue = map(rssi, -20, -90, 0, 64);
		analogWrite(IND_G_PIN, gValue);
		analogWrite(IND_R_PIN, rValue);

		// Serial.print("Rate=");
		// Serial.print(packetRate);
		// Serial.print("Hz\tRSSI=");
		// Serial.println(rssi, DEC);

		rocketHBPacketCount = 0;
		lastPacketPrint = currentMillis;
	}

	if (currentMillis - lastDisplayUpdateTime >= displayUpdateDelay) {
		u8g2.clearBuffer();

		//Write TLM rate
		u8g2.setFont(u8g2_font_helvR08_tf);
		u8g2.setCursor(0,8);
    	u8g2.print("TLM Hz");
    	u8g2.setCursor(0,32);
    	u8g2.setFont(u8g2_font_logisoso16_tf);
    	u8g2.print(packetRate);

    	//Write rssi
    	u8g2.setFont(u8g2_font_helvR08_tf);
		u8g2.setCursor(40,8);
    	u8g2.print("RSSI");
    	u8g2.setCursor(40,32);
    	u8g2.setFont(u8g2_font_logisoso16_tf);
    	u8g2.print(rssi);

    	//Write pyro states
    	u8g2.setFont(u8g2_font_helvR08_tf);
		u8g2.setCursor(70,32);
    	u8g2.print("Cont=");
    	u8g2.print((telem.pyro1Cont)?"T":"F");
    	u8g2.print((telem.pyro2Cont)?"T":"F");
    	u8g2.print((telem.pyro3Cont)?"T":"F");
    	u8g2.print((telem.pyro4Cont)?"T":"F");
    	u8g2.print((telem.pyro5Cont)?"T":"F");

    	//Write voltage rails
    	u8g2.setFont(u8g2_font_helvR08_tf);
		u8g2.setCursor(70,8);
    	u8g2.print("V=");
    	u8g2.print(telem.battV);
    	u8g2.print(",");
    	u8g2.print(telem.servoV);

    	//Write altitude
    	u8g2.setFont(u8g2_font_helvR08_tf);
		u8g2.setCursor(70,20);
    	u8g2.print("Alt=");
    	u8g2.print(telem.alt);

    	u8g2.sendBuffer();

    	lastDisplayUpdateTime = currentMillis;
	}

	if (currentMillis - lastHeartbeat > heartbeatDelay) {
		sendRadioPacket(HEARTBEAT, 0, 0, 0, 0);
		lastHeartbeat = currentMillis;
	}

	if (Serial.available() > 0) {
		char inChar = Serial.read();

		if (inChar == ';') {
			serialBuffer.toLowerCase().trim();

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
			} else if (serialBuffer == "launch") {
				if (!confirmLaunch) { //Are we sure?
					u8g2.clearBuffer();
					u8g2.setFont(u8g2_font_logisoso22_tf);
					u8g2.setCursor(0, 30);
					u8g2.print("Confirm?");
					u8g2.sendBuffer();
					for (int i=0; i<3; i++) {
						tone(BUZZER_PIN, 1400);
						delay(100);
						tone(BUZZER_PIN, 1700);
						delay(100);
					}
					noTone(BUZZER_PIN);
					delay(2000);
					confirmLaunch = true;
				} else { //WE ARE CONFIRMED! Go for it!
					u8g2.clearBuffer();
					u8g2.setFont(u8g2_font_logisoso16_tf);
					u8g2.setCursor(0, 30);
					u8g2.print("Confirmed :)");
					u8g2.sendBuffer();
					
					delay(1000);
					confirmLaunch = false; //reset flag

					for (int i=15; i>0; i--) {
						u8g2.clearBuffer();
						u8g2.setCursor(55, 30);
						u8g2.print(i);
						u8g2.sendBuffer();

						if (i > 3) {
							tone(BUZZER_PIN, 1400);
							delay(100);
							noTone(BUZZER_PIN);
							delay(900);
						} else {
							for (int i=0; i<2; i++) {
								tone(BUZZER_PIN, 1400);
								delay(100);
								tone(BUZZER_PIN, 2000);
								delay(100);
							}
							noTone(BUZZER_PIN);
							delay(600);
						}
					}

					sendRadioPacket(SETSTATE, 0, 2, 0, 0); //Set state to launch mode
					delay(100);
					sendRadioPacket(SETSTATE, 0, 2, 0, 0);

				}
			} else {
				Serial.println("Command not understood");
			}
			serialBuffer = "";
		} else {
			serialBuffer += inChar;
		}
	}

	if (radio.available()) {
		RadioPacket rx;
		uint8_t datalen = sizeof(rx);
		radio.recv((uint8_t*)&rx, &datalen);

		if (rx.id != 3 || true) {
			Serial.print("GOT CMD: ");
			Serial.println(rx.id);
		}
		switch (rx.id) { //TODO copy to the phat struct
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

void sendRadioPacket(uint8_t id, uint8_t subID, float data1, float data2, float data3) {
	RadioPacket tx;
	tx.id = id;
	tx.subID = subID;
	tx.data1 = data1;
	tx.data2 = data2;
	tx.data3 = data3;

	radio.send((uint8_t*)&tx, sizeof(tx)); //Gotta love this cursed line of C
	radio.waitPacketSent();
	radio.waitAvailableTimeout(1); //FIXME: THIS IS A HACK THIS SHOULD NOT BE HERE
}