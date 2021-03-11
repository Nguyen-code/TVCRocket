
#include <XBee.h>
#include "libs/states.h"
#include "libs/AES/AES.cpp"

/*
Test sending encrypted packets using XBee API mode and an AES library
Should be uploaded to board with MY address of 0 and DL set to 0x0
*/


/*
TELEMETRY STRUCTS
*/
struct S_TELEMETRY_STATE {
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
	byte pyroCont;
	byte pyroFire;
};

struct S_TELEMETRY_SENSOR {
	//Raw sensor data
	float gyroX;
	float gyroY;
	float gyroZ;
	float accX;
	float accY;
	float accZ;
	long GNSSLat;
	long GNSSLon;

	//Calculated state vector data
	float oriX;
	float oriY;
	float oriZ;
	float posX;
	float posY;
	float posZ;
	float velX;
	float velY;
	float velZ;

	//TVC Data
	float tvcY;
	float tvcZ;
	bool tvcActive;

	//Roll wheel data
	float rollPercent;
	float rollSetpoint;
};

static struct S_TELEMETRY_STATE telem_state;
static struct S_TELEMETRY_SENSOR telem_sensor;

// create the XBee object
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();

void setup() {
	Serial.begin(115200);
	Serial1.begin(115200);
	xbee.setSerial(Serial1);
	delay(1000);
}

void loop() {
	xbee.readPacket();
	if (xbee.getResponse().isAvailable()) {
		if (xbee.getResponse().getApiId() == RX_16_RESPONSE) { //expect rx_16 only
			xbee.getResponse().getRx16Response(rx16);
			int packetLength = rx16.getDataLength();
			Serial.print("PL");
			Serial.println(packetLength);
			switch (packetLength) {
				default:
					Serial.println("Got packet of unknown size");
					break;
				case sizeof(S_TELEMETRY_STATE):
					Serial.println("Got state packet");
					S_TELEMETRY_STATE recv = (S_TELEMETRY_STATE &)*rx16.getData();
					telem_state.timeSinceStartup = recv.timeSinceStartup;
					break;
				case sizeof(S_TELEMETRY_SENSOR):
					Serial.println("Got sensor packet");
					break;
			}
		}

		if (xbee.getResponse().isError()) {
		  Serial.print("Error: ");
		  Serial.println(xbee.getResponse().getErrorCode());
		} 
		

		Serial.print("TelemTime: "); Serial.print(telem_state.timeSinceStartup);

		uint8_t rssi = rx16.getRssi();
		Serial.print(" RSSI: "); Serial.println(rssi);
	}
}
