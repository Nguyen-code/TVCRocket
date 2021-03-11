
#include <XBee.h>
#include "libs/states.h"
#include "libs/AES/AES.cpp"
#define KEYBITS 128

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

//Define structs used to hold data
static struct S_TELEMETRY_STATE telem_state;
static struct S_TELEMETRY_SENSOR telem_sensor;

//Find closest buffer size to packet sizes that are divisible by 16
const uint8_t state_aes_buffer_size = sizeof(S_TELEMETRY_STATE) - (sizeof(S_TELEMETRY_STATE) % 16) + 16;
const uint8_t sensor_aes_buffer_size = sizeof(S_TELEMETRY_SENSOR) - (sizeof(S_TELEMETRY_SENSOR) % 16) + 16;

//Allocate AES buffer
uint8_t state_aes_buffer[state_aes_buffer_size];
uint8_t sensor_aes_buffer[sensor_aes_buffer_size];

//Define AES stuff (at 128 bit encryption, this is a 16 byte key)
const uint8_t aes_key[KEYLENGTH(KEYBITS)] = {0x80, 0x08, 0x13, 0x58, 0x00, 0x81, 0x35, 0x80, 0x08, 0x13, 0x58, 0x00, 0x81, 0x35, 0x00, 0x01};
uint32_t rk[RKLENGTH(KEYBITS)];


// create the XBee object
XBee xbee = XBee();

Tx16Request tx = Tx16Request(0x0000, (uint8_t*)&telem_state, sizeof(S_TELEMETRY_STATE));
TxStatusResponse txStatus = TxStatusResponse();

void setup() {
	Serial.begin(115200);
	Serial1.begin(115200);
	xbee.setSerial(Serial1);
	delay(100);

	//Setup initial values
	telem_state.timeSinceStartup = millis();
	telem_state.pyroFire = 0x12;

	Serial.println("AES-128 KEY:");
	for (int i=0; i<KEYLENGTH(KEYBITS); i++) {
		Serial.print("0x");
		Serial.print(aes_key[i], HEX);
		Serial.print(" ");
	}
	Serial.println("\n");

	//Copy data into plain buffer
	uint8_t telemPlain[sizeof(S_TELEMETRY_STATE)];
	memcpy(telemPlain, (uint8_t*)&telem_state, sizeof(S_TELEMETRY_STATE));
	Serial.println("PLAINTEXT ------");
	for (int i=0; i<sizeof(S_TELEMETRY_STATE); i++) {
		Serial.print("0x");
		Serial.print(telemPlain[i], HEX);
		Serial.print(" ");
	}
	Serial.println("\n----------\n");


	//Encrypt data
	int nroundsEnc = aesSetupEncrypt(rk, *aes_key, KEYBITS);
	memset(state_aes_buffer, 0, sizeof(state_aes_buffer)); //Reset state_aes_buffer
	memcpy(state_aes_buffer, (uint8_t*)&telem_state, sizeof(S_TELEMETRY_STATE)); //Copy plaintext telemetry data into buffer
	Serial.println("PLAINTEXT PADDED TO %16 ------");
	for (int i=0; i<state_aes_buffer_size; i++) {
		Serial.print("0x");
		Serial.print(state_aes_buffer[i], HEX);
		Serial.print(" ");
	}
	Serial.println("\n----------\n");

	//Proceed block by block and encrypt the data
	for (int i=0; i<state_aes_buffer_size/16; i++) {
		uint8_t toEncrypt[16];
		uint8_t encrypted[16];
		memcpy(toEncrypt, state_aes_buffer+(i*16), 16); //copy single block to be encrypted
		aesEncrypt(rk, nroundsEnc, toEncrypt, encrypted);
		memcpy(state_aes_buffer+(i*16), encrypted, 16); //copy data back into buffer
	}
	Serial.println("ENCRYPTED ------");
	for (int i=0; i<state_aes_buffer_size; i++) {
		Serial.print("0x");
		Serial.print(state_aes_buffer[i], HEX);
		Serial.print(" ");
	}
	Serial.println("\n----------\n");

	//Decrypt data
	int nroundsDec = aesSetupDecrypt(rk, *aes_key, KEYBITS);
	
	//Proceed block by block and encrypt the data
	for (int i=0; i<state_aes_buffer_size/16; i++) {
		uint8_t toDecrypt[16];
		uint8_t decrypted[16];
		memcpy(toDecrypt, state_aes_buffer+(i*16), 16); //copy single block to be decrypted
		aesDecrypt(rk, nroundsDec, toDecrypt, decrypted);
		memcpy(state_aes_buffer+(i*16), decrypted, 16); //copy data back into buffer
	}

	Serial.println("DECRYPTED ------");
	for (int i=0; i<state_aes_buffer_size; i++) {
		Serial.print("0x");
		Serial.print(state_aes_buffer[i], HEX);
		Serial.print(" ");
	}
	Serial.println("\n----------\n");


	

}

void loop() {
	//xbee.send(tx);
	//telem_state.timeSinceStartup = millis();

	xbee.readPacket();
	if (xbee.getResponse().isAvailable()) {
		if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
			xbee.getResponse().getTxStatusResponse(txStatus);

			// get the delivery status, the fifth byte
			if (txStatus.getStatus() == SUCCESS) {
				// success.  time to celebrate
				Serial.print("Send ok; receiver ok");
			} else {
				// the remote XBee did not receive our packet. is it powered on?
				Serial.println("Send ok; no receiver?");
			}
		} else if (xbee.getResponse().isError()) {
			Serial.println("Send ok; recv corrupt packet");    
		} else {
			Serial.println("Send fail; radio ok?");
		}
	}

	delay(50);
}
