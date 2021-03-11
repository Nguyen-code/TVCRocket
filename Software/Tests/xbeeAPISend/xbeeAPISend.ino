
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

//Define structs used to hold data
static struct S_TELEMETRY_STATE telem_state;
static struct S_TELEMETRY_SENSOR telem_sensor;

//Define AES stuff (at 128 bit encryption, this is a 16 byte key)
#define KEYBITS 128
#define getAESBufferSize(unpadded_size) unpadded_size - (unpadded_size % 16) + 16

const uint8_t aes_key[KEYLENGTH(KEYBITS)] = {0x80, 0x08, 0x13, 0x58, 0x00, 0x81, 0x35, 0x80, 0x08, 0x13, 0x58, 0x00, 0x81, 0x35, 0x00, 0x01};
uint32_t rk[RKLENGTH(KEYBITS)];

uint8_t state_aes_buffer[getAESBufferSize(sizeof(S_TELEMETRY_STATE))];
uint8_t sensor_aes_buffer[getAESBufferSize(sizeof(S_TELEMETRY_SENSOR))];

// create the XBee object
XBee xbee = XBee();

Tx16Request txStateEncrypted = Tx16Request(0x0000, state_aes_buffer, sizeof(state_aes_buffer));
Tx16Request txSensorEncrypted = Tx16Request(0x0000, sensor_aes_buffer, sizeof(sensor_aes_buffer)); 
TxStatusResponse txStatus = TxStatusResponse();

void setup() {
	Serial.begin(115200);
	Serial1.begin(115200);
	xbee.setSerial(Serial1);
	delay(1000);

	//Setup initial values
	telem_state.timeSinceStartup = millis();
	telem_state.pyroFire = 0x12;

	//DEMO STUFF
	/*
	Serial.println("AES-128 KEY:");
	for (int i=0; i<KEYLENGTH(KEYBITS); i++) {
		Serial.print("0x");
		Serial.print(aes_key[i], HEX);
		Serial.print(" ");
	}
	Serial.println("\n");

	uint8_t state_aes_buffer[getAESBufferSize(sizeof(S_TELEMETRY_STATE))];
	encryptAESBuffer(state_aes_buffer, &telem_state, sizeof(S_TELEMETRY_STATE));

	S_TELEMETRY_STATE out;
	decryptAESBuffer(&out, state_aes_buffer, sizeof(S_TELEMETRY_STATE));

	Serial.println("TSS, PF1");
	Serial.println(telem_state.timeSinceStartup);
	Serial.println(telem_state.pyroFire);
	Serial.println("TSS, PF2");
	Serial.println(out.timeSinceStartup);
	Serial.println(out.pyroFire);
	
	printBuffer((uint8_t*)&telem_state, sizeof(S_TELEMETRY_STATE));
	printBuffer((uint8_t*)&out, sizeof(S_TELEMETRY_STATE));
	*/
}

void encryptAESBuffer(uint8_t *aes_buffer, void *unpaddedData, const int data_size) {
	//Get nRounds, or number of rounds of encryption to do
	int nroundsEnc = aesSetupEncrypt(rk, aes_key, KEYBITS);

	//Find closest buffer size to packet sizes that are divisible by 16
	const uint8_t aes_buffer_size = data_size - (data_size % 16) + 16;

	//Reset aes_buffer
	memset(aes_buffer, 0, aes_buffer_size);
	//Copy plaintext data into buffer
	memcpy(aes_buffer, unpaddedData, data_size);

	//Proceed block by block and encrypt the data
	for (int i=0; i<aes_buffer_size/16; i++) {
		//Declare single block arrays
		uint8_t toEncrypt[16];
		uint8_t encrypted[16];
		memcpy(toEncrypt, aes_buffer+(i*16), 16); //copy single block to be encrypted
		aesEncrypt(rk, nroundsEnc, toEncrypt, encrypted); //Do the encryption
		memcpy(aes_buffer+(i*16), encrypted, 16); //copy data back into buffer
	}
}

void decryptAESBuffer(void *out, uint8_t *aes_buffer, const int data_size) {
	//Get nRounds, or number of rounds of decryption
	int nroundsDec = aesSetupDecrypt(rk, aes_key, KEYBITS);

	const int buffer_size = getAESBufferSize(data_size);

	//Ensure the buffer has a correct block size
	if (buffer_size % 16 != 0) {
		return;
	} else {	
		//Proceed block by block and decrypt the data
		for (int i=0; i<buffer_size/16; i++) {
			uint8_t toDecrypt[16];
			uint8_t decrypted[16];
			memcpy(toDecrypt, aes_buffer+(i*16), 16); //copy single block to be decrypted
			aesDecrypt(rk, nroundsDec, toDecrypt, decrypted);
			memcpy(aes_buffer+(i*16), decrypted, 16); //copy data back into buffer
		}
	}

	//Copy data into final array, without copying decrypted padding
	memcpy(out, aes_buffer, data_size);
}

void printBuffer(uint8_t *buffer, const int buffer_size) {
	Serial.println("BUFFER ------");
	for (int i=0; i<buffer_size; i++) {
		Serial.print("0x");
		Serial.print(buffer[i], HEX);
		Serial.print(" ");
	}
	Serial.println("\n----------\n");
}

void loop() {
	unsigned long start = micros();
	encryptAESBuffer(state_aes_buffer, &telem_state, sizeof(S_TELEMETRY_STATE));
	unsigned long enc = micros();
	xbee.send(txStateEncrypted);
	unsigned long send = micros();

	Serial.print("AESEncryptTime: ");
	Serial.print((enc-start)/1000.0f, 3);
	Serial.print("ms, XBeeSendTime: ");
	Serial.print((send-enc)/1000.0f, 3);
	Serial.println("ms");
	Serial.print("FULLS_STATE: ");
	Serial.print((send-start)/1000.0f, 3);
	Serial.println("ms");

	start = micros();
	encryptAESBuffer(sensor_aes_buffer, &telem_sensor, sizeof(S_TELEMETRY_SENSOR));
	enc = micros();
	xbee.send(txSensorEncrypted);
	send = micros();

	Serial.print("AESEncryptTime: ");
	Serial.print((enc-start)/1000.0f, 3);
	Serial.print("ms, XBeeSendTime: ");
	Serial.print((send-enc)/1000.0f, 3);
	Serial.println("ms");
	Serial.print("FULLS_SENSOR: ");
	Serial.print((send-start)/1000.0f, 3);
	Serial.println("ms");

	telem_state.timeSinceStartup = millis();
	telem_sensor.GNSSLat = millis()+100;

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
