#include <Wire.h>

#define SLAVE_ADDRESS 0x05

#define LED_R 3
#define LED_G 4
#define LED_B 5

struct S_I2C_MESSAGE {
	byte commandID;
	byte data1;
	byte data2;
	byte data3;
};
S_I2C_MESSAGE tx;

void setup() {
	Serial.begin(115200);
	Wire.setClock(100000);
	Wire.begin(SLAVE_ADDRESS);
	Wire.onRequest(wireRequest);
	Wire.onReceive(wireReceive);
	delay(1000);
}

bool direction = true;
int rValue, gValue, bValue;

void loop() {
	analogWrite(LED_R, rValue);
	analogWrite(LED_G, gValue);
	analogWrite(LED_B, bValue);

	if (direction) {
		rValue++;
		tx.data2++; //g value of remote board
		if (rValue >= 178) {
			direction = false;
		}
	} else {
		rValue--;
		tx.data2--;
		if (rValue <= 50) {
			direction = true;
		}
	}

	delay(5);
}

void wireReceive(int nBytes) {
	Serial.print("Receive get, nBytes="); Serial.println(nBytes);
}

void wireRequest() {
	Serial.println("Request get");
	Wire.write((uint8_t *)&tx, sizeof(S_I2C_MESSAGE));
	tx.commandID++;
}

void printBuffer(uint8_t *buffer, const int buffer_size) {
	Serial.print("BUFFER SIZE=");
	Serial.print(buffer_size);
	Serial.println(" ------");
	for (int i=0; i<buffer_size; i++) {
		Serial.print("0x");
		Serial.print(buffer[i], HEX);
		Serial.print("\t");
		if ((i+1)%6 == 0 && i != 0) Serial.println();
	}
	Serial.println("\n----------\n");
}