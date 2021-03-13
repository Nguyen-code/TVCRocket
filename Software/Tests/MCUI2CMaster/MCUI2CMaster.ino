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

void setup() {
	Serial.begin(115200);
	Wire.setClock(100000);
	Wire.begin();
	delay(1000);
}

void loop() {
	Wire.requestFrom(SLAVE_ADDRESS, sizeof(S_I2C_MESSAGE));

	S_I2C_MESSAGE rx;
	uint8_t buf[sizeof(S_I2C_MESSAGE)];
	int point = 0;
	while(Wire.available()) {
		buf[point] = (uint8_t)Wire.read();
		point++;
	}
	printBuffer(buf, sizeof(S_I2C_MESSAGE));

	memcpy(&rx, &buf, sizeof(S_I2C_MESSAGE));
	
	Serial.print("CID="); Serial.print(rx.commandID); Serial.print(", D1="); Serial.print(rx.data1); Serial.print(", D2="); Serial.print(rx.data2); Serial.print(", D3="); Serial.println(rx.data3);
	analogWrite(LED_R, rx.data1);
	analogWrite(LED_G, rx.data2);
	analogWrite(LED_B, rx.data3);

	delay(100);
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