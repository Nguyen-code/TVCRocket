// set this to the hardware serial port you wish to use
#define XBSerial Serial1
#define XBBaudRate 115200

/*
 * XBEE CONFIG INFO
 * 
 * NI: Depends on module used. RECV_A and RECV_B for the two receivers, TRANS_A for the vehicle
 * CH: C
 * PAN ID: 6942
 * FIRMWARE: XB3-24, 802.15.4, Rev 200C
 * Baud rate: 115200
 * AP: Mode 2
 * A MY+DH+DL: 0x0 0x0 0x1
 * B MY+DH+DL: 0x1 0x0 0x0
 */
void setup() {
  Serial.begin(115200);
  XBSerial.begin(XBBaudRate);
}

bool sendTestData = false;
unsigned long lastTestTime = 0;

void loop() {
  int incomingByte;

  if (sendTestData && millis()-lastTestTime > 500) {
    XBSerial.write("Hello world");
    Serial.println("Sent test data");
    lastTestTime = millis();
  }

  

  if (Serial.available() > 0) {
    XBSerial.write(Serial.read());
  }
  if (XBSerial.available() > 0) {
    Serial.write(XBSerial.read());
  }
}
