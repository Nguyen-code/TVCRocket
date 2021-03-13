#include <SPI.h>
#include <RH_RF95.h>
#include <XBee.h>

#define RFM95_CS 31
#define RFM95_RST 33
#define RFM95_INT 32

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 868.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// create the XBee object
XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();

// Blinky on receipt
#define LED 13

void setup() 
{
  pinMode(LED, OUTPUT);     
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  while (!Serial);
  Serial.begin(115200);
  delay(100);

  //Serial.println("Arduino LoRa RX Test!");
  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  //Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  //Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  Serial1.begin(115200);
  xbee.setSerial(Serial1);
  //Serial.println("XBee serialStart OK");

  //Serial plotter stuff
  Serial.println("RF95 XBEE");
}

int rf95_rssi;
int xbee_rssi;
unsigned long lastRf95Packet;
unsigned long lastXbeePacket;

void loop()
{
  if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      digitalWrite(LED, HIGH);
      //Serial.print("RF95 RSSI: "); Serial.println(rf95.lastRssi(), DEC);
      rf95_rssi = rf95.lastRssi();
      lastRf95Packet = millis();

      // Send a reply
      uint8_t data[] = "And hello back to you";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent(100);
      //Serial.println("Sent a reply");
      digitalWrite(LED, LOW);
    }
    else
    {
      // Serial.println("RF95 receive failed");
      rf95_rssi = -150;
    }
  }

  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == RX_16_RESPONSE) { //expect rx_16 only
      xbee.getResponse().getRx16Response(rx16);
      
      uint8_t dataLength = rx16.getDataLength();
      uint8_t rx[dataLength];

      // uint8_t rssi = rx16.getRssi();
      // Serial.print("XBee RSSI: "); Serial.println(rssi);
      xbee_rssi = -rx16.getRssi();
      lastXbeePacket = millis();
    }

    if (xbee.getResponse().isError()) {
      Serial.print("Error code: ");
      Serial.println(xbee.getResponse().getErrorCode());
    }
  }

  if (millis() - lastRf95Packet > 3000) {
    rf95_rssi = -150;
  }
  if (millis() - lastXbeePacket > 3000) {
    xbee_rssi = -150;
  }

  //Serial plotter stuff
  Serial.print(rf95_rssi); Serial.print(" "); Serial.println(xbee_rssi);
  delay(50);
}
