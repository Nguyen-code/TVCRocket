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

uint8_t testPacket[2] = {0x00, 0x01};

// create the XBee object
XBee xbee = XBee();

Tx16Request txTestPacket = Tx16Request(0x0000, testPacket, sizeof(testPacket));
TxStatusResponse txStatus = TxStatusResponse();
 
void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
 
  while (!Serial);
  Serial.begin(115200);
  delay(100);
 
  Serial.println("Arduino LoRa TX Test!");
 
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
 
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  Serial1.begin(115200);
  xbee.setSerial(Serial1);
  Serial.println("XBee serialStart OK");
}
 
int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop()
{
  //Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  
  char radiopacket[20] = "Hello World #      ";
  itoa(packetnum++, radiopacket+13, 10);
  //Serial.print("Sending "); Serial.println(radiopacket);
  radiopacket[19] = 0;
  
  //Serial.println("Sending..."); delay(10);
  rf95.send((uint8_t *)radiopacket, 20);
 
  //Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
 
  //Serial.println("Waiting for reply..."); delay(10);
  if (rf95.waitAvailableTimeout(500))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      //Serial.print("RF95 RSSI: "); Serial.println(rf95.lastRssi(), DEC);
      Serial.println("RF95 OK");
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("RF95 FAIL");
  }
  
  xbee.send(txTestPacket);
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
      xbee.getResponse().getTxStatusResponse(txStatus);

      // get the delivery status, the fifth byte
      if (txStatus.getStatus() == SUCCESS) {
        // success.  time to celebrate
        Serial.println("XBEE OK");
      } else {
        // the remote XBee did not receive our packet. is it powered on?
        Serial.println("XBEE FAIL");
      }
    } else if (xbee.getResponse().isError()) {
      Serial.println("Send ok; recv corrupt packet");
      Serial.print("Error code: ");
      Serial.println(xbee.getResponse().getErrorCode(), HEX); 
    } else {
      Serial.println("Send fail; radio ok?");
      Serial.print("Weird API ID in response: ");
      Serial.println(xbee.getResponse().getApiId(), HEX);
    }
  }
}
