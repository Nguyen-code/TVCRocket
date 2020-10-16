#include <SPI.h>
#include <RH_RF95.h>

// Singleton instance of the radio driver
RH_RF95 radio(31, 32);

struct RadioPacket {
  uint8_t id;
  uint8_t subID;
  float data1;
  float data2;
  float data3;
};

void setup() 
{
   pinMode(21, OUTPUT);
  Serial.begin(115200);
  if (!radio.init())
    Serial.println("init failed");  

  radio.setTxPower(23, false);
  radio.setFrequency(868);
}

void loop()
{
  if (radio.available()) {
    RadioPacket rx;
    uint8_t datalen = sizeof(rx);
    radio.recv((uint8_t*)&rx, &datalen);

    Serial.print("Got packet! ID=");
    Serial.print(rx.id);
    Serial.print(", subID=");
    Serial.print(rx.subID);
    Serial.print(", d1=");
    Serial.print(rx.data1);
    Serial.print(", d2=");
    Serial.print(rx.data2);
    Serial.print(", d3=");
    Serial.println(rx.data3);
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