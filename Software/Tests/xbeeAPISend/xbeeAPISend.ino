
#include <XBee.h>
#include "states.h"

/*
This example is for Series 2 XBee
 Sends a ZB TX request with the value of analogRead(pin5) and checks the status response for success
*/


/*
TELEMETRY STRUCT
*/
struct TELEMETRY {
  //Raw vehicle information
  unsigned long timeSinceStartup;
  unsigned long missionElapsedTime;

//  //State stuff
//  FlightMode fMode;
//  PyroStates pState;
//  ChuteStates cState;
//  DataLoggingStates dState;
//  TelemSendStates tSState;
//  TelemConnStates tCState;
//
//  //Basic sensor data
//  float battV;
//  float servoV;
//  float rollMotorV;
//  float boardTemp;
//
//  //Raw sensor data
//  double gyroX;
//  double gyroY;
//  double gyroZ;
//  double accX;
//  double accY;
//  double accZ;
//  long GNSSLat;
//  long GNSSLon;

  //Calculated state vector data
//  double oriX;
//  double oriY;
//  double oriZ;
//  double posX;
//  double posY;
//  double posZ;
//  double velX;
//  double velY;
//  double velZ;
//
//  //TVC Data
//  double tvcY;
//  double tvcZ;
//  bool tvcActive;
//
//  //Roll wheel data
//  float rollPercent;
//  float rollSetpoint;
//
//  //Vehicle estimation
//  float twr;
//  float mass;
//  
//  //GNSS locking data
//  byte GNSSFix;
//  int GNSSpDOP;
//  byte GNSSSats;
//  int GNSSAccHoriz; //mm
//  int GNSSAccVert; //mm
//  int GNSSAccVel; //mm
//
//  //Pyro channel data
//  bool pyro1Cont;
//  bool pyro2Cont;
//  bool pyro3Cont;
//  bool pyro4Cont;
//  bool pyro5Cont;
//  bool pyro1Fire;
//  bool pyro2Fire;
//  bool pyro3Fire;
//  bool pyro4Fire;
//  bool pyro5Fire;
};
static struct TELEMETRY telem;

// create the XBee object
XBee xbee = XBee();

Tx16Request tx = Tx16Request(0x0001, (uint8_t*)&telem, sizeof(TELEMETRY));
TxStatusResponse txStatus = TxStatusResponse();

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  xbee.setSerial(Serial1);
  Serial.println(sizeof(telem));
  delay(1000);
}

void loop() {
  xbee.send(tx);
  Serial.println("Sent test");
  telem.timeSinceStartup = millis();

  if (xbee.readPacket(500)) {
    if (xbee.getResponse().getApiId() == TX_STATUS_RESPONSE) {
         xbee.getResponse().getTxStatusResponse(txStatus);
        
         // get the delivery status, the fifth byte
           if (txStatus.getStatus() == SUCCESS) {
              // success.  time to celebrate
              Serial.println("Send ok; receiver ok");
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

}
