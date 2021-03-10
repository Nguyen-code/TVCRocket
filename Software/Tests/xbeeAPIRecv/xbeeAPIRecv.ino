
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
//
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
//
//  //Calculated state vector data
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
XBeeResponse response = XBeeResponse();
Rx16Response rx16 = Rx16Response();

void setup() {
  Serial.begin(9600);
  Serial1.begin(115200);
  xbee.setSerial(Serial1);
  delay(1000);
}

void loop() {
  xbee.readPacket();
  if (xbee.getResponse().isAvailable()) {
    xbee.getResponse().getRx16Response(rx16);
    TELEMETRY recv = (TELEMETRY &)*rx16.getData();
    telem.timeSinceStartup = recv.timeSinceStartup;

    Serial.print("TelemTime: "); Serial.print(telem.timeSinceStartup);

    uint8_t rssi = rx16.getRssi();
    Serial.print(" RSSI: "); Serial.println(rssi);
  }
}
