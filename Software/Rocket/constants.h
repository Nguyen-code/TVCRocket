#ifndef constants_h
#define constants_h

#include "Arduino.h"

/*
TVC OFFSETS
*/

//Channel 1
#define TVC_X_CH1_OFFSET 0.0
#define TVC_Y_CH1_OFFSET 0.0
//Channel 2
#define TVC_X_CH2_OFFSET 0.0
#define TVC_Y_CH2_OFFSET 0.0

/*
PID VALUES
*/

const float PID_Z_P = 1.5;
const float PID_Z_I = 0.00;
const float PID_Z_D = 0.2;

const float PID_Y_P = 1.5;
const float PID_Y_I = 0.00;
const float PID_Y_D = 0.2;

//WORKING
// const float PID_Z_P = 1.5;
// const float PID_Z_I = 0.00;
// const float PID_Z_D = 0.2;

// const float PID_Y_P = 1.5;
// const float PID_Y_I = 0.00;
// const float PID_Y_D = 0.2;


/*
SERVO CONSTANTS
*/
#define SERVO_MULT 5
#define SERVO_RANGE SERVO_MULT*1.5

/*
ADC RESISTOR DIVIDER CALCUALTIONS
*/
const short ADC_VBUS_DIVD_RES1 = 1000; //VBUS = main input voltage, 12.6V max
const short ADC_VBUS_DIVD_RES2 = 330;
const double ADC_RES_DIV_FACTOR_VBUS = (double)ADC_VBUS_DIVD_RES2/((double)ADC_VBUS_DIVD_RES1+(double)ADC_VBUS_DIVD_RES2);

const short ADC_VSERVO_DIVD_RES1 = 1000; //VSERVO = servo voltage rail
const short ADC_VSERVO_DIVD_RES2 = 620;
const double ADC_RES_DIV_FACTOR_VSERVO = (double)ADC_VSERVO_DIVD_RES2/((double)ADC_VSERVO_DIVD_RES1+(double)ADC_VSERVO_DIVD_RES2);

const short ADC_VMOTOR_DIVD_RES1 = 560; //VMOTOR = roll motor voltage rail
const short ADC_VMOTOR_DIVD_RES2 = 680;
const double ADC_RES_DIV_FACTOR_VMOTOR = (double)ADC_VMOTOR_DIVD_RES2/((double)ADC_VMOTOR_DIVD_RES1+(double)ADC_VMOTOR_DIVD_RES2);

const short ADC_PYRO_DIVD_RES1 = 1000; //PYRO = Pyro voltage, same as VBUS in this version of the flight controller
const short ADC_PYRO_DIVD_RES2 = 330;
const double ADC_RES_DIV_FACTOR_PYRO = (double)ADC_PYRO_DIVD_RES2/((double)ADC_PYRO_DIVD_RES1+(double)ADC_PYRO_DIVD_RES2);

const double ADC_MAX_V = 4.096;
const double ADC_DIV_FACTOR_V = ADC_MAX_V/2047; //11 bit mantissa

const float ADC_PYRO_THRESH_CONT = 10.9; //voltage drop in v for cont

/*
MISC CONSTANT SETTINGS
*/

const bool buzzerEnabled = false;
const bool indicatorEnabled = false;


#endif