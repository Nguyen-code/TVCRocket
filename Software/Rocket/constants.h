#ifndef constants_h
#define constants_h

#include "Arduino.h"


/*
TVC OFFSETS
*/

//Channel 1
#define TVC_CH1_X_OFFSET 0.0
#define TVC_CH1_Y_OFFSET 0.0
//Channel 2
#define TVC_CH2_X_OFFSET 0.0
#define TVC_CH2_Y_OFFSET 0.0

/*
PID VALUES
*/

const float PID_X_P = 0.1;
const float PID_X_I = 0.0;
const float PID_X_D = 0.0;

const float PID_Y_P = 0.1;
const float PID_Y_I = 0.0;
const float PID_Y_D = 0.0;

/*
ADC RESISTOR DIVIDER CALCUALTIONS
*/
#define ADC_VBUS_DIVD_RES1 1000; //VBUS = main input voltage, 12.6V max
#define ADC_VBUS_DIVD_RES2 330;
const double ADC_RES_DIV_FACTOR_VBUS = (double)ADC_VBUS_DIVD_RES2/((double)ADC_VBUS_DIVD_RES1+(double)ADC_VBUS_DIVD_RES2);

#define ADC_VSERVO_DIVD_RES1 1000; //VSERVO = servo voltage rail
#define ADC_VSERVO_DIVD_RES2 620;
const double ADC_RES_DIV_FACTOR_VSERVO = (double)ADC_VSERVO_DIVD_RES2/((double)ADC_VSERVO_DIVD_RES1+(double)ADC_VSERVO_DIVD_RES2);

#define ADC_VMOTOR_DIVD_RES1 560; //VMOTOR = roll motor voltage rail
#define ADC_VMOTOR_DIVD_RES2 680;
const double ADC_RES_DIV_FACTOR_VMOTOR = (double)ADC_VMOTOR_DIVD_RES2/((double)ADC_VMOTOR_DIVD_RES1+(double)ADC_VMOTOR_DIVD_RES2);

#define ADC_PYRO_DIVD_RES1 1000; //PYRO = Pyro voltage, same as VBUS in this version of the flight controller
#define ADC_PYRO_DIVD_RES2 330;
const double ADC_RES_DIV_FACTOR_PYRO = (double)ADC_PYRO_DIVD_RES2/((double)ADC_PYRO_DIVD_RES1+(double)ADC_PYRO_DIVD_RES2);

const double ADC_MAX_V = 6.144;
const double ADC_DIV_FACTOR_V = ADC_MAX_V/4096;

const float ADC_PYRO_THRESH_CONT = 0.22; //voltage drop in v for short
const float ADC_PYRO_THRESH_SHORT = 0.03; //voltage drop in v for short

#endif