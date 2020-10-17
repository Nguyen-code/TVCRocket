#include "PID.h"
#include "Arduino.h"

double PID::update(double input, double dt)
{
    double error = setpoint - input;
    integral += error * dt;

    double derivative = (error - prevError) / dt;
    prevError = error;
    
    double output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastOutput = output;
    return output;
}

double PID::getLast() {
	return lastOutput;
}

void PID::resetIntegrator() {
	integral = 0; //reset it!
}