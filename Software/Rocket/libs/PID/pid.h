#ifndef PIDlib_h
#define PIDlib_h

class PID
{
public:
    double Kp = 0, Ki = 0, Kd = 0;
    double integral = 0;
    double setpoint = 0;
    double input;
    double lastOutput;

    PID() {  }; // Default initializer
    PID(double p, double i, double d) { Kp = p, Ki = i; Kd = d; }; // Gains initializer
    PID(double p, double i, double d, double s) { Kp = p, Ki = i; Kd = d; setpoint = s; }; // Full initializer

    double update(double input, double dt); // Updates PID maths and returns new output
    double getLast(); //Gets last PID value
private:
    double prevError = 0;
};

#endif