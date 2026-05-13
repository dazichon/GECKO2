#ifndef PID_H
#define PID_H

class PID {
private:
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float lastError;
    float outputMin, outputMax;
    float deadzone = 50;

public:
    PID(float Kp, float Ki, float Kd, float minOutput = -255, float maxOutput = 255);

    void setDeadzone(float dz) { deadzone = dz; }

    float get_set();

    void setSetpoint(float sp);

    void setTunings(float kp, float ki, float kd);

    float compute(float input, float dt);

    void reset();
};

#endif