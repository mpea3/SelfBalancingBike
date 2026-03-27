#pragma once

#include <Arduino.h>

// =============================================================================
// PID Controller
// General-purpose PID with anti-windup, derivative filtering, and output limits
// =============================================================================

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float masterGain);

    // Compute PID output given current error and time step
    float compute(float setpoint, float measurement, float dt);

    // Reset internal state (integral sum, previous error)
    void reset();

    // Runtime gain adjustment
    void setGains(float kp, float ki, float kd);
    void setMasterGain(float gain) { _masterGain = gain; }
    void setOutputLimits(float min, float max);
    void setIntegralLimits(float min, float max);

    // Accessors for debugging
    float getProportional() const { return _pTerm; }
    float getIntegral() const { return _iTerm; }
    float getDerivative() const { return _dTerm; }
    float getLastError() const { return _prevError; }

private:
    float _kp, _ki, _kd;
    float _masterGain;

    float _pTerm = 0.0f;
    float _iTerm = 0.0f;
    float _dTerm = 0.0f;

    float _integral = 0.0f;
    float _prevError = 0.0f;
    bool _firstRun = true;

    float _outputMin = -100.0f;
    float _outputMax = 100.0f;
    float _integralMin = -50.0f;
    float _integralMax = 50.0f;
};
