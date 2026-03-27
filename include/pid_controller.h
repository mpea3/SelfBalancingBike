#pragma once

#include <Arduino.h>

// =============================================================================
// PID Controller
// Features: derivative-on-measurement, first-order derivative low-pass filter,
// anti-windup, configurable output limits
// =============================================================================

class PIDController {
public:
    PIDController(float kp, float ki, float kd);

    // Compute PID output given setpoint, measurement, and time step
    // Uses derivative-on-measurement to avoid derivative kick
    float compute(float setpoint, float measurement, float dt);

    // Reset internal state (integral sum, previous measurement, filter state)
    void reset();

    // Runtime gain adjustment
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setIntegralLimits(float min, float max);
    void setDerivativeFilterN(float n) { _filterN = n; }

    // Accessors for debugging
    float getProportional() const { return _pTerm; }
    float getIntegral() const { return _iTerm; }
    float getDerivative() const { return _dTerm; }
    float getLastError() const { return _prevError; }

private:
    float _kp, _ki, _kd;

    float _pTerm = 0.0f;
    float _iTerm = 0.0f;
    float _dTerm = 0.0f;

    float _integral = 0.0f;
    float _prevError = 0.0f;
    float _prevMeasurement = 0.0f;
    float _filteredDerivative = 0.0f;
    bool _firstRun = true;

    // Derivative low-pass filter coefficient
    // Higher N = less filtering (faster response, more noise)
    // Lower N = more filtering (slower response, less noise)
    float _filterN = 20.0f;

    float _outputMin = -100.0f;
    float _outputMax = 100.0f;
    float _integralMin = -50.0f;
    float _integralMax = 50.0f;
};
