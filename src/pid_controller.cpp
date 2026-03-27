#include "pid_controller.h"
#include "config.h"

PIDController::PIDController(float kp, float ki, float kd, float masterGain)
    : _kp(kp), _ki(ki), _kd(kd), _masterGain(masterGain) {
    setOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    setIntegralLimits(PID_INTEGRAL_MIN, PID_INTEGRAL_MAX);
}

float PIDController::compute(float setpoint, float measurement, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    // Proportional term
    _pTerm = _kp * error;

    // Integral term with anti-windup clamping
    _integral += error * dt;
    _integral = constrain(_integral, _integralMin, _integralMax);
    _iTerm = _ki * _integral;

    // Derivative term (on error; skip first iteration)
    if (_firstRun) {
        _dTerm = 0.0f;
        _firstRun = false;
    } else {
        _dTerm = _kd * (error - _prevError) / dt;
    }

    _prevError = error;

    // Combine and apply master gain
    float output = _masterGain * (_pTerm + _iTerm + _dTerm);

    // Clamp output
    return constrain(output, _outputMin, _outputMax);
}

void PIDController::reset() {
    _integral = 0.0f;
    _prevError = 0.0f;
    _pTerm = 0.0f;
    _iTerm = 0.0f;
    _dTerm = 0.0f;
    _firstRun = true;
}

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setOutputLimits(float min, float max) {
    _outputMin = min;
    _outputMax = max;
}

void PIDController::setIntegralLimits(float min, float max) {
    _integralMin = min;
    _integralMax = max;
}
