#include "pid_controller.h"
#include "config.h"

PIDController::PIDController(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd), _filterN(DERIVATIVE_FILTER_N) {
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

    // Derivative term: computed on measurement (not error) to avoid derivative kick
    // when the setpoint changes. Also applies a first-order low-pass filter to
    // suppress high-frequency sensor noise.
    if (_firstRun) {
        _dTerm = 0.0f;
        _filteredDerivative = 0.0f;
        _firstRun = false;
    } else {
        // Raw derivative of measurement (negative sign because d(error)/dt = -d(measurement)/dt
        // when setpoint is constant, and we want the same sign convention)
        float rawDerivative = -(measurement - _prevMeasurement) / dt;

        // First-order low-pass filter: y[n] = alpha * y[n-1] + (1-alpha) * x[n]
        // alpha = N*dt / (1 + N*dt) gives filter coefficient from time constant
        float alpha = (_filterN * dt) / (1.0f + _filterN * dt);
        _filteredDerivative = (1.0f - alpha) * _filteredDerivative + alpha * rawDerivative;

        _dTerm = _kd * _filteredDerivative;
    }

    _prevError = error;
    _prevMeasurement = measurement;

    // Combine terms
    float output = _pTerm + _iTerm + _dTerm;

    // Clamp output
    return constrain(output, _outputMin, _outputMax);
}

void PIDController::reset() {
    _integral = 0.0f;
    _prevError = 0.0f;
    _prevMeasurement = 0.0f;
    _filteredDerivative = 0.0f;
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
