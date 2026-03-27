#include "motor_control.h"
#include "config.h"
#include <ArduinoMotorCarrier.h>

bool MotorControl::begin() {
    // Initialize communication with the Motor Carrier's SAMD11 co-processor
    if (!controller.begin()) {
        Serial.println("ERROR: Motor Carrier not detected!");
        return false;
    }

    // Ensure motors are stopped at startup
    stopAll();
    return true;
}

void MotorControl::setFlywheelDuty(float duty) {
    duty = constrain(duty, -FLYWHEEL_MAX_DUTY, FLYWHEEL_MAX_DUTY);
    duty = applyDeadZone(duty);

    // Motor Carrier expects -100 to +100 duty cycle
    int pwmVal = static_cast<int>(duty);

    if (FLYWHEEL_MOTOR_INDEX == 1) {
        M1.setDuty(pwmVal);
    } else if (FLYWHEEL_MOTOR_INDEX == 2) {
        M2.setDuty(pwmVal);
    } else if (FLYWHEEL_MOTOR_INDEX == 3) {
        M3.setDuty(pwmVal);
    } else {
        M4.setDuty(pwmVal);
    }
}

void MotorControl::setDriveDuty(float duty) {
    duty = constrain(duty, -100.0f, 100.0f);
    int pwmVal = static_cast<int>(duty);

    if (DRIVE_MOTOR_INDEX == 1) {
        M1.setDuty(pwmVal);
    } else if (DRIVE_MOTOR_INDEX == 2) {
        M2.setDuty(pwmVal);
    } else if (DRIVE_MOTOR_INDEX == 3) {
        M3.setDuty(pwmVal);
    } else {
        M4.setDuty(pwmVal);
    }
}

void MotorControl::stopAll() {
    M1.setDuty(0);
    M2.setDuty(0);
    M3.setDuty(0);
    M4.setDuty(0);
}

long MotorControl::getFlywheelEncoderCount() {
    if (FLYWHEEL_ENCODER_INDEX == 1) {
        return encoder1.getRawCount();
    }
    return encoder2.getRawCount();
}

void MotorControl::resetFlywheelEncoder() {
    if (FLYWHEEL_ENCODER_INDEX == 1) {
        encoder1.resetCounter(0);
    } else {
        encoder2.resetCounter(0);
    }
    _prevEncoderCount = 0;
    _flywheelSpeed = 0.0f;
    _saturated = false;
}

float MotorControl::getFlywheelSpeed(float dt) {
    if (dt <= 0.0f) return _flywheelSpeed;

    long currentCount = getFlywheelEncoderCount();
    long delta = currentCount - _prevEncoderCount;
    _prevEncoderCount = currentCount;

    _flywheelSpeed = static_cast<float>(delta) / dt;

    // Check saturation: flywheel spinning too fast in either direction
    _saturated = (fabsf(_flywheelSpeed) > FLYWHEEL_SATURATION_SPEED);

    return _flywheelSpeed;
}

float MotorControl::applyDeadZone(float duty) {
    // Smooth dead zone compensation using a continuous piecewise-linear ramp.
    // Instead of an abrupt offset that causes oscillation at zero crossing,
    // we smoothly map the [0..1] range into the dead zone and scale the rest.
    if (fabsf(duty) < 0.5f) {
        return 0.0f;
    }

    // Smooth ramp: output = sign(duty) * (deadzone + |duty| * (100 - deadzone) / 100)
    // This maps duty=0.5 → deadzone, duty=100 → 100
    float sign = (duty > 0) ? 1.0f : -1.0f;
    float absDuty = fabsf(duty);
    float scaled = FLYWHEEL_DEAD_ZONE + absDuty * (FLYWHEEL_MAX_DUTY - FLYWHEEL_DEAD_ZONE) / FLYWHEEL_MAX_DUTY;
    return sign * scaled;
}

void MotorControl::keepAlive() {
    // The Motor Carrier requires periodic pings to stay active
    controller.ping();
}
