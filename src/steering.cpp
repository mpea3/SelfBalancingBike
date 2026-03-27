#include "steering.h"
#include "config.h"
#include <ArduinoMotorCarrier.h>

bool Steering::begin() {
    center();
    return true;
}

void Steering::setAngle(int angle) {
    _currentAngle = constrain(angle, SERVO_MIN, SERVO_MAX);

    if (STEERING_SERVO_INDEX == 1) {
        servo1.setAngle(_currentAngle);
    } else if (STEERING_SERVO_INDEX == 2) {
        servo2.setAngle(_currentAngle);
    } else if (STEERING_SERVO_INDEX == 3) {
        servo3.setAngle(_currentAngle);
    } else {
        servo4.setAngle(_currentAngle);
    }
}

void Steering::center() {
    setAngle(SERVO_CENTER);
}

void Steering::steer(int offset) {
    setAngle(SERVO_CENTER + offset);
}
