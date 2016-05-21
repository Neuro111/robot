#include "motorcontroller.h"
#include "Arduino.h"



MotorController::MotorController(int pwmR, int dirR, int pwmL, int dirL)
{
    _pwmR = pwmR;
    _pwmL = pwmL;
    _dirR = dirR;
    _dirL = dirL;

    pinMode(_pwmR, OUTPUT);
    pinMode(_pwmL, OUTPUT);
    pinMode(_dirR, OUTPUT);
    pinMode(_dirL, OUTPUT);
}


void MotorController::move(int speed, int minAbsSpeed)
{
    int direction = 1;

    if (speed < 0)
    {
        direction = -1;

        speed = min(speed, -1*minAbsSpeed);
        speed = max(speed, -255);
    }
    else
    {
        speed = max(speed, minAbsSpeed);
        speed = min(speed, 255);
    }

    if (speed == _currentSpeed) return;

    int realSpeed = max(minAbsSpeed, abs(speed));

    digitalWrite(_dirL, speed > 0 ? HIGH : LOW);
    digitalWrite(_dirR, speed > 0 ? HIGH : LOW);
    analogWrite(_pwmL, abs(realSpeed));
    analogWrite(_pwmR, abs(realSpeed));

    _currentSpeed = direction * realSpeed;
}


void MotorController::move(int speed)
{
    if (speed == _currentSpeed) return;

    if (speed > 255) speed = 255;
    else if (speed < -255) speed = -255;

    digitalWrite(_dirL, speed > 0 ? HIGH : LOW);
    digitalWrite(_dirR, speed > 0 ? HIGH : LOW);
    analogWrite(_pwmL, abs(speed));
    analogWrite(_pwmR, abs(speed));

    _currentSpeed = speed;
}


void MotorController::stopMoving()
{
    digitalWrite(_dirL, LOW);
    digitalWrite(_dirR, LOW);
    digitalWrite(_pwmL, HIGH);
    digitalWrite(_pwmR, HIGH);

    _currentSpeed = 0;
}
