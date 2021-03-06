#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "Arduino.h"


class MotorController
{
protected:
    int _pwmR, _pwmL, _dirR, _dirL;
    int _currentSpeed;
    int _currentTurn;
public:
    MotorController(int pwmR, int dirR, int pwmL, int dirL);
    void move(int speed);
    void move(int speed, int minAbsSpeed);
    void stopMoving();
    void moveTurn(int speed, int turn);
};


#endif
