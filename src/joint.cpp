#include "joint.h"

namespace xarm {

Joint::Joint(int minServo, int maxServo, double minAngle, double maxAngle)
{
    this->minServo = minServo;
    this->maxServo = maxServo;
    this->minAngle = minAngle;
    this->maxAngle = maxAngle;
}

int Joint::getServo()
{
    return this->servo;
}

void Joint::setServo(int servo)
{
    // check with bounds

    this->servo = servo;
}

double Joint::getAngle()
{
    return convertToRadian(this->servo);
}

void Joint::setAngle(double angle)
{
    // check with bounds

    this->servo = convertToServoReading(angle);
}

double Joint::convertToRadian(int servo)
{
    // check with bounds

    const auto diff = servo - minServo;
    const auto ratio = (maxAngle - minAngle) / (maxServo - minServo);
    return (minAngle + diff * ratio);
}

int Joint::convertToServoReading(double angle)
{
    // check with bounds

    const auto diff = angle - minAngle;
    const auto ratio = (maxServo - minServo) / (maxAngle - minAngle);
    return (minServo + diff * ratio);
}

}
