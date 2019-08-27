#include <array>
#include <vector>
#include <iostream>
#include <cmath>

// platform
#include <windows.h>

#include "xarm.h"
#include "joint.h"
#include "hid.h"

static double pi()
{
    return std::atan(1) * 4;
}

namespace xarm
{

Arm::Arm()
{
    initialize();
}

void Arm::initialize()
{
    auto const degreeToRadianWorker = [](double degree) -> double {
        return degree * pi() / 180;
    };

    static const std::array<double, Arm::numJoints> minServo = {
        130,
        0,
        0,
        20,
        140,
        0};
    static const std::array<double, Arm::numJoints> maxServo = {
        690,
        700,
        700,
        700,
        880,
        1000};
    static const std::array<double, Arm::numJoints> minAngle = {
        degreeToRadianWorker(-10),
        degreeToRadianWorker(-90),
        degreeToRadianWorker(-130),
        degreeToRadianWorker(-110),
        degreeToRadianWorker(-90),
        degreeToRadianWorker(-150)};
    static const std::array<double, Arm::numJoints> maxAngle = {
        degreeToRadianWorker(90),
        degreeToRadianWorker(90),
        degreeToRadianWorker(40),
        degreeToRadianWorker(45),
        degreeToRadianWorker(90),
        degreeToRadianWorker(90)};

    for (auto i = 0; i < Arm::numJoints; ++i)
    {
        joints[i] = std::make_unique<Joint>(minServo[i], maxServo[i], minAngle[i], maxAngle[i]);
    }
}

std::array<double, Arm::numJoints> Arm::convertToRadian(const std::array<int, Arm::numJoints> &servoReadings)
{
    std::array<double, Arm::numJoints> radian;
    for (int i = 0; i < servoReadings.size(); i++)
    {
        radian[i] = this->joints[i]->convertToRadian(servoReadings[i]);
    }
    return radian;
}

std::array<int, Arm::numJoints> Arm::convertToServoReadings(const std::array<double, Arm::numJoints> &radian)
{
    std::array<int, Arm::numJoints> servoReadings;
    for (int i = 0; i < radian.size(); i++)
    {
        servoReadings[i] = this->joints[i]->convertToServoReading(radian[i]);
    }
    return servoReadings;
}

void Arm::resetPosition(::HANDLE device)
{
    const uint16_t actionTime = 1000;
    setServoPositions(device, actionTime, {500, 500, 500, 500, 500, 500}, 50);
}

void Arm::setPosition(::HANDLE device, const std::array<double, Arm::numJoints> &pos)
{
    const uint16_t actionTime = 1000;
    const auto servoPositions = convertToServoReadings(pos);
    setServoPositions(device, actionTime, servoPositions, 50);
}

std::array<double, Arm::numJoints> Arm::getPosition(::HANDLE device)
{
    const auto servoReadings = readServoPositions(device);
    return convertToRadian(servoReadings);
}

} // namespace xarm
