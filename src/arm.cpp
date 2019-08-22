#include <vector>
#include <iostream>

// platform
#include <windows.h>

#include "arm.h"
#include "hid.h"

namespace xarm {

static const double pi = 3.14159265358979323846;
static const double minPos[6] = {130, 0, 0, 20, 140, 0};
static const double maxPos[6] = {690, 700, 700, 700, 880, 1000};
static const double minRad[6] = {-10 * (pi / 180), -90 * (pi / 180), -130 * (pi / 180), -110 * (pi / 180), -90 * (pi / 180), -150 * (pi / 180)};
static const double maxRad[6] = {90 * (pi / 180), 90 * (pi / 180), 40 * (pi / 180), 45 * (pi / 180), 90 * (pi / 180), 90 * (pi / 180)};

std::vector<double> convertToRadian(const std::vector<int>& servoReadings)
{
    std::vector<double> radian;
    for (int i = 0; i < servoReadings.size(); i++)
    {
        radian.push_back(((servoReadings[i] - minPos[i]) * ((maxRad[i] - minRad[i]) / (maxPos[i] - minPos[i]))) + minRad[i]);
    }
    return radian;
}

std::vector<int> convertToServoReadings(const std::vector<double>& radian)
{
    std::vector<int> servoReadings;
    for (int i = 0; i < radian.size(); i++)
    {
        servoReadings.push_back(((radian[i] - minRad[i]) / ((maxRad[i] - minRad[i]) / (maxPos[i] - minPos[i]))) + minPos[i]);
    }
    return servoReadings;
}

void MoveToPosition(::HANDLE device, const Joints& pos)
{
    const uint16_t actionTime = 1000;
    const auto servoPositions = convertToServoReadings(pos.data);
    setServoPositions(device, actionTime, servoPositions, 50);
}

Joints readPosition(::HANDLE device)
{
    const auto servoReadings = readServoPositions(device, { 1, 2, 3, 4, 5, 6 });
    Joints j;
    j.data = convertToRadian(servoReadings);
    return j;
}

void PrintCurrentPosition(::HANDLE device)
{
    auto joints = readPosition(device);
    for (const auto& joint : joints.data) {
        std::cout << joint << " ";
    }
    std::cout << std::endl;
}

} // namespace xarm
