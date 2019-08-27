#include <vector>
#include <iostream>

// platform
#include <windows.h>

#include "xarm_api.h"
#include "xarm_hid.h"

namespace xarm_api {

static const double pi = 3.14159265358979323846;
static const double minPos[6] = {130, 0, 0, 20, 140, 0};
static const double maxPos[6] = {690, 700, 700, 700, 880, 1000};
static const double minRad[6] = {-10 * (pi / 180), -90 * (pi / 180), -130 * (pi / 180), -110 * (pi / 180), -90 * (pi / 180), -150 * (pi / 180)};
static const double maxRad[6] = {90 * (pi / 180), 90 * (pi / 180), 40 * (pi / 180), 45 * (pi / 180), 90 * (pi / 180), 90 * (pi / 180)};

xarm_api::xarm_api(HANDLE deviceHandle)
{
    deviceHandle = hid::OpenHIDByVidPid(L"0483", L"5750");
}
       
xarm_api::~xarm_api()
{
    ::CloseHandle(deviceHandle);
}

std::vector<double> xarm_api::convertToRadian(const std::vector<int>& servoReadings)
{
    std::vector<double> radian;
    for (int i = 0; i < servoReadings.size(); i++)
    {
        radian.push_back(((servoReadings[i] - minPos[i]) * ((maxRad[i] - minRad[i]) / (maxPos[i] - minPos[i]))) + minRad[i]);
    }
    return radian;
}

std::vector<int> xarm_api::convertToServoReadings(const std::vector<double>& radian)
{
    std::vector<int> servoReadings;
    for (int i = 0; i < radian.size(); i++)
    {
        servoReadings.push_back(((radian[i] - minRad[i]) / ((maxRad[i] - minRad[i]) / (maxPos[i] - minPos[i]))) + minPos[i]);
    }
    return servoReadings;
}

void xarm_api::moveToPosition(::HANDLE device, const Joints& pos)
{
    const uint16_t actionTime = 1000;
    const auto servoPositions = xarm_api::convertToServoReadings(pos.data);
    xarm_hid::setServoPositions(device, actionTime, servoPositions, 50);
}

xarm_api::Joints xarm_api::readPosition(::HANDLE device)
{
    const auto servoReadings = xarm_hid::readServoPositions(device, { 1, 2, 3, 4, 5, 6 });
    xarm_api::Joints j;
    j.data = xarm_api::convertToRadian(servoReadings);
    return j;
}

void xarm_api::printCurrentPosition(::HANDLE device)
{
    auto joints = readPosition(device);
    for (const auto& joint : joints.data) {
        std::cout << joint << " ";
    }
    std::cout << std::endl;
}

} // namespace xarm
