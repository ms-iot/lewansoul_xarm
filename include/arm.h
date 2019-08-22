#pragma once

#include <vector>

// platform
#include <windows.h>

namespace xarm {

struct Joints {
    Joints()
    {
        data.resize(6, -1);
    }

    std::vector<double> data;
};

std::vector<double> convertToRadian(const std::vector<int>& servoReadings);

std::vector<int> convertToServoReadings(const std::vector<double>& radian);

void MoveToPosition(::HANDLE device, const Joints& pos);

Joints readPosition(::HANDLE device);

void PrintCurrentPosition(::HANDLE device);

} // namespace xarm
