#ifndef _XARM_ARM_H
#define _XARM_ARM_H

#include <array>
#include <memory>

// platform
#include <windows.h>

#include "joint.h"

namespace xarm {

class Arm
{
public:
    Arm();

    static const int numJoints = 6;

    std::array<double, Arm::numJoints> convertToRadian(const std::array<int, Arm::numJoints>& servoReadings);
    std::array<int, Arm::numJoints> convertToServoReadings(const std::array<double, Arm::numJoints>& radian);

private:
    void initialize();

    std::array<std::unique_ptr<Joint>, Arm::numJoints> joints;
};

void MoveToPosition(::HANDLE device, const std::array<double, 6>& pos);

std::array<double, 6> readPosition(::HANDLE device);

void PrintCurrentPosition(::HANDLE device);

} // namespace xarm

#endif
