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

    void resetJointPositions();
    void setJointPositions(const std::array<double, Arm::numJoints>& pos);
    std::array<double, Arm::numJoints> getJointPositions();

private:
    void initializeDevice();
    void initializeJoints();

    std::array<double, Arm::numJoints> convertToRadian(const std::array<int, Arm::numJoints>& servoReadings);
    std::array<int, Arm::numJoints> convertToServoReadings(const std::array<double, Arm::numJoints>& radian);

    ::HANDLE device = nullptr;
    std::array<std::unique_ptr<Joint>, Arm::numJoints> joints;
};

} // namespace xarm

#endif
