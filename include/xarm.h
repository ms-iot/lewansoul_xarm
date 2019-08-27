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
    ~Arm();

    static const int numJoints = 6;

    void resetJointPositions();
    void setJointPositions(const std::array<double, Arm::numJoints>& pos);
    std::array<double, Arm::numJoints> getJointPositions();

private:
    enum class Commands {
        Read = 0x15,
        Write = 0x03,
    };

    void initializeDevice();
    void initializeJoints();

    // joint-servo convertion
    std::array<double, Arm::numJoints> convertToRadian(const std::array<int, Arm::numJoints>& servoReadings);
    std::array<int, Arm::numJoints> convertToServoReadings(const std::array<double, Arm::numJoints>& radian);

    // device control
    void sendCommand(const Arm::Commands& command, const std::vector<unsigned char>& arguments);
    void setServoPositions(int actionTime, const std::array<int, Arm::numJoints>& positions, int epsilon, bool wait = false);
    std::vector<int> readServoPositions(const std::vector<int>& ids);
    std::array<int, Arm::numJoints> readServoPositions();

    ::HANDLE device = nullptr;

    // bool ready = true;
    std::array<std::unique_ptr<Joint>, Arm::numJoints> joints;
};

} // namespace xarm

#endif
