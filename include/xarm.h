#ifndef _XARM_ARM_H
#define _XARM_ARM_H

#include <array>
#include <memory>

#include "ihid.h"
#include "joint.h"

namespace xarm {


const std::string left_gripper_joint = "left_gripper_joint_0";
const std::string wrist_joint_1 = "wrist_joint_1";
const std::string wrist_joint_2 = "wrist_joint_2";
const std::string elbow_joint = "elbow_joint_3";
const std::string shoulder_joint = "shoulder_joint_4";
const std::string base_joint = "base_joint_5";


class Arm
{
    std::shared_ptr<rclcpp::Node> _node;
public:
    Arm(std::shared_ptr<rclcpp::Node> node);

    static const int numJoints = 6;

    bool init();

    void resetJointPositions();
    void setJointPositions(const std::array<double, Arm::numJoints>& pos);
    std::array<double, Arm::numJoints> getJointPositions();

private:
    enum class Commands {
        Read = 0x15,
        Write = 0x03,
    };

    // joint-servo convertion
    std::array<double, Arm::numJoints> convertToRadian(const std::array<int, Arm::numJoints>& servoReadings);
    std::array<int, Arm::numJoints> convertToServoReadings(const std::array<double, Arm::numJoints>& radian);

    // device control
    void sendCommand(const Arm::Commands& command, const std::vector<unsigned char>& arguments);
    void setServoPositions(int actionTime, const std::array<int, Arm::numJoints>& positions, int epsilon, bool wait = false);
    std::vector<int> readServoPositions(const std::vector<int>& ids);
    std::array<int, Arm::numJoints> readServoPositions();

    // bool ready = true;
    std::unique_ptr<IHid> device;
    std::array<std::unique_ptr<Joint>, Arm::numJoints> joints;

    void handleJointStateRequest(const sensor_msgs::msg::JointState::SharedPtr msg);

};

} // namespace xarm

#endif
