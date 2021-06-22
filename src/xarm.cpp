#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <array>
#include <vector>
#include <iostream>
#include <cmath>
#include <functional>

#include "ihid.h"
#include "xarm.h"
#include "joint.h"
#include "hid.h"

namespace xarm
{

Arm::Arm(std::shared_ptr<rclcpp::Node> node)
: _node(node)
{
}

bool Arm::init()
{
    auto sub = _node->create_subscription<sensor_msgs::msg::JointState>("/joint_states_goal", 1, 
        std::bind(&Arm::handleJointStateRequest, this, std::placeholders::_1));

    device = std::make_unique<Hid>();

    if (!device->init())
    {
        return false;
    }

    auto const degreeToRadianWorker = [](double degree) -> double {
        static const auto pi = std::atan(1) * 4;
        return degree * pi / 180;
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

    return true;
}


void Arm::resetJointPositions()
{
    const uint16_t actionTime = 1000;
    setServoPositions(actionTime, {500, 500, 500, 500, 500, 500}, 50);
}

void Arm::setJointPositions(const std::array<double, Arm::numJoints> &pos)
{
    const uint16_t actionTime = 1000;
    const auto servoPositions = convertToServoReadings(pos);
    setServoPositions(actionTime, servoPositions, 50);
}

std::array<double, Arm::numJoints> Arm::getJointPositions()
{
    const auto servoReadings = readServoPositions();
    return convertToRadian(servoReadings);
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

void Arm::sendCommand(const Arm::Commands& command, const std::vector<unsigned char>& arguments)
{
    // @todo: verify parameters

    std::vector<unsigned char> packet = {
        0x55, // first byte of header
        0x55, // second byte of header
    };
    size_t dataLength = arguments.size() + 2;
    packet.push_back(static_cast<unsigned char>(dataLength));
    packet.push_back(static_cast<unsigned char>(command));
    packet.insert(packet.end(), arguments.begin(), arguments.end());
    device->sendData(packet);
}

void Arm::setServoPositions(int actionTime, const std::array<int, Arm::numJoints>& positions, int epsilon, bool wait)
{
    std::vector<unsigned char> arguments;
    arguments.push_back(static_cast<unsigned char>(positions.size()));
    arguments.push_back(0x00ff & actionTime);
    arguments.push_back((0xff00 & actionTime) >> 8);
    for (auto i = 0; i < positions.size(); ++i)
    {
        arguments.push_back(static_cast<unsigned char>(i + 1));
        arguments.push_back(0x00ff & positions[i]);
        arguments.push_back((0xff00 & positions[i]) >> 8);
    }
    sendCommand(Arm::Commands::Write, arguments);
    if (!wait)
    {
        return;
    }

    auto pl = positions;
    for (auto& p : pl)
    {
        p -= epsilon;
    }
    auto ph = positions;
    for (auto& p : ph)
    {
        p += epsilon;
    }
    while (true)
    {
        const auto pos = readServoPositions();
        bool pass = true;
        for (auto i = 0; i < pos.size(); ++i)
        {
            if (pos[i] < pl[i] || pos[i] > ph[i])
            {
                pass = false;
                break;
            }
        }
        if (pass)
        {
            break;
        }
    }
}

std::vector<int> Arm::readServoPositions(const std::vector<int>& ids)
{
    std::vector<unsigned char> arguments;
    arguments.push_back(static_cast<unsigned char>(ids.size()));
    for (const auto& id : ids)
    {
        arguments.push_back(static_cast<unsigned char>(id));
    }
    sendCommand(Arm::Commands::Read, arguments);

    const auto reading = device->recvData();
    if (reading[0] != static_cast<unsigned char>(Arm::Commands::Read))
    {
        throw;
    }
    const auto numServo = static_cast<size_t>(reading[1]);
    if (reading.size() != numServo * 3 + 2)
    {
        throw;
    }
    std::vector<int> servoPositions(6, -1);
    for (auto i = 0; i < numServo; ++i)
    {
        // actual reading starts from byte 3, each servo reading takes 3 bytes
        auto p = 2 + 3 * i;
        const auto& topBits = reading[p + 2];
        const auto& bottomBits = reading[p + 1];
        servoPositions[static_cast<unsigned>(reading[p]) - 1] = (topBits << 8) | bottomBits;
    }
    return servoPositions;
}

std::array<int, Arm::numJoints> Arm::readServoPositions()
{
    std::vector<unsigned char> arguments;
    arguments.push_back(static_cast<unsigned char>(6));
    for (auto id = 1; id <= 6; ++id)
    {
        arguments.push_back(static_cast<unsigned char>(id));
    }
    sendCommand(Arm::Commands::Read, arguments);

    const auto reading = device->recvData();
    if (reading[0] != static_cast<unsigned char>(Arm::Commands::Read))
    {
        throw;
    }
    const auto numServo = static_cast<size_t>(reading[1]);
    if (reading.size() != numServo * 3 + 2)
    {
        throw;
    }
    std::array<int, 6> servoPositions = { 0 };
    for (auto i = 0; i < numServo; ++i)
    {
        // actual reading starts from byte 3, each servo reading takes 3 bytes
        auto p = 2 + 3 * i;
        const auto& topBits = reading[p + 2];
        const auto& bottomBits = reading[p + 1];
        servoPositions[static_cast<unsigned>(reading[p]) - 1] = (topBits << 8) | bottomBits;
    }
    return servoPositions;
}


void Arm::handleJointStateRequest(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    const auto &name = msg->name;
    const auto &position = msg->position;
    // const auto &velocity = msg->velocity;
    // const auto &effort = msg->effort;

    // @todo: sanity check with joint names
    if (name.size() != position.size())
    {
        RCLCPP_ERROR(_node->get_logger(),"name [size: %d] and position [size: %d] don't match!", name.size(), position.size());
        return;
    }

    RCLCPP_INFO(_node->get_logger(), "received joint states!");
    std::array<double, Arm::numJoints> pos = {0};
    for (auto i = 0; i < name.size(); i++)
    {
        const auto &n = name[i];
        const auto &p = position[i];
        if (n == left_gripper_joint)
        {
            pos[0] = p;
        }
        else if (n == wrist_joint_1)
        {
            pos[1] = p;
        }
        else if (n == wrist_joint_2)
        {
            pos[2] = p;
        }
        else if (n == elbow_joint)
        {
            pos[3] = p;
        }
        else if (n == shoulder_joint)
        {
            pos[4] = p;
        }
        else if (n == base_joint)
        {
            pos[5] = p;
        }
    };

    setJointPositions(pos);
}


} // namespace xarm
