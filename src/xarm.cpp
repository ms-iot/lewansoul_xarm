#include <array>
#include <vector>
#include <iostream>
#include <cmath>

// platform
#include <windows.h>

#include "xarm.h"
#include "joint.h"
#include "hid.h"

namespace xarm
{

Arm::Arm()
{
    initializeDevice();
    initializeJoints();
}

Arm::~Arm()
{
    // @todo: move to device class
    ::CloseHandle(device);
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

void Arm::initializeDevice()
{
    device = hid::initializeDevice(L"0483", L"5750");
}

void Arm::initializeJoints()
{
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
    // Header:
    // Received two consecutive 0x55, indicates that the data packets arrived.
    // Length:
    // Equal to the parameter number N plus a command and plus the byte length occupied by the data length itself. That means the data length is equal to the parameter N plus 2. (Length = N + command + a byte length = N + 2)
    // Command:
    // Various control instructions.
    // Parameter:
    // In addition to the command, the need to add control information

    // @todo: verify parameters

    std::vector<unsigned char> packet = {
        0x55, // first byte of header
        0x55, // second byte of header
    };
    const int dataLength = arguments.size() + 2;
    packet.push_back(static_cast<unsigned char>(dataLength));
    packet.push_back(static_cast<unsigned char>(command));
    packet.insert(packet.end(), arguments.begin(), arguments.end());
    hid::sendData(device, packet);
}

void Arm::setServoPositions(int actionTime, const std::array<int, Arm::numJoints>& positions, int epsilon, bool wait)
{
    std::vector<unsigned char> arguments;
    arguments.push_back(static_cast<BYTE>(positions.size()));
    arguments.push_back(0x00ff & actionTime);
    arguments.push_back((0xff00 & actionTime) >> 8);
    for (auto i = 0; i < positions.size(); ++i)
    {
        arguments.push_back(static_cast<BYTE>(i + 1));
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
    std::vector<BYTE> arguments;
    arguments.push_back(static_cast<BYTE>(ids.size()));
    for (const auto& id : ids)
    {
        arguments.push_back(static_cast<BYTE>(id));
    }
    sendCommand(Arm::Commands::Read, arguments);

    const auto reading = hid::recvData(device);
    if (reading[0] != static_cast<BYTE>(Arm::Commands::Read))
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
    std::vector<BYTE> arguments;
    arguments.push_back(static_cast<BYTE>(6));
    for (auto id = 1; id <= 6; ++id)
    {
        arguments.push_back(static_cast<BYTE>(id));
    }
    sendCommand(Arm::Commands::Read, arguments);

    const auto reading = hid::recvData(device);
    if (reading[0] != static_cast<BYTE>(Arm::Commands::Read))
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

} // namespace xarm
