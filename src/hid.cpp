#include <vector>
#include <chrono>
#include <thread>

// platform
#include <windows.h>

#include "hid.h"

std::vector<BYTE> generateCommandPacket(ArmHidCommands command, const std::vector<BYTE>& arguments)
{
    std::vector<BYTE> packet;
    const int dataLength = 1 + 1 + arguments.size();
    packet.push_back(static_cast<BYTE>(dataLength));
    packet.push_back(static_cast<BYTE>(command));
    packet.insert(packet.end(), arguments.begin(), arguments.end());
    return packet;
}

void setServoPositions(::HANDLE device, int actionTime, const std::vector<int>& positions, int epsilon, bool wait)
{
    std::vector<BYTE> arguments;
    arguments.push_back(static_cast<BYTE>(positions.size()));
    arguments.push_back(0x00ff & actionTime);
    arguments.push_back((0xff00 & actionTime) >> 8);
    for (auto i = 0; i < positions.size(); ++i)
    {
        arguments.push_back(static_cast<BYTE>(i + 1));
        arguments.push_back(0x00ff & positions[i]);
        arguments.push_back((0xff00 & positions[i]) >> 8);
    }
    const auto command = generateCommandPacket(ArmHidCommands::Write, arguments);
    sendData(device, command);
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
        const auto pos = readServoPositions(device, { 1, 2, 3, 4, 5, 6 });
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

std::vector<int> readServoPositions(::HANDLE device, const std::vector<int>& ids)
{
    std::vector<BYTE> arguments;
    arguments.push_back(static_cast<BYTE>(ids.size()));
    for (const auto& id : ids)
    {
        arguments.push_back(static_cast<BYTE>(id));
    }
    const auto command = generateCommandPacket(ArmHidCommands::Read, arguments);
    sendData(device, command);

    const auto reading = recvData(device);
    if (reading[0] != static_cast<BYTE>(ArmHidCommands::Read))
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

void sendData(::HANDLE device, const std::vector<BYTE>& data)
{
    if (!device || device == INVALID_HANDLE_VALUE)
    {
        throw;
    }

    std::vector<BYTE> packet = {
        0x00, // HID report ID
        0x55, // first byte of header
        0x55, // second byte of header
    };
    packet.insert(packet.end(), data.begin(), data.end());

    const auto HIDReportSize = 65;
    if (packet.size() > HIDReportSize)
    {
        throw;
    }
    packet.resize(HIDReportSize, 0x00);

    if (!::WriteFile(device, &packet[0], packet.size(), nullptr, nullptr))
    {
        const auto error = ::GetLastError();
        throw error;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

std::vector<BYTE> recvData(::HANDLE device)
{
    if (!device || device == INVALID_HANDLE_VALUE)
    {
        throw;
    }

    const auto HIDReportSize = 65;
    std::vector<BYTE> buffer(HIDReportSize, 0x00);
    if (!::ReadFile(device, &buffer[0], buffer.size(), nullptr, nullptr))
    {
        const auto error = ::GetLastError();
        throw error;
    }

    if (buffer[0] != 0x00)
    {
        // first byte should be HID report ID
        throw;
    }

    if (buffer[1] != 0x55 || buffer[2] != 0x55)
    {
        // should start with header bytes
        throw;
    }

    // actual data starts from the next byte, so actual length is dataLength - 1
    auto dataLength = static_cast<unsigned>(buffer[3]) - 1;
    std::vector<BYTE> data(buffer.begin() + 4, buffer.begin() + 4 + dataLength);
    return data;
}
