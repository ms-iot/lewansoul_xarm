#pragma once

#include <vector>

// platform
#include <windows.h>

namespace hid 
{
    class hid
    {
        public:

        enum class ArmHidCommands {
            Read = 0x15,
            Write = 0x03,
        };

        std::vector<BYTE> generateCommandPacket(ArmHidCommands command, const std::vector<BYTE>& arguments);

        void setServoPositions(::HANDLE device, int actionTime, const std::vector<int>& joints, int epsilon, bool wait = false);

        std::vector<int> readServoPositions(::HANDLE device, const std::vector<int>& ids);

        void sendData(::HANDLE device, const std::vector<BYTE>& data);

        std::vector<BYTE> recvData(::HANDLE device);
   
    };
}
