#pragma once

#include <vector>

// platform
#include <windows.h>

#include "hid.h"

namespace xarm_api {

    class xarm_api
    {
        private:
        HANDLE m_deviceHandle = hid::OpenHIDByVidPid(L"0483", L"5750");

        public:
        xarm_api();
        ~xarm_api();

        HANDLE getxArmHandle()
        {
            return this->m_deviceHandle;
        }

        struct Joints 
        {
            Joints()
            {
                data.resize(6, -1);
            }
            std::vector<double> data;
        };
        
        std::vector<double> convertToRadian(const std::vector<int>& servoReadings);
        std::vector<int> convertToServoReadings(const std::vector<double>& radian);
        void moveToPosition(::HANDLE device, const Joints& pos);
        Joints readPosition(::HANDLE device);
        void printCurrentPosition(::HANDLE device);
    };

} // namespace xarm
