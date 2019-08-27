#pragma once

#include <vector>

// platform
#include <windows.h>

#include "hid.h"

namespace xarm_api {

    class xarm_api
    {
        private:
        HANDLE deviceHandle;

        public:
        xarm_api(HANDLE deviceHandle);
        ~xarm_api();

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
