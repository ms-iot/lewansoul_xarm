#include <array>
#include <vector>
#include <iostream>
#include <cmath>

// platform
#include <stdint.h>
#include <stdlib.h>
#include <string.h> // for wcscpy_s, wcscat_s
#include <string>
#include <windows.h>
#include <wchar.h>
#include <hidsdi.h>
#include <setupapi.h>
#include <initguid.h>
#include <devpkey.h>    // device property keys		   (DEVPKEY_Device_HardwareIds)
#include <devpropdef.h> // device property definitions (DEVPROP_TYPE_STRING_LIST)
#include <windows.h>

#include "xarm.h"
#include "joint.h"
#include "hid.h"

static double pi()
{
    return std::atan(1) * 4;
}

namespace xarm
{

Arm::Arm()
{
    initializeDevice();
    initializeJoints();
}

wchar_t *composeDevId(wchar_t *buff, const DWORD buffSize,
                      const wchar_t *vid, const wchar_t *pid)
{
    // compose the device identifier
    wcscpy_s(buff, buffSize, L"VID_");
    wcscat_s(buff, buffSize, vid);
    wcscat_s(buff, buffSize, L"&PID_");
    wcscat_s(buff, buffSize, pid);

    return buff;
}

HDEVINFO getDevInfoSet(const LPGUID devClassGuid)
{
    return SetupDiGetClassDevs(devClassGuid, nullptr, nullptr,
                               DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
}

bool getDevInfoByPIDVID(const HDEVINFO devInfoSet, const LPGUID devClassGuid,
                        const wchar_t *devId, PSP_DEVINFO_DATA devInfoBuffer)
{
    DEVPROPTYPE devPropType = DEVPROP_TYPE_STRING; // hardware id property type specifier
    wchar_t szBuff[256] = {'\0'};
    DWORD dwBuffReqSize = 0;

    SP_DEVINFO_DATA currDevInfo; // device info iterator
    currDevInfo.cbSize = sizeof(SP_DEVINFO_DATA);

    for (DWORD idx = 0; SetupDiEnumDeviceInfo(devInfoSet, idx, &currDevInfo); idx++)
    {
        // https://docs.microsoft.com/en-us/windows/win32/api/setupapi/nf-setupapi-setupdigetdevicepropertyw
        // SetupAPI supports only a Unicode version of SetupDiGetDeviceProperty
        if (::SetupDiGetDevicePropertyW(
                devInfoSet, &currDevInfo,                         // device info
                &DEVPKEY_Device_HardwareIds, &devPropType,        // key info
                (PBYTE)szBuff, sizeof(szBuff), &dwBuffReqSize, 0) // out buffers
            && wcsstr(szBuff, devId))
        {
            *devInfoBuffer = currDevInfo;
            return true;
        }
    }

    return false;
}

bool extractInterfData(const HDEVINFO devInfoSet, PSP_DEVINFO_DATA devInfo,
                       const LPGUID devClassGuid, PSP_DEVICE_INTERFACE_DATA interfData)
{
    SP_DEVICE_INTERFACE_DATA currentInterfData;
    currentInterfData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);

    // TODO consider devices with multiple interfaces (ours now has only one)
    if (SetupDiEnumDeviceInterfaces(devInfoSet, devInfo,
                                    devClassGuid, 0, &currentInterfData))
    {
        *interfData = currentInterfData;
        return true;
    }

    return false;
}

HANDLE getHandleToInterface(const HDEVINFO devInfoSet, const PSP_DEVICE_INTERFACE_DATA devInterfData)
{
    HANDLE handle = nullptr;

    // 1. get device interface detail buffer size

    DWORD requiredSize = 0;
    SetupDiGetDeviceInterfaceDetail(devInfoSet, devInterfData, nullptr, 0, &requiredSize, nullptr);

    // 2. get interface detail and open handle

    PSP_DEVICE_INTERFACE_DETAIL_DATA devInterfDetail =
        (PSP_DEVICE_INTERFACE_DETAIL_DATA)malloc(requiredSize);

    if (devInterfDetail != nullptr)
    {
        devInterfDetail->cbSize = sizeof(SP_INTERFACE_DEVICE_DETAIL_DATA);
        if (SetupDiGetDeviceInterfaceDetail(devInfoSet, devInterfData,
                                            devInterfDetail, requiredSize, nullptr, nullptr))
        {
            handle = CreateFile(devInterfDetail->DevicePath, GENERIC_READ | GENERIC_WRITE,
                                FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        }

        // clean up temporary buffer
        free(devInterfDetail);
    }

    return handle;
}

HANDLE OpenHIDByVidPid(const WCHAR *vid, const WCHAR *pid)
{
    HANDLE devHandle = nullptr;

    // 1. get device interface class

    GUID devIClassGuid;
    HidD_GetHidGuid(&devIClassGuid);

    // 2. get set of devices with interfaces within such class

    HDEVINFO devInfoSet = getDevInfoSet(&devIClassGuid);

    // 3. filter through set looking for device information with specific pid and vid identifiers

    DWORD hardwareIDLength = wcslen(vid) + wcslen(pid) + 8 + 2; // should be 16 + termination char
    wchar_t *hardwareId = new wchar_t[hardwareIDLength];
    composeDevId(hardwareId, hardwareIDLength, vid, pid);

    SP_DEVINFO_DATA devInfo;
    if (getDevInfoByPIDVID(devInfoSet, &devIClassGuid, hardwareId, &devInfo))
    {
        // 4. get interface data from device information

        SP_DEVICE_INTERFACE_DATA devInterfData;
        if (extractInterfData(devInfoSet, &devInfo, &devIClassGuid, &devInterfData))
        {
            // 5. get handle from device path contained in device interface details structure
            devHandle = getHandleToInterface(devInfoSet, &devInterfData);
        }
    }

    // 6. cleanup

    SetupDiDestroyDeviceInfoList(devInfoSet);

    return devHandle;
}

void Arm::initializeDevice()
{
    device = OpenHIDByVidPid(L"0483", L"5750");
}

void Arm::initializeJoints()
{
    auto const degreeToRadianWorker = [](double degree) -> double {
        return degree * pi() / 180;
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

void Arm::resetJointPositions()
{
    const uint16_t actionTime = 1000;
    setServoPositions(device, actionTime, {500, 500, 500, 500, 500, 500}, 50);
}

void Arm::setJointPositions(const std::array<double, Arm::numJoints> &pos)
{
    const uint16_t actionTime = 1000;
    const auto servoPositions = convertToServoReadings(pos);
    setServoPositions(device, actionTime, servoPositions, 50);
}

std::array<double, Arm::numJoints> Arm::getJointPositions()
{
    const auto servoReadings = readServoPositions(device);
    return convertToRadian(servoReadings);
}

} // namespace xarm
