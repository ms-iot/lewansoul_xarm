#include <array>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

// platform
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

#include "hid.h"

namespace hid {

static wchar_t* composeDevId(wchar_t *buff, const DWORD buffSize,
                      const wchar_t *vid, const wchar_t *pid)
{
    // compose the device identifier
    wcscpy_s(buff, buffSize, L"VID_");
    wcscat_s(buff, buffSize, vid);
    wcscat_s(buff, buffSize, L"&PID_");
    wcscat_s(buff, buffSize, pid);

    return buff;
}

static HDEVINFO getDevInfoSet(const LPGUID devClassGuid)
{
    return SetupDiGetClassDevs(devClassGuid, nullptr, nullptr,
                               DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
}

static bool getDevInfoByPIDVID(const HDEVINFO devInfoSet, const LPGUID devClassGuid,
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

static bool extractInterfData(const HDEVINFO devInfoSet, PSP_DEVINFO_DATA devInfo,
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

static HANDLE getHandleToInterface(const HDEVINFO devInfoSet, const PSP_DEVICE_INTERFACE_DATA devInterfData)
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

::HANDLE initializeDevice(const std::wstring& vid, const std::wstring& pid)
{
    HANDLE devHandle = nullptr;

    // 1. get device interface class

    GUID devIClassGuid;
    HidD_GetHidGuid(&devIClassGuid);

    // 2. get set of devices with interfaces within such class

    HDEVINFO devInfoSet = getDevInfoSet(&devIClassGuid);

    // 3. filter through set looking for device information with specific pid and vid identifiers

    DWORD hardwareIDLength = vid.length() + pid.length() + 8 + 2; // should be 16 + termination char
    wchar_t *hardwareId = new wchar_t[hardwareIDLength];
    composeDevId(hardwareId, hardwareIDLength, vid.c_str(), pid.c_str());

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

void sendData(::HANDLE device, const std::vector<BYTE>& data)
{
    if (!device || device == INVALID_HANDLE_VALUE)
    {
        throw;
    }

    std::vector<BYTE> packet = {
        0x00, // HID report ID
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

}
