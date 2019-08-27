#pragma once

#include <windows.h>
#include <setupapi.h>		// HDEVINFO, SP_DEVINFO_DATA, etc
#include <devpkey.h>		// device property keys (DEVPKEY_Device_HardwareIds)
#include <hidsdi.h>			// HidD_GetHidGuid

namespace hid
{
    wchar_t* composeDevId(wchar_t* buff, const DWORD buffSize, const wchar_t* vid, const wchar_t* pid);
    HDEVINFO getDevInfoSet(const LPGUID devClassGuid);
    bool getDevInfoByPIDVID(const HDEVINFO devInfoSet, const LPGUID devClassGuid, const wchar_t* devId, PSP_DEVINFO_DATA devInfoBuffer);
    bool extractInterfData(const HDEVINFO devInfoSet, PSP_DEVINFO_DATA devInfo, const LPGUID devClassGuid, PSP_DEVICE_INTERFACE_DATA interfData);
    HANDLE getHandleToInterface(const HDEVINFO devInfoSet, const PSP_DEVICE_INTERFACE_DATA devInterfData);
    HANDLE OpenHIDByVidPid(const WCHAR* vid, const WCHAR* pid);
}