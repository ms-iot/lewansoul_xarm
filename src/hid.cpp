#include "hid.h"
#include "setupapi.h"

wchar_t* hid::composeDevId(wchar_t* buff, const DWORD buffSize,
	const wchar_t* vid, const wchar_t* pid)
{
	// compose the device identifier
	wcscpy_s(buff, buffSize, L"VID_");
	wcscat_s(buff, buffSize, vid);
	wcscat_s(buff, buffSize, L"&PID_");
	wcscat_s(buff, buffSize, pid);

	return buff;
}

HDEVINFO hid::getDevInfoSet(const LPGUID devClassGuid)
{
	return SetupDiGetClassDevs(devClassGuid, nullptr, nullptr,
		DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
}

bool hid::getDevInfoByPIDVID(const HDEVINFO devInfoSet, const LPGUID devClassGuid, const wchar_t* devId, PSP_DEVINFO_DATA devInfoBuffer)
{
	DEVPROPTYPE devPropType = DEVPROP_TYPE_STRING; // hardware id property type specifier
	wchar_t szBuff[256] = { '\0' };
	DWORD dwBuffReqSize = 0;

	SP_DEVINFO_DATA currDevInfo; // device info iterator
	currDevInfo.cbSize = sizeof(SP_DEVINFO_DATA);

	for (DWORD idx = 0; SetupDiEnumDeviceInfo(devInfoSet, idx, &currDevInfo); idx++)
	{
		if (SetupDiGetDeviceProperty(
			devInfoSet, &currDevInfo,						   // device info
			&DEVPKEY_Device_HardwareIds, &devPropType,		   // key info
			(PBYTE)szBuff, sizeof(szBuff), &dwBuffReqSize, 0)  // out buffers
			&& wcsstr(szBuff, devId))
		{
			*devInfoBuffer = currDevInfo;
			return true;
		}
	}

	return false;
}

bool hid::extractInterfData(const HDEVINFO devInfoSet, PSP_DEVINFO_DATA devInfo,
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

HANDLE hid::getHandleToInterface(const HDEVINFO devInfoSet, const PSP_DEVICE_INTERFACE_DATA devInterfData)
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

HANDLE hid::OpenHIDByVidPid(const WCHAR* vid, const WCHAR* pid)
{
	HANDLE devHandle = nullptr;

	// 1. get device interface class

	GUID devIClassGuid;
	HidD_GetHidGuid(&devIClassGuid);

	// 2. get set of devices with interfaces within such class

	HDEVINFO devInfoSet = getDevInfoSet(&devIClassGuid);

	// 3. filter through set looking for device information with specific pid and vid identifiers

	DWORD hardwareIDLength = wcslen(vid) + wcslen(pid) + 8 + 2; // should be 16 + termination char
	wchar_t* hardwareId = new wchar_t[hardwareIDLength];
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