#ifndef _XARM_HID_H
#define _XARM_HID_H

#include <string>
#include <vector>

// platform
#include <windows.h>

namespace hid {

::HANDLE initializeDevice(const std::wstring& vid, const std::wstring& pid);
void sendData(::HANDLE device, const std::vector<BYTE>& data);
std::vector<BYTE> recvData(::HANDLE device);

}

#endif
