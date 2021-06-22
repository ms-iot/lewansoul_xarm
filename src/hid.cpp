#include <array>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

// platform
#if defined(_WIN32)
    #include <windows.h>
#endif

#include "dep/hidapi.h"

#include "hid.h"

namespace xarm {

Hid::Hid()
{
    hid_init();
    device = hid_open(0x0483, 0x5750, nullptr);
}

Hid::~Hid()
{
    hid_close(device);
    hid_exit();
}

void Hid::sendData(const std::vector<unsigned char>& data)
{
    std::vector<unsigned char> packet = {
        0x00, // HID report ID
    };
    packet.insert(packet.end(), data.begin(), data.end());

    const auto HIDReportSize = 65;
    if (packet.size() > HIDReportSize)
    {
        throw;
    }
    packet.resize(HIDReportSize, 0x00);

    hid_write(device, &packet[0], packet.size());
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

std::vector<unsigned char> Hid::recvData()
{
    const auto HIDReportSize = 65;
    std::vector<unsigned char> buffer(HIDReportSize, 0x00);
    hid_read(device, &buffer[0], buffer.size());

    // when buffer is read with ::ReadFile, the first byte is the HID report ID
    // but buffer read from hid_read does not contain this byte
    // if (buffer[0] != 0x00)
    // {
    //     // first byte should be HID report ID
    //     throw;
    // }

    if (buffer[0] != 0x55 || buffer[1] != 0x55)
    {
        // should start with header bytes
        throw;
    }
    const auto headerBytes = 2;

    // actual data starts from the next byte, so actual length is dataLength - 1
    auto dataLength = static_cast<unsigned>(buffer[headerBytes]) - 1;

    const auto headerAndLengthBytes = headerBytes + 1;
    std::vector<unsigned char> data(buffer.begin() + headerAndLengthBytes, buffer.begin() + headerAndLengthBytes + dataLength);
    return data;
}

}
