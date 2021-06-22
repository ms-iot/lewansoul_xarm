#ifndef _XARM_HID_H
#define _XARM_HID_H

#include <string>
#include <vector>

#include "dep/hidapi.h"

#include "ihid.h"

namespace xarm {

class Hid : public IHid
{
public:
    Hid();
    ~Hid();

    void sendData(const std::vector<unsigned char>& data);
    std::vector<unsigned char> recvData();

    bool init();

private:
    hid_device* device = nullptr;
};

}

#endif
