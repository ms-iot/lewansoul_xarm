#ifndef _XARM_IHID_H
#define _XARM_IHID_H

#include <vector>

namespace xarm {

class IHid
{
public:
    ~IHid() = default;

    virtual void sendData(const std::vector<unsigned char>& data) = 0;
    virtual std::vector<unsigned char> recvData() = 0;
};

}

#endif
