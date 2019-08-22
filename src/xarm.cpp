#define UNICODE

#include "std_msgs/String.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <iostream>		// for cout
#include <iomanip>		// for setfill, setw
#include <vector>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>		// for wcscpy_s, wcscat_s
#include <string>
#include <windows.h>
#include <wchar.h>
#include <hidsdi.h>
#include <setupapi.h>
#include <initguid.h>	
#include <devpkey.h>    // device property keys		   (DEVPKEY_Device_HardwareIds)
#include <devpropdef.h> // device property definitions (DEVPROP_TYPE_STRING_LIST)

#include "arm.h"

using namespace xarm;

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
        if (SetupDiGetDeviceProperty(
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

void xArmJointState_Callback(const std_msgs::String::ConstPtr& msg)
{
}

int main(int argc, char **argv)
{
    HANDLE device = OpenHIDByVidPid(L"0483", L"5750");
    // setup (center)
    Joints pos;
    pos.data = convertToRadian({ 500, 500, 500, 500, 500, 500});

    MoveToPosition(device, pos);

    ros::init(argc, argv, "xarm_rviz_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Subscriber sub = nh.subscribe("xarm_joint_state_topic", 1, xArmJointState_Callback);
    ros::Rate loop_rate(20);

    tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "Base";

    while (ros::ok())
    {
        // get joint state in radian
        const auto jointState = readPosition(device);

        // update joint_state
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(6);
        joint_state.position.resize(6);
        joint_state.name[0] = "left_gripper_joint_0";
        joint_state.position[0] = jointState.data[0];
        joint_state.name[1] = "wrist_joint_1";
        joint_state.position[1] = jointState.data[1];
        joint_state.name[2] = "wrist_joint_2";
        joint_state.position[2] = jointState.data[2];
        joint_state.name[3] = "elbow_joint_3";
        joint_state.position[3] = jointState.data[3];
        joint_state.name[4] = "shoulder_joint_4";
        joint_state.position[4] = jointState.data[4];
        joint_state.name[5] = "base_joint_5";
        joint_state.position[5] = jointState.data[5];

        //update transform
        odom_trans.header.stamp = ros::Time::now();

        //send the joint state and transform
        pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
