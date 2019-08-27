#define UNICODE

#include "std_msgs/String.h"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include "xarm_api.h"

const std::string left_gripper_joint = "left_gripper_joint_0";
const std::string wrist_joint_1 = "wrist_joint_1";
const std::string wrist_joint_2 = "wrist_joint_2";
const std::string elbow_joint = "elbow_joint_3";
const std::string shoulder_joint = "shoulder_joint_4";
const std::string base_joint = "base_joint_5";

void xArmJointState_Callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    const auto& names = msg->name;
    const auto& positions = msg->position;
    if (names.size() != positions.size())
    {
        ROS_ERROR("names [size: %d] and positions [size: %d] don't match!", names.size(), positions.size());
        return;
    }

    ROS_INFO("received joint states:");
    Joints pos;
    for (auto i = 0; i < names.size(); i++)
    {
        const auto& name = names[i];
        const auto& position = positions[i];
        if (name == left_gripper_joint)
        {
            pos.data[0] = position;
        }
        else if (name == wrist_joint_1)
        {
            pos.data[1] = position;
        }
        else if (name == wrist_joint_2)
        {
            pos.data[2] = position;
        }
        else if (name == elbow_joint)
        {
            pos.data[3] = position;
        }
        else if (name == shoulder_joint)
        {
            pos.data[4] = position;
        }
        else if (name == base_joint)
        {
            pos.data[5] = position;
        }
    }

    MoveToPosition(device, pos);
}

int main(int argc, char **argv)
{
    // initialize
    device = OpenHIDByVidPid(L"0483", L"5750");
    // setup (center)
    Joints pos;
    pos.data = convertToRadian({ 500, 500, 500, 500, 500, 500});

    MoveToPosition(device, pos);

    ros::init(argc, argv, "xarm");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);
    ros::Subscriber sub = nh.subscribe("/joint_states_goal", 1, xArmJointState_Callback);
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
        joint_state.name[0] = left_gripper_joint;
        joint_state.position[0] = jointState.data[0];
        joint_state.name[1] = wrist_joint_1;
        joint_state.position[1] = jointState.data[1];
        joint_state.name[2] = wrist_joint_2;
        joint_state.position[2] = jointState.data[2];
        joint_state.name[3] = elbow_joint;
        joint_state.position[3] = jointState.data[3];
        joint_state.name[4] = shoulder_joint;
        joint_state.position[4] = jointState.data[4];
        joint_state.name[5] = base_joint;
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
