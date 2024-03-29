#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <array>
#include <functional>

// todo: remove dependency on boost/function.hpp and make handler a member function
#include <boost/function.hpp>

#include "xarm.h"

using namespace xarm;

const std::string left_gripper_joint = "left_gripper_joint_0";
const std::string wrist_joint_1 = "wrist_joint_1";
const std::string wrist_joint_2 = "wrist_joint_2";
const std::string elbow_joint = "elbow_joint_3";
const std::string shoulder_joint = "shoulder_joint_4";
const std::string base_joint = "base_joint_5";

void handleJointStateRequest(Arm& arm, const sensor_msgs::JointState::ConstPtr& msg)
{
    const auto &name = msg->name;
    const auto &position = msg->position;
    // const auto &velocity = msg->velocity;
    // const auto &effort = msg->effort;

    // @todo: sanity check with joint names
    if (name.size() != position.size())
    {
        ROS_ERROR("name [size: %d] and position [size: %d] don't match!", name.size(), position.size());
        return;
    }

    ROS_INFO("received joint states!");
    std::array<double, Arm::numJoints> pos = {0};
    for (auto i = 0; i < name.size(); i++)
    {
        const auto &n = name[i];
        const auto &p = position[i];
        if (n == left_gripper_joint)
        {
            pos[0] = p;
        }
        else if (n == wrist_joint_1)
        {
            pos[1] = p;
        }
        else if (n == wrist_joint_2)
        {
            pos[2] = p;
        }
        else if (n == elbow_joint)
        {
            pos[3] = p;
        }
        else if (n == shoulder_joint)
        {
            pos[4] = p;
        }
        else if (n == base_joint)
        {
            pos[5] = p;
        }
    };
    arm.setJointPositions(pos);
}

int main(int argc, char** argv)
{
    Arm arm;

    ROS_INFO("Arm state: [ready]!");
    arm.resetJointPositions(); 

    ros::init(argc, argv, "xarm");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 1);

    // ros::Subscriber does not take std::function
    boost::function<void(const sensor_msgs::JointState::ConstPtr&)> actuator = std::bind(handleJointStateRequest, std::ref(arm), std::placeholders::_1);
    ros::Subscriber sub = nh.subscribe("/joint_states_goal", 1, actuator);
    ros::Rate loop_rate(20);

    tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "Base";

    while (ros::ok())
    {
        // get joint state in radian
        const auto jointState = arm.getJointPositions();

        // update joint_state
        sensor_msgs::JointState joint_state;
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(6);
        joint_state.position.resize(6);
        joint_state.name[0] = left_gripper_joint;
        joint_state.position[0] = jointState[0];
        joint_state.name[1] = wrist_joint_1;
        joint_state.position[1] = jointState[1];
        joint_state.name[2] = wrist_joint_2;
        joint_state.position[2] = jointState[2];
        joint_state.name[3] = elbow_joint;
        joint_state.position[3] = jointState[3];
        joint_state.name[4] = shoulder_joint;
        joint_state.position[4] = jointState[4];
        joint_state.name[5] = base_joint;
        joint_state.position[5] = jointState[5];

        // update transform
        odom_trans.header.stamp = ros::Time::now();

        // send the joint state and transform
        pub.publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        ros::spinOnce();
        loop_rate.sleep();
    }
}
