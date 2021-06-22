
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <array>
#include <functional>

#include "xarm.h"

using namespace xarm;

rclcpp::Node::SharedPtr node;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("lewansoul_node");

    auto arm = std::make_shared<Arm>(node);

    if (!arm->init())
    {
        RCLCPP_ERROR(node->get_logger(),"Could not connect to Robot Arm");

        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Arm state: [ready]!");

    auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);

    rclcpp::Rate loop_rate(20);

    tf2_ros::TransformBroadcaster broadcaster(node);
    geometry_msgs::msg::TransformStamped odom_trans;

    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "Base";

    while (rclcpp::ok())
    {
        // get joint state in radian
        const auto jointPositions = arm->getJointPositions();

        // update joint_state
        sensor_msgs::msg::JointState joint_state;
        joint_state.header.stamp = rclcpp::Time();
        joint_state.name.resize(6);
        joint_state.position.resize(6);
        joint_state.name[0] = left_gripper_joint;
        joint_state.position[0] = jointPositions[0];
        joint_state.name[1] = wrist_joint_1;
        joint_state.position[1] = jointPositions[1];
        joint_state.name[2] = wrist_joint_2;
        joint_state.position[2] = jointPositions[2];
        joint_state.name[3] = elbow_joint;
        joint_state.position[3] = jointPositions[3];
        joint_state.name[4] = shoulder_joint;
        joint_state.position[4] = jointPositions[4];
        joint_state.name[5] = base_joint;
        joint_state.position[5] = jointPositions[5];

        // update transform
        odom_trans.header.stamp = rclcpp::Time();

        // send the joint state and transform
        pub->publish(joint_state);
        broadcaster.sendTransform(odom_trans);

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
}
