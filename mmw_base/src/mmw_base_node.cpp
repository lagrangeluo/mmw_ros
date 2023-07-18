#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include "mmw_sdk/platforms/mmw_base.hpp"
#include "mmw_base/mmw_messenger.hpp"

using namespace wescore;

MmwBase robot;

void DetachRobot(int signal) {
  robot.Terminate();
}

int main(int argc, char **argv)
{
    // setup ROS node
    ros::init(argc, argv, "mmw_odom");
    ros::NodeHandle node(""), private_node("~");
    std::signal(SIGINT, DetachRobot);

    // instantiate a robot object
    MmwBase robot;
    MmwROSMessenger messenger(&robot, &node);

    // fetch parameters before connecting to robot
    std::string port_name;
    private_node.param<std::string>("port_name", port_name, std::string("can0"));

    // connect to robot and setup ROS subscription
    if (port_name.find("can") != std::string::npos)
    {
        robot.Connect(port_name);
        ROS_INFO("Using CAN bus to talk with the robot");
    }
    else
    {
        ROS_ERROR("Only connect with can!");
    }
    messenger.SetupSubscription();

    // publish robot state at 50Hz while listening to twist commands
    ros::Rate rate_50hz(50); // 50Hz
    while (true)
    {
        messenger.PublishStateToROS();
        ros::spinOnce();
        rate_50hz.sleep();
    }

    return 0;
}
