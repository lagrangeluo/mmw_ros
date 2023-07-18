#ifndef MMW_MESSENGER_HPP
#define MMW_MESSENGER_HPP

#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// #include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "mmw_sdk/platforms/mmw_base.hpp"

namespace wescore
{
class MmwROSMessenger
{
public:
    explicit MmwROSMessenger(ros::NodeHandle *nh);
    MmwROSMessenger(MmwBase *mmw, ros::NodeHandle *nh);

    std::string base_frame_;

    int count_send = 0;

    void SetupSubscription();

    void PublishStateToROS();

private:
    MmwBase *mmw_;
    ros::NodeHandle *nh_;

    ros::Publisher mmw_status_publisher_;

    ros::Time last_time_;
    ros::Time current_time_;
};
} // namespace wescore

#endif /* MMW_MESSENGER_HPP */
