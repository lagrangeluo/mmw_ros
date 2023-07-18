#include "mmw_base/mmw_messenger.hpp"

#include <tf/transform_broadcaster.h>

#include "mmw_msgs/MmwStatus.h"

namespace wescore
{
MmwROSMessenger::MmwROSMessenger(ros::NodeHandle *nh) : mmw_(nullptr), nh_(nh)
{
}

MmwROSMessenger::MmwROSMessenger(MmwBase *mmw, ros::NodeHandle *nh) : mmw_(mmw), nh_(nh)
{
}

void MmwROSMessenger::SetupSubscription()
{
    // status publisher
    mmw_status_publisher_ = nh_->advertise<mmw_msgs::MmwStatus>("/mmw_status", 10);
}

void MmwROSMessenger::PublishStateToROS()
{
    current_time_ = ros::Time::now();

    static bool init_run = true;
    if (init_run)
    {
        last_time_ = current_time_;
        init_run = false;
        return;
    }

    auto state = mmw_->GetMmwState();
    // publish mmw state message

    mmw_msgs::MmwStatus status_msg;
    mmw_msgs::MmwTargetInfo target_info;
    status_msg.header.stamp = current_time_;

    size_t n = status_msg.data.size();
    for(size_t i=0; i<n; i++)
    {
        // radar state
        status_msg.data[i].radar_states.radar_id = state.radar_state[i].radar_id;
        status_msg.data[i].radar_states.radar_mode = state.radar_state[i].radar_mode;
        status_msg.data[i].radar_states.radar_roll_count = state.radar_state[i].radar_roll_count;
        status_msg.data[i].radar_states.radar_output_type = state.radar_state[i].radar_output_type;
        status_msg.data[i].radar_states.radar_mount_dir = state.radar_state[i].radar_mount_dir;

        // target state
        status_msg.data[i].target_states.num_of_cluster = state.target_state[i].num_of_cluster;
        status_msg.data[i].target_states.cluster_status_roll_count = state.target_state[i].cluster_status_roll_count;

        // target info
        for(int j = 0; j < state.target_state[i].num_of_cluster ; j++)
        {
            target_info.cluster_index = state.target_info[i][j].cluster_index;
            target_info.cluster_range = state.target_info[i][j].cluster_range;
            target_info.cluster_azimuth = state.target_info[i][j].cluster_azimuth;
            target_info.cluster_vrel = state.target_info[i][j].cluster_vrel;
            target_info.cluster_rollcount = state.target_info[i][j].cluster_rollcount;

            status_msg.data[i].target_info.push_back(target_info);
        }
    }


    if(state.count_get > count_send || state.count_get+2 == count_send)
    {
       count_send = (count_send + 1) % 3;
       mmw_status_publisher_.publish(status_msg);
    }

    // record time for next integration
    last_time_ = current_time_;
    //mmw_status_publisher_.publish(status_msg);
    ros::Duration(0.1).sleep();



}

} // namespace wescore
