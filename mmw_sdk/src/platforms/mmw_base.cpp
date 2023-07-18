#include "mmw_sdk/platforms/mmw_base.hpp"

#include <string>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <ratio>
#include <thread>

namespace wescore
{
MmwBase::~MmwBase()
{
    if (cmd_thread_.joinable())
        cmd_thread_.join();
}

void MmwBase::Connect(std::string dev_name, int32_t baud_rate)
{
    if (baud_rate == 0)
    {
        ConfigureCANBus(dev_name);
    }
    else
    {
        std::cerr << "ERROR: Failed to connect to can" << std::endl;
    }
}

void MmwBase::Terminate() {
  std::terminate();
}

void MmwBase::ConfigureCANBus(const std::string &can_if_name)
{
    can_if_ = std::make_shared<ASyncCAN>(can_if_name);

    can_if_->set_receive_callback(std::bind(&MmwBase::ParseCANFrame, this, std::placeholders::_1));

    can_connected_ = true;
}

MmwState MmwBase::GetMmwState()
{
    std::lock_guard<std::mutex> guard(mmw_state_mutex_);
    return mmw_state_;
}

void MmwBase::ParseCANFrame(can_frame *rx_frame)
{
    MmwMessage status_msg;
    DecodeMmwMsgFromCAN(rx_frame, &status_msg);
    NewStatusMsgReceivedCallback(status_msg);
}

void MmwBase::NewStatusMsgReceivedCallback(const MmwMessage &msg)
{
    // std::cout << "new status msg received" << std::endl;
    std::lock_guard<std::mutex> guard(mmw_state_mutex_);
    UpdateMmwState(msg, mmw_state_);
}

void MmwBase::UpdateMmwState(const MmwMessage &status_msg, MmwState &state)
{
    switch (status_msg.type)
    {
    case RadarStatusMsg:
    {
        const RadarStatusMessage &msg = status_msg.body.radar_status_msg;

   //     if(msg.radar_id == RADAR0_ID)
     //   {
            state.count_get = (state.count_get + 1) % 3 ;
            for(int i = 0; i < state.radar_num; i++)
            {
                state.target_info_id[i] = 0;
            }
    //    }

        state.radar_state[msg.radar_id].radar_id = (msg.data.status.data0 ) & 0x0F;
        state.radar_state[msg.radar_id].radar_mode = msg.data.status.data0 >> 4;
        state.radar_state[msg.radar_id].radar_roll_count = msg.data.status.data1 & 0x03;
        state.radar_state[msg.radar_id].radar_output_type = msg.data.status.data7 & 0x01;
        state.radar_state[msg.radar_id].radar_mount_dir = (msg.data.status.data7 & 0x02) >> 1;
        break;

    }
    case TargetStatusMsg:
    {
        const TargetStatusMessage &msg = status_msg.body.target_status_msg;

        state.target_state[msg.radar_id].num_of_cluster = msg.data.status.data0;
        state.target_state[msg.radar_id].cluster_status_roll_count = msg.data.status.data1 & 0x03;
        break;
    }
    case TargetInfoMsg:
    {
        const TargetInfoMessage &msg = status_msg.body.target_info_msg;
        int target_info_id = state.target_info_id[msg.radar_id];

        state.target_info[msg.radar_id][target_info_id].cluster_index = msg.data.info.data0;
        state.target_info[msg.radar_id][target_info_id].cluster_rcSValue = msg.data.info.data1 * 0.5 - 50;
        state.target_info[msg.radar_id][target_info_id].cluster_range
            = (static_cast<uint16_t>(msg.data.info.data3) |
               static_cast<uint16_t>(msg.data.info.data2) << 8)
               * 0.01;  //m
        state.target_info[msg.radar_id][target_info_id].cluster_azimuth = msg.data.info.data4 - 90;
        int16_t vrel = (static_cast<uint16_t>(msg.data.info.data6)  |
                        static_cast<uint16_t>(msg.data.info.data5 & 0x07) << 8);
        state.target_info[msg.radar_id][target_info_id].cluster_vrel = (vrel * 0.05 - 35) * 1.0;  // m/s
        state.target_info[msg.radar_id][target_info_id].cluster_rollcount = (msg.data.info.data5 & 0xC0) >> 6;
        state.target_info_id[msg.radar_id]++;
        break;
    }
    default:break;
    }
}
} // namespace wescore
