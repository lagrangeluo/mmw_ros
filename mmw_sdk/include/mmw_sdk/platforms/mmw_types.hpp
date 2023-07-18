#ifndef MMW_TYPE_HPP
#define MMW_TYPE_HPP

#include <cstdint>
#include <iostream>

namespace wescore
{
struct MmwState
{
    struct RadarState
    {
      uint8_t radar_id = 0;
      uint8_t radar_mode = 0;
      uint8_t radar_roll_count = 0;
      uint8_t radar_output_type = 0;
      uint8_t radar_mount_dir = 0;
    };
    struct TargetState
    {
      uint8_t num_of_cluster = 0;
      uint8_t cluster_status_roll_count = 0;
    };
    struct TargetInfo
    {
      uint8_t cluster_index = 0;
      int8_t cluster_rcSValue = 0;
      double cluster_range = 0;
      int8_t cluster_azimuth = 0;
      double cluster_vrel = 0;
      uint8_t cluster_rollcount = 0;
    };

    int count_get = 0;

    // mmw state
    static constexpr uint8_t radar_num = 8;
    int target_info_id[radar_num];
    RadarState radar_state[radar_num];
    TargetState target_state[radar_num];
    TargetInfo target_info[radar_num][256];

};

} // namespace westonrobot

#endif /* MMW_TYPE_HPP */
