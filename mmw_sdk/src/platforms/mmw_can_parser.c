#include "mmw_sdk/platforms/mmw_can_parser.h"

#include "string.h"

bool DecodeMmwMsgFromCAN(const struct can_frame *rx_frame, MmwMessage *msg)
{
    msg->type = MmwMsgNone;
    switch (rx_frame->can_id)
    {
    case CAN_MSG_RADAR0_STATUS_ID:
    case CAN_MSG_RADAR1_STATUS_ID:
    case CAN_MSG_RADAR2_STATUS_ID:
    case CAN_MSG_RADAR3_STATUS_ID:
    case CAN_MSG_RADAR4_STATUS_ID:
    case CAN_MSG_RADAR5_STATUS_ID:
    case CAN_MSG_RADAR6_STATUS_ID:
    case CAN_MSG_RADAR7_STATUS_ID:
    {
        msg->type = RadarStatusMsg;
        msg->body.radar_status_msg.radar_id = (rx_frame->can_id - CAN_MSG_RADAR0_STATUS_ID) >> 4;
        memcpy(msg->body.radar_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_TARGET0_STATUS_ID:
    case CAN_MSG_TARGET1_STATUS_ID:
    case CAN_MSG_TARGET2_STATUS_ID:
    case CAN_MSG_TARGET3_STATUS_ID:
    case CAN_MSG_TARGET4_STATUS_ID:
    case CAN_MSG_TARGET5_STATUS_ID:
    case CAN_MSG_TARGET6_STATUS_ID:
    case CAN_MSG_TARGET7_STATUS_ID:
    {
        msg->type = TargetStatusMsg;
        msg->body.target_status_msg.radar_id = (rx_frame->can_id - CAN_MSG_TARGET0_STATUS_ID) >> 4;
        memcpy(msg->body.target_status_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
    case CAN_MSG_TARGET0_INFO_ID:
    case CAN_MSG_TARGET1_INFO_ID:
    case CAN_MSG_TARGET2_INFO_ID:
    case CAN_MSG_TARGET3_INFO_ID:
    case CAN_MSG_TARGET4_INFO_ID:
    case CAN_MSG_TARGET5_INFO_ID:
    case CAN_MSG_TARGET6_INFO_ID:
    case CAN_MSG_TARGET7_INFO_ID:
    {
        msg->type = TargetInfoMsg;
        msg->body.target_info_msg.radar_id = (rx_frame->can_id - CAN_MSG_TARGET0_INFO_ID) >> 4;
        memcpy(msg->body.target_info_msg.data.raw, rx_frame->data, rx_frame->can_dlc * sizeof(uint8_t));
        break;
    }
  }
    return true;
}
