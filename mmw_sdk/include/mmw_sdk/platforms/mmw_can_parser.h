#ifndef MMW_CAN_PARSER_H
#define MMW_CAN_PARSER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "mmw_protocol.h"

#ifdef __linux__
#include <linux/can.h>
#else
struct can_frame
{
    uint32_t can_id;
    uint8_t can_dlc;
    uint8_t data[8]__attribute__((aligned(8)));
};
#endif

bool DecodeMmwMsgFromCAN(const struct can_frame *rx_frame, MmwMessage *msg);

#ifdef __cplusplus
}
#endif

#endif /* MMW_CAN_PARSER_H */
