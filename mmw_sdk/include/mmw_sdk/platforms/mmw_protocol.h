#ifndef MMW_PROTOCOL_H
#define MMW_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define RADAR0_ID                       ((uint8_t)0x00)
#define RADAR1_ID                       ((uint8_t)0x01)
#define RADAR2_ID                       ((uint8_t)0x02)
#define RADAR3_ID                       ((uint8_t)0x03)
#define RADAR4_ID                       ((uint8_t)0x04)
#define RADAR5_ID                       ((uint8_t)0x05)
#define RADAR6_ID                       ((uint8_t)0x06)
#define RADAR7_ID                       ((uint8_t)0x07)

// CAN Definitions
#define CAN_MSG_RADAR0_STATUS_ID            ((uint32_t)0x60A)
#define CAN_MSG_TARGET0_STATUS_ID           ((uint32_t)0x70B)
#define CAN_MSG_TARGET0_INFO_ID             ((uint32_t)0x70C)

#define CAN_MSG_RADAR1_STATUS_ID            ((uint32_t)0x61A)
#define CAN_MSG_TARGET1_STATUS_ID           ((uint32_t)0x71B)
#define CAN_MSG_TARGET1_INFO_ID             ((uint32_t)0x71C)

#define CAN_MSG_RADAR2_STATUS_ID            ((uint32_t)0x62A)
#define CAN_MSG_TARGET2_STATUS_ID           ((uint32_t)0x72B)
#define CAN_MSG_TARGET2_INFO_ID             ((uint32_t)0x72C)

#define CAN_MSG_RADAR3_STATUS_ID            ((uint32_t)0x63A)
#define CAN_MSG_TARGET3_STATUS_ID           ((uint32_t)0x73B)
#define CAN_MSG_TARGET3_INFO_ID             ((uint32_t)0x73C)

#define CAN_MSG_RADAR4_STATUS_ID            ((uint32_t)0x64A)
#define CAN_MSG_TARGET4_STATUS_ID           ((uint32_t)0x74B)
#define CAN_MSG_TARGET4_INFO_ID             ((uint32_t)0x74C)

#define CAN_MSG_RADAR5_STATUS_ID            ((uint32_t)0x65A)
#define CAN_MSG_TARGET5_STATUS_ID           ((uint32_t)0x75B)
#define CAN_MSG_TARGET5_INFO_ID             ((uint32_t)0x75C)

#define CAN_MSG_RADAR6_STATUS_ID            ((uint32_t)0x66A)
#define CAN_MSG_TARGET6_STATUS_ID           ((uint32_t)0x76B)
#define CAN_MSG_TARGET6_INFO_ID             ((uint32_t)0x76C)

#define CAN_MSG_RADAR7_STATUS_ID            ((uint32_t)0x67A)
#define CAN_MSG_TARGET7_STATUS_ID           ((uint32_t)0x77B)
#define CAN_MSG_TARGET7_INFO_ID             ((uint32_t)0x77C)

/*-------------------- Feedback Messages -----------------------*/

/* No padding in the struct */
#pragma pack(push, 1)

// Radar Status
typedef struct {
    uint8_t radar_id;
    union
    {
        struct
        {
            uint8_t data0;
            uint8_t data1;
            uint8_t data2;
            uint8_t data3;
            uint8_t data4;
            uint8_t data5;
            uint8_t data6;
            uint8_t data7;
        } status;
        uint8_t raw[8];
    } data;
} RadarStatusMessage;

//Target Status
typedef struct {
    uint8_t radar_id;
    union
    {
        struct
        {
            uint8_t data0;
            uint8_t data1;
            uint8_t data2;
            uint8_t data3;
            uint8_t data4;
            uint8_t data5;
            uint8_t data6;
            uint8_t data7;
        } status;
        uint8_t raw[8];
    } data;
} TargetStatusMessage;

//Target Info
typedef struct {
    uint8_t radar_id;
    union
    {
        struct
        {
            uint8_t data0;
            uint8_t data1;
            uint8_t data2;
            uint8_t data3;
            uint8_t data4;
            uint8_t data5;
            uint8_t data6;
            uint8_t data7;
        } info;
        uint8_t raw[8];
    } data;
} TargetInfoMessage;

// For convenience to access status message
typedef enum
{
    MmwMsgNone = 0x00,
    RadarStatusMsg = 0x05,
    TargetStatusMsg = 0x06,
    TargetInfoMsg = 0x07,
} MmwMsgType;

typedef struct 
{
    MmwMsgType type;
    union {
        RadarStatusMessage radar_status_msg;
        TargetStatusMessage target_status_msg;
        TargetInfoMessage target_info_msg;
    } body;
} MmwMessage;

#pragma pack(pop)

#ifdef __cplusplus
}
#endif

#endif /* MMW_PROTOCOL_H */
