#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

/* ================= Frame Header ================= */

#define TELEMETRY_HEADER_1     0xAA
#define TELEMETRY_HEADER_2     0xAA

#define TELEMETRY_DOWN_HEADER_1  0xAA
#define TELEMETRY_DOWN_HEADER_2  0xAF

#define TELEMETRY_MAX_DATA_SIZE  32

/* ================= Packet Structure ================= */

typedef struct
{
    uint8_t msg_id;
    uint8_t length;
    uint8_t data[TELEMETRY_MAX_DATA_SIZE];
} telemetry_packet_t;

/* ================= Message IDs (Upward) ================= */

typedef enum
{
    TELEMETRY_ID_VERSION        = 0x00,
    TELEMETRY_ID_STATUS         = 0x01,
    TELEMETRY_ID_SENSOR         = 0x02,
    TELEMETRY_ID_RC             = 0x03,
    TELEMETRY_ID_GPS            = 0x04,
    TELEMETRY_ID_POWER          = 0x05,
    TELEMETRY_ID_MOTOR          = 0x06,
    TELEMETRY_ID_SENSOR2        = 0x07,
    TELEMETRY_ID_FLIGHT_MODE    = 0x0A,
    TELEMETRY_ID_SPEED          = 0x0B,

    TELEMETRY_ID_PID1           = 0x10,
    TELEMETRY_ID_PID2           = 0x11,
    TELEMETRY_ID_PID3           = 0x12,
    TELEMETRY_ID_PID4           = 0x13,
    TELEMETRY_ID_PID5           = 0x14,
    TELEMETRY_ID_PID6           = 0x15,

    TELEMETRY_ID_RADIO          = 0x40,
    TELEMETRY_ID_MESSAGE        = 0xEE,
    TELEMETRY_ID_CHECK          = 0xEF,

    TELEMETRY_ID_REMOTER        = 0x50,
    TELEMETRY_ID_PRINTF         = 0x51,

    TELEMETRY_ID_USERDATA_BASE  = 0xF1   /* 0xF1 ~ 0xFA */
} telemetry_msg_id_t;

/* ================= Downward Message IDs ================= */

typedef enum
{
    TELEMETRY_DOWN_COMMAND  = 0x01,
    TELEMETRY_DOWN_ACK      = 0x02,
    TELEMETRY_DOWN_RC       = 0x03,
    TELEMETRY_DOWN_POWER    = 0x05,
    TELEMETRY_DOWN_MODE     = 0x0A,

    TELEMETRY_DOWN_PID1     = 0x10,
    TELEMETRY_DOWN_PID2     = 0x11,
    TELEMETRY_DOWN_PID3     = 0x12,
    TELEMETRY_DOWN_PID4     = 0x13,
    TELEMETRY_DOWN_PID5     = 0x14,
    TELEMETRY_DOWN_PID6     = 0x15,

    TELEMETRY_DOWN_RADIO    = 0x40,
    TELEMETRY_DOWN_REMOTER  = 0x50,
} telemetry_down_id_t;

/* ================= Downward Commands ================= */

#define TELEMETRY_CMD_ACC_CALIB        0x01
#define TELEMETRY_CMD_GYRO_CALIB       0x02
#define TELEMETRY_CMD_MAG_CALIB        0x04
#define TELEMETRY_CMD_BARO_CALIB       0x05

#define TELEMETRY_CMD_FLIGHT_LOCK      0xA0
#define TELEMETRY_CMD_FLIGHT_UNLOCK    0xA1

#define TELEMETRY_ACK_READ_PID         0x01
#define TELEMETRY_ACK_RESET_PARAM      0xA1

/* ================= Public Send APIs ================= */

void telemetry_send_status(float roll,
                           float pitch,
                           float yaw,
                           int32_t altitude,
                           uint8_t flight_mode,
                           uint8_t armed);

void telemetry_send_sensor(int16_t ax, int16_t ay, int16_t az,
                           int16_t gx, int16_t gy, int16_t gz,
                           int16_t mx, int16_t my, int16_t mz);

void telemetry_send_rc(uint16_t thrust,
                       uint16_t yaw,
                       uint16_t roll,
                       uint16_t pitch);

void telemetry_send_power(uint16_t voltage,
                          uint16_t current);

void telemetry_send_motor(uint16_t m1,
                          uint16_t m2,
                          uint16_t m3,
                          uint16_t m4);

void telemetry_send_user_data(uint8_t group,
                               int16_t d1,
                               int16_t d2,
                               int16_t d3,
                               int16_t d4,
                               int16_t d5,
                               int16_t d6);

#endif /* __TELEMETRY_H */