#include <stdint.h>
#include "telemetry.h"

/* External UART send function (must be implemented in uart.c) */
extern void uart_send_char(char c);

/* ======================= Byte Macros ======================= */

#define BYTE_H(x)   ((uint8_t)((x) >> 8))
#define BYTE_L(x)   ((uint8_t)((x) & 0xFF))

#define BYTE0(dw)   ((uint8_t)((dw) & 0xFF))
#define BYTE1(dw)   ((uint8_t)(((dw) >> 8) & 0xFF))
#define BYTE2(dw)   ((uint8_t)(((dw) >> 16) & 0xFF))
#define BYTE3(dw)   ((uint8_t)(((dw) >> 24) & 0xFF))

/* ======================= Checksum ======================= */

static uint8_t telemetry_checksum(uint8_t id, uint8_t len, uint8_t *data)
{
    uint8_t sum = 0;

    sum += TELEMETRY_HEADER_1;
    sum += TELEMETRY_HEADER_2;
    sum += id;
    sum += len;

    for(uint8_t i = 0; i < len; i++)
        sum += data[i];

    return sum;
}

/* ======================= Low-Level Packet Send ======================= */

static void telemetry_send_packet(telemetry_packet_t *pkt)
{
    uint8_t sum = telemetry_checksum(pkt->msg_id, pkt->length, pkt->data);

    uart_send_char(TELEMETRY_HEADER_1);
    uart_send_char(TELEMETRY_HEADER_2);
    uart_send_char(pkt->msg_id);
    uart_send_char(pkt->length);

    for(uint8_t i = 0; i < pkt->length; i++)
        uart_send_char(pkt->data[i]);

    uart_send_char(sum);
}

/* ======================= STATUS ======================= */

void telemetry_send_status(float roll, float pitch, float yaw,
                           int32_t altitude,
                           uint8_t flight_mode,
                           uint8_t armed)
{
    telemetry_packet_t pkt;
    uint8_t cnt = 0;

    int16_t r = (int16_t)(roll  * 100.0f);
    int16_t p = (int16_t)(pitch * 100.0f);
    int16_t y = (int16_t)(yaw   * 100.0f);

    pkt.msg_id = TELEMETRY_ID_STATUS;

    pkt.data[cnt++] = BYTE_H(r);
    pkt.data[cnt++] = BYTE_L(r);

    pkt.data[cnt++] = BYTE_H(p);
    pkt.data[cnt++] = BYTE_L(p);

    pkt.data[cnt++] = BYTE_H(y);
    pkt.data[cnt++] = BYTE_L(y);

    pkt.data[cnt++] = BYTE3(altitude);
    pkt.data[cnt++] = BYTE2(altitude);
    pkt.data[cnt++] = BYTE1(altitude);
    pkt.data[cnt++] = BYTE0(altitude);

    pkt.data[cnt++] = flight_mode;
    pkt.data[cnt++] = armed;

    pkt.length = cnt;

    telemetry_send_packet(&pkt);
}

/* ======================= SENSOR ======================= */

void telemetry_send_sensor(int16_t ax, int16_t ay, int16_t az,
                           int16_t gx, int16_t gy, int16_t gz,
                           int16_t mx, int16_t my, int16_t mz)
{
    telemetry_packet_t pkt;
    uint8_t cnt = 0;

    pkt.msg_id = TELEMETRY_ID_SENSOR;

    int16_t values[9] = {ax, ay, az, gx, gy, gz, mx, my, mz};

    for(uint8_t i = 0; i < 9; i++)
    {
        pkt.data[cnt++] = BYTE_H(values[i]);
        pkt.data[cnt++] = BYTE_L(values[i]);
    }

    pkt.length = cnt;

    telemetry_send_packet(&pkt);
}

/* ======================= RC DATA ======================= */

void telemetry_send_rc(uint16_t thrust,
                       uint16_t yaw,
                       uint16_t roll,
                       uint16_t pitch)
{
    telemetry_packet_t pkt;
    uint8_t cnt = 0;

    pkt.msg_id = TELEMETRY_ID_RC;

    uint16_t values[4] = {thrust, yaw, roll, pitch};

    for(uint8_t i = 0; i < 4; i++)
    {
        pkt.data[cnt++] = BYTE_H(values[i]);
        pkt.data[cnt++] = BYTE_L(values[i]);
    }

    pkt.length = cnt;

    telemetry_send_packet(&pkt);
}

/* ======================= POWER ======================= */

void telemetry_send_power(uint16_t voltage, uint16_t current)
{
    telemetry_packet_t pkt;
    uint8_t cnt = 0;

    pkt.msg_id = TELEMETRY_ID_POWER;

    pkt.data[cnt++] = BYTE_H(voltage);
    pkt.data[cnt++] = BYTE_L(voltage);

    pkt.data[cnt++] = BYTE_H(current);
    pkt.data[cnt++] = BYTE_L(current);

    pkt.length = cnt;

    telemetry_send_packet(&pkt);
}

/* ======================= MOTOR PWM ======================= */

void telemetry_send_motor(uint16_t m1,
                          uint16_t m2,
                          uint16_t m3,
                          uint16_t m4)
{
    telemetry_packet_t pkt;
    uint8_t cnt = 0;

    pkt.msg_id = TELEMETRY_ID_MOTOR;

    uint16_t motors[4] = {m1, m2, m3, m4};

    for(uint8_t i = 0; i < 4; i++)
    {
        pkt.data[cnt++] = BYTE_H(motors[i]);
        pkt.data[cnt++] = BYTE_L(motors[i]);
    }

    pkt.length = cnt;

    telemetry_send_packet(&pkt);
}

/* ======================= USER DATA ======================= */

void telemetry_send_user_data(uint8_t group,
                               int16_t d1, int16_t d2, int16_t d3,
                               int16_t d4, int16_t d5, int16_t d6)
{
    telemetry_packet_t pkt;
    uint8_t cnt = 0;

    pkt.msg_id = TELEMETRY_ID_USERDATA_BASE + group - 1;

    int16_t values[6] = {d1, d2, d3, d4, d5, d6};

    for(uint8_t i = 0; i < 6; i++)
    {
        pkt.data[cnt++] = BYTE_H(values[i]);
        pkt.data[cnt++] = BYTE_L(values[i]);
    }

    pkt.length = cnt;

    telemetry_send_packet(&pkt);
}