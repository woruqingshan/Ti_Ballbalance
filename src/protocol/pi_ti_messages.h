#ifndef PI_TI_MESSAGES_H
#define PI_TI_MESSAGES_H

#include "protocol/protocol.h"
#include <stdbool.h>
#include <stdint.h>

#define PI_TI_PROTOCOL_PROFILE_VERSION 0x0001U
#define PI_TI_ACK_PAYLOAD_SIZE          12U
#define PI_TI_MOTOR_STATUS_SIZE         32U
#define PI_TI_LINK_STATS_SIZE           32U
#define PI_TI_BALL_PREVIEW_SIZE         32U

typedef enum {
    PI_CMD_HELLO = 1,
    PI_CMD_GET_STATUS = 2,
    PI_CMD_STARTUP_HORIZONTAL = 3,
    PI_CMD_SET_OFFSET_MDEG = 4,
    PI_CMD_RETURN_HORIZONTAL = 5,
    PI_CMD_ENABLE_MOTOR = 6,
    PI_CMD_DISABLE_MOTOR = 7,
    PI_CMD_EMERGENCY_DISABLE = 8,
    PI_CMD_ARM_PREVIEW = 9,
    PI_CMD_DISARM_PREVIEW = 10,
    PI_CMD_SET_BALL_TARGET_CENTI_CM = 11,
    PI_CMD_CLEAR_LINK_STATS = 12
} PiTiCommand;

typedef enum {
    PI_ACK_OK = 0,
    PI_ACK_ACCEPTED = 1,
    PI_ACK_BAD_COMMAND = 2,
    PI_ACK_BAD_STATE = 3,
    PI_ACK_OUT_OF_RANGE = 4,
    PI_ACK_BUSY = 5,
    PI_ACK_DISABLED = 6,
    PI_ACK_UNSUPPORTED = 7,
    PI_ACK_DUPLICATE = 8
} PiTiAckCode;

enum {
    PI_MOTOR_FLAG_ENABLED = 1U << 0,
    PI_MOTOR_FLAG_STARTUP_COMPLETE = 1U << 1,
    PI_MOTOR_FLAG_BUSY = 1U << 2,
    PI_MOTOR_FLAG_SOFT_LIMIT = 1U << 3,
    PI_MOTOR_FLAG_LINK_ALIVE = 1U << 4,
    PI_MOTOR_FLAG_PREVIEW_ARMED = 1U << 5,
    PI_MOTOR_FLAG_OUTPUT_COMPILED = 1U << 6,
    PI_MOTOR_FLAG_OUTPUT_ACTIVE = 1U << 7
};

enum {
    PI_PREVIEW_FLAG_HAS_BALL = 1U << 0,
    PI_PREVIEW_FLAG_CONTROL_VALID = 1U << 1,
    PI_PREVIEW_FLAG_ARMED = 1U << 2,
    PI_PREVIEW_FLAG_OUTPUT_COMPILED = 1U << 3,
    PI_PREVIEW_FLAG_OUTPUT_ACTIVE = 1U << 4,
    PI_PREVIEW_FLAG_PREDICTED = 1U << 5,
    PI_PREVIEW_FLAG_FOUND = 1U << 6
};

static inline void pi_ti_put_i32_le(uint8_t *p, int32_t value)
{
    protocol_put_u32_le(p, (uint32_t) value);
}

static inline int32_t pi_ti_get_i32_le(const uint8_t *p)
{
    return (int32_t) protocol_get_u32_le(p);
}

static inline void pi_ti_pack_ack(uint8_t payload[PI_TI_ACK_PAYLOAD_SIZE],
                                  uint8_t command,
                                  uint8_t result,
                                  int32_t detail,
                                  uint32_t token)
{
    payload[0] = command;
    payload[1] = result;
    protocol_put_u16_le(&payload[2], PI_TI_PROTOCOL_PROFILE_VERSION);
    protocol_put_u32_le(&payload[4], token);
    pi_ti_put_i32_le(&payload[8], detail);
}

static inline void pi_ti_pack_motor_status(
    uint8_t payload[PI_TI_MOTOR_STATUS_SIZE],
    uint8_t mode,
    uint8_t flags,
    uint8_t control_state,
    uint8_t last_ack,
    uint32_t uptime_ms,
    uint32_t last_rx_age_ms,
    int32_t target_mdeg,
    int32_t commanded_mdeg,
    int32_t target_pulse,
    int32_t commanded_pulse,
    uint32_t faults)
{
    payload[0] = mode;
    payload[1] = flags;
    payload[2] = control_state;
    payload[3] = last_ack;
    protocol_put_u32_le(&payload[4], uptime_ms);
    protocol_put_u32_le(&payload[8], last_rx_age_ms);
    pi_ti_put_i32_le(&payload[12], target_mdeg);
    pi_ti_put_i32_le(&payload[16], commanded_mdeg);
    pi_ti_put_i32_le(&payload[20], target_pulse);
    pi_ti_put_i32_le(&payload[24], commanded_pulse);
    protocol_put_u32_le(&payload[28], faults);
}

static inline void pi_ti_pack_link_stats(
    uint8_t payload[PI_TI_LINK_STATS_SIZE],
    uint32_t rx_frames,
    uint32_t rx_sequence_gaps,
    uint32_t ball_packets,
    uint32_t ball_sequence_gaps,
    uint32_t crc_errors,
    uint32_t length_errors,
    uint32_t version_errors,
    uint32_t uart_overflow)
{
    protocol_put_u32_le(&payload[0], rx_frames);
    protocol_put_u32_le(&payload[4], rx_sequence_gaps);
    protocol_put_u32_le(&payload[8], ball_packets);
    protocol_put_u32_le(&payload[12], ball_sequence_gaps);
    protocol_put_u32_le(&payload[16], crc_errors);
    protocol_put_u32_le(&payload[20], length_errors);
    protocol_put_u32_le(&payload[24], version_errors);
    protocol_put_u32_le(&payload[28], uart_overflow);
}

static inline void pi_ti_pack_ball_preview(
    uint8_t payload[PI_TI_BALL_PREVIEW_SIZE],
    uint8_t flags,
    uint8_t invalid_reason,
    uint32_t frame_id,
    uint32_t age_ms,
    int16_t position_centi_cm,
    int16_t velocity_centi_cm_s,
    int16_t raw_control_milli,
    int16_t limited_control_milli,
    int32_t target_offset_mdeg,
    int32_t target_physical_mdeg,
    uint32_t ball_sequence_gaps)
{
    payload[0] = flags;
    payload[1] = invalid_reason;
    protocol_put_u16_le(&payload[2], PI_TI_PROTOCOL_PROFILE_VERSION);
    protocol_put_u32_le(&payload[4], frame_id);
    protocol_put_u32_le(&payload[8], age_ms);
    protocol_put_i16_le(&payload[12], position_centi_cm);
    protocol_put_i16_le(&payload[14], velocity_centi_cm_s);
    protocol_put_i16_le(&payload[16], raw_control_milli);
    protocol_put_i16_le(&payload[18], limited_control_milli);
    pi_ti_put_i32_le(&payload[20], target_offset_mdeg);
    pi_ti_put_i32_le(&payload[24], target_physical_mdeg);
    protocol_put_u32_le(&payload[28], ball_sequence_gaps);
}

#endif
