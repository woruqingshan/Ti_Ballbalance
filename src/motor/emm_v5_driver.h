#ifndef EMM_V5_DRIVER_H
#define EMM_V5_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int32_t raw_units;
    uint32_t received_timestamp_ms;
} EmmV5Position;

typedef struct {
    uint32_t sent;
    uint32_t rejected;
    uint32_t position_requests;
    uint32_t position_responses;
    uint32_t position_timeouts;
    uint32_t position_bad_frames;
} EmmV5Stats;

void emm_v5_init(void);
void emm_v5_poll(uint32_t now_ms);
bool emm_v5_enable(bool enable);

/*
 * Relative position command. Positive pulses use direction byte 0x00 (CW),
 * negative pulses use direction byte 0x01 (CCW). The command's mode byte is
 * explicitly 0x00, which is relative-position mode in the Emm V5.0 protocol.
 */
bool emm_v5_move_relative(int32_t signed_pulses,
                          uint16_t speed_rpm,
                          uint8_t acceleration);

/* Set current motor angle / position error / pulse count to zero. */
bool emm_v5_clear_current_position(void);

bool emm_v5_request_current_position(uint32_t now_ms);
bool emm_v5_take_current_position(EmmV5Position *position_out);
bool emm_v5_position_request_pending(void);
const EmmV5Stats *emm_v5_stats(void);

#endif
