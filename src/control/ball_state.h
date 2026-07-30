#ifndef BALL_STATE_H
#define BALL_STATE_H
#include <stdbool.h>
#include <stdint.h>
typedef struct {
    uint32_t frame_id;
    uint32_t source_timestamp_ms;
    uint32_t received_timestamp_ms;
    float position_cm;
    float velocity_cm_s;
    float confidence;
    uint16_t vision_latency_ms;
    uint8_t invalid_reason;
    bool found;
    bool predicted;
    bool valid;
} BallState;
#endif
