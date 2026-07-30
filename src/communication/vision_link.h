#ifndef VISION_LINK_H
#define VISION_LINK_H
#include <stdbool.h>
#include <stdint.h>
#include "control/ball_state.h"
#include "protocol/protocol.h"
typedef struct {
    ProtocolParser parser;
    BallState latest;
    uint32_t last_sequence;
    uint32_t packets_ok;
    uint32_t sequence_gaps;
    bool has_state;
    bool has_command;
    uint8_t command;
    uint8_t command_argument;
    uint32_t command_token;
} VisionLink;
void vision_link_init(VisionLink *v);
void vision_link_poll(VisionLink *v, uint32_t now_ms);
bool vision_link_get_latest(const VisionLink *v, BallState *out);
bool vision_link_take_command(VisionLink *v, uint8_t *command, uint8_t *argument, uint32_t *token);
void vision_link_send_frame(uint8_t type, const uint8_t *payload, uint16_t len, uint32_t now_ms);
#endif
