#ifndef BBC_PROTOCOL_H
#define BBC_PROTOCOL_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define PROTOCOL_SOF0 0xA5U
#define PROTOCOL_SOF1 0x5AU
#define PROTOCOL_VERSION 0x01U
#define PROTOCOL_MAX_PAYLOAD 64U
#define PROTOCOL_MAX_FRAME (16U + PROTOCOL_MAX_PAYLOAD)

typedef enum {
    MSG_BALL_STATE = 0x01,
    MSG_PI_STATUS = 0x02,
    MSG_CONTROL_COMMAND = 0x10,
    MSG_TASK_EVENT = 0x81,
    MSG_CONTROL_TELEMETRY = 0x82,
    MSG_HEARTBEAT = 0x83,
    MSG_COMMAND_ACK = 0x84,
    MSG_MOTOR_STATUS = 0x85,
    MSG_LINK_STATS = 0x86,
    MSG_BALL_PREVIEW = 0x87
} ProtocolMessageType;

typedef struct {
    uint8_t type;
    uint16_t payload_len;
    uint32_t sequence;
    uint32_t timestamp_ms;
    uint8_t payload[PROTOCOL_MAX_PAYLOAD];
} ProtocolFrame;

typedef struct {
    uint8_t buffer[PROTOCOL_MAX_FRAME];
    uint16_t index;
    uint16_t expected;
    uint32_t crc_errors;
    uint32_t length_errors;
    uint32_t version_errors;
} ProtocolParser;

void protocol_parser_init(ProtocolParser *p);
bool protocol_parser_push(ProtocolParser *p, uint8_t byte, ProtocolFrame *out);
size_t protocol_encode(uint8_t type, uint32_t sequence, uint32_t timestamp_ms,
                       const uint8_t *payload, uint16_t payload_len,
                       uint8_t *out, size_t out_capacity);
uint16_t protocol_get_u16_le(const uint8_t *p);
uint32_t protocol_get_u32_le(const uint8_t *p);
int16_t protocol_get_i16_le(const uint8_t *p);
void protocol_put_u16_le(uint8_t *p, uint16_t v);
void protocol_put_i16_le(uint8_t *p, int16_t v);
void protocol_put_u32_le(uint8_t *p, uint32_t v);
#endif
