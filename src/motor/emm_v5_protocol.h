#ifndef EMM_V5_PROTOCOL_H
#define EMM_V5_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define EMM_V5_DEFAULT_ADDRESS                 (0x01U)
#define EMM_V5_CHECK_BYTE                      (0x6BU)
#define EMM_V5_CMD_READ_CURRENT_POSITION       (0x36U)
#define EMM_V5_POSITION_RESPONSE_SIZE          (8U)

typedef struct {
    uint8_t frame[EMM_V5_POSITION_RESPONSE_SIZE];
    uint8_t index;
    uint32_t bad_frames;
} EmmV5PositionParser;

void emm_v5_position_parser_init(EmmV5PositionParser *parser);
size_t emm_v5_build_read_current_position(uint8_t address,
                                          uint8_t *out,
                                          size_t out_capacity);
bool emm_v5_position_parser_push(EmmV5PositionParser *parser,
                                 uint8_t address,
                                 uint8_t byte,
                                 int32_t *position_units_out);

#endif
