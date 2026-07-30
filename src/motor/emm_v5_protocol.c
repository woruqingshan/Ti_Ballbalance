#include "emm_v5_protocol.h"

void emm_v5_position_parser_init(EmmV5PositionParser *parser)
{
    parser->index = 0U;
    parser->bad_frames = 0U;
}

size_t emm_v5_build_read_current_position(uint8_t address,
                                          uint8_t *out,
                                          size_t out_capacity)
{
    if ((out == NULL) || (out_capacity < 3U)) {
        return 0U;
    }

    out[0] = address;
    out[1] = EMM_V5_CMD_READ_CURRENT_POSITION;
    out[2] = EMM_V5_CHECK_BYTE;
    return 3U;
}

static void resync(EmmV5PositionParser *parser, uint8_t address, uint8_t byte)
{
    parser->index = (byte == address) ? 1U : 0U;
    if (parser->index == 1U) {
        parser->frame[0] = byte;
    }
}

bool emm_v5_position_parser_push(EmmV5PositionParser *parser,
                                 uint8_t address,
                                 uint8_t byte,
                                 int32_t *position_units_out)
{
    uint32_t magnitude;

    if ((parser == NULL) || (position_units_out == NULL)) {
        return false;
    }

    if (parser->index == 0U) {
        if (byte == address) {
            parser->frame[0] = byte;
            parser->index = 1U;
        }
        return false;
    }

    if (parser->index == 1U) {
        if (byte != EMM_V5_CMD_READ_CURRENT_POSITION) {
            resync(parser, address, byte);
            return false;
        }
        parser->frame[1] = byte;
        parser->index = 2U;
        return false;
    }

    parser->frame[parser->index++] = byte;
    if (parser->index < EMM_V5_POSITION_RESPONSE_SIZE) {
        return false;
    }

    parser->index = 0U;
    if ((parser->frame[0] != address) ||
        (parser->frame[1] != EMM_V5_CMD_READ_CURRENT_POSITION) ||
        (parser->frame[2] > 1U) ||
        (parser->frame[7] != EMM_V5_CHECK_BYTE)) {
        parser->bad_frames++;
        resync(parser, address, byte);
        return false;
    }

    magnitude = ((uint32_t) parser->frame[3] << 24) |
                ((uint32_t) parser->frame[4] << 16) |
                ((uint32_t) parser->frame[5] << 8) |
                ((uint32_t) parser->frame[6]);

    if (magnitude > 0x7FFFFFFFU) {
        parser->bad_frames++;
        return false;
    }

    *position_units_out = (parser->frame[2] == 1U)
        ? -(int32_t) magnitude
        : (int32_t) magnitude;
    return true;
}
