#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "../src/protocol/pi_ti_messages.h"

int main(void)
{
    uint8_t ack[PI_TI_ACK_PAYLOAD_SIZE] = {0};
    uint8_t status[PI_TI_MOTOR_STATUS_SIZE] = {0};
    uint8_t preview[PI_TI_BALL_PREVIEW_SIZE] = {0};

    pi_ti_pack_ack(ack, PI_CMD_SET_OFFSET_MDEG, PI_ACK_OK, -27600, 0x12345678U);
    assert(ack[0] == PI_CMD_SET_OFFSET_MDEG);
    assert(ack[1] == PI_ACK_OK);
    assert(protocol_get_u32_le(&ack[4]) == 0x12345678U);
    assert(pi_ti_get_i32_le(&ack[8]) == -27600);

    pi_ti_pack_motor_status(status, 11U, 0x15U, 1U, PI_ACK_OK,
                            1000U, 20U, -28100, -28125, -250, -250, 0U);
    assert(status[0] == 11U);
    assert(status[1] == 0x15U);
    assert(pi_ti_get_i32_le(&status[12]) == -28100);
    assert(pi_ti_get_i32_le(&status[24]) == -250);

    pi_ti_pack_ball_preview(preview, 0x07U, 0U, 9U, 30U,
                            125, -250, 100, 80, 250, -27850, 3U);
    assert(protocol_get_u32_le(&preview[4]) == 9U);
    assert(protocol_get_i16_le(&preview[12]) == 125);
    assert(pi_ti_get_i32_le(&preview[24]) == -27850);
    return 0;
}
