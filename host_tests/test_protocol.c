#include <assert.h>
#include <stdio.h>
#include "protocol/protocol.h"
#include "control/ball_controller.h"
#include "motor/emm_v5_protocol.h"

static void test_main_protocol(void)
{
    uint8_t payload[3] = {1, 2, 3};
    uint8_t buffer[128];
    ProtocolParser parser;
    ProtocolFrame frame;
    size_t length;
    size_t i;

    protocol_parser_init(&parser);
    length = protocol_encode(MSG_HEARTBEAT,
                             7U,
                             99U,
                             payload,
                             3U,
                             buffer,
                             sizeof(buffer));
    assert(length == 19U);
    for (i = 0U; i < length; ++i) {
        if (protocol_parser_push(&parser, buffer[i], &frame)) {
            assert(frame.type == MSG_HEARTBEAT);
            assert(frame.sequence == 7U);
            assert(frame.timestamp_ms == 99U);
            assert(frame.payload_len == 3U);
        }
    }
}

static void test_ball_controller(void)
{
    BallController controller;
    BallState ball = {0};
    ball_controller_init(&controller, 0.5f, 0.1f, 4.0f, 0.5f);
    ball.position_cm = 0.0f;
    ball.velocity_cm_s = 0.0f;
    assert(ball_controller_update(&controller, 5.0f, &ball) > 0.0f);
}

static void test_emm_position_parser(void)
{
    const uint8_t positive[] = {
        0x01U, 0x36U, 0x00U, 0x00U, 0x01U, 0x00U, 0x00U, 0x6BU
    };
    const uint8_t negative[] = {
        0x01U, 0x36U, 0x01U, 0x00U, 0x00U, 0x01U, 0x00U, 0x6BU
    };
    uint8_t command[3];
    EmmV5PositionParser parser;
    int32_t position = 0;
    size_t i;

    assert(emm_v5_build_read_current_position(0x01U,
                                               command,
                                               sizeof(command)) == 3U);
    assert(command[0] == 0x01U);
    assert(command[1] == 0x36U);
    assert(command[2] == 0x6BU);

    emm_v5_position_parser_init(&parser);
    for (i = 0U; i < sizeof(positive); ++i) {
        if (emm_v5_position_parser_push(&parser,
                                        0x01U,
                                        positive[i],
                                        &position)) {
            assert(i == sizeof(positive) - 1U);
        }
    }
    assert(position == 65536);

    for (i = 0U; i < sizeof(negative); ++i) {
        (void) emm_v5_position_parser_push(&parser,
                                           0x01U,
                                           negative[i],
                                           &position);
    }
    assert(position == -256);
}

int main(void)
{
    test_main_protocol();
    test_ball_controller();
    test_emm_position_parser();
    puts("host tests passed");
    return 0;
}
