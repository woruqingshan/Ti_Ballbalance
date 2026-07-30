#include "ti_msp_dl_config.h"
#include "app_config.h"
#include "bsp/bsp_time.h"
#include "communication/vision_uart.h"
#include "communication/vision_link.h"
#include "control/ball_controller.h"
#include "motor/emm_v5_driver.h"
#include "motor/motor_position.h"
#include "motor/motor_scheduler.h"
#include "motor/rod_motor_mapper.h"
#include "safety/safety.h"
#include "safety/faults.h"
#include "task/task3.h"
#include "telemetry/telemetry.h"
#include "test_modes/emm_position_calibration_test.h"
#include "test_modes/motor_sign_test.h"
#include "test_modes/motor_startup_horizontal_test.h"
#if APP_MODE == APP_MODE_ROD_MOTOR_MANUAL_CONTROL
#include "test_modes/motor_manual_control_test.h"
#elif APP_MODE == APP_MODE_MOTOR_COMMAND_SEMANTICS_TEST
#include "test_modes/motor_command_semantics_test.h"
#elif APP_MODE == APP_MODE_PI_TI_MANUAL_LINK
#include "test_modes/pi_ti_manual_link_test.h"
#elif APP_MODE == APP_MODE_BALL_STATE_SIMULATION || \
      APP_MODE == APP_MODE_REAL_VISION_OBSERVER
#include "test_modes/ball_state_link_test.h"
#elif APP_MODE == APP_MODE_PI_TI_UART0_DIAG
#include "test_modes/pi_ti_uart0_diag_test.h"
#elif APP_MODE == APP_MODE_BALL_OBSERVATION_RX_DIAG
#include "test_modes/ball_observation_rx_diag.h"
#elif APP_MODE == APP_MODE_PI_MOTOR_TARGET_STREAM
#include "app/pi_motor_target_stream_app.h"
#elif APP_MODE == APP_MODE_PI_TI_PROTOCOL_TX_DIAG || \
      APP_MODE == APP_MODE_PI_TI_PROTOCOL_RX_POLL_DIAG || \
      APP_MODE == APP_MODE_PI_TI_PROTOCOL_RX_IRQ_DIAG
#include "test_modes/pi_ti_protocol_diag_test.h"
#endif
#include "vehicle/vehicle_control_port.h"

static VisionLink g_vision;
static BallController g_controller;
static MotorPosition g_motor;
static MotorScheduler g_motor_scheduler;
static SafetyState g_safety;
static Task3 g_task3;
static BallState g_ball;
static bool g_has_ball;

static void process_control_commands(void)
{
    uint8_t command, argument;
    uint32_t token;
    (void) argument;
    (void) token;

    while (vision_link_take_command(&g_vision,
                                    &command,
                                    &argument,
                                    &token)) {
        switch (command) {
        case 1U:
            task3_arm(&g_task3);
            safety_clear(&g_safety);
            break;
        case 2U:
            task3_request_start(&g_task3);
            break;
        case 3U:
            task3_abort(&g_task3, 0x80000000U);
            motor_scheduler_set_enabled(&g_motor_scheduler, false);
            break;
        case 4U:
            if (!task3_running(&g_task3)) {
                motor_position_set_zero(&g_motor);
            }
            break;
        case 5U:
            motor_scheduler_set_enabled(&g_motor_scheduler, true);
            break;
        case 6U:
            motor_scheduler_set_enabled(&g_motor_scheduler, false);
            break;
        default:
            break;
        }
    }
}

static void send_task_event(Task3State state,
                            uint32_t now,
                            uint32_t reason)
{
    uint8_t payload[12] = {0};
    payload[0] = 3U;
    payload[1] = (uint8_t) state;
    protocol_put_u32_le(&payload[4], reason);
    protocol_put_u32_le(&payload[8], now);
    vision_link_send_frame(MSG_TASK_EVENT,
                           payload,
                           sizeof(payload),
                           now);
}

static void init_app(void)
{
    SYSCFG_DL_init();
    bsp_time_init();
    vision_uart_init();
    vision_link_init(&g_vision);
    emm_v5_init();
    vehicle_init();
    ball_controller_init(&g_controller,
                         APP_POSITION_KP,
                         APP_VELOCITY_KD,
                         APP_CONTROL_MAX,
                         APP_CONTROL_SLEW_PER_UPDATE);
    motor_position_init(&g_motor,
                        APP_MOTOR_SOFT_MIN_PULSE,
                        APP_MOTOR_SOFT_MAX_PULSE);
    motor_position_set_zero(&g_motor);
    safety_init(&g_safety);
    task3_init(&g_task3);
    motor_scheduler_init(
        &g_motor_scheduler,
        (APP_MODE != APP_MODE_TASK3_LIVE) &&
        (APP_MODE != APP_MODE_EMM_STEP_SWEEP) &&
        (APP_MODE != APP_MODE_EMM_ENABLE_TEST));

#if APP_MODE == APP_MODE_TASK3_LIVE
#if APP_EMM_COMMAND_SEMANTICS_VERIFIED == 0
#error APP_MODE_TASK3_LIVE requires APP_EMM_COMMAND_SEMANTICS_VERIFIED=1
#endif
#endif
}

/*
 * T1 intentionally initializes only the board clock, millisecond time base and
 * verified UART0 IRQ/ring receive path. It does not initialize VisionLink,
 * UART1/Emm, RodMotorControl, vehicle logic, telemetry or any motor output.
 */
#if APP_MODE == APP_MODE_BALL_OBSERVATION_RX_DIAG
static void init_ball_observation_rx_diag(void)
{
    SYSCFG_DL_init();
    bsp_time_init();
    vision_uart_init();
}
#endif

#if APP_MODE == APP_MODE_EMM_POSITION_CALIBRATION
static void run_position_calibration_mode(void)
{
    EmmPositionCalibrationTest test;
    emm_position_calibration_test_init(&test, bsp_time_ms());
    for (;;) {
        uint32_t now = bsp_time_ms();
        emm_v5_poll(now);
        emm_position_calibration_test_update(&test, now);
        __WFI();
    }
}
#endif

#if APP_MODE == APP_MODE_MOTOR_SIGN_TEST
static void run_motor_sign_test_mode(void)
{
    MotorSignTest test;
    motor_sign_test_init(&test, bsp_time_ms());
    for (;;) {
        uint32_t now = bsp_time_ms();
        emm_v5_poll(now);
        motor_sign_test_update(&test, now);
        __WFI();
    }
}
#endif

int main(void)
{
#if APP_MODE == APP_MODE_BALL_OBSERVATION_RX_DIAG
    init_ball_observation_rx_diag();
#else
    init_app();
#endif

#if APP_MODE == APP_MODE_EMM_POSITION_CALIBRATION
    run_position_calibration_mode();
#elif APP_MODE == APP_MODE_MOTOR_SIGN_TEST
    run_motor_sign_test_mode();
#elif APP_MODE == APP_MODE_MOTOR_STARTUP_HORIZONTAL
    motor_startup_horizontal_test_run();
#elif APP_MODE == APP_MODE_ROD_MOTOR_MANUAL_CONTROL
    motor_manual_control_test_run();
#elif APP_MODE == APP_MODE_MOTOR_COMMAND_SEMANTICS_TEST
    motor_command_semantics_test_run();
#elif APP_MODE == APP_MODE_PI_TI_MANUAL_LINK
    pi_ti_manual_link_test_run();
#elif APP_MODE == APP_MODE_BALL_STATE_SIMULATION
    ball_state_link_test_run(true);
#elif APP_MODE == APP_MODE_REAL_VISION_OBSERVER
    ball_state_link_test_run(false);
#elif APP_MODE == APP_MODE_PI_TI_UART0_DIAG
    pi_ti_uart0_diag_test_run();
#elif APP_MODE == APP_MODE_BALL_OBSERVATION_RX_DIAG
    ball_observation_rx_diag_run();
#elif APP_MODE == APP_MODE_PI_MOTOR_TARGET_STREAM
    pi_motor_target_stream_app_run();
#elif APP_MODE == APP_MODE_PI_TI_PROTOCOL_TX_DIAG
    pi_ti_protocol_tx_diag_run();
#elif APP_MODE == APP_MODE_PI_TI_PROTOCOL_RX_POLL_DIAG
    pi_ti_protocol_rx_poll_diag_run();
#elif APP_MODE == APP_MODE_PI_TI_PROTOCOL_RX_IRQ_DIAG
    pi_ti_protocol_rx_irq_diag_run();
#else
    uint32_t last_control = 0U;
    uint32_t last_task = 0U;
    uint32_t last_safety = 0U;
    uint32_t last_motor = 0U;
    uint32_t last_telemetry = 0U;
    uint32_t last_heartbeat = 0U;
#if APP_MODE == APP_MODE_EMM_ENABLE_TEST || APP_MODE == APP_MODE_EMM_STEP_SWEEP
    uint32_t last_test = 0U;
#endif
#if APP_MODE == APP_MODE_EMM_STEP_SWEEP
    bool sweep_direction = false;
#endif
    Task3State previous_state = g_task3.state;

#if APP_MODE == APP_MODE_EMM_ENABLE_TEST
    motor_scheduler_set_enabled(&g_motor_scheduler, true);
#elif APP_MODE == APP_MODE_EMM_STEP_SWEEP
    motor_scheduler_set_enabled(&g_motor_scheduler, true);
#elif APP_MODE == APP_MODE_TASK3_DRY_RUN || \
      APP_MODE == APP_MODE_TASK3_LIVE || \
      APP_MODE == APP_MODE_SIMULATED_BALL_CONTROL
    motor_scheduler_set_enabled(&g_motor_scheduler, true);
    task3_arm(&g_task3);
    task3_request_start(&g_task3);
#endif

    for (;;) {
        uint32_t now = bsp_time_ms();
        emm_v5_poll(now);
        vision_link_poll(&g_vision, now);
        g_has_ball = vision_link_get_latest(&g_vision, &g_ball);
        process_control_commands();

#if APP_MODE == APP_MODE_EMM_ENABLE_TEST
        if ((uint32_t) (now - last_test) >= 2000U) {
            last_test = now;
            (void) emm_v5_enable(((now / 2000U) & 1U) != 0U);
        }
#elif APP_MODE == APP_MODE_EMM_STEP_SWEEP
        if ((uint32_t) (now - last_test) >= 2000U) {
            last_test = now;
            sweep_direction = !sweep_direction;
            (void) emm_v5_move_relative(sweep_direction ? 20 : -20,
                                        120U,
                                        40U);
        }
#elif APP_MODE == APP_MODE_SIMULATED_BALL_CONTROL
        if (!g_has_ball) {
            g_has_ball = true;
            g_ball.valid = true;
            g_ball.position_cm = 0.0f;
            g_ball.velocity_cm_s = 0.0f;
            g_ball.received_timestamp_ms = now;
        }
#endif

        if ((uint32_t) (now - last_safety) >= APP_SAFETY_PERIOD_MS) {
            last_safety = now;
            safety_update(&g_safety,
                          &g_ball,
                          g_has_ball,
                          &g_motor,
                          now);
            if (!g_safety.control_allowed && task3_running(&g_task3)) {
                task3_abort(&g_task3, g_safety.faults);
            }
        }

        if ((uint32_t) (now - last_task) >= APP_TASK_PERIOD_MS) {
            last_task = now;
            task3_update(&g_task3,
                         &g_ball,
                         g_has_ball,
                         vehicle_is_stationary(),
                         now);
            if (g_task3.state != previous_state) {
                send_task_event(g_task3.state,
                                now,
                                g_task3.result_reason);
                previous_state = g_task3.state;
            }
        }

        if ((uint32_t) (now - last_control) >= APP_CONTROL_PERIOD_MS) {
            last_control = now;
            if (task3_running(&g_task3) && g_safety.control_allowed) {
                float control = ball_controller_update(&g_controller,
                                                       g_task3.target_cm,
                                                       &g_ball);
                motor_position_set_target(
                    &g_motor, rod_motor_map_control(control));
            } else {
                ball_controller_reset(&g_controller);
                motor_position_set_target(&g_motor, 0);
            }
        }

        if ((uint32_t) (now - last_motor) >=
            APP_MOTOR_COMMAND_PERIOD_MS) {
            last_motor = now;
            motor_scheduler_update(&g_motor_scheduler, &g_motor, now);
        }

        if ((uint32_t) (now - last_telemetry) >=
            APP_TELEMETRY_PERIOD_MS) {
            last_telemetry = now;
            telemetry_send(&g_task3,
                           &g_ball,
                           g_has_ball,
                           &g_controller,
                           &g_motor,
                           &g_motor_scheduler,
                           &g_safety,
                           now);
        }

        if ((uint32_t) (now - last_heartbeat) >=
            APP_HEARTBEAT_PERIOD_MS) {
            last_heartbeat = now;
            telemetry_send_heartbeat(now, APP_MODE);
        }
        __WFI();
    }
#endif

    return 0;
}
