#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_MODE_PROTOCOL_LOOPBACK               0
#define APP_MODE_EMM_ENABLE_TEST                 1
#define APP_MODE_EMM_STEP_SWEEP                  2
#define APP_MODE_SIMULATED_BALL_CONTROL          3
#define APP_MODE_TASK3_DRY_RUN                   4
#define APP_MODE_TASK3_LIVE                      5
#define APP_MODE_EMM_POSITION_CALIBRATION        6
#define APP_MODE_MOTOR_SIGN_TEST                 7
#define APP_MODE_MOTOR_STARTUP_HORIZONTAL        8
#define APP_MODE_ROD_MOTOR_MANUAL_CONTROL        9
#define APP_MODE_MOTOR_COMMAND_SEMANTICS_TEST   10
#define APP_MODE_PI_TI_MANUAL_LINK              11
#define APP_MODE_BALL_STATE_SIMULATION          12
#define APP_MODE_REAL_VISION_OBSERVER           13
#define APP_MODE_PI_TI_UART0_DIAG               14
#define APP_MODE_PI_TI_PROTOCOL_TX_DIAG         15
#define APP_MODE_PI_TI_PROTOCOL_RX_POLL_DIAG    16
#define APP_MODE_PI_TI_PROTOCOL_RX_IRQ_DIAG     17
#define APP_MODE_BALL_OBSERVATION_RX_DIAG       18
#define APP_MODE_BALL_CONTROL_DRY_RUN           19
#define APP_MODE_PI_MOTOR_TARGET_STREAM         20
#define APP_MODE_PI_BALL_OBSERVATION_CONTROL    21

#define APP_MODE_PI_MOTOR_TARGET_CH340_DIAG \
    APP_MODE_PI_TI_PROTOCOL_RX_POLL_DIAG

/* Safe default after applying this patch. Select Mode 19 or 21 explicitly. */
#define APP_MODE APP_MODE_BALL_OBSERVATION_RX_DIAG

#define APP_EMM_COMMAND_SEMANTICS_VERIFIED 0

#define APP_CONTROL_PERIOD_MS             20U
#define APP_TASK_PERIOD_MS                10U
#define APP_SAFETY_PERIOD_MS              10U
#define APP_TELEMETRY_PERIOD_MS           50U
#define APP_HEARTBEAT_PERIOD_MS          500U
#define APP_MOTOR_COMMAND_PERIOD_MS       25U
#define APP_VISION_TIMEOUT_MS            120U
#define APP_TASK3_TIMEOUT_MS            4800U

#define APP_MOTOR_SOFT_MIN_PULSE        (-240)
#define APP_MOTOR_SOFT_MAX_PULSE         (240)
#define APP_MOTOR_MAX_DELTA_PULSE         20
#define APP_MOTOR_MIN_DELTA_PULSE          2
#define APP_MOTOR_SPEED_RPM               120U
#define APP_MOTOR_ACCELERATION             40U
#define APP_PULSES_PER_CONTROL_UNIT       20.0f

#define APP_POSITION_KP                    0.35f
#define APP_VELOCITY_KD                    0.035f
#define APP_CONTROL_MAX                    4.0f
#define APP_CONTROL_SLEW_PER_UPDATE        0.35f

#define APP_TASK3_ARRIVAL_TOL_CM            0.7f
#define APP_TASK3_PLUS_MAX_SPEED_CM_S        4.0f
#define APP_TASK3_FINAL_MAX_SPEED_CM_S       2.0f
#define APP_TASK3_PLUS_HOLD_MS             150U
#define APP_TASK3_FINAL_HOLD_MS            300U

#define APP_EMM_POSITION_QUERY_PERIOD_MS    100U
#define APP_EMM_POSITION_UNITS_PER_REVOLUTION (65536LL)
#define APP_SIGN_TEST_DEFAULT_STEP_PULSE     10
#define APP_SIGN_TEST_SOFT_LIMIT_PULSE       30
#define APP_SIGN_TEST_SPEED_RPM              30U
#define APP_SIGN_TEST_ACCELERATION           10U
#define APP_SIGN_TEST_POSITION_SETTLE_MS     400U

#define APP_STARTUP_TEST_BOOT_DELAY_MS       2000U
#define APP_STARTUP_TEST_COMMAND_GAP_MS       250U
#define APP_STARTUP_TEST_TARGET_ANGLE_MDEG  (-28100L)
#define APP_STARTUP_TEST_PULSES_PER_REV       3200L
#define APP_STARTUP_TEST_SPEED_RPM              20U
#define APP_STARTUP_TEST_ACCELERATION            10U

#define APP_ROD_MOTOR_PULSES_PER_REV          3200L
#define APP_ROD_PHYSICAL_MIN_MDEG           (-70000L)
#define APP_ROD_PHYSICAL_MAX_MDEG                0L
#define APP_ROD_HORIZONTAL_MDEG             (-28100L)
#define APP_ROD_CONTROL_MIN_MDEG            (-31100L)
#define APP_ROD_CONTROL_MAX_MDEG            (-25100L)

#define APP_ROD_BOOT_DELAY_MS                  2000U
#define APP_ROD_COMMAND_GAP_MS                  250U
#define APP_ROD_STARTUP_SETTLE_MS              1500U
#define APP_ROD_COMMAND_MARGIN_MS               250U
#define APP_ROD_MANUAL_SPEED_RPM                 20U
#define APP_ROD_MANUAL_ACCELERATION              10U
#define APP_ROD_MANUAL_DEFAULT_STEP_MDEG         500L

#define APP_SEMANTICS_TEST_OFFSET_MDEG          2000L
#define APP_SEMANTICS_TEST_SPEED_RPM              10U
#define APP_SEMANTICS_TEST_ACCELERATION            5U
#define APP_SEMANTICS_DEFAULT_GAP_MS              100U
#define APP_SEMANTICS_RESULT_SETTLE_MS           1500U
#define APP_SEMANTICS_QUERY_WAIT_MS               180U

#define APP_PI_LINK_TIMEOUT_MS                   800U
#define APP_PI_STATUS_PERIOD_MS                  100U
#define APP_PI_HEARTBEAT_PERIOD_MS               500U
#define APP_PI_LINK_STATS_PERIOD_MS             1000U
#define APP_PI_PREVIEW_PERIOD_MS                  50U
#define APP_PI_UART0_DIAG_PERIOD_MS              500U
#define APP_PI_PROTOCOL_DIAG_TIMEOUT_MS          5000U
#define APP_PI_MANUAL_MAX_OFFSET_MDEG            3000L

#define APP_PI_TARGET_FRAME_SIZE                    8U
#define APP_PI_TARGET_RX_BUDGET_BYTES              32U
#define APP_PI_TARGET_MIN_OFFSET_MDEG           (-3000L)
#define APP_PI_TARGET_MAX_OFFSET_MDEG             3000L
#define APP_PI_TARGET_TIMEOUT_MS                    500U
#define APP_PI_TARGET_DIRECTION_SIGN                  1L
#define APP_PI_TARGET_DIAG_ECHO_EACH_BYTE             1U

/* Fixed-14 BallObservation receive, gate and control settings. */
#define APP_BALL_OBS_FRAME_SIZE                      14U
#define APP_BALL_OBS_RX_BUDGET_BYTES                32U
#define APP_BALL_OBS_OBSERVER_PERIOD_MS              10U
#define APP_BALL_OBS_EXPECTED_PERIOD_MS              25U
#define APP_BALL_OBS_VALID_MAX_AGE_MS               120U
#define APP_BALL_OBS_LINK_TIMEOUT_MS                600U
#define APP_BALL_OBS_REACQUIRE_FRAMES                 3U

#define APP_BALL_OBS_POSITION_MIN_CENTI_CM        (-1300)
#define APP_BALL_OBS_POSITION_MAX_CENTI_CM          1300
#define APP_BALL_OBS_MAX_ABS_VELOCITY_CENTI_CM_S   5000U
#define APP_BALL_OBS_MIN_CONFIDENCE_MILLI           350U
#define APP_BALL_OBS_ALLOW_PREDICTED                   0U

#define APP_BALL_CONTROL_TARGET_CENTI_CM               0
#define APP_BALL_CONTROL_KP_MDEG_PER_CM                60L
#define APP_BALL_CONTROL_KD_MDEG_PER_CM_S              15L
#define APP_BALL_CONTROL_MAX_OFFSET_MDEG              200L
#define APP_BALL_CONTROL_MAX_SLEW_MDEG_PER_S          800L
#define APP_BALL_CONTROL_MOTOR_SIGN                      1L

/* Mode 21 live-output settings. Keep the initial range deliberately small. */
#define APP_BALL_LIVE_MOTOR_UPDATE_PERIOD_MS          25U
#define APP_BALL_LIVE_DISABLE_ON_LINK_TIMEOUT           1U
#define APP_BALL_LIVE_AUTO_REENABLE_AFTER_RECOVERY      1U

#define APP_PREVIEW_POSITION_KP                  0.15f
#define APP_PREVIEW_VELOCITY_KD                  0.020f
#define APP_PREVIEW_CONTROL_MAX                  2.0f
#define APP_PREVIEW_CONTROL_SLEW_PER_UPDATE      0.15f
#define APP_PREVIEW_MDEG_PER_CONTROL_UNIT         250L
#define APP_PREVIEW_MAX_OFFSET_MDEG               500L

#define APP_VISION_MIN_CONFIDENCE_MILLI           350U
#define APP_VISION_MAX_AGE_MS                      120U
#define APP_VISION_ALLOW_PREDICTED                   1U

#define APP_PHASE6_MOTOR_OUTPUT_ENABLED              0U
#define APP_PHASE6_MOTOR_UPDATE_PERIOD_MS          100U

#endif
