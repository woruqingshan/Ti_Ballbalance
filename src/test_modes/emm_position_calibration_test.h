#ifndef EMM_POSITION_CALIBRATION_TEST_H
#define EMM_POSITION_CALIBRATION_TEST_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool have_position;
    bool zero_valid;
    int32_t current_raw_units;
    int32_t zero_raw_units;
    int32_t min_relative_units;
    int32_t max_relative_units;
    uint32_t last_query_ms;
    uint32_t last_reported_timeout_count;
} EmmPositionCalibrationTest;

void emm_position_calibration_test_init(EmmPositionCalibrationTest *test,
                                        uint32_t now_ms);
void emm_position_calibration_test_update(EmmPositionCalibrationTest *test,
                                          uint32_t now_ms);

#endif
