#ifndef TEST_CONSOLE_H
#define TEST_CONSOLE_H

#include <stdint.h>

void test_console_write(const char *text);
void test_console_write_line(const char *text);
void test_console_write_i32(int32_t value);
void test_console_write_u32(uint32_t value);
void test_console_write_fixed_milli(int64_t milli_value);
void test_console_newline(void);

#endif
