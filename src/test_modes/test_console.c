#include "test_console.h"
#include "communication/vision_uart.h"
#include <stddef.h>

static size_t text_length(const char *text)
{
    size_t length = 0U;
    while ((text != NULL) && (text[length] != '\0')) {
        length++;
    }
    return length;
}

void test_console_write(const char *text)
{
    if (text != NULL) {
        vision_uart_write((const uint8_t *) text, text_length(text));
    }
}

void test_console_newline(void)
{
    static const uint8_t newline[] = {'\r', '\n'};
    vision_uart_write(newline, sizeof(newline));
}

void test_console_write_line(const char *text)
{
    test_console_write(text);
    test_console_newline();
}

void test_console_write_u32(uint32_t value)
{
    uint8_t buffer[10];
    size_t length = 0U;

    do {
        buffer[length++] = (uint8_t) ('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (length < sizeof(buffer)));

    while (length > 0U) {
        length--;
        vision_uart_write(&buffer[length], 1U);
    }
}

void test_console_write_i32(int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        static const uint8_t minus = '-';
        vision_uart_write(&minus, 1U);
        magnitude = (uint32_t) (-(int64_t) value);
    } else {
        magnitude = (uint32_t) value;
    }
    test_console_write_u32(magnitude);
}

void test_console_write_fixed_milli(int64_t milli_value)
{
    uint64_t magnitude;
    uint32_t whole;
    uint32_t fraction;
    uint8_t digit;

    if (milli_value < 0) {
        static const uint8_t minus = '-';
        vision_uart_write(&minus, 1U);
        magnitude = (uint64_t) (-milli_value);
    } else {
        magnitude = (uint64_t) milli_value;
    }

    whole = (uint32_t) (magnitude / 1000U);
    fraction = (uint32_t) (magnitude % 1000U);
    test_console_write_u32(whole);
    digit = '.';
    vision_uart_write(&digit, 1U);
    digit = (uint8_t) ('0' + ((fraction / 100U) % 10U));
    vision_uart_write(&digit, 1U);
    digit = (uint8_t) ('0' + ((fraction / 10U) % 10U));
    vision_uart_write(&digit, 1U);
    digit = (uint8_t) ('0' + (fraction % 10U));
    vision_uart_write(&digit, 1U);
}
