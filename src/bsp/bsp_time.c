#include "bsp_time.h"
#include "ti_msp_dl_config.h"
static volatile uint32_t g_ms;
void bsp_time_init(void)
{
    g_ms = 0U;
    (void) SysTick_Config(CPUCLK_FREQ / 1000U);
}
uint32_t bsp_time_ms(void) { return g_ms; }
void SysTick_Handler(void) { g_ms++; }
