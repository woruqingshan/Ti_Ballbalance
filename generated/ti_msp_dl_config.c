#include "ti_msp_dl_config.h"

static const DL_UART_Main_ClockConfig gUartClockConfig = {
    .clockSel = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUartConfig = {
    .mode = DL_UART_MAIN_MODE_NORMAL,
    .direction = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity = DL_UART_MAIN_PARITY_NONE,
    .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_VISION_UART_init();
    SYSCFG_DL_EMM_UART_init();
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_UART_Main_reset(VISION_UART_INST);
    DL_UART_Main_reset(EMM_UART_INST);
    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_UART_Main_enablePower(VISION_UART_INST);
    DL_UART_Main_enablePower(EMM_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{
    DL_GPIO_initPeripheralOutputFunction(GPIO_VISION_UART_IOMUX_TX,
                                          GPIO_VISION_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_VISION_UART_IOMUX_RX,
                                         GPIO_VISION_UART_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(GPIO_EMM_UART_IOMUX_TX,
                                          GPIO_EMM_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_EMM_UART_IOMUX_RX,
                                         GPIO_EMM_UART_IOMUX_RX_FUNC);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
}

static void init_uart(UART_Regs *inst, bool rx_interrupt)
{
    DL_UART_Main_setClockConfig(inst, (DL_UART_Main_ClockConfig *) &gUartClockConfig);
    DL_UART_Main_init(inst, (DL_UART_Main_Config *) &gUartConfig);
    DL_UART_Main_setOversampling(inst, DL_UART_OVERSAMPLING_RATE_16X);
    /* 32 MHz BUSCLK, 115200 baud: IBRD=17, FBRD=23. */
    DL_UART_Main_setBaudRateDivisor(inst, 17U, 23U);
    if (rx_interrupt) {
        DL_UART_Main_enableInterrupt(inst, DL_UART_MAIN_INTERRUPT_RX);
    }
    DL_UART_Main_enable(inst);
}

SYSCONFIG_WEAK void SYSCFG_DL_VISION_UART_init(void)
{
    init_uart(VISION_UART_INST, true);
}

SYSCONFIG_WEAK void SYSCFG_DL_EMM_UART_init(void)
{
    init_uart(EMM_UART_INST, false);
}
