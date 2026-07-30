#ifndef TI_MSP_DL_CONFIG_H
#define TI_MSP_DL_CONFIG_H

#define CONFIG_LP_MSPM0G3507
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__) || defined(__ARMCC_VERSION)
#define SYSCONFIG_WEAK __attribute__((weak))
#else
#define SYSCONFIG_WEAK
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POWER_STARTUP_DELAY (16)
#define CPUCLK_FREQ (32000000U)

#define VISION_UART_INST UART0
#define VISION_UART_INST_FREQUENCY (32000000U)
#define VISION_UART_INST_IRQHandler UART0_IRQHandler
#define VISION_UART_INST_INT_IRQN UART0_INT_IRQn
#define GPIO_VISION_UART_RX_PORT GPIOA
#define GPIO_VISION_UART_TX_PORT GPIOA
#define GPIO_VISION_UART_RX_PIN DL_GPIO_PIN_11
#define GPIO_VISION_UART_TX_PIN DL_GPIO_PIN_10
#define GPIO_VISION_UART_IOMUX_RX (IOMUX_PINCM22)
#define GPIO_VISION_UART_IOMUX_TX (IOMUX_PINCM21)
#define GPIO_VISION_UART_IOMUX_RX_FUNC IOMUX_PINCM22_PF_UART0_RX
#define GPIO_VISION_UART_IOMUX_TX_FUNC IOMUX_PINCM21_PF_UART0_TX

#define EMM_UART_INST UART1
#define EMM_UART_INST_FREQUENCY (32000000U)
#define EMM_UART_INST_IRQHandler UART1_IRQHandler
#define EMM_UART_INST_INT_IRQN UART1_INT_IRQn
#define GPIO_EMM_UART_RX_PORT GPIOB
#define GPIO_EMM_UART_TX_PORT GPIOB
#define GPIO_EMM_UART_RX_PIN DL_GPIO_PIN_5
#define GPIO_EMM_UART_TX_PIN DL_GPIO_PIN_4
#define GPIO_EMM_UART_IOMUX_RX (IOMUX_PINCM18)
#define GPIO_EMM_UART_IOMUX_TX (IOMUX_PINCM17)
#define GPIO_EMM_UART_IOMUX_RX_FUNC IOMUX_PINCM18_PF_UART1_RX
#define GPIO_EMM_UART_IOMUX_TX_FUNC IOMUX_PINCM17_PF_UART1_TX

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_VISION_UART_init(void);
void SYSCFG_DL_EMM_UART_init(void);

#ifdef __cplusplus
}
#endif
#endif
