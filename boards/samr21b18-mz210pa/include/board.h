#ifndef BOARD_H
#define BOARD_H

#include "cpu.h"
#include "periph_conf.h"
#include "periph_cpu.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   xtimer configuration
 * @{
 */
#define XTIMER_DEV          TIMER_1
#define XTIMER_CHAN         (0)
/** @} */

/**
 * @name AT86RF233 configuration
 *
 * {spi bus, spi speed, cs pin, int pin, reset pin, sleep pin}
 */
#define AT86RF2XX_PARAMS_BOARD      {.spi = SPI_DEV(0), \
                                     .spi_clk = SPI_CLK_5MHZ, \
                                     .cs_pin = GPIO_PIN(PB, 31), \
                                     .int_pin = GPIO_PIN(PB, 0), \
                                     .sleep_pin = GPIO_PIN(PA, 20), \
                                     .reset_pin = GPIO_PIN(PB, 15)}

/**
 * @brief   LED pin definitions and handlers
 * @{
 */
#define LED0_PIN            GPIO_PIN(0, 7)

#define LED_PORT            PORT->Group[0]
#define LED0_MASK           (1 << 7)

#define LED0_ON             (LED_PORT.OUTCLR.reg = LED0_MASK)
#define LED0_OFF            (LED_PORT.OUTSET.reg = LED0_MASK)
#define LED0_TOGGLE         (LED_PORT.OUTTGL.reg = LED0_MASK)
/** @} */

/**
 * @name SW0 (Button) pin definitions
 * @{
 */
#define BUTTON_PORT         PORT->Group[0]
#define BUTTON_PIN          (28)
#define BUTTON_GPIO         GPIO_PIN(0, BUTTON_PIN)
/** @} */

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_H */
/** @} */
