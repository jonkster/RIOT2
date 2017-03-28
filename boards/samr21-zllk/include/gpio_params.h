#ifndef GPIO_PARAMS_H
#define GPIO_PARAMS_H

#include "board.h"
#include "saul/periph.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief    GPIO pin configuration
 */
static const  saul_gpio_params_t saul_gpio_params[] =
{
    {
        .name = "LED(red)",
        .pin = LED0_PIN,
        .mode = GPIO_OUT
    },
    {
        .name = "LED(yellow)",
        .pin = LED1_PIN,
        .mode = GPIO_OUT
    },
    {
        .name = "LED(RGB_R)",
        .pin = LED_RGB_R_PIN,
        .mode = GPIO_OUT
    },
    {
        .name = "LED(RGB_G)",
        .pin = LED_RGB_G_PIN,
        .mode = GPIO_OUT
    },
    {
        .name = "LED(RGB_B)",
        .pin = LED_RGB_B_PIN,
        .mode = GPIO_OUT
    },
    {
        .name = "Button(SW0)",
        .pin = BUTTON_GPIO,
        .mode = GPIO_IN_PU
    },
};

#ifdef __cplusplus
}
#endif

#endif /* GPIO_PARAMS_H */
/** @} */
