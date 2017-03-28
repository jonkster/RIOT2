#include "board.h"
#include "periph/gpio.h"

void board_init(void)
{
    /* initialize the on-board LED */
    gpio_init(LED0_PIN, GPIO_OUT);
    gpio_init(LED1_PIN, GPIO_OUT);
    gpio_init(LED_RGB_R_PIN, GPIO_OUT);
    gpio_init(LED_RGB_G_PIN, GPIO_OUT);
    gpio_init(LED_RGB_B_PIN, GPIO_OUT);

    /* initialize the CPU */
    cpu_init();
}
