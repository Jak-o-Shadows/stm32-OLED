#include "taskBlink.hpp"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>


void HeartbeatTask::setup() {
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}

void HeartbeatTask::loop() {
    gpio_toggle(GPIOC, GPIO13);
    sleep(200);
    gpio_toggle(GPIOC, GPIO13);
    sleep(200);
    gpio_toggle(GPIOC, GPIO13);
    sleep(150);
    gpio_toggle(GPIOC, GPIO13);
    sleep(1000);
}
