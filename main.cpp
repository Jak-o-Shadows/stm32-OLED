#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/i2c.h>
#include <libopencm3/stm32/dma.h>

#include <libopencm3/cm3/nvic.h>

#include <stdint.h>
#include <string.h>

// FreeRTOS includes
#include "FreeRTOS.h"
#include "task.h"

#include "status.hpp"

#include "taskBlink.hpp"
#include "OledTask.hpp"

#include "devices/gy521/gy521.hpp"


HeartbeatTask heartbeat;
OledTask oled;


void clock_setup(void)
{
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    /* Enable GPIOA clock (for LED GPIOs). */
    rcc_periph_clock_enable(RCC_GPIOC);

    rcc_periph_clock_enable(RCC_GPIOA);

    /* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
    rcc_periph_clock_enable(RCC_USART2);
    rcc_periph_clock_enable(RCC_AFIO);

    rcc_periph_clock_enable(RCC_GPIOB);
    //I2C
    rcc_periph_clock_enable(RCC_I2C2);
    rcc_periph_clock_enable(RCC_DMA1);
}

void usart_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART2);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2); //USART 2 TX is A2
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO3);                   //USART 2 RX is A3

    usart_set_baudrate(USART2, 9600);
    usart_set_databits(USART2, 8);
    usart_set_stopbits(USART2, USART_STOPBITS_1);
    usart_set_parity(USART2, USART_PARITY_NONE);
    usart_set_mode(USART2, USART_MODE_TX_RX);
    usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);
    //enable interrupt rx
    USART_CR1(USART2) |= USART_CR1_RXNEIE;

    usart_enable(USART2);
}

static void nvic_setup(void)
{
    //nvic_set_priority(NVIC_USART2_IRQ, 2);
    //nvic_enable_irq(NVIC_USART2_IRQ);

    nvic_set_priority(NVIC_DMA1_CHANNEL4_IRQ, 0);
    nvic_enable_irq(NVIC_DMA1_CHANNEL4_IRQ);

    nvic_set_priority(NVIC_TIM2_IRQ, 2);
    nvic_enable_irq(NVIC_TIM2_IRQ);
}

static void gpio_setup(void)
{
    gpio_set(GPIOC, GPIO13);

    /* Setup GPIO for LED use. */
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13 | GPIO14);

    //setup i2c pins
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,
                  GPIO10 | GPIO11); //B10 =SCL, B11=SDA
                                    //		  GPIO6 | GPIO7);
}

static void timer_setup(void)
{

    rcc_periph_clock_enable(RCC_TIM2);

    rcc_periph_reset_pulse(RST_TIM2);

    //Timer global mode:
    //	no divider
    //	alignment edge
    //	direction up
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    // Note that TIM2 on APB1 is running at double frequency according to
    //	https://github.com/libopencm3/libopencm3-examples/blob/master/examples/stm32/f1/stm32-h103/timer/timer.c
    timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 10000));

    // Disable preload
    timer_disable_preload(TIM2);
    timer_continuous_mode(TIM2);

    // Count the full range, as the compare value is used to set the value
    timer_set_period(TIM2, 65535);

    timer_set_oc_value(TIM2, TIM_OC1, 10); //was 10000

    timer_enable_counter(TIM2);

    timer_enable_irq(TIM2, TIM_DIER_CC1IE);
}



/////////////////////////////////////////////////////
////////////// Main Loop   //////////////////////////
/////////////////////////////////////////////////////

int main(void)
{

    clock_setup();
    gpio_setup();
    usart_setup();
    timer_setup();
    nvic_setup();

    const uint32_t i2c = I2C2; //i2c2
    i2cMaster_setup(i2c);

    GY521_t gy521_dev = {
        .address = 0x68,
        .i2c = I2C2,
    };
    int16_t accels[3];
    int16_t gyros[3];
    
    Status sts;
    sts = gy521_init(&gy521_dev);

    while(1){
        for(int i=0;i<50000;i++){
            __asm__("nop");
        };
        sts = gy521_read(&gy521_dev, accels, gyros);

    };


    /*
    heartbeat.start("heartbeat", configMINIMAL_STACK_SIZE, 1);
    oled.start("oled", configMINIMAL_STACK_SIZE, 1);

    vTaskStartScheduler();
    for(;;);
    */
    return 0;

}

/////////////////////////////////////////////////////
////////////// Comms Stuff //////////////////////////
/////////////////////////////////////////////////////

// Interrupt Functions

void usart2_isr(void)
{
    static uint8_t data = 'A';
    static uint8_t reply = 0;

    /* Check if we were called because of RXNE. */
    if (((USART_CR1(USART2) & USART_CR1_RXNEIE) != 0) &&
        ((USART_SR(USART2) & USART_SR_RXNE) != 0))
    {

        // Receieve the data, using the MiniSSC protocol
        //	This protocol has a header byte (0xFF), followed
        //	by a number (0->254) followed by a number (0-254)
        data = usart_recv(USART2);
        //reply = dobyte(data);

        /* Enable transmit interrupt so it sends back the data. */
        USART_CR1(USART2) |= USART_CR1_TXEIE;
    }

    /* Check if we were called because of TXE. */
    if (((USART_CR1(USART2) & USART_CR1_TXEIE) != 0) &&
        ((USART_SR(USART2) & USART_SR_TXE) != 0))
    {

        /* Indicate that we are sending out data. */
        // gpio_toggle(GPIOA, GPIO7);

        /* Put data into the transmit register. */
        usart_send(USART2, reply);

        /* Disable the TXE interrupt as we don't need it anymore. */
        USART_CR1(USART2) &= ~USART_CR1_TXEIE;
    }
}

void tim2_isr(void)
{
    // This timer ticks every 1ms
    if (timer_get_flag(TIM2, TIM_SR_CC1IF))
    {
        timer_clear_flag(TIM2, TIM_SR_CC1IF);

        // Setup next compare time
        uint16_t compare_time = timer_get_counter(TIM2);
        timer_set_oc_value(TIM2, TIM_OC1, 10 + compare_time);

        // Debug toggle LED
        gpio_toggle(GPIOC, GPIO13);
    }
}

void dma1_channel4_isr(void)
{
    // I2C Transmit DMA channel

    // First clear flag
    if (dma_get_interrupt_flag(DMA1, DMA_CHANNEL4, DMA_TCIF))
    {
        dma_clear_interrupt_flags(DMA1, DMA_CHANNEL4, DMA_TCIF);
    }

    if (oled.segment == 0)
    {
        OLED_address(&oled.dev, 0, 0);
    }
    oled.segment++;
    if ((oled.segment*oled.display.segment_width*oled.display.segment_height) >= 
        (oled.display.display_width * oled.display.display_height) )
    {
        oled.segment = 0;
        // Disable TCIF interrupt flag
        dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    }
    else
    {
        // Must re-enable TCIF interrupt
        dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    }

    // Send the initial buffer
    oled.i2cSendBytesDMA(oled.dev.address, oled.display.segments[oled.segmentIdx].data.bytes, oled.display.segments[oled.segmentIdx].bytes_len);

    // Then swap buffer to prepare the next one.
    oled.segmentIdx++;
    if (oled.segmentIdx >= oled.display.num_segments)
    {
        oled.segmentIdx = 0;
    }

    // Then prepare next buffer to send

    // First clear it
    uint8_t fill = 0;
    if (oled.segmentIdx==0){
        fill = 0;
    } else {
        fill = 0xFF;
    }
    memset(oled.display.segments[oled.segmentIdx].data.components.pixelBuffer, fill, oled.display.segments[oled.segmentIdx].bytes_len-1);

    // Draw a line across X near the bottom
    uint8_t offset;
    for (uint8_t row = 0; row < oled.display.display_height; row++)
    {
        offset = oled.segment*4;
        offset = offset % oled.display.display_height;
        pixelInvert(&oled.display.segments[oled.segmentIdx], oled.display.getSegmentXCoord(oled.segment), oled.display.getSegmentYCoord(oled.segment), oled.display.segment_width - offset, row);
    }

    // Draw some characters
     char word[] = "Jak_o_Shadows";
    uint8_t wordLength = 13;
    uint8_t start = oled.rotateX + oled.display.display_height/2;
    words(&oled.display.segments[oled.segmentIdx], oled.display.getSegmentXCoord(oled.segment), oled.display.getSegmentYCoord(oled.segment), start, 0, word, wordLength);

    gpio_toggle(GPIOC, GPIO14);

}