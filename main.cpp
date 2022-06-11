#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/i2c.h>
#include <libopencm3/stm32/dma.h>

#include <libopencm3/cm3/nvic.h>

#include <stdint.h>
#include <string.h>

#include "macros.h"
#include "status.hpp"
#include "devices/ssd1306/ssd1306.hpp"
#include "protocol/drawing/drawing.hpp"
#include "periph/i2c/i2cMaster.hpp"

// Older Bits

// I2C and DMA

Status i2cSendBytesDMA(uint8_t addr, uint8_t data[], uint8_t numData)
{

    dma_disable_channel(DMA1, DMA_CHANNEL4);

    dma_set_memory_address(DMA1, DMA_CHANNEL4, (uint32_t)data);

    dma_set_number_of_data(DMA1, DMA_CHANNEL4, numData);

    // Start the transaction
    uint32_t reg32 __attribute__((unused));

    //send start
    i2c_send_start(I2C2);

    //wait for the switch to master mode.
    while (!((I2C_SR1(I2C2) & I2C_SR1_SB) &
             (I2C_SR2(I2C2) & (I2C_SR2_MSL | I2C_SR2_BUSY))))
        ;

    //send address
    i2c_send_7bit_address(I2C2, addr, I2C_WRITE);
    //check for the ack
    while (!(I2C_SR1(I2C2) & I2C_SR1_ADDR))
        ;
    //must read SR2 to clear it
    reg32 = I2C_SR2(I2C2);

    // Send the data
    i2c_enable_dma(I2C2);

    dma_enable_channel(DMA1, DMA_CHANNEL4);

    uint16_t maxCheck = 65000;
    while (maxCheck--)
    {
        if ((I2C_SR1(I2C2) & I2C_SR1_BTF))
        {
            break;
        }
    }

    // Finish the transaction
    i2c_send_stop(I2C2);
    for (int i = 0; i < 200; i++)
    {
        __asm__("nop");
    }

    // Disable DMA
    i2c_disable_dma(I2C2);
    //dma_disable_channel(DMA1, DMA_CHANNEL4);

    return STATUSok;
}

void i2cDma_setup(void)
{

    // Setup DMA with the i2c
    //  @100 kHz, one byte takes 0.08 ms. => 1 pixel takes 0.01 ms
    //  Hence the whole 128x32 screen takes a minimum of 4096*0.01 ms
    //      = 40.96 ms.
    // Ignoring a couple of things, this makes a full update rate of
    //  just 24 Hz
    //
    // If using a line-by line paradigm, with a 32 pixel line, each line
    //  takes 0.32 ms to transfer. This is ONLY an update rate of 3125 Hz
    //      => the line by line paradigm probably works pretty damn easy
    //
    // i2c is slow...

    // Use DMA to transfer
    // I2C2 is DMA1 Channel 4 (TX)
    // Setup DMA
    dma_channel_reset(DMA1, DMA_CHANNEL4);

    dma_set_peripheral_address(DMA1, DMA_CHANNEL4, (uint32_t)&I2C2_DR);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL4);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL4, DMA_CCR_PSIZE_8BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL4, DMA_CCR_MSIZE_8BIT);
    dma_set_read_from_memory(DMA1, DMA_CHANNEL4);

    dma_disable_channel(DMA1, DMA_CHANNEL4);
}


// Working variables
static SSD1306_t dev;
static uint8_t segment;
static uint8_t segmentIdx;
static Segment_t buffer1;
static Segment_t buffer2;
static Segment_t *buffers[] = {&buffer1, &buffer2};
static const uint8_t bufferCount = 2;

// TEMP - drawing function
static uint8_t rotateX = 0;
static const uint16_t msRotate = 20;
static uint16_t msRotateCounter = 0;



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
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

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

    const uint32_t i2c = I2C2; //i2c2


    clock_setup();
    gpio_setup();
    usart_setup();
    i2cMaster_setup(i2c);
    i2cDma_setup();


    // Initialise device properties
    dev.address = 0b0111100;
    dev.mode = PAGE;
    dev.state = DEAD;
    dev.i2c = I2C2;

    // Initialise segment buffers
    buffer1.size_x = 32;
    buffer1.size_y = 1;
    buffer2.size_x = 32;
    buffer2.size_y = 1;
    buffer1.data.components.dataCommand = 0x40;
    buffer2.data.components.dataCommand = 0x40;
    memset(buffer1.data.components.pixelBuffer, 0xFF, segmentPixelCount / 8);
    memset(buffer2.data.components.pixelBuffer, 0, segmentPixelCount / 8);
    // Initialise double buffer & tracking variables
    segment = 0;
    segmentIdx = 0;

    init(&dev);

    // Send some data

    // Delay
    for (int i = 0; i < 10000; i++)
    {
        __asm__("nop");
    }

    // Clear the display
    OLED_address(&dev, 0, 0);
    for (int pxByte = 0; pxByte < 128 * 32 / 8; pxByte++)
    {
        uint8_t bytes[] = {
            0x40,
            0};
        i2cMaster_send(I2C2, dev.address, bytes, 2);
    }

    // kick it all off
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    uint8_t bytes[] = {0xFF};
    i2cSendBytesDMA(dev.address, bytes, 1);

    gpio_toggle(GPIOC, GPIO13);

    //dma1_channel4_isr();
    for (int i = 0; i < 50000; i++)
    {
        __asm__("nop");
        gpio_toggle(GPIOC, GPIO13);
    }

    timer_setup();
    nvic_setup();


    setupScroll(&dev, false);
    scrollState(&dev, false);

    bool inverted = false;
    uint8_t commandsInvert[1];
    while (1)
    {
        for (int i = 0; i < 5000000; i++)
        {
            __asm__("nop");
        }
        // Then invert display
        if (inverted)
        {
            commandsInvert[0] = 0xA7;
        }
        else
        {
            commandsInvert[0] = 0xA6;
        }
        //sendCommands(&dev, commandsInvert, 1);
        inverted = !inverted;
    }
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

        // Do the work
        msRotateCounter++;
        if (msRotateCounter > msRotate)
        {
            msRotateCounter = 0;

            // Do the update
            rotateX = rotateX - 5;
            // Kick off update
            dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
            uint8_t bytes[] = {0xFF};
            i2cSendBytesDMA(dev.address, bytes, 1);

            // Debug toggle LED
            gpio_toggle(GPIOC, GPIO13);
        }
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

    if (segment == 0)
    {
        OLED_address(&dev, 0, 0);
    }
    segment++;
    if (segment > ((WIDTH * HEIGHT) / segmentPixelCount))
    {
        segment = 0;
        // Disable TCIF interrupt flag
        dma_disable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    }
    else
    {
        // Must re-enable TCIF interrupt
        dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    }

    // Then swap buffer
    Segment_t activeBuffer = *buffers[segmentIdx];
    segmentIdx++;
    if (segmentIdx >= bufferCount)
    {
        segmentIdx = 0;
    }

    // Then send
    i2cSendBytesDMA(dev.address, activeBuffer.data.bytes, segmentCommandSize);

    // Then prepare next buffer to send
    Segment_t *nextBuffer = buffers[segmentIdx];
    memset(nextBuffer->data.components.pixelBuffer, 0xFF, segmentPixelCount / 8);

    // Draw a line across X near the bottom
    for (uint8_t row = 0; row < HEIGHT; row++)
    {
        pixelClear(nextBuffer, 0, segment, WIDTH - 4, row);
    }

    // Draw some characters
    char word[] = "Jak_o_Shadows";
    uint8_t wordLength = 13;
    uint8_t start = rotateX + HEIGHT / 2; // Works because overflow
    words(nextBuffer, 0, segment, start, 1, word, wordLength);
}