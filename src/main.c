#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/i2c.h>
#include <libopencm3/stm32/dma.h>

#include <libopencm3/cm3/nvic.h>

#include <stdint.h>
#include <string.h>

typedef enum Status_e
{
    STATUSok = 0,
    STATUSbad = 1
} Status;

// Generic i2c
Status i2cSendBytes(uint32_t i2c, uint8_t addr, uint8_t data[], uint8_t numData)
{

    uint32_t reg32 __attribute__((unused));

    //send start
    i2c_send_start(i2c);

    //wait for the switch to master mode.
    while (!((I2C_SR1(i2c) & I2C_SR1_SB) &
             (I2C_SR2(i2c) & (I2C_SR2_MSL | I2C_SR2_BUSY))))
        ;

    //send address
    i2c_send_7bit_address(i2c, addr, I2C_WRITE);
    //check for the ack
    while (!(I2C_SR1(i2c) & I2C_SR1_ADDR))
        ;
    //must read SR2 to clear it
    reg32 = I2C_SR2(i2c);

    for (int i = 0; i < numData; i++)
    {
        i2c_send_data(i2c, data[i]);
        while (!(I2C_SR1(i2c) & I2C_SR1_BTF))
            ; //wait for byte transferred flag
    }

    //send stop condition
    i2c_send_stop(i2c);
    for (int i = 0; i < 200; i++)
    {
        __asm__("nop");
    }

    return STATUSok;
}

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

    while (!(I2C_SR1(I2C2) & I2C_SR1_BTF))
        ; //wait for byte transferred flag

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

// SDS1036 OLED Specific

typedef enum state_e
{
    DEAD = 0,
    INIT,
    OK
} state_t;

typedef enum mode_e
{
    PAGE = 0,
    HORI,
    VERT
} mode_t;

typedef struct SSD1306_s
{
    state_t state;
    uint8_t address;
    mode_t mode;
} SSD1306;

#define WIDTH 32
#define HEIGHT 128
#define segmentPixelCount WIDTH
#define segmentCommandSize segmentPixelCount / 8 + 1
typedef union Segment_u {
    uint8_t bytes[segmentCommandSize];
    struct components_s
    {
        uint8_t dataCommand;
        uint8_t pixelBuffer[segmentPixelCount / 8];
    } components;
} Segment_t;

// Working variables
static SSD1306 dev;
static uint8_t segment;
static uint8_t segmentIdx;
static Segment_t buffer1;
static Segment_t buffer2;
static Segment_t *buffers[] = {&buffer1, &buffer2};
static const uint8_t bufferCount = 2;

// Start functions

Status sendCommands(SSD1306 *dev, uint8_t commands[], uint8_t numCommands)
{

    uint8_t commandAddress = 0; // D/C = 0, Co = 0

    // Probably a nicer way of doing this?
    uint8_t commandBuffer[1 + 256]; // Size hardcoded to type of `numCommands`

    // Starts with the commandAddress
    commandBuffer[0] = commandAddress;
    for (int i = 0; i < numCommands; i++)
    {
        commandBuffer[i + 1] = commands[i];
    }
    return i2cSendBytes(I2C2, dev->address, commandBuffer, 1 + numCommands);
}

Status init(SSD1306 *dev)
{

    uint8_t commands[] = {
        // Display Off
        0xAE,
        // Display Offset
        0xD3, // Command
        0,    // Command payload, (No Offset)
        // Display Start line
        0x40,
        // Segment Re-Map
        0xA0 | 0x1, // Rotate screen 180
        // Vertical Mode - works better with the 'line based' paradigm
        0x20, // Set memory mode command
        1,    // Vertical
        // COM SCAN DEC
        0xC8,
        // COM PINS
        0xDA,
        0x02, // Specific to 128x32 px
        // Multiplex
        0xA8,
        32 - 1,
        // Set Oscillator Frequency
        0xD5, // Command
        0x80, // Recommended value (p64 of App Note)
        // Charge Pump
        0x8D,
        // pre-Charge
        0xD9,
        0xF1,
        // Internal VCC
        0x14,
        // VCOM Detect
        0xD8,
        0x40,
        // Display Contrast
        0x81, // Command
        128,  // Set between 1ish and 255ish
        // Display resume
        0xA4,
        // Normal display
        0xA6,
        // Display on
        0xAF};

    static const uint8_t numCommands = 25;

    return sendCommands(dev, commands, numCommands);

    return STATUSok;
}

// Older Bits

// Define Functions
#define IsHigh(BIT, PORT) ((PORT & (1 << BIT)) != 0)
#define IsLow(BIT, PORT) ((PORT & (1 << BIT)) == 0)
#define SetBit(BIT, PORT) PORT |= (1 << BIT)
#define ClearBit(BIT, PORT) PORT &= ~(1 << BIT)

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

    nvic_enable_irq(NVIC_TIM2_IRQ);

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

/////////////////////////////////////////////////////////
//////////                     //////////////////////////
/////////////////////////////////////////////////////////

void i2c_setup(void)
{

    //disable i2c before changing config
    i2c_peripheral_disable(I2C2);
    i2c_reset(I2C2);

    i2c_set_standard_mode(I2C2);
    i2c_set_clock_frequency(I2C2, I2C_CR2_FREQ_16MHZ);

    i2c_set_ccr(I2C2, 0xAA);
    i2c_set_dutycycle(I2C2, I2C_CCR_DUTY_DIV2);

    i2c_set_trise(I2C2, 0x11);

    i2c_enable_ack(I2C2);

    //enable it
    i2c_peripheral_enable(I2C2);

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

    // Enable TCIF
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);

    dma_disable_channel(DMA1, DMA_CHANNEL4);
}

/////////////////////////////////////////////////////
////////////// Main Loop   //////////////////////////
/////////////////////////////////////////////////////

void OLED_address(SSD1306 *dev, uint8_t x, uint8_t y)
{
    uint8_t commands[] = {
        // Set column addressing
        0x21,
        0x00,
        128 - 1,
        // Set page addressing extents
        0x22,
        0x00,
        32 / 8 - 1};
    sendCommands(dev, commands, 6);
}

int main(void)
{
    clock_setup();
    gpio_setup();
    timer_setup();
    usart_setup();
    i2c_setup();
    nvic_setup();

    uint32_t i2c = I2C2; //i2c2

    // Initialise device properties
    dev.address = 0b0111100;
    dev.mode = DEAD;
    dev.state = PAGE;

    // Initialise segment buffers
    buffer1.components.dataCommand = 0x40;
    buffer2.components.dataCommand = 0x40;
    memset(buffer1.components.pixelBuffer, 0xFF, segmentPixelCount / 8);
    memset(buffer2.components.pixelBuffer, 0, segmentPixelCount / 8);
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
        i2cSendBytes(I2C2, dev.address, bytes, 2);
    }

    // kick it all off
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL4);
    uint8_t bytes[] = {0xFF};
    i2cSendBytesDMA(dev.address, bytes, 1);

    //dma1_channel4_isr();

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
    if (segment > (WIDTH * HEIGHT) / segmentPixelCount)
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
    i2cSendBytesDMA(dev.address, activeBuffer.bytes, segmentCommandSize);

    // Then process next buffer
    Segment_t *nextBuffer = buffers[segmentIdx];
    memset(nextBuffer->components.pixelBuffer, segmentIdx, segmentPixelCount / 8);

    // Artificial delay for development
    for (int i = 0; i < 100000; i++)
    {
        __asm__("nop");
    }
}