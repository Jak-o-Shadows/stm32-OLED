#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/f1/i2c.h>

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
        0x00, // Command payload, (No Offset)
        // Display Start line
        0x40, // 0x40 + bits (up to 7F)
        // Segment Re-Map
        0xA1, // Loop around when writing data back to zero (matches adafruit library)
        // Set COM port outputs and direction
        0xC8, // Remapped mode. N-1 to 0 (matches adafruit library)
        // Set COM Pins hardware configuration
        0xDA, // Set COM Pins command
        0x02, // For 128x32 matrix
        // Set Contrast
        0x81, // Set contrast command
        0x8F, // Contrast value (0->FF)
        // Set display off
        0xAE,
        // Set normal display
        0xA6, // 1 = on, 0 = off per pixel
        // Set memory mode
        0x20, // Set memory mode command
        0x00, // b00 = horizontal
        // Set Oscillator Frequency
        0xD5, // Command
        0x80, // Recommended value (p64 of App Note)
        // Enable charge pump regulator
        0x8D, // Command
        0x14, // Recommended value (p64 of App Note)
        // Display on
        0xAF};

    static const uint8_t numCommands = 17;
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
    /* Without this the RTC interrupt routine will never be called. */
    nvic_enable_irq(NVIC_USART2_IRQ);
    nvic_set_priority(NVIC_USART2_IRQ, 2);
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
}

/////////////////////////////////////////////////////
////////////// Main Loop   //////////////////////////
/////////////////////////////////////////////////////

int main(void)
{
    clock_setup();
    gpio_setup();
    timer_setup();
    usart_setup();
    i2c_setup();
    nvic_setup();

    uint32_t i2c = I2C2; //i2c2

    SSD1306 dev;
    dev.address = 0b0111100;
    dev.mode = DEAD;
    dev.state = PAGE;

    init(&dev);

    // Send some data

    // Delay
    for (int i = 0; i < 10000; i++)
    {
        __asm__("nop");
    }

    // Set where to write the data to
    uint8_t commands[] = {
        // Set page address
        0x22, // Command
        0,    // Page start
        0xFF, // Page end (not really, but it'll do according to adafruit)
        // Set Column Address
        0x21, // Command
        0     // Column Start
    };
    sendCommands(&dev, commands, 5);
    uint8_t commands2[] = {
        128 - 1 // Column end (128x32)
    };
    sendCommands(&dev, commands2, 1);

    // Set the whole thing off
    for (int pxByte = 0; pxByte < 128 * 32 / 8; pxByte++)
    {
        uint8_t bytes[] = {
            0x40,
            pxByte};
        sendCommands(&dev, bytes, 2);
    }
    /*
    sendCommands(&dev, commands, 5);
    sendCommands(&dev, commands2, 1);

    // Delay
    for (int i = 0; i < 10000; i++)
    {
        __asm__("nop");
    }

    uint8_t line[128 + 1];
    memset(&line, 0xFF, 128);
    line[0] = 0x40;

    // Send the data
    for (int i = 0; i < 32 / 8; i++)
    {
        //uint8_t bytes[] = {
        //    0x40, // D/C, Co command
        //    0xFF};

        i2cSendBytes(i2c, dev.address, line, 128 + 1);
        memset(&line[1], 0xFF * (i % 2), 128);
    }
*/
    bool inverted = false;
    uint8_t commandsInvert[1];

    for (int invertCount = 0; invertCount < 51; invertCount++)
    {
        for (int i = 0; i < 1000000; i++)
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
        sendCommands(&dev, commandsInvert, 1);
        inverted = !inverted;
    }

    while (1)
    {
        __asm__("nop");
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
