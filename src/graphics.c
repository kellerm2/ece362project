// graphics.c
// Self-contained graphics module for STM32F091 Audio Visualizer project

#include "stm32f0xx.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

// -----------------------------------------------------------------------------
// Definitions and Macros
// -----------------------------------------------------------------------------

#define TFT_WIDTH  240
#define TFT_HEIGHT 320

// SPI Configuration
#define TFT_SPI       SPI1
#define TFT_SPI_AF    0   // Adjust based on your MCU's SPI1 alternate function
#define TFT_SPI_CLK   RCC_APB2ENR_SPI1EN
#define TFT_SCK_PIN   GPIO_PIN_5    // PA5
#define TFT_MOSI_PIN  GPIO_PIN_7    // PA7
#define TFT_CS_PIN    GPIO_PIN_4    // PA4
#define TFT_DC_PIN    GPIO_PIN_3    // PA3
#define TFT_RST_PIN   GPIO_PIN_2    // PA2

// Color Definitions (RGB565)
#define COLOR_BLACK   0x0000
#define COLOR_WHITE   0xFFFF
#define COLOR_RED     0xF800
#define COLOR_GREEN   0x07E0
#define COLOR_BLUE    0x001F
#define COLOR_YELLOW  0xFFE0
#define COLOR_ORANGE  0xFD20
#define COLOR_PURPLE  0xF81F
#define COLOR_CYAN    0x07FF
#define COLOR_MAGENTA 0xF81F

// -----------------------------------------------------------------------------
// Global Variables
// -----------------------------------------------------------------------------

volatile uint16_t current_color = COLOR_WHITE;

// -----------------------------------------------------------------------------
// Function Prototypes
// -----------------------------------------------------------------------------

void graphics_init(void);
void spi_init(void);
void spi_send(uint8_t data);
void tft_command(uint8_t cmd);
void tft_data(uint8_t data);
void tft_reset(void);
void tft_init(void);
void tft_set_address_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void tft_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
void tft_clear_screen(uint16_t color);
void tft_draw_rectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
void render_waveform(uint16_t* audio_buffer, uint16_t buffer_size);
void render_spectrum(float* frequency_bins, uint16_t num_bins);
uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b);

// -----------------------------------------------------------------------------
// Function Implementations
// -----------------------------------------------------------------------------

// Initialize SPI peripheral for TFT communication
void spi_init(void) {
    // Enable clocks for GPIOA and SPI1
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure GPIO pins for SPI1: PA5 (SCK), PA7 (MOSI)
    GPIOA->MODER &= ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER7_Msk | GPIO_MODER_MODER3_Msk | GPIO_MODER_MODER4_Msk | GPIO_MODER_MODER2_Msk);
    GPIOA->MODER |= (2 << GPIO_MODER_MODER5_Pos) | // SCK
                     (2 << GPIO_MODER_MODER7_Pos) | // MOSI
                     (1 << GPIO_MODER_MODER3_Pos) | // DC as General Output
                     (1 << GPIO_MODER_MODER4_Pos) | // CS as General Output
                     (1 << GPIO_MODER_MODER2_Pos);  // RST as General Output

    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_5 | GPIO_OTYPER_OT_7); // Push-pull
    GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEEDR5_Pos) | // High speed
                       (3 << GPIO_OSPEEDR_OSPEEDR7_Pos) |
                       (3 << GPIO_OSPEEDR_OSPEEDR3_Pos) |
                       (3 << GPIO_OSPEEDR_OSPEEDR4_Pos) |
                       (3 << GPIO_OSPEEDR_OSPEEDR2_Pos);

    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5_Msk | GPIO_PUPDR_PUPDR7_Msk |
                      GPIO_PUPDR_PUPDR3_Msk | GPIO_PUPDR_PUPDR4_Msk |
                      GPIO_PUPDR_PUPDR2_Msk);
    // No pull-up, no pull-down

    // Initialize SPI1
    TFT_SPI->CR1 = 0; // Reset configuration
    TFT_SPI->CR1 |= SPI_CR1_MSTR;             // Master mode
    TFT_SPI->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Software slave management
    TFT_SPI->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1; // Baud rate control (f_PCLK/8)
    TFT_SPI->CR1 |= SPI_CR1_SPE;              // Enable SPI
}

// Send a single byte via SPI
void spi_send(uint8_t data) {
    // Wait until TX buffer is empty
    while (!(TFT_SPI->SR & SPI_SR_TXE));
    TFT_SPI->DR = data;
    // Wait until transmission complete
    while (!(TFT_SPI->SR & SPI_SR_RXNE));
    // Read data to clear RX buffer
    (void)TFT_SPI->DR;
}

// Reset the TFT LCD
void tft_reset(void) {
    // Configure RST pin as output
    GPIOA->MODER |= GPIO_MODER_MODER2_0; // PA2 as output
    // Reset sequence
    GPIOA->BSRR = GPIO_BSRR_BR_2; // RST low
    for (volatile int i = 0; i < 100000; i++);
    GPIOA->BSRR = GPIO_BSRR_BS_2; // RST high
    for (volatile int i = 0; i < 100000; i++);
}

// Send a command to the TFT
void tft_command(uint8_t cmd) {
    // Set DC low for command
    GPIOA->BSRR = GPIO_BSRR_BR_3; // DC low
    // Set CS low
    GPIOA->BSRR = GPIO_BSRR_BR_4; // CS low
    spi_send(cmd);
    // Set CS high
    GPIOA->BSRR = GPIO_BSRR_BS_4; // CS high
}

// Send data to the TFT
void tft_data(uint8_t data) {
    // Set DC high for data
    GPIOA->BSRR = GPIO_BSRR_BS_3; // DC high
    // Set CS low
    GPIOA->BSRR = GPIO_BSRR_BR_4; // CS low
    spi_send(data);
    // Set CS high
    GPIOA->BSRR = GPIO_BSRR_BS_4; // CS high
}

// Initialize the TFT LCD
void tft_init(void) {
    tft_reset();

    tft_command(0x01);             // Software Reset
    nano_wait(500000);              // Delay
    tft_command(0x28);             // Display OFF

    // Initialize display settings (commands may vary based on TFT driver)
    tft_command(0xC0);             // Power Control
    tft_data(0x23);
    tft_command(0xC1);
    tft_data(0x10);

    tft_command(0xC2);             // Power Control 2
    tft_data(0x3E);
    tft_data(0x28);

    tft_command(0xC5);             // VCOM Control
    tft_data(0x07);
    tft_data(0xE2);

    tft_command(0x36);             // Memory Data Access Control
    tft_data(0x08);                 // Landscape mode

    tft_command(0x3A);             // COLMOD: Pixel Format Set
    tft_data(0x05);                 // 16-bit/pixel

    tft_command(0xB1);             // Frame Rate Control
    tft_data(0x00);
    tft_data(0x18);

    tft_command(0xB6);             // Display Function Control
    tft_data(0x08);
    tft_data(0x82);
    tft_data(0x27);

    tft_command(0xF2);             // Enable 3G
    tft_data(0x00);

    tft_command(0x26);             // Gamma Set
    tft_data(0x01);

    tft_command(0xE0);             // Positive Gamma Correction
    tft_data(0x0F);
    tft_data(0x31);
    tft_data(0x2B);
    tft_data(0x0C);
    tft_data(0x0E);
    tft_data(0x08);
    tft_data(0x4E);
    tft_data(0xF1);
    tft_data(0x37);
    tft_data(0x07);
    tft_data(0x10);
    tft_data(0x03);
    tft_data(0x0E);
    tft_data(0x09);
    tft_data(0x00);

    tft_command(0xE1);             // Negative Gamma Correction
    tft_data(0x00);
    tft_data(0x0E);
    tft_data(0x14);
    tft_data(0x03);
    tft_data(0x11);
    tft_data(0x07);
    tft_data(0x31);
    tft_data(0xC1);
    tft_data(0x48);
    tft_data(0x08);
    tft_data(0x0F);
    tft_data(0x0C);
    tft_data(0x31);
    tft_data(0x36);
    tft_data(0x0F);

    tft_command(0x29);             // Display ON
    nano_wait(500000);              // Delay
    tft_command(0x2C);             // Memory Write
}

// Set the address window for pixel drawing
void tft_set_address_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    tft_command(0x2A); // Column Address Set
    tft_data(x0 >> 8);
    tft_data(x0 & 0xFF);
    tft_data(x1 >> 8);
    tft_data(x1 & 0xFF);

    tft_command(0x2B); // Page Address Set
    tft_data(y0 >> 8);
    tft_data(y0 & 0xFF);
    tft_data(y1 >> 8);
    tft_data(y1 & 0xFF);

    tft_command(0x2C); // Memory Write
}

// Draw a single pixel at (x, y) with the specified color
void tft_draw_pixel(uint16_t x, uint16_t y, uint16_t color) {
    if (x >= TFT_WIDTH || y >= TFT_HEIGHT)
        return;

    tft_set_address_window(x, y, x, y);
    // Send color data (MSB first)
    tft_data(color >> 8);
    tft_data(color & 0xFF);
}

// Clear the entire screen with the specified color
void tft_clear_screen(uint16_t color) {
    tft_set_address_window(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
    // Stream color data for the entire screen
    GPIOA->BSRR = GPIO_BSRR_BR_4; // CS low
    GPIOA->BSRR = GPIO_BSRR_BR_3; // DC low (command)
    spi_send(0x2C);               // Memory Write command
    GPIOA->BSRR = GPIO_BSRR_BS_3; // DC high (data)

    for (uint32_t i = 0; i < TFT_WIDTH * TFT_HEIGHT; i++) {
        spi_send(color >> 8);
        spi_send(color & 0xFF);
    }

    GPIOA->BSRR = GPIO_BSRR_BS_4; // CS high
}

// Draw a filled rectangle at (x, y) with specified width, height, and color
void tft_draw_rectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color) {
    for (uint16_t i = x; i < x + width && i < TFT_WIDTH; i++) {
        for (uint16_t j = y; j < y + height && j < TFT_HEIGHT; j++) {
            tft_draw_pixel(i, j, color);
        }
    }
}

// Convert 8-bit RGB to RGB565 format
uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | 
           ((g & 0xFC) << 3) | 
           ((b & 0xF8) >> 3);
}

// Render a waveform based on audio buffer data
void render_waveform(uint16_t* audio_buffer, uint16_t buffer_size) {
    // Define waveform area
    uint16_t start_x = 10;
    uint16_t start_y = TFT_HEIGHT / 2;
    uint16_t end_x = TFT_WIDTH - 10;
    uint16_t end_y = TFT_HEIGHT / 2;
    uint16_t amplitude = TFT_HEIGHT / 4;

    // Clear waveform area
    tft_draw_rectangle(start_x, start_y - amplitude, end_x - start_x, 2 * amplitude, COLOR_BLACK);

    // Draw waveform
    uint16_t prev_x = start_x;
    uint16_t prev_y = start_y;

    for (uint16_t i = 0; i < buffer_size; i++) {
        float normalized = (float)audio_buffer[i] / 4096.0f; // Assuming 12-bit ADC
        float offset = normalized - 0.5f; // Center at 0
        uint16_t x = start_x + ((end_x - start_x) * i) / buffer_size;
        uint16_t y = start_y + (uint16_t)(offset * 2 * amplitude);

        tft_draw_pixel(prev_x, prev_y, COLOR_GREEN);
        tft_draw_pixel(x, y, COLOR_GREEN);

        prev_x = x;
        prev_y = y;
    }
}

// Render frequency spectrum based on FFT data
void render_spectrum(float* frequency_bins, uint16_t num_bins) {
    // Define spectrum area
    uint16_t start_x = 10;
    uint16_t start_y = TFT_HEIGHT - 10;
    uint16_t width = TFT_WIDTH - 20;
    uint16_t height = TFT_HEIGHT / 2;
    uint16_t bin_width = width / num_bins;

    // Clear spectrum area
    tft_draw_rectangle(start_x, start_y - height, width, height, COLOR_BLACK);

    // Draw spectrum bars
    for (uint16_t i = 0; i < num_bins; i++) {
        float magnitude = frequency_bins[i];
        if (magnitude > 1.0f) magnitude = 1.0f; // Clamp
        uint16_t bar_height = (uint16_t)(magnitude * height);
        uint16_t x = start_x + i * bin_width;
        uint16_t y = start_y - bar_height;

        tft_draw_rectangle(x, y, bin_width - 2, bar_height, COLOR_RED);
    }
}

// Initialize the graphics module
void graphics_init(void) {
    spi_init();
    tft_init();
    tft_clear_screen(COLOR_BLACK);
}

// -----------------------------------------------------------------------------
// Example Usage within an Infinite Loop (Optional)
// -----------------------------------------------------------------------------

/*
int main(void) {
    // Existing initializations...
    internal_clock();
    enable_ports();
    setup_dma();
    enable_dma();
    init_tim15();
    init_tim7();
    setup_adc();
    init_tim2();
    init_wavetable();
    setup_dac();
    init_tim6();

    // Initialize Graphics
    graphics_init();

    // Example audio buffer and frequency bins
    uint16_t audio_buffer[256];
    float frequency_bins[128];

    while (1) {
        // Assume audio_buffer is filled via DMA
        render_waveform(audio_buffer, 256);

        // Assume frequency_bins is filled via FFT
        render_spectrum(frequency_bins, 128);

        nano_wait(100000); // Small delay to control update rate
    }
}
*/

// -----------------------------------------------------------------------------
// Utility Function: Wait for 'n' nanoseconds
// -----------------------------------------------------------------------------

void nano_wait(unsigned int n) {
    asm volatile(
        " mov r0, %0        \n"
        "repeat:             \n"
        " sub r0, #83        \n" // Adjust cycle count as needed
        " bgt repeat         \n"
        : : "r"(n) : "r0", "cc");
}