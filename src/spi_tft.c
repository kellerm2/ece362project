// spi_tft.c
// Self-contained SPI communication module for STM32F091 Audio Visualizer project

#include "stm32f0xx.h"
#include <stdint.h>

// ----------------------------------------------------------------------------
// Definitions and Macros
// ----------------------------------------------------------------------------

// TFT LCD Specifications
#define TFT_WIDTH  240
#define TFT_HEIGHT 320

// SPI Configuration Constants
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

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------

// Current drawing color
volatile uint16_t current_color = COLOR_WHITE;

// ----------------------------------------------------------------------------
// Function Prototypes
// ----------------------------------------------------------------------------

void spi_tft_init(void);
void spi_send_byte(uint8_t data);
void tft_reset(void);
void tft_command(uint8_t cmd);
void tft_data(uint8_t data);
void tft_init(void);
void tft_set_address_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void tft_draw_pixel(uint16_t x, uint16_t y, uint16_t color);
void tft_clear_screen(uint16_t color);
void spi_send_data_block(uint8_t* data, uint16_t length);

// ----------------------------------------------------------------------------
// Function Implementations
// ----------------------------------------------------------------------------

// Initializes the SPI interface for communication with the TFT LCD
void spi_tft_init(void) {
    // Enable clocks for GPIOA and SPI1
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // Configure GPIO pins for SPI1: PA5 (SCK), PA7 (MOSI), PA4 (CS), PA3 (DC), PA2 (RST)
    GPIOA->MODER &= ~(GPIO_MODER_MODER5_Msk | GPIO_MODER_MODER7_Msk |
                      GPIO_MODER_MODER4_Msk | GPIO_MODER_MODER3_Msk |
                      GPIO_MODER_MODER2_Msk);
    GPIOA->MODER |= (2 << GPIO_MODER_MODER5_Pos) | // SCK - Alternate Function
                     (2 << GPIO_MODER_MODER7_Pos) | // MOSI - Alternate Function
                     (1 << GPIO_MODER_MODER4_Pos) | // CS - General Output
                     (1 << GPIO_MODER_MODER3_Pos) | // DC - General Output
                     (1 << GPIO_MODER_MODER2_Pos);  // RST - General Output

    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL5_Msk | GPIO_AFRL_AFRL7_Msk);
    GPIOA->AFR[0] |= (TFT_SPI_AF << GPIO_AFRL_AFRL5_Pos) | // SCK
                     (TFT_SPI_AF << GPIO_AFRL_AFRL7_Pos);  // MOSI

    // Configure output type as push-pull
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_5 | GPIO_OTYPER_OT_7 |
                       GPIO_OTYPER_OT_4 | GPIO_OTYPER_OT_3 |
                       GPIO_OTYPER_OT_2);

    // Configure speed to high
    GPIOA->OSPEEDR |= (3 << GPIO_OSPEEDR_OSPEEDR5_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEEDR7_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEEDR4_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEEDR3_Pos) |
                      (3 << GPIO_OSPEEDR_OSPEEDR2_Pos);
    
    // No pull-up, no pull-down
    GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR5_Msk | GPIO_PUPDR_PUPDR7_Msk |
                      GPIO_PUPDR_PUPDR4_Msk | GPIO_PUPDR_PUPDR3_Msk |
                      GPIO_PUPDR_PUPDR2_Msk);

    // Initialize SPI1
    TFT_SPI->CR1 = 0; // Reset configuration
    TFT_SPI->CR1 |= SPI_CR1_MSTR;             // Master mode
    TFT_SPI->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Software slave management
    TFT_SPI->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1; // Baud rate control (f_PCLK/8)
    TFT_SPI->CR1 |= SPI_CR1_SPE;              // Enable SPI
}

// Sends a single byte of data over the SPI interface to the TFT
// SPI1: Transmits a single byte to the TFT LCD and waits for the transmission to complete
void spi_send_byte(uint8_t data) {
    // Wait until TX buffer is empty
    while (!(TFT_SPI->SR & SPI_SR_TXE));
    TFT_SPI->DR = data;
    // Wait until transmission complete
    while (!(TFT_SPI->SR & SPI_SR_RXNE));
    // Read data to clear RX buffer
    (void)TFT_SPI->DR;
}

// Reset the TFT LCD by toggling the reset pin
void tft_reset(void) {
    // Configure RST pin as output (already done in spi_tft_init)
    // Reset sequence
    GPIOA->BSRR = GPIO_BSRR_BR_2; // RST low
    for (volatile int i = 0; i < 100000; i++);
    GPIOA->BSRR = GPIO_BSRR_BS_2; // RST high
    for (volatile int i = 0; i < 100000; i++);
}

// Sends a command byte to the TFT LCD to configure settings or execute a specific action
// GPIOA: Uses PA3 as the data/command (DC) pin and PA4 as the chip select (CS) pin to signal a command.
// SPI1: Sends the command byte to the TFT.
// Sets DC low to indicate a command.
// Lowers CS to start transmission, sends the command via SPI, then raises CS to complete the transaction.
void tft_command(uint8_t cmd) {
    // Set DC low for command
    GPIOA->BSRR = GPIO_BSRR_BR_3; // DC low
    // Set CS low
    GPIOA->BSRR = GPIO_BSRR_BR_4; // CS low
    // Send command byte
    spi_send_byte(cmd);
    // Set CS high
    GPIOA->BSRR = GPIO_BSRR_BS_4; // CS high
}

// Sends data (as opposed to a command) to the TFT LCD
// GPIOA: Uses PA3 (DC) for setting data mode and PA4 (CS) for enabling/disabling SPI transmission.
// SPI1: Transmits the data byte to the TFT
// Sets DC high to indicate data mode.
// Lowers CS, sends the data via SPI, then raises CS.
void tft_data(uint8_t data) {
    // Set DC high for data
    GPIOA->BSRR = GPIO_BSRR_BS_3; // DC high
    // Set CS low
    GPIOA->BSRR = GPIO_BSRR_BR_4; // CS low
    // Send data byte
    spi_send_byte(data);
    // Set CS high
    GPIOA->BSRR = GPIO_BSRR_BS_4; // CS high
}

// Initialize the TFT LCD
// PI1 and GPIOA: Sends initialization commands and data to set up the display’s properties
void tft_init(void) {
    tft_reset();

    tft_command(0x01);             // Software Reset
    for (volatile int i = 0; i < 50000; i++); // Delay
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
    tft_data(0x08);                // Landscape mode

    tft_command(0x3A);             // COLMOD: Pixel Format Set
    tft_data(0x05);                // 16-bit/pixel

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
    for (volatile int i = 0; i < 50000; i++); // Delay
    tft_command(0x2C);             // Memory Write
}

// Set the address window for pixel operations
// Sends commands to set the column (0x2A) and row (0x2B) addresses.
// Defines the upper left (x0, y0) and lower right (x1, y1) corners of the area to be modified.
// Prepares the TFT to receive pixel data within this window
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
// Calls tft_set_address_window() to set the pixel location.
// Sends the color data (in RGB565 format) to draw the pixel
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
    // Set DC high for data
    GPIOA->BSRR = GPIO_BSRR_BS_3; // DC high
    // Set CS low
    GPIOA->BSRR = GPIO_BSRR_BR_4; // CS low

    // Send Memory Write command
    spi_send_byte(0x2C);

    // Send color data for the entire screen
    for (uint32_t i = 0; i < TFT_WIDTH * TFT_HEIGHT; i++) {
        spi_send_byte(color >> 8);
        spi_send_byte(color & 0xFF);
    }

    // Set CS high
    GPIOA->BSRR = GPIO_BSRR_BS_4; // CS high
}

// Send a block of data via SPI (useful for fast pixel streaming)
// (e.g., drawing lines or large filled areas)
void spi_send_data_block(uint8_t* data, uint16_t length) {
    // Set DC high for data
    GPIOA->BSRR = GPIO_BSRR_BS_3; // DC high
    // Set CS low
    GPIOA->BSRR = GPIO_BSRR_BR_4; // CS low

    for (uint16_t i = 0; i < length; i++) {
        spi_send_byte(data[i]);
    }

    // Set CS high
    GPIOA->BSRR = GPIO_BSRR_BS_4; // CS high
}

// ----------------------------------------------------------------------------
// Example Usage within an Application (Optional)
// ----------------------------------------------------------------------------

/*
int main(void) {
    // Initialize SPI and TFT
    spi_tft_init();
    tft_init();
    tft_clear_screen(COLOR_BLACK);

    // Draw a red pixel at (10, 10)
    tft_draw_pixel(10, 10, COLOR_RED);

    // Draw a filled green rectangle
    for (uint16_t y = 50; y < 150; y++) {
        for (uint16_t x = 60; x < 180; x++) {
            tft_draw_pixel(x, y, COLOR_GREEN);
        }
    }

    // Fill the screen with blue
    tft_clear_screen(COLOR_BLUE);

    while (1) {
        // Main loop can handle other tasks or enter low-power mode
    }
}
*/

// ----------------------------------------------------------------------------
// Utility Function: Additional Delays (If Needed)
// ----------------------------------------------------------------------------

// You can include additional delay functions here if precise timing is required.
// However, it's recommended to use hardware timers for timing-critical operations.