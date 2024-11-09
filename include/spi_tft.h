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