/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Jan 24 2024
  * @brief   ECE 362 Project
  ******************************************************************************
*/

/**
******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "vanderg0";

/******************************************************************************
*/ 

#include "stm32f0xx.h"
#include <math.h>   // for M_PI
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "lcd.h"

void nano_wait(int);

//=============================================================================
// Part 1: 7-segment display update with DMA
//=============================================================================

#define TFT_WIDTH 240
#define TFT_HEIGHT 320
uint16_t adc_buffer[TFT_WIDTH];
uint16_t scaled_buffer[TFT_WIDTH];
#define MAX_AUDIO_AMPLITUDE 4095 // because we have 12-bit ADC ??

//============================================================================
// setup_dma() + enable_dma()
//============================================================================
void setup_dma(void) {
    
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CMAR = (uint32_t)adc_buffer;
    DMA1_Channel5->CPAR = (uint32_t)&(ADC1->DR);
    DMA1_Channel5->CNDTR = TFT_WIDTH;
    DMA1_Channel5->CCR |= DMA_CCR_DIR;
    DMA1_Channel5->CCR |= DMA_CCR_MINC;
    DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0;
    DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;
    DMA1_Channel5->CCR |= DMA_CCR_CIRC;
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    ADC1->CFGR1 |= ADC_CFGR1_DMAEN;


}


//============================================================================
// setup_adc()
//============================================================================
void setup_adc(void) {

RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
GPIOA->MODER &= ~(0x3 << (1 * 2));
GPIOA->MODER |= (0x3 << (1 * 2));
RCC->APB2ENR |= RCC_APB2ENR_ADCEN;
RCC->CR2 |= RCC_CR2_HSI14ON;
while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0){}
ADC1->CR |= ADC_CR_ADEN;
while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {}
ADC1->CHSELR |= ADC_CHSELR_CHSEL1;
while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {}
    

}

void scale_adc_values(void) {
    for (int i = 0; i < TFT_HEIGHT; i++) {
        scaled_buffer[i] = (adc_buffer[i] * TFT_HEIGHT) / 4096;
    }
}

// void spi_init(void) {
    
//     RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//     RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

//     GPIOA->MODER &= ~((3 << (5 * 2)) | (3 << (7 * 2)));
//     GPIOA->MODER |= ((2 << (5 * 2)) | (2 << (7 * 2)));
//     GPIOA->AFR[0] &= ~((15 << (5 * 4)) | (15 << (7 * 4)));

//     SPI1->CR1 &= ~SPI_CR1_SPE;

//     SPI1->CR1 = 0;
//     SPI1->CR1 |= SPI_CR1_MSTR;
//     SPI1->CR1 |= SPI_CR1_BR;

//     SPI1->CR2 &= ~SPI_CR2_DS;
//     SPI1->CR2 |= (0x7 << SPI_CR2_DS_Pos);

//     SPI1->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP;
//     SPI1->CR2 |= SPI_CR2_TXDMAEN;

//     SPI1->CR1 |= SPI_CR1_SPE;

// }

void init_spi1_slow() {
    // Enable the clocks for GPIOB and SPI1
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  // Enable GPIOB clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;   // Enable SPI1 clock

    SPI1->CR1 &= ~SPI_CR1_SPE;

    // Configure GPIOB pins for SPI (PB3 = SCK, PB4 = MISO, PB5 = MOSI)
    GPIOB->MODER &= ~((0b11 << (3 * 2)) | (0b11 << (4 * 2)) | (0b11 << (5 * 2))); // Clear mode
    GPIOB->MODER |= ((0b10 << (3 * 2)) | (0b10 << (4 * 2)) | (0b10 << (5 * 2)));       // Set to Alternate Function (AF)
    
    GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL3_Msk | GPIO_AFRL_AFSEL4_Msk | GPIO_AFRL_AFSEL5_Msk); // Clear AFR
    GPIOB->AFR[0] |= (0 << GPIO_AFRL_AFSEL3_Pos) | (0 << GPIO_AFRL_AFSEL4_Pos) | (0 << GPIO_AFRL_AFSEL5_Pos); // Set AF5 (SPI1)

    GPIOB->OSPEEDR |= ((0b11 << (3 * 2)) | (0b11 << (4 * 2)) | (0b11 << (5 * 2))); // High-speed GPIO
    // GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3_Msk | GPIO_PUPDR_PUPD4_Msk | GPIO_PUPDR_PUPD5_Msk); // No pull-up/pull-down

    // // Reset SPI1 to default state
    // SPI1->CR1 = 0;
    // SPI1->CR2 = 0;

    // Configure SPI1 settings
    SPI1->CR1 |= SPI_CR1_MSTR;              // Master mode
    SPI1->CR1 |= SPI_CR1_BR;                // Set baud rate divisor to max (111 = fPCLK / 256)
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Software Slave Management and Internal Slave Select
    SPI1->CR2 |= (0b0111 << SPI_CR2_DS_Pos);   // 8-bit data size (default)
    SPI1->CR2 |= SPI_CR2_FRXTH;             // Set FIFO reception threshold to 8-bit

    // Enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
}

void enable_sdcard() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  // Enable GPIOB clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;   // Enable SPI1 clock

    GPIOB->BRR = 1 << 2;
}

void disable_sdcard() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  // Enable GPIOB clock
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;   // Enable SPI1 clock

    GPIOB->BSRR = 1 << 2;
}

void init_sdcard_io() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;  // Enable GPIOB clock
    init_spi1_slow();
    GPIOB->MODER &= ~(0b11 << (2 * 2)); //Configures PB2 as an output.
    GPIOB->MODER |= (0b01 << (2 * 2));
    disable_sdcard();
}

void sdcard_io_high_speed() {
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    SPI1->CR1 &= ~SPI_CR1_SPE; // Disable the SPI1 channel.
    SPI1->CR1 |= 0b001 << SPI_CR1_BR_Pos; // Set the SPI1 Baud Rate register so that the clock rate is 12 MHz. (You may need to set this lower if your SD card does not reliably work at this rate.)
    SPI1->CR1 |= SPI_CR1_SPE; // Re-enable the SPI1 channel.
}

void init_lcd_spi() {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; 
    // Configure PB8, PB11, and PB14 as GPIO outputs. Don't forget the clock to GPIOB.
    GPIOB->MODER &= ~(0b11 << (8 * 2));
    GPIOB->MODER &= ~(0b11 << (11 * 2));
    GPIOB->MODER &= ~(0b11 << (14 * 2));
    GPIOB->MODER |= (0b01 << (8 * 2));
    GPIOB->MODER |= (0b01 << (11 * 2));
    GPIOB->MODER |= (0b01 << (14 * 2));
    init_spi1_slow(); //Call init_spi1_slow() to configure SPI1.
    sdcard_io_high_speed(); //Call sdcard_io_high_speed() to make SPI1 fast
}

// uint8_t spi_transfer(uint8_t data) {
//     // Send data
//     SPI1->DR = data;
//     // Wait for the transmit buffer to be empty
//     while (!(SPI1->SR & SPI_SR_TXE));
//     // Wait for a received byte
//     while (!(SPI1->SR & SPI_SR_RXNE));
//     // Return the received data
//     return SPI1->DR;
// }

void draw_visualizer_bars() {
    int data_length = TFT_WIDTH;
    int bar_width = TFT_WIDTH / data_length;  // Calculate width of each bar
    int max_bar_height = TFT_HEIGHT;         // Max height for scaling

    for (int i = 0; i < data_length; i++) {
        scale_adc_values(); // // Normalize audio data to fit within the display height
        int bar_height = scaled_buffer[i]; // CHECK THIS??

        // Determine the x and y position for the bar
        int x = i * bar_width;
        int y = TFT_HEIGHT - bar_height; // Bars grow upwards

        // Draw a filled rectangle for the bar
        // if bar_height thresholds, then change the color that is called with fill Rect
        LCD_DrawFillRectangle(x, y, bar_width, bar_height, WHITE); // NEED LIBRARY FOR THIS
    }
}


//============================================================================
// All the things you need to test your subroutines.
//============================================================================
int main(void) {
    internal_clock();
    setup_adc();
    setup_dma();
    ADC1->CR |= ADC_CR_ADSTART;
    //void init_lcd_spi();

    //LCD_Setup();
    //LCD_Clear(BLUE);
    while (1)
    {
        enable_sdcard();
        draw_visualizer_bars(); // not done yet- look into tft libraries
        disable_sdcard();
        nano_wait(1000000000);
    }

}
