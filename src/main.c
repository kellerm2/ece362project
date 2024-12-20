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
#include "commands.h"

void nano_wait(int);
void draw_visualizer_bars();
void init_tim2();
void sdcard_io_high_speed(void);

//=============================================================================
// Part 1: 7-segment display update with DMA
//=============================================================================

#define TFT_WIDTH 240
#define TFT_HEIGHT 320
uint16_t adc_buffer[TFT_WIDTH]; //= {3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,3000,5000,5000,5000,5000,5000,5000,
// 5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,
// 5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,
// 5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,
// 5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,
// 5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,
// 5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000,5000};
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
    DMA1_Channel5->CCR &= ~DMA_CCR_DIR;
    DMA1_Channel5->CCR |= DMA_CCR_MINC;
    DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0;
    DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;
    DMA1_Channel5->CCR |= DMA_CCR_CIRC;
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    
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
ADC1->CFGR1 |= ADC_CFGR1_DMAEN;
}

void scale_adc_values(void) {
    for (int i = 0; i < TFT_WIDTH; i++) {
        float temp = (float) adc_buffer[i] / 4096; 
        scaled_buffer[i] = temp * TFT_HEIGHT;
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
    SPI1->CR1 |= 0b000 << SPI_CR1_BR_Pos;   // Set baud rate divisor to max (000 = fPCLK / 2)
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Software Slave Management and Internal Slave Select
    SPI1->CR2 |= (0b0111 << SPI_CR2_DS_Pos);   // 8-bit data size (default)
    SPI1->CR2 |= SPI_CR2_FRXTH;             // Set FIFO reception threshold to 8-bit
    SPI1->CR2 |= SPI_CR2_TXDMAEN;

    // Enable SPI1
    SPI1->CR1 |= SPI_CR1_SPE;
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
    sdcard_io_high_speed();
}

void sdcard_io_high_speed(){
    SPI1 -> CR1 &= ~SPI_CR1_SPE;

    SPI1 -> CR1 &= ~SPI_CR1_BR;
    SPI1 -> CR1 |= SPI_CR1_BR_0;

    SPI1 -> CR1 |= SPI_CR1_SPE;

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

// void LCD_Setup() {
//     init_lcd_spi();
//     tft_select(0);
//     tft_reset(0);
//     tft_reg_select(0);
//     LCD_Init(tft_reset, tft_select, tft_reg_select);
// }
void setup_dma_interrupt(void) {
    // Enable DMA interrupt for Channel 1 (ADC DMA)
    DMA1_Channel5->CCR |= DMA_CCR_TCIE; // Enable transfer complete interrupt
    NVIC_EnableIRQ(DMA1_Ch4_7_DMA2_Ch3_5_IRQn);  // Enable the interrupt in the NVIC
}

void DMA1_Ch4_7_DMA2_Ch3_5_IRQHandler();
void DMA1_Ch4_7_DMA2_Ch3_5_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF5) {  // Check if transfer complete flag is set
        DMA1->IFCR |= DMA_IFCR_CTCIF5; // Clear the transfer complete flag
        
        // Now that the data is in adc_buffer[], trigger the visualizer
        draw_visualizer_bars();
    }
}

void init_tim15(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    // 48M / 1k = 48k = PSC * ARR
    TIM15->PSC = 9999; // (999 + 1) = 1k
    TIM15->ARR = 47; // (47 + 1) = 48

    TIM15->DIER |= TIM_DIER_UDE;
    // NVIC_EnableIRQ(TIM15_IRQn);
    NVIC_SetPriority(TIM15_IRQn, 0); 
    TIM15->CR1 |= TIM_CR1_CEN;
}

void TIM15_IRQHandler();

void TIM15_IRQHandler() {
    TIM15->SR &= ~TIM_SR_UIF;

    //LCD_DrawFillRectangle(0, 0, 100, 100, BLUE);
    draw_visualizer_bars();
}

void draw_visualizer_bars() {
    int data_length = TFT_WIDTH;
    int bar_width = TFT_WIDTH / data_length;  // Calculate width of each bar
    int max_bar_height = TFT_HEIGHT;         // Max height for scaling
    scale_adc_values();

    for (int i = 0; i < data_length; i++) {
         // // Normalize audio data to fit within the display height
        int bar_height = scaled_buffer[i]; // CHECK THIS??

        // Determine the x and y position for the bar
        int x = i;
        int y = 0; // Bars grow upwards

        // Draw a filled rectangle for the bar
        // if bar_height thresholds, then change the color that is called with fill Rect
        char str[200];
        snprintf(str, sizeof(str), "BH: %d\n", bar_height);
        LCD_DrawString(10, 10, BLACK, WHITE, str, 16, 0);
        if (bar_height < 0.1 * TFT_HEIGHT) LCD_DrawFillRectangle(x, y, x+1, bar_height, BLUE);
        else if (bar_height < 0.2 * TFT_HEIGHT) LCD_DrawFillRectangle(x, y, x+1, bar_height, GREEN);
        else if (bar_height < 0.3 * TFT_HEIGHT) LCD_DrawFillRectangle(x, y, x+1, bar_height, YELLOW);
        else LCD_DrawFillRectangle(x, y, x+1, bar_height, RED);
        nano_wait(1000000);
    }
    LCD_Clear(WHITE);
}

void TIM2_IRQHandler();

void TIM2_IRQHandler() {
    TIM2->SR &= ~TIM_SR_UIF; //acknowledge the interrupt first

    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
        // Wait until the EOC bit is set in the ISR
    }
}

void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    // 48M / 10 = 4.8M = PSC * ARR
    TIM2->PSC = 99; // (9999 + 1) = 10k
    TIM2->ARR = 47; // (479 + 1) = 480

    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;

    NVIC_SetPriority(TIM2_IRQn, 3);

}

//============================================================================
// All the things you need to test your subroutines.
//============================================================================
int main(void) {

    internal_clock();

    init_lcd_spi();

    LCD_Setup(); // should call init_lcd_spi() ??
    // LCD_Clear(GREEN);

    setup_adc();
    setup_dma();
    setup_dma_interrupt();
    init_tim15();
    init_tim2();
    
    //LCD_DrawFillRectangle(0, 0, 200, 300, BLUE);
    //int div = 1;
    while (1)
    {
        //for (int i = 0; i <TFT_WIDTH; i++) {
        //    adc_buffer[i] = adc_buffer[i] / div;
        //}
        //draw_visualizer_bars(); // does this need to be interrupt ????
        //LCD_Clear(MAGENTA);//     nano_wait(1000000000);
        //div *= 2;
    }

    return 0;
}
