#include "stm32f0xx.h"
#include <math.h>   // for M_PI
#include <stdint.h>
#include <stdio.h>

#include "spi_tft.h"
#include "audio_input.h"
#include "display_control.h"
#include "graphics.h"


void enable_ports(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOB->MODER &= 0xFFEAAAAA;
    GPIOB->MODER |= 0x00155555;
    GPIOC->MODER &= ~0xFF00;
    GPIOC->MODER |= 0x5500;
    GPIOC->OTYPER &= 0xFF0F;
    GPIOC->OTYPER |= 0x00F0;
    GPIOC->PUPDR &= 0xFFFFFFAA;
    GPIOC->PUPDR |= 0x00000055;
}

//============================================================================
// setup_dma() + enable_dma()
//============================================================================
void setup_dma(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    DMA1_Channel1->CMAR = (uint32_t)(&(ADC1->DR));
    DMA1_Channel1->CPAR = (uint32_t)(audio_buffer); // & is address of
    DMA1_Channel1->CNDTR = (uint32_t) AUDIO_BUFFER_SIZE;
    DMA1_Channel1->CCR |= DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0 | DMA_CCR_CIRC;

    // Clear any pending flags
    DMA1->IFCR = DMA_IFCR_CTCIF1; // needed to call process_audio

    DMA1_Channel5->CCR |= DMA_CCR_EN;
    // Enable DMA1 Channel1 interrupt in NVIC
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
}


//============================================================================
// init_tim15()
//============================================================================
void init_tim15(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    // 48M / 1k = 48k = PSC * ARR
    TIM15->PSC = 999; // (999 + 1) = 1k
    TIM15->ARR = 47; // (47 + 1) = 48

    TIM15->DIER |= TIM_DIER_UDE;
    TIM15->CR1 |= TIM_CR1_CEN;
}

//============================================================================
// setup_adc()
//============================================================================
void setup_adc(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= 0x0000000C; // set PA1 to 11 for analog mode

    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    RCC->CR2 |= RCC_CR2_HSI14ON;

    while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) {
        // Wait for the 14 MHz clock to be ready.
    } 

    ADC1->CR |= ADC_CR_ADEN;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
        // Wait for the ADC to be ready
    } 

    ADC1->CHSELR |= ADC_CHSELR_CHSEL1;
    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {
        // Wait for the ADC to be ready
    }
}
//============================================================================
// Varables for boxcar averaging.
//============================================================================
#define BCSIZE 32
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;
//============================================================================
// Timer 2 ISR
//============================================================================
// Write the Timer 2 ISR here.  Be sure to give it the right name.
void TIM2_IRQHandler();

void TIM2_IRQHandler() {
    TIM2->SR &= ~TIM_SR_UIF; //acknowledge the interrupt first

    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {
        // Wait until the EOC bit is set in the ISR
    }

    bcsum -= boxcar[bcn];
    bcsum += boxcar[bcn] = ADC1->DR;
    bcn += 1;
    if (bcn >= BCSIZE)
        bcn = 0;
    volume = bcsum / BCSIZE; // !!!! what is volume here, do we need this
}


//============================================================================
// init_tim2()
//============================================================================
void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    // 48M / 10 = 4.8M = PSC * ARR
    TIM2->PSC = 9999; // (9999 + 1) = 10k
    TIM2->ARR = 479; // (479 + 1) = 480

    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;

    NVIC_SetPriority(TIM2_IRQn, 3); // sets lower priority for running againt ISR TIM6
}

// Process Audio Data (e.g., Calculate Amplitude, Perform FFT)
void process_audio_data(void) {
    // Example: Calculate amplitude
    float amplitude = calculate_amplitude((uint16_t*)audio_buffer, AUDIO_BUFFER_SIZE);
    
    // TODO: Map amplitude to visualization parameters or trigger events
    
    // Example: Perform FFT (requires additional implementation)
    
    float fft_input[AUDIO_BUFFER_SIZE];
    float fft_output[AUDIO_BUFFER_SIZE / 2];
    for(uint16_t i = 0; i < AUDIO_BUFFER_SIZE; i++) {
        fft_input[i] = (audio_buffer[i] - 2048) / 2048.0f; // Normalize ADC data (assuming 12-bit ADC)
    }
    compute_fft(fft_input, fft_output, AUDIO_BUFFER_SIZE);
    
    // TODO: Use fft_output for frequency spectrum visualization
    
}

void DMA1_Channel1_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        // Clear the DMA transfer complete flag
        DMA1->IFCR |= DMA_IFCR_CTCIF1;
        
        // Process the audio data
        process_audio_data();
    }
}

// MAIN
int main() {
    // enable
    internal_clock();
    enable_ports(); // not set up
    setup_dma();
    // enable_dma();
    init_tim15(); // used for DMA
    setup_adc();
    init_tim2(); // used for ADC

    // Initialize Audio Input
    audio_setup();
    
    // Initialize Graphics
    graphics_init();
    display_init();
    tft_clear_screen(COLOR_BLACK);
    
    // Example audio buffer and frequency bins
    uint16_t audio_buffer[256];
    float frequency_bins[128];
    uint16_t FFT_BIN_COUNT = 128; // need for refresh in display_ctrl.c
    // AUDIO_BUFFER_SIZE defined in audio_input.h

    // SPI TFT
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

    while(1) {
        if (display_update_flag) {
            refresh_display();
            display_update_flag = 0;
        }

        nano_wait(100000); // Small delay to control update rate
        // Rendering and visualization handled in graphics.c via DMA data
        // The process_audio_data() function can trigger updates or set parameters for graphics
    }

    return 0;
}

