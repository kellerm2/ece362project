// audio_input.c
// Self-contained audio input module for STM32F091 Audio Visualizer project

#include "stm32f0xx.h"
#include <stdint.h>
#include <math.h>

// ----------------------------------------------------------------------------
// Definitions and Macros
// ----------------------------------------------------------------------------

// Audio Configuration
#define AUDIO_ADC ADC1
#define AUDIO_PIN GPIO_PIN_1       // PA1 (ADC Channel 1)
#define AUDIO_GPIO GPIOA
#define AUDIO_DMA_CHANNEL DMA1_Channel1
#define AUDIO_BUFFER_SIZE 1024     // Adjust as needed

// Buffer to store ADC samples
volatile uint16_t audio_buffer[AUDIO_BUFFER_SIZE];
volatile uint16_t buffer_index = 0;

// ----------------------------------------------------------------------------
// Function Prototypes
// ----------------------------------------------------------------------------

void audio_init(void);
void adc_dma_init(void);
void process_audio_data(void);
float calculate_amplitude(uint16_t* buffer, uint16_t size);
void compute_fft(float* input, float* output, uint16_t size);

// ----------------------------------------------------------------------------
// Function Implementations
// ----------------------------------------------------------------------------

// Initialize GPIO for Audio Input
void audio_init(void) {
    // Enable clocks for GPIOA and ADC
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    
    // Configure PA1 as analog input
    AUDIO_GPIO->MODER |= GPIO_MODER_MODER1;  // Set both bits for analog mode
    
    // Enable HSI14 (14 MHz internal RC oscillator) for ADC
    RCC->CR2 |= RCC_CR2_HSI14ON;
    while(!(RCC->CR2 & RCC_CR2_HSI14RDY));
    
    // Configure ADC1
    if (ADC1->CR & ADC_CR_ADEN) {
        ADC1->CR &= ~ADC_CR_ADEN;  // Disable ADC if already enabled
    }
    
    // Wait for ADC to be disabled
    while(ADC1->CR & ADC_CR_ADEN);
    
    // Configure ADC settings
    ADC1->CR = 0;                  // Reset CR register
    ADC1->CFGR1 &= ~ADC_CFGR1_RES;  // Clear resolution bits
    ADC1->CFGR1 |= ADC_CFGR1_RES_1;  // Set 12-bit resolution
    
    // Enable continuous conversion and DMA
    ADC1->CFGR1 |= ADC_CFGR1_CONT;   // Continuous mode
    ADC1->CFGR1 |= ADC_CFGR1_DMAEN;  // Enable DMA
    ADC1->CFGR1 |= ADC_CFGR1_DMACFG; // Circular DMA mode
    
    // Select channel 1 (PA1)
    ADC1->CHSELR = 0;                 // Clear channel selection
    ADC1->CHSELR |= ADC_CHSELR_CHSEL1; // Select channel 1
    
    // Calibrate ADC
    ADC1->CR |= ADC_CR_ADCAL;
    while(ADC1->CR & ADC_CR_ADCAL);
    
    // Enable ADC
    ADC1->CR |= ADC_CR_ADEN;
    while(!(ADC1->ISR & ADC_ISR_ADRDY));
    
    // Start ADC conversions
    ADC1->CR |= ADC_CR_ADSTART;
}

// Initialize DMA for ADC Data Transfer
void adc_dma_init(void) {
    // Enable clock for DMA1
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    
    // Disable DMA channel before configuration
    DMA1_Channel1->CCR &= ~DMA_CCR_EN;
    
    // Configure DMA1 Channel1 for ADC1
    DMA1_Channel1->CCR = 0; // Reset configuration
    DMA1_Channel1->CCR |= DMA_CCR_MINC |    // Memory increment mode
                         DMA_CCR_MSIZE_0 |   // Memory size 16-bit
                         DMA_CCR_PSIZE_0 |   // Peripheral size 16-bit
                         DMA_CCR_CIRC |      // Circular mode
                         DMA_CCR_TCIE;       // Transfer complete interrupt
    
    // Set peripheral address (ADC data register)
    DMA1_Channel1->CPAR = (uint32_t)&(ADC1->DR);
    
    // Set memory address (audio_buffer)
    DMA1_Channel1->CMAR = (uint32_t)audio_buffer;
    
    // Set number of data items
    DMA1_Channel1->CNDTR = AUDIO_BUFFER_SIZE;
    
    // Clear any pending flags
    DMA1->IFCR = DMA_IFCR_CTCIF1;
    
    // Enable DMA1 Channel1
    DMA1_Channel1->CCR |= DMA_CCR_EN;
    
    // Enable DMA1 Channel1 interrupt in NVIC
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
}


// DMA1 Channel1 Interrupt Handler
void DMA1_Channel1_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        // Clear the DMA transfer complete flag
        DMA1->IFCR |= DMA_IFCR_CTCIF1;
        
        // Process the audio data
        process_audio_data();
    }
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

// Calculate the amplitude of the audio signal
float calculate_amplitude(uint16_t* buffer, uint16_t size) {
    float sum = 0.0f;
    for(uint16_t i = 0; i < size; i++) {
        float sample = (float)(buffer[i] - 2048) / 2048.0f; // Normalize to [-1, 1]
        sum += fabsf(sample);
    }
    return sum / size;
}

// Placeholder Function for FFT Processing
// Implement a proper FFT algorithm or integrate a library as needed
void compute_fft(float* input, float* output, uint16_t size) {
    // Implement FFT computation here or call an external library
    // This is a placeholder and does not perform actual FFT
    for(uint16_t i = 0; i < size / 2; i++) {
        output[i] = 0.0f;
    }
}

// ----------------------------------------------------------------------------
// Initialization Function
// ----------------------------------------------------------------------------

void audio_setup(void) {
    audio_init();
    adc_dma_init();
}

// ----------------------------------------------------------------------------
// Example Usage within `main.c`
// ----------------------------------------------------------------------------

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
    
    // Initialize Audio Input
    audio_setup();
    
    // Initialize Graphics
    graphics_init();
    tft_clear_screen(COLOR_BLACK);
    
    // Main loop
    while(1) {
        // Rendering and visualization handled in graphics.c via DMA data
        // The process_audio_data() function can trigger updates or set parameters for graphics
    }
}
*/

// ----------------------------------------------------------------------------
// Utility Function: Example Implementations (Optional)
// ----------------------------------------------------------------------------

// Implement any additional utility functions here as needed.

// ----------------------------------------------------------------------------
// End of `audio_input.c`
// ----------------------------------------------------------------------------