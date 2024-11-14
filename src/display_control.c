// display_control.c
// Self-contained display control module for STM32F091 Audio Visualizer project

#include "stm32f0xx.h"
#include <stdint.h>

// ----------------------------------------------------------------------------
// Definitions and Macros
// ----------------------------------------------------------------------------

// Timer Configuration for Refresh Rate
#define DISPLAY_TIMER TIM15
#define DISPLAY_TIMER_IRQ TIM15_IRQn
#define DISPLAY_PRESCALER 1         // Adjust based on desired refresh rate
#define DISPLAY_ARR 300             // Auto-reload value

// PWM Configuration for Brightness Control (Optional)
//#define PWM_TIMER TIM3
//#define PWM_CHANNEL TIM3->CCER |= TIM_CCER_CC1E; // Enable Channel 1
#define PWM_PIN GPIO_PIN_6        // PA6 for TIM3_CH1
#define PWM_GPIO GPIOA
#define PWM_PRESCALER 1
#define PWM_ARR 255

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------

// Flag to indicate that a display update is required
volatile uint8_t display_update_flag = 0;

// Current brightness level (0-255)
volatile uint8_t brightness_level = 255;

// ----------------------------------------------------------------------------
// Function Prototypes
// ----------------------------------------------------------------------------

void display_init(void);
void init_display_timer(void);
void init_pwm_brightness(void);
void TIM15_IRQHandler(void);
void TIM3_IRQHandler(void); // Optional for brightness control
void refresh_display(void);
void adjust_brightness(uint8_t level);

// ----------------------------------------------------------------------------
// Function Implementations
// ----------------------------------------------------------------------------

// Initialize the display control module
void display_init(void) {
    init_display_timer();
}

// Initialize TIM15 for display refresh rate management
void init_display_timer(void) {
    // Enable clock for TIM15
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    
    // Configure TIM15
    DISPLAY_TIMER->PSC = DISPLAY_PRESCALER - 1;  // Prescaler
    DISPLAY_TIMER->ARR = DISPLAY_ARR;            // Auto-reload value
    DISPLAY_TIMER->DIER |= TIM_DIER_UIE;         // Enable update interrupt
    DISPLAY_TIMER->CR1 |= TIM_CR1_CEN;           // Enable TIM15
    
    // Enable TIM15 interrupt in NVIC
    NVIC_EnableIRQ(DISPLAY_TIMER_IRQ);
    NVIC_SetPriority(DISPLAY_TIMER_IRQ, 2);      // Set priority (adjust as needed)
}

// TIM15 Interrupt Handler for Display Refresh
void TIM15_IRQHandler(void) {
    if (DISPLAY_TIMER->SR & TIM_SR_UIF) {
        DISPLAY_TIMER->SR &= ~TIM_SR_UIF;  // Clear update interrupt flag
        display_update_flag = 1;            // Set flag to update display
    }
}

// ----------------------------------------------------------------------------
// Main Loop Integration (Example within main.c)
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
    
    // Initialize Graphics and Display Control
    graphics_init();
    display_init();
    tft_clear_screen(COLOR_BLACK);
    
    // Initialize audio input
    audio_setup();
    
    // Main loop
    while (1) {
        if (display_update_flag) {
            refresh_display();
            display_update_flag = 0;
        }
        
        // Additional application logic
        
        // Optional: Adjust brightness based on user input or ambient conditions
        // adjust_brightness(new_level);
    }
}
*/

// ----------------------------------------------------------------------------
// Utility Functions (Optional)
// ----------------------------------------------------------------------------

// Implement any additional utility functions here as needed.

// ----------------------------------------------------------------------------
// End of `display_control.c`
// ----------------------------------------------------------------------------