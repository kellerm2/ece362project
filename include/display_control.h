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
#define PWM_TIMER TIM3
#define PWM_CHANNEL TIM3->CCER |= TIM_CCER_CC1E; // Enable Channel 1
#define PWM_PIN GPIO_PIN_6        // PA6 for TIM3_CH1
#define PWM_GPIO GPIOA
#define PWM_PRESCALER 1
#define PWM_ARR 255

// ----------------------------------------------------------------------------
// Global Variables
// ----------------------------------------------------------------------------



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