

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