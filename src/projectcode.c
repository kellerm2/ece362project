#include "stm32f0xx.h"
#include <math.h>   // for M_PI
#include <stdint.h>
#include <stdio.h>

// #include "spi_tft.h"
// #include "audio_input.h"
// #include "dma_handler.h"
// #include "support.h"
// #include "syscalls.h"
// #include "display_control.h"
// #include "graphics.h"

// SETUP HERE - ENABLING STUFF

// AUDIO INPUT HERE

// DMA HANDLER HERE

// SPI TFT HERE

// GRAPHICS HERE

// DISPLAY CTRL HERE
// SETUP HERE - ENABLING STUFF
#include "spi_tft.h"
#include "audio_input.h"
#include "dma_handler.h"
#include "support.h"
#include "syscalls.h"
#include "display_control.h"
#include "graphics.h"

// ENABLE GPIO AND STUFF

// AUDIO INPUT HERE


// DMA HANDLER HERE


// SPI TFT HERE


// GRAPHICS HERE


// DISPLAY CTRL HERE


// MAIN
int main() {
    // enable
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
    display_init();
    tft_clear_screen(COLOR_BLACK);
    
    // Example audio buffer and frequency bins
    uint16_t audio_buffer[256];
    float frequency_bins[128];

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

    // Fill the screen with blue
    tft_clear_screen(COLOR_BLUE);

    while(1) {
        // if (display_update_flag) {
        //     refresh_display();
        //     display_update_flag = 0;
        // }
        // Assume audio_buffer is filled via DMA
        render_waveform(audio_buffer, 256);

        // Assume frequency_bins is filled via FFT
        render_spectrum(frequency_bins, 128);

        nano_wait(100000); // Small delay to control update rate
        // Rendering and visualization handled in graphics.c via DMA data
        // The process_audio_data() function can trigger updates or set parameters for graphics
    }

    return 0;
}

