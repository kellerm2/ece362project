#include "stm32f0xx.h"
#include <math.h>   // for M_PI
#include <stdint.h>
#include <stdio.h>

#include "spi_tft.h"
#include "audio_input.h"
#include "dma_handler.h"
#include "support.h"
#include "syscalls.h"
#include "display_control.h"
#include "graphics.h"

