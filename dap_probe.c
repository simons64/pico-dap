#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/uart.h"
#include "hardware/dma.h"

#include "dap.pio.h"

static PIO pio_hw_dap;
static uint pio_sm_dap;
static uint offset_dap;
#define GPIO_DAP_CLK 0

static uint dma_channel_burst;
#define DMA_IRQ_TO_USE 0


#define BUFFER_SIZE 64
uint32_t buffer_burst[BUFFER_SIZE] = {0};


int main()
{
    for(uint32_t i=0; i<BUFFER_SIZE; ++i) {
        // if ((i % 2) == 0) buffer_burst[i] = 0xFF;
        // else buffer_burst[i] = 0;
        buffer_burst[i] = i;
    }


    stdio_init_all();

    // setup pio for dap clock
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(&dap_program, &pio_hw_dap, &pio_sm_dap, &offset_dap, GPIO_DAP_CLK, 1, true)) {
        panic("failed to allocate pio for rx");
    }
    dap_program_init(pio_hw_dap, pio_sm_dap, offset_dap, GPIO_DAP_CLK, 10000);
    
    dma_channel_burst = dma_claim_unused_channel(true);
    dma_channel_config dma_config = dma_channel_get_default_config(dma_channel_burst);
    channel_config_set_transfer_data_size(&dma_config, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_config, true);
    channel_config_set_write_increment(&dma_config, false);
    // enable irq for tx
    dma_irqn_set_channel_enabled(DMA_IRQ_TO_USE, dma_channel_burst, true);

    // setup dma to write to pio fifo
    channel_config_set_dreq(&dma_config, pio_get_dreq(pio_hw_dap, pio_sm_dap, true));
    dma_channel_configure(dma_channel_burst, &dma_config, &pio_hw_dap->txf[pio_sm_dap], buffer_burst, BUFFER_SIZE - 1, false); // dma not started
    dma_channel_start(dma_channel_burst);

    // Just wait for dma tx to finish
    dma_channel_wait_for_finish_blocking(dma_channel_burst);

    // Never return from main: the SDK's _exit() locks the core on bkpt and
    // the board then needs BOOTSEL to be flashed again.
    while (true) {
        printf("Hello, cruel world\n");
        sleep_ms(1000);
    }
}
