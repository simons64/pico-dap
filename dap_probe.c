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

int main()
{
    stdio_init_all();

    // setup pio for dap clock
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(&dap_program, &pio_hw_dap, &pio_sm_dap, &offset_dap, GPIO_DAP_CLK, 1, true)) {
        panic("failed to allocate pio for rx");
    }
    dap_program_init(pio_hw_dap, pio_sm_dap, offset_dap, GPIO_DAP_CLK, 1000);
    

    // Never return from main: the SDK's _exit() locks the core on bkpt and
    // the board then needs BOOTSEL to be flashed again.
    while (true) {
        printf("Hello, cruel world\n");
        sleep_ms(1000);
    }
}
