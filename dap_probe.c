#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "dap.pio.h"

#define GPIO_DAP_DATA 1
#define GPIO_DAP_CLK  0

// The PIO program clocks a fixed number of command bits out per transaction
// (the three 6-bit loops in dap.pio). Frames longer than this are refused by
// dap_request() instead of being driven truncated.
#define DAP_TX_BITS 18

// The state machine clocks DAP_RX_BITS reply bits back in, which fills two
// FIFO words. Both are drained by DMA.
#define DAP_RX_WORDS 2

// arglen value meaning "this command has no argument" (an 18-bit frame).
#define DAP_NO_ARG 0x3F

#define DAP_REQUEST_TIMEOUT_MS 100

// ---------------------------------------------------------------------------
// Frame format: start(1) | cmd(5) | arglen(6) | arg(arglen) | crc6(6).
// Payload fields are clocked LSB-first, the CRC is clocked MSB-first, and the
// CRC covers the start bit plus every cmd/len/arg bit before it.
// ---------------------------------------------------------------------------

// CRC-6/ITU (poly x^6+x+1, init 0x01) over the first `nbits` bits, in the order
// they were clocked. Same polynomial and bit order as the TriCore DAP analyzer.
static uint8_t dap_crc6(uint64_t bits, unsigned nbits)
{
    uint8_t crc = 0x01;
    for (unsigned i = 0; i < nbits; i++) {
        uint8_t feedback = (uint8_t)(((crc >> 5) & 1u) ^ ((bits >> i) & 1u));
        crc = (uint8_t)((crc << 1) & 0x3F);
        if (feedback)
            crc ^= 0x03;
    }
    return crc;
}

// The 6 CRC bits arrive MSB-first: crc = (crc << 1) | bit. Reading them with a
// little-endian mask returns the bit reversal.
static uint8_t dap_crc6_field(uint64_t bits, unsigned first_bit)
{
    uint8_t crc = 0;
    for (unsigned i = 0; i < 6; i++)
        crc = (uint8_t)((crc << 1) | ((bits >> (first_bit + i)) & 1u));
    return crc;
}

// Inverse of dap_crc6_field: place a CRC MSB-first at `first_bit`, which is how
// it has to sit in the word the state machine shifts out.
static uint64_t dap_put_crc6(uint8_t crc, unsigned first_bit)
{
    uint64_t bits = 0;
    for (unsigned i = 0; i < 6; i++)
        bits |= (uint64_t)((crc >> (5 - i)) & 1u) << (first_bit + i);
    return bits;
}

typedef struct {
    unsigned start;
    uint8_t cmd;
    uint8_t arglen;
    uint64_t arg;
    uint8_t crc;
    uint8_t crc_expected;
    bool crc_ok;
} dap_frame_t;

static const char *dap_command_name(uint8_t cmd)
{
    switch (cmd) {
    case 0x02: return "jtag_swap_dr";
    case 0x08: return "write_register";
    case 0x09: return "write_block";
    case 0x0A: return "read_block";
    case 0x10: return "sync";
    case 0x11: return "dapisc";
    case 0x13: return "jtag_set_ir";
    case 0x15: return "jtag_reset";
    case 0x1A: return "read_register";
    case 0x1C: return "set_io_client";
    default:   return "unknown";
    }
}

// Split a bit stream into CMD / LEN / ARG. Bit 0 is the start bit, i.e. bit i of
// `bits` is the i-th bit clocked.
static dap_frame_t dap_parse_frame(uint64_t bits)
{
    dap_frame_t f = {0};
    f.start = (unsigned)(bits & 1u);
    f.cmd = (uint8_t)((bits >> 1) & 0x1Fu);
    f.arglen = (uint8_t)((bits >> 6) & 0x3Fu);

    unsigned narg = (f.arglen == DAP_NO_ARG) ? 0u : f.arglen;
    if (narg > 46) // 12 header bits + arg + 6 CRC bits have to fit in 64
        narg = 46;
    for (unsigned i = 0; i < narg; i++)
        f.arg |= (uint64_t)((bits >> (12 + i)) & 1u) << i;

    f.crc = dap_crc6_field(bits, 12 + narg);
    f.crc_expected = dap_crc6(bits, 12 + narg);
    f.crc_ok = (f.crc == f.crc_expected);
    return f;
}

// Total clocks a frame occupies on the wire.
static unsigned dap_frame_len(unsigned arglen)
{
    return 12 + (arglen == DAP_NO_ARG ? 0u : arglen) + 6;
}

// Pack CMD / LEN / ARG into the bit stream the state machine shifts out:
// start bit at bit 0, then cmd, len, arg (LSB-first each) and the CRC on top.
static uint64_t dap_build_frame(uint8_t cmd, unsigned arglen, uint64_t arg)
{
    unsigned narg = (arglen == DAP_NO_ARG) ? 0u : arglen;
    uint64_t f = 1u;
    f |= (uint64_t)(cmd & 0x1Fu) << 1;
    f |= (uint64_t)(arglen & 0x3Fu) << 6;
    if (narg != 0 && narg <= 46)
        f |= (arg & ((1ull << narg) - 1u)) << 12;

    unsigned covered = 12 + narg;
    f |= dap_put_crc6(dap_crc6(f, covered), covered);
    return f;
}

// How many payload bits a command's reply carries, or 0 when it carries none.
static unsigned dap_reply_bits(const dap_frame_t *cmd)
{
    switch (cmd->cmd) {
    case 0x10: // sync: a fixed 32-bit pattern
        return 32;
    case 0x1A: { // read_register: size encoded in the argument
        unsigned size_code = (unsigned)((cmd->arg >> 4) & 7u);
        return size_code == 4 ? 16u : size_code == 5 ? 32u : size_code == 6 ? 64u : 0u;
    }
    default:
        return 0;
    }
}

// ---------------------------------------------------------------------------
// Hardware: one PIO state machine, a TX channel feeding command words and an RX
// channel draining reply words. Nothing outside this section touches buffers.
// ---------------------------------------------------------------------------

typedef enum {
    DAP_OK = 0,
    DAP_ERR_TX_BITS,   // frame longer than the PIO clocks out, nothing sent
    DAP_ERR_NO_REPLY,  // no reply within DAP_REQUEST_TIMEOUT_MS
} dap_status_t;

typedef struct {
    dap_status_t status;
    unsigned tx_bits;
    uint64_t tx; // frame sent, bit 0 = start bit
    uint64_t rx; // bits received, bit 0 = start bit
} dap_reply_t;

static PIO pio_dap;
static uint sm_dap;
static uint offset_dap;
static uint dma_tx;
static uint dma_rx;
static uint32_t tx_word;
static uint32_t rx_words[DAP_RX_WORDS];

static void dap_init(unsigned freq_hz)
{
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(&dap_program, &pio_dap, &sm_dap,
                                                          &offset_dap, GPIO_DAP_DATA, 2, true))
        panic("failed to allocate pio for dap");
    dap_program_init(pio_dap, sm_dap, offset_dap, GPIO_DAP_DATA, GPIO_DAP_CLK, freq_hz);

    // TX: one command word into the TX FIFO, DREQ paced by the state machine.
    dma_tx = dma_claim_unused_channel(true);
    dma_channel_config tx = dma_channel_get_default_config(dma_tx);
    channel_config_set_transfer_data_size(&tx, DMA_SIZE_32);
    channel_config_set_read_increment(&tx, true);
    channel_config_set_write_increment(&tx, false);
    channel_config_set_dreq(&tx, pio_get_dreq(pio_dap, sm_dap, true));
    dma_channel_configure(dma_tx, &tx, &pio_dap->txf[sm_dap], &tx_word,
                          dma_encode_transfer_count(1), false);

    // RX: drain the reply words as the state machine pushes them, so `push
    // iffull` never stalls the clock mid-transaction.
    dma_rx = dma_claim_unused_channel(true);
    dma_channel_config rx = dma_channel_get_default_config(dma_rx);
    channel_config_set_transfer_data_size(&rx, DMA_SIZE_32);
    channel_config_set_read_increment(&rx, false);
    channel_config_set_write_increment(&rx, true);
    channel_config_set_dreq(&rx, pio_get_dreq(pio_dap, sm_dap, false));
    dma_channel_configure(dma_rx, &rx, rx_words, &pio_dap->rxf[sm_dap],
                          dma_encode_transfer_count(DAP_RX_WORDS), false);
}

// Send one frame and collect the reply. This is the whole buffer lifetime: the
// caller supplies cmd/len/arg, nothing else.
static dap_reply_t dap_request(uint8_t cmd, unsigned arglen, uint64_t arg)
{
    dap_reply_t r = {0};
    r.tx_bits = dap_frame_len(arglen);
    r.tx = dap_build_frame(cmd, arglen, arg);

    if (r.tx_bits > DAP_TX_BITS) {
        r.status = DAP_ERR_TX_BITS;
        return r;
    }

    tx_word = (uint32_t)r.tx;

    // Arm RX first: it has to be waiting on the RX DREQ when the state machine
    // pushes the first reply word.
    if (dma_channel_is_busy(dma_rx))
        dma_channel_abort(dma_rx);
    dma_channel_transfer_to_buffer_now(dma_rx, rx_words,
                                       dma_encode_transfer_count(DAP_RX_WORDS));
    dma_channel_transfer_from_buffer_now(dma_tx, &tx_word,
                                         dma_encode_transfer_count(1));

    absolute_time_t deadline = make_timeout_time_ms(DAP_REQUEST_TIMEOUT_MS);
    while (dma_channel_is_busy(dma_rx)) {
        if (time_reached(deadline)) {
            dma_channel_abort(dma_rx);
            r.status = DAP_ERR_NO_REPLY;
            return r;
        }
        tight_loop_contents();
    }

    r.rx = ((uint64_t)rx_words[1] << 32) | rx_words[0];
    r.status = DAP_OK;
    return r;
}

// sync: no argument, answered with a fixed 32-bit pattern plus CRC.
static dap_reply_t dap_sync(void)
{
    return dap_request(0x10, DAP_NO_ARG, 0);
}

static void dap_print_reply(const dap_reply_t *r)
{
    dap_frame_t tx = dap_parse_frame(r->tx);
    printf("tx raw: 0x%llX (%u bits)\n", (unsigned long long)r->tx, r->tx_bits);
    printf("tx cmd: 0x%02X (%s) len: 0x%02X arg: 0x%llX crc: 0x%02X (%s)\n",
           tx.cmd, dap_command_name(tx.cmd), tx.arglen,
           (unsigned long long)tx.arg, tx.crc, tx.crc_ok ? "OK" : "MISMATCH");

    if (r->status == DAP_ERR_TX_BITS) {
        printf("rx: not sent, frame is %u bits and the PIO clocks %u\n",
               r->tx_bits, DAP_TX_BITS);
        return;
    }
    if (r->status == DAP_ERR_NO_REPLY) {
        printf("rx: no reply within %d ms\n", DAP_REQUEST_TIMEOUT_MS);
        return;
    }

    printf("rx raw: 0x%016llX (word0 0x%08X word1 0x%08X)\n",
           (unsigned long long)r->rx, (unsigned)r->rx, (unsigned)(r->rx >> 32));

    dap_frame_t rx = dap_parse_frame(r->rx);
    printf("rx start: %u\n", rx.start);
    printf("rx cmd: 0x%02X (%s)\n", rx.cmd, dap_command_name(rx.cmd));
    printf("rx len: 0x%02X (%u bits)\n", rx.arglen,
           rx.arglen == DAP_NO_ARG ? 0u : (unsigned)rx.arglen);
    printf("rx arg: 0x%llX\n", (unsigned long long)rx.arg);
    printf("rx crc: 0x%02X expected: 0x%02X (%s)\n", rx.crc, rx.crc_expected,
           rx.crc_ok ? "OK" : "MISMATCH");

    unsigned rbits = dap_reply_bits(&tx);
    if (rbits != 0 && rbits <= 62) {
        uint64_t mask = (1ull << rbits) - 1u;
        uint64_t payload = (r->rx >> 1) & mask;
        uint8_t crc = dap_crc6_field(r->rx, 1 + rbits);
        uint8_t crc_expected = dap_crc6(r->rx, 1 + rbits);
        printf("reply: %u bits payload: 0x%llX crc: 0x%02X expected: 0x%02X (%s)\n",
               rbits, (unsigned long long)payload, crc, crc_expected,
               crc == crc_expected ? "OK" : "MISMATCH");
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000); // give a host time to attach to USB stdio

    dap_init(100000);

    dap_reply_t r = dap_sync();
    dap_print_reply(&r);

    // Never return from main: the SDK's _exit() locks the core on bkpt and
    // the board then needs BOOTSEL to be flashed again.
    int k = 0;
    while (true) {
        printf("running %d   \r", k);
        k++;
        sleep_ms(1000);
    }
}
