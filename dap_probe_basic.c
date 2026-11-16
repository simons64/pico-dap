#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"

#include "dap.pio.h"

#define GPIO_DAP_DATA 1
#define GPIO_DAP_CLK  0

// The command stream is word 0 = metadata (two counts) followed by the frame,
// which spans one word per 32 bits: a 66-bit dapisc frame needs three data
// words, so 96 bits are available for the frame.
#define DAP_TX_DATA_WORDS 3
#define DAP_TX_WORDS      (1 + DAP_TX_DATA_WORDS)

// The state machine clocks DAP_RX_BITS reply bits back in, which fills two
// FIFO words. Both are drained by DMA.
#define DAP_RX_WORDS 2

// arglen value meaning "this command has no argument".
#define DAP_NO_ARG 0x3F

// Bits a frame occupies besides its argument: start(1) + cmd(5) + arglen(6) +
// crc6(6). Same arithmetic as TricoreDapAnalyzer's `12 + arg_len + 6`.
#define DAP_FRAME_BITS 18

// Argument bits in a frame. arglen 0x3f is the "no argument" marker, not 63
// bits - TricoreDapAnalyzer does `len == 63 ? 0 : len` for exactly this reason,
// and using it raw would make every argument-less frame 63 bits too long.
#define DAP_ARG_BITS(arglen) ((arglen) == DAP_NO_ARG ? 0u : (uint32_t)(arglen))

#define DAP_REQUEST_TIMEOUT_MS 100

// ---------------------------------------------------------------------------
// Frame format: start(1) | cmd(5) | arglen(6) | arg(arglen) | crc6(6).
// Payload fields are clocked LSB-first, the CRC is clocked MSB-first, and the
// CRC covers the start bit plus every cmd/len/arg bit before it.
// ---------------------------------------------------------------------------

// CRC-6/ITU (poly x^6+x+1, init 0x01) over the first `nbits` bits, in the order
// they were clocked. Same polynomial and bit order as the TriCore DAP analyzer.
static uint8_t dap_crc6(uint64_t bits, uint32_t nbits)
{
    uint8_t crc = 0x01;
    for (uint32_t i = 0; i < nbits; i++) {
        uint8_t feedback = (uint8_t)(((crc >> 5) & 1u) ^ ((bits >> i) & 1u));
        crc = (uint8_t)((crc << 1) & 0x3F);
        if (feedback)
            crc ^= 0x03;
    }
    return crc;
}

// The 6 CRC bits arrive MSB-first: crc = (crc << 1) | bit. Reading them with a
// little-endian mask returns the bit reversal.
static uint8_t dap_crc6_field(uint64_t bits, uint32_t first_bit)
{
    uint8_t crc = 0;
    for (uint32_t i = 0; i < 6; i++)
        crc = (uint8_t)((crc << 1) | ((bits >> (first_bit + i)) & 1u));
    return crc;
}

// Inverse of dap_crc6_field: place a CRC MSB-first at `first_bit`, which is how
// it has to sit in the word the state machine shifts out.
static uint64_t dap_put_crc6(uint8_t crc, uint32_t first_bit)
{
    uint64_t bits = 0;
    for (uint32_t i = 0; i < 6; i++)
        bits |= (uint64_t)((crc >> (5 - i)) & 1u) << (first_bit + i);
    return bits;
}

typedef struct {
    uint32_t start;
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
    f.start = (uint32_t)(bits & 1u);
    f.cmd = (uint8_t)((bits >> 1) & 0x1Fu);
    f.arglen = (uint8_t)((bits >> 6) & 0x3Fu);

    uint32_t narg = (f.arglen == DAP_NO_ARG) ? 0u : f.arglen;
    if (narg > 46) // 12 header bits + arg + 6 CRC bits have to fit in 64
        narg = 46;
    for (uint32_t i = 0; i < narg; i++)
        f.arg |= (uint64_t)((bits >> (12 + i)) & 1u) << i;

    f.crc = dap_crc6_field(bits, 12 + narg);
    f.crc_expected = dap_crc6(bits, 12 + narg);
    f.crc_ok = (f.crc == f.crc_expected);
    return f;
}

// ---------------------------------------------------------------------------
// The command frame spans several words, so it has no single-register form: a
// 66-bit dapisc frame does not fit in a uint64_t. Bits are written and read at a
// bit position across the data words, LSB of word 0 first.
// ---------------------------------------------------------------------------

static void dap_bits_put(uint32_t *words, uint32_t pos, uint64_t value, uint32_t nbits)
{
    for (uint32_t i = 0; i < nbits; i++)
        if ((value >> i) & 1u)
            words[(pos + i) / 32u] |= 1u << ((pos + i) % 32u);
}

static uint64_t dap_bits_get(const uint32_t *words, uint32_t pos, uint32_t nbits)
{
    uint64_t v = 0;
    for (uint32_t i = 0; i < nbits && i < 64; i++)
        v |= (uint64_t)((words[(pos + i) / 32u] >> ((pos + i) % 32u)) & 1u) << i;
    return v;
}

// The 6 CRC bits are clocked MSB-first, so the field is read big-endian.
static uint8_t dap_bits_get_crc_field(const uint32_t *words, uint32_t pos)
{
    uint8_t crc = 0;
    for (uint32_t i = 0; i < 6; i++)
        crc = (uint8_t)((crc << 1) | ((words[(pos + i) / 32u] >> ((pos + i) % 32u)) & 1u));
    return crc;
}

static uint8_t dap_crc6_words(const uint32_t *words, uint32_t pos, uint32_t nbits)
{
    uint8_t crc = 0x01;
    for (uint32_t i = 0; i < nbits; i++) {
        uint8_t bit = (uint8_t)((words[(pos + i) / 32u] >> ((pos + i) % 32u)) & 1u);
        uint8_t feedback = (uint8_t)(((crc >> 5) & 1u) ^ bit);
        crc = (uint8_t)((crc << 1) & 0x3F);
        if (feedback)
            crc ^= 0x03;
    }
    return crc;
}

// Pack start | cmd | arglen | arg | crc6 into `data` (DAP_TX_DATA_WORDS words),
// returning the frame length in bits, or 0 when it does not fit.
static uint32_t dap_build_frame(uint32_t data[DAP_TX_DATA_WORDS], uint8_t cmd,
                                uint32_t arglen, uint64_t arg)
{
    uint32_t narg = DAP_ARG_BITS(arglen);
    uint32_t nbits = DAP_FRAME_BITS + narg;

    if (nbits > 32u * DAP_TX_DATA_WORDS)
        return 0;

    for (uint32_t i = 0; i < DAP_TX_DATA_WORDS; i++)
        data[i] = 0;

    dap_bits_put(data, 0, 1u, 1);                // start bit
    dap_bits_put(data, 1, cmd & 0x1Fu, 5);       // cmd
    dap_bits_put(data, 6, arglen & 0x3Fu, 6);    // arglen
    if (narg != 0)
        dap_bits_put(data, 12, arg, narg);       // arg

    uint8_t crc = dap_crc6_words(data, 0, 12u + narg);
    for (uint32_t i = 0; i < 6; i++)             // crc, MSB-first
        if ((crc >> (5 - i)) & 1u)
            dap_bits_put(data, 12u + narg + i, 1u, 1);

    return nbits;
}

// How many payload bits a command's reply carries, or 0 when it carries none.
static uint32_t dap_reply_bits(const dap_frame_t *cmd)
{
    switch (cmd->cmd) {
    case 0x10: // sync: a fixed 32-bit pattern
        return 32;
    case 0x11: // dapisc: 2 bytes, sometimes with status bits OR'ed in
        return 16;
    case 0x02: // jtag_swap_dr: 32 bits
        return 32;
    case 0x1A: { // read_register: size encoded in the argument
        uint32_t size_code = (uint32_t)((cmd->arg >> 4) & 7u);
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
    uint32_t tx_len;
    uint32_t rx_len;
    uint64_t tx; // frame sent, bit 0 = start bit
    uint64_t rx; // bits received, bit 0 = start bit
} dap_reply_t;

static PIO pio_dap;
static uint sm_dap;
static uint offset_dap;
static uint dma_tx;
static uint dma_rx;
static uint32_t tx_words[DAP_TX_WORDS];
static uint32_t rx_words[DAP_RX_WORDS];

static void dap_init(uint32_t freq_hz)
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
    dma_channel_configure(dma_tx, &tx, &pio_dap->txf[sm_dap], &tx_words,
                          dma_encode_transfer_count(DAP_TX_WORDS), false);

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
static dap_reply_t dap_request(uint8_t cmd, uint32_t arglen, uint64_t arg, uint32_t reslen)
{
    dap_reply_t r = {0};

    // Frame: DAP_FRAME_BITS plus its argument, written into the data words
    // (word 0 is the metadata). Three data words hold 96 bits, so dapisc's 66
    // bits fit; arglen 0x3f means "no argument", handled by DAP_ARG_BITS.
    r.tx_len = dap_build_frame(&tx_words[1], cmd, arglen, arg);
    if (r.tx_len == 0) {
        r.status = DAP_ERR_TX_BITS;
        return r;
    }
    r.tx = ((uint64_t)tx_words[2] << 32) | tx_words[1]; // low 64 bits, for display

    // Word 0 field [13:7] is the reply window: reslen plus the reply's own
    // start/CRC overhead. sync passes 21 (39 bits), dapisc 5 (23 bits). The PIO's
    // receive loop is `jmp y--` without a guard, so it clocks one more than this.
    uint32_t reply_clocks = reslen + DAP_FRAME_BITS;
    uint32_t clocked = reply_clocks + 1u;
    tx_words[0] = ((reply_clocks & 0x7Fu) << 7) | r.tx_len;

    // Drain exactly the words the receive loop will push: ceil(clocked/32). A
    // short reply (dapisc is 24 clocks) fills one word only, and arming the RX
    // DMA for two would wait forever.
    uint32_t nrx = (clocked + 31u) / 32u;
    if (nrx > DAP_RX_WORDS)
        nrx = DAP_RX_WORDS;

    // The state machine pulls one metadata word plus one word per 32 frame bits,
    // and stops there: queue exactly that many, or the leftover word sits in the
    // TX FIFO and is taken as the metadata of the next request (X and Y then
    // decode as garbage and the reply never arrives). This is why two consecutive
    // dap_sync() calls failed while re-running dap_init() "fixed" it - dap_init()
    // clears the FIFOs.
    unsigned ntx = 1u + (r.tx_len + 31u) / 32u;
    if (ntx > DAP_TX_WORDS) {
        r.status = DAP_ERR_TX_BITS;
        return r;
    }

    // The state machine pushes one word more than the reply needs whenever the
    // clocked bit count is a multiple of 32 (two full words plus its trailing
    // `push`), so a word can be left in the RX FIFO. Wait until it is parked at
    // its metadata `pull` (address 1 - the program starts with `set pindirs, 1`)
    // and drop whatever is there.
    for (uint32_t spins = 0; pio_sm_get_pc(pio_dap, sm_dap) != offset_dap + 1; spins++) {
        if (spins > 1000000u) // never spin forever if the SM is wedged
            break;
        tight_loop_contents();
    }
    while (!pio_sm_is_rx_fifo_empty(pio_dap, sm_dap))
        (void)pio_sm_get(pio_dap, sm_dap);

    // Arm RX first: it has to be waiting on the RX DREQ when the state machine
    // pushes the first reply word.
    if (dma_channel_is_busy(dma_rx))
        dma_channel_abort(dma_rx);
    dma_channel_transfer_to_buffer_now(dma_rx, rx_words,
                                       dma_encode_transfer_count(nrx));
    dma_channel_transfer_from_buffer_now(dma_tx, tx_words,
                                         dma_encode_transfer_count(ntx));

    absolute_time_t deadline = make_timeout_time_ms(DAP_REQUEST_TIMEOUT_MS);
    while (dma_channel_is_busy(dma_rx)) {
        if (time_reached(deadline)) {
            dma_channel_abort(dma_rx);
            r.status = DAP_ERR_NO_REPLY;
            return r;
        }
        tight_loop_contents();
    }

    // Combine the reply words. The ISR shifts right, so the word that holds the
    // tail of the reply keeps those bits in its *top*: 40 clocked bits means
    // word1 carries only 8 bits, up at positions 24..31, and must be shifted
    // down before it can be concatenated.
    r.rx_len = reply_clocks + 1u; // the PIO's `jmp y--` runs y+1 times
    r.rx = 0;
    for (uint32_t i = 0; i < DAP_RX_WORDS; i++) {
        uint32_t first = 32u * i;
        if (r.rx_len <= first)
            break;
        uint32_t in_word = r.rx_len - first;
        uint32_t word = rx_words[i];
        if (in_word < 32u)
            word >>= (32u - in_word);
        r.rx |= (uint64_t)word << first;
    }
    r.status = DAP_OK;
    return r;
}

// sync: no argument, answered with a fixed 32-bit pattern plus CRC.
static dap_reply_t dap_sync(void)
{
    return dap_request(0x10, DAP_NO_ARG, 0, 21);
}

static void dap_print_reply(const dap_reply_t *r)
{
    // Decode the command from the command words, not from r->tx: a frame longer
    // than 64 bits (dapisc is 66) has its CRC field past the end of a uint64_t.
    const uint32_t *tw = &tx_words[1];
    uint8_t cmd = (uint8_t)dap_bits_get(tw, 1, 5);
    uint32_t len = (uint32_t)dap_bits_get(tw, 6, 6);
    uint32_t narg = DAP_ARG_BITS(len);
    uint64_t arg = dap_bits_get(tw, 12, narg);
    uint8_t crc_field = dap_bits_get_crc_field(tw, 12u + narg);
    uint8_t crc_calc = dap_crc6_words(tw, 0, 12u + narg);

    printf("tx: %u bits words 0x%08X 0x%08X 0x%08X\n", r->tx_len,
           tx_words[1], tx_words[2], tx_words[3]);
    printf("tx cmd: 0x%02X (%s) len: 0x%02X arg: 0x%llX crc: 0x%02X (%s)\n",
           cmd, dap_command_name(cmd), len, (uint64_t)arg, crc_field,
           crc_field == crc_calc ? "OK" : "MISMATCH");

    dap_frame_t tx = dap_parse_frame(r->tx); // for dap_reply_bits() below

    // if (r->status == DAP_ERR_TX_BITS) {
    //     printf("rx: not sent, frame is %u bits and the PIO clocks %u\n",
    //            r->tx_bits, DAP_TX_BITS);
    //     return;
    // }
    if (r->status == DAP_ERR_NO_REPLY) {
        printf("rx: no reply within %d ms\n", DAP_REQUEST_TIMEOUT_MS);
        return;
    }

    printf("rx raw: 0x%016llX (word0 0x%08X word1 0x%08X)\n",
           (uint64_t)r->rx, (uint32_t)r->rx, (uint32_t)(r->rx >> 32));

    dap_frame_t rx = dap_parse_frame(r->rx);
    printf("rx start: %u\n", rx.start);
    printf("rx cmd: 0x%02X (%s)\n", rx.cmd, dap_command_name(rx.cmd));
    printf("rx len: 0x%02X (%u bits)\n", rx.arglen,
           rx.arglen == DAP_NO_ARG ? 0u : (uint32_t)rx.arglen);
    printf("rx arg: 0x%llX\n", (uint64_t)rx.arg);
    printf("rx crc: 0x%02X expected: 0x%02X (%s)\n", rx.crc, rx.crc_expected,
           rx.crc_ok ? "OK" : "MISMATCH");

    uint32_t rbits = dap_reply_bits(&tx);
    if (rbits != 0 && rbits <= 62) {
        uint64_t mask = (1ull << rbits) - 1u;
        uint64_t payload = (r->rx >> 1) & mask;
        uint8_t crc = dap_crc6_field(r->rx, 1 + rbits);
        uint8_t crc_expected = dap_crc6(r->rx, 1 + rbits);
        printf("reply: %u bits payload: 0x%llX crc: 0x%02X expected: 0x%02X (%s)\n",
               rbits, (uint64_t)payload, crc, crc_expected,
               crc == crc_expected ? "OK" : "MISMATCH");
    }
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000); // give a host time to attach to USB stdio

    // 3 MHz DAP clock (the divider is now derived from 4 SM cycles per bit, so
    // this is the frequency actually on the wire). dapisc was measured to answer
    // at 3 MHz and not at 200 kHz.
    dap_init(1000000);

    // Two consecutive syncs on one init: the second one used to come back shifted
    // by 32 bits because the first left a word in the RX FIFO.
    dap_reply_t r = dap_sync();
    dap_print_reply(&r);

    dap_reply_t r2 = dap_sync();
    dap_print_reply(&r2);

    // batch.dap_dapisc(48, 0x4ABBAF530F00)
    // batch.exec()
    dap_reply_t r3 = dap_request(17, 48, 0x4ABBAF530F00, 5);
    dap_print_reply(&r3);

    // dap_reply_t r3 = dap_request(21, 0, 0); // batch.dap_telegram(21, 0, None, 3)  
    // dap_print_reply(&r3);        // jtag_reset

    // batch.exec()
    // batch.dap_telegram(28, 2, 1, 3)             # io client 1
    // batch.exec()
    // first = status()
    // second = status()
    // batch.dap_write_ojconf(0x3)
    // batch.exec()

    // Never return from main: the SDK's _exit() locks the core on bkpt and
    // the board then needs BOOTSEL to be flashed again.
    int k = 0;
    while (true) {
        printf("running %d   \r", k);
        k++;
        sleep_ms(1000);
    }
}
