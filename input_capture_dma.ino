/*
 * Distributed under the 2-clause BSD licence (See LICENCE.TXT file at the
 * repository root).
 *
 * Copyright (c) 2016 Tilo Nitzsche.  All rights reserved.
 *
 * https://github.com/tni/teensy-samples
 */

#include <WProgram.h>
#include <DMAChannel.h>
#include <array>

// This code works with Kinetis K (Teensy 3.x)
// Both edges of the signal at Pin 3 are being captured and transferred
// via DMA into a ring buffer.
// Pin 21 is configured as output and connected to pin 3. It is used
// to generate a test signal.

const uint8_t out_pin = 21;
const uint8_t capture_pin = 3;


#define GPIO_BITBAND_ADDR(reg, bit) (((uint32_t)&(reg) - 0x40000000) * 32 + (bit) * 4 + 0x42000000)
#define GPIO_BITBAND_PTR(reg, bit) ((volatile uint32_t *)GPIO_BITBAND_ADDR((reg), (bit)))
#define OUT_PIN_BITBAND_REG GPIO_BITBAND_PTR(CORE_PIN21_PORTREG, CORE_PIN21_BIT)

// The Arduino preprocessor is buggy and screws up template functions outside namespaces.
namespace {
template<size_t N> __attribute__((always_inline)) void generateNopsHelper() {
    generateNopsHelper<N-1>();
    asm volatile ("nop");
}
template<> __attribute__((always_inline)) void generateNopsHelper<0>() {}

template<size_t N> __attribute__((always_inline)) FASTRUN void generateNops() {
    generateNopsHelper<N>();
}

template<size_t high_time, size_t low_time>
__attribute__((noinline)) FASTRUN void generateDoublePulse() {
    // force 0 and 1 to be loaded in registers, so that we can get precise pulses
    uint32_t v_low = 0;
    uint32_t v_high = 1;
    volatile uint32_t dummy = v_low; dummy = v_high;
    asm volatile ("" ::: "memory");
    (void) *OUT_PIN_BITBAND_REG; // load OUT_PIN_BITBAND_REG into register
    noInterrupts();
    *OUT_PIN_BITBAND_REG = v_high;
    generateNops<high_time>();
    *OUT_PIN_BITBAND_REG = v_low;
    generateNops<low_time>();
    *OUT_PIN_BITBAND_REG = v_high;
    generateNops<high_time>();
    *OUT_PIN_BITBAND_REG = v_low;
    generateNops<low_time>();
    interrupts();
}

} // namespace


struct PulseCapture {
    volatile uint16_t timer_at_rising;
    volatile uint16_t timer_at_falling;
};

static DMAChannel dma;
// use ring buffer of pairs of [timer at rising edge, timer at falling edge]
const size_t buffer_capture_cnt = 16;
static std::array<PulseCapture, buffer_capture_cnt> buffer;
const size_t buffer_byte_cnt = sizeof(buffer[0]) * buffer.size();

size_t getCurrentDmaIndex() {
    auto dest_addr = static_cast<decltype(buffer.data())>(dma.destinationAddress());
    return size_t(dest_addr - buffer.data());
}

auto& serial = Serial;

void setup() {
    // setup FTM1 in free running mode, counting from 0 - 0xFFFF
    FTM1_SC = 0;
    FTM1_CNT = 0;
    FTM1_MOD = 0xFFFF;
    // pin 3 config to FTM mode; pin 3 is connected to FTM1 channel 0
    CORE_PIN3_CONFIG = PORT_PCR_MUX(3);
    // set FTM1 clock source to system clock; FTM_SC_PS(0): divide clock by 1
    FTM1_SC = (FTM_SC_CLKS(1) | FTM_SC_PS(0));

    // need to unprotect FTM1_COMBINE register
    FTM1_MODE = FTM_MODE_WPDIS;
    // set FTM1 CH0 to dual edge capture, paired channels
    FTM1_COMBINE = FTM_COMBINE_DECAP0 | FTM_COMBINE_DECAPEN0;
    FTM1_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;

    // DMA transfers use a minor loop which we use to read FTM1_C0V and FTM1_C1V and
    // a major loop which we use to iterate over our capture ring buffer.

    // We read 2 bytes from FTM1_C0V and 2 bytes from FTM1_C1V
    dma.TCD->SADDR = &FTM1_C0V;
    dma.TCD->ATTR_SRC = 1; // 16-bit read from source (timer value is 16 bits)
    dma.TCD->SOFF = 8;     // increment source address by 8 (switch from reading FTM1_C0V to FTM1_C1V)
    // transfer 4 bytes total per minor loop (2 from FTM1_C0V + 2 from FTM1_C1V); go back to
    // FTM1_C0V after minor loop (-16 bytes).
    dma.TCD->NBYTES_MLOFFYES = DMA_TCD_NBYTES_SMLOE | DMA_TCD_NBYTES_MLOFFYES_NBYTES(4) |
                               DMA_TCD_NBYTES_MLOFFYES_MLOFF(-16);
    // source addr adjustment at major loop end (the minor loop adjustment doesn't get executed)
    dma.TCD->SLAST = -16;
    dma.TCD->DADDR = buffer.data();
    dma.TCD->DOFF = 2;     // 2 bytes destination increment
    dma.TCD->ATTR_DST = 1; // 16-bit write to dest
    // set major loop count
    dma.TCD->BITER = buffer_capture_cnt;
    dma.TCD->CITER = buffer_capture_cnt;
    // use buffer as ring buffer, go back 'buffer_byte_cnt' after buffer_capture_cnt captures
    // have been done
    dma.TCD->DLASTSGA = -buffer_byte_cnt;
    // disable channel linking, keep DMA running continously
    dma.TCD->CSR = 0;

    // trigger a DMA transfer whenever a new pulse has been captured
    dma.triggerAtHardwareEvent(DMAMUX_SOURCE_FTM1_CH1);
    dma.enable();

    // channel 0, capture rising edge; FTM_CSC_MSA --> continous capture mode
    FTM1_C0SC = FTM_CSC_ELSA | FTM_CSC_MSA;
    // channel 1, capture falling edge and trigger DMA, FTM_CSC_MSA --> continous capture mode
    FTM1_C1SC = FTM_CSC_CHIE | FTM_CSC_DMA| FTM_CSC_ELSB | FTM_CSC_MSA;


    pinMode(out_pin, OUTPUT);

    serial.begin(115200);
    delay(2000);
}

static uint32_t iteration_cnt = 0;

void loop() {
    if(iteration_cnt % 2 == 0) generateDoublePulse<20, 80>();
    else generateDoublePulse<80, 20>();

    PulseCapture p1, p2;
    size_t buffer_index = getCurrentDmaIndex();
    p1 = buffer[(buffer_index - 2) % buffer.size()];
    p2 = buffer[(buffer_index - 1) % buffer.size()];

    serial.printf("Iteration: %u Buffer index: %u\n", iteration_cnt, buffer_index);
    serial.printf("Buffer addr: %x DMA dest: %x DMA src: %x\n",
                  (uint32_t) buffer.data(),
                  (uint32_t) dma.destinationAddress(),
                  (uint32_t) dma.sourceAddress());
    serial.printf("Pulse 1: %u %u\n", p1.timer_at_rising, p1.timer_at_falling);
    serial.printf("Pulse 2: %u %u\n", p2.timer_at_rising, p2.timer_at_falling);
    serial.printf("Pulse width, 1: %i 2: %i\n",
                  uint16_t(p1.timer_at_falling - p1.timer_at_rising),
                  uint16_t(p2.timer_at_falling - p2.timer_at_rising));
    serial.printf("Pulse to pulse: %i\n\n", uint16_t(p2.timer_at_rising - p1.timer_at_rising));

    iteration_cnt++;
    delay(500);
}
