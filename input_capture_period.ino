/*
 * Distributed under the 2-clause BSD licence (See LICENCE.TXT file at the
 * repository root).
 *
 * Copyright (c) 2017 Tilo Nitzsche.  All rights reserved.
 *
 * https://github.com/tni/teensy-samples
 */

#include <WProgram.h>
#include <DMAChannel.h>
#include <array>

// This code works with Kinetis K (Teensy 3.x)
// Measure period between 2 rising edges of signal. (Or alternately, measure pulse width.)
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

    // measure period between 2 rising edges
    // channel 0, capture rising edge; FTM_CSC_MSA --> continous capture mode
    FTM1_C0SC = FTM_CSC_ELSA | FTM_CSC_MSA;
    // channel 1, capture rising edge; FTM_CSC_MSA --> continous capture mode
    FTM1_C1SC = FTM_CSC_ELSA | FTM_CSC_MSA | FTM_CSC_MSA;

    // to measure pulse width, use the following config for FTM1_C1SC:
    // channel 1, capture falling edge and trigger DMA, FTM_CSC_MSA --> continous capture mode
    //FTM1_C1SC = FTM_CSC_CHIE | FTM_CSC_DMA| FTM_CSC_ELSB | FTM_CSC_MSA;

    pinMode(out_pin, OUTPUT);

    serial.begin(115200);
    delay(2000);
}

static uint32_t iteration_cnt = 0;

void loop() {
    if(iteration_cnt % 2 == 0) generateDoublePulse<20, 80>();
    else generateDoublePulse<40, 160>();

    // The two FTM value registers are synchronized. FTM1_C0V must be read before
    // FTM1_C1V. The FTM1_C1V read will reflect the correct capture that happened
    // immeditely after the FTM1_C0V capture.
    // (FTM1_C0V, FTM1_C1V) are continously updated with the current captured edge
    // pair by the FTM timer and reflect the most current measurement.
    uint16_t first_edge_time = FTM1_C0V;
    uint16_t second_edge_time = FTM1_C1V;

    serial.printf("Iteration: %u\n", iteration_cnt);
    serial.printf("first_edge_time: %u second_edge_time: %u\n", first_edge_time, second_edge_time);
    serial.printf("Period: %i\n\n", uint16_t(second_edge_time - first_edge_time));

    iteration_cnt++;
    delay(1000);
}
