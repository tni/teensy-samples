/*
 * Distributed under the 2-clause BSD licence (See LICENCE.TXT file at the
 * repository root).
 *
 * Copyright (c) 2017 Tilo Nitzsche.  All rights reserved.
 *
 * https://github.com/tni/teensy-samples
 */

// Example for Teensy 3.x DAC output using the FIFO.
// Precise, jitter-free timing is achieved, using the PDB to trigger
// loading of new values from the FIFO. The FIFO is filled using DMA.
//
// The FIFO is not really a FIFO (there is no support for a push or pop operation),
// but rather a ringbuffer. There is an index to the currently used entry.
//
// DMA transfers can be triggered by the FIFO index being at 0 and
// the FIFO watermark. This code uses these triggers to transfer 4 entries
// at a time. The lower half of the FIFO (entries 0 - 3) is updated when
// the FIFO index reaches 4 (the watermark), the upper half (entries 4 - 7)
// is updated when the FIFO index wraps to 0.
//
// The FIFO DMA destination is set up using the DMA modulo feature,
// wrapping the DMA destination pointer at 8 FIFO entries.
//
// While the FIFO theoretically has 16 entries, they can't be utilized since
// the watermark pointer is limited. So only 8 FIFO entries are used.


#include <array>
#include <DMAChannel.h>

const size_t buffer_alignment = 8;
std::array<uint16_t, 512> buffer __attribute__ ((aligned (buffer_alignment))) DMAMEM;
constexpr size_t buffer_byte_count = sizeof(buffer);
static_assert(buffer_byte_count % buffer_alignment == 0, "DMA buffer end not properly aligned");
static_assert(buffer.size() % 8 == 0, "Buffer size must be multiple of DAC buffer.");

void initBuffer() {
    for(size_t i = 0; i < buffer.size(); i += 4) {
        buffer[i+0] = 0;
        buffer[i+1] = 1000;
        buffer[i+2] = 2000;
        buffer[i+3] = 3000;
    }
}

DMAChannel dma;

void setup() {
    Serial.begin(9600);
    delay(2000);
    Serial.println("PDB DAC sample. Starting...");

    initBuffer();
    SIM_SCGC2 |= SIM_SCGC2_DAC0;  // enable DAC clock
    DAC0_C0 |= DAC_C0_DACEN;      // enable DAC
    DAC0_C0 |= DAC_C0_DACRFS;     // use 3.3V VDDA as reference voltage
    DAC0_C0 |= DAC_C0_DACBWIEN;   // enable DMA trigger at watermark
    DAC0_C0 |= DAC_C0_DACBTIEN;   // enable DMA trigger at 0
    DAC0_C1 = DAC_C1_DACBFWM(2) | // watermark for DMA trigger
                                  // --> DMA triggered when DAC buffer index is 4
              DAC_C1_DMAEN      | // enable DMA
              DAC_C1_DACBFEN;     // enable DAC buffer

    DAC0_C2 = DAC_C2_DACBFUP(7);  // set buffer size to 8

    DAC0_SR &= ~(DAC_SR_DACBFWMF); // clear watermark flag
    DAC0_SR &= ~(DAC_SR_DACBFRTF); // clear top pos flag
    DAC0_SR &= ~(DAC_SR_DACBFRBF); // clear bottom pos flag

    // Init DAC FIFO with the last 8 buffer elements. This makes setting up the circular
    // DMA transfer easier.
    using aliased_uint16 = uint16_t __attribute__((__may_alias__));
    using aliased_uint16_vptr = volatile aliased_uint16*;
    for(size_t i = 0; i < 8; i++) {
        ((aliased_uint16_vptr) &DAC0_DAT0L)[i] = buffer[buffer.size() - 8 + i];
    }

    // The modulo feature of the DMA controller is used. The destination
    // pointer wraps at +16 bytes.
    dma.destinationCircular(((volatile uint16_t*) &DAC0_DAT0L), 16);

    dma.TCD->SADDR = buffer.data();
    dma.TCD->SOFF = 2;
    dma.TCD->ATTR_SRC = 1;
    dma.TCD->NBYTES = 8;
    dma.TCD->SLAST = -buffer_byte_count;
    dma.TCD->BITER = buffer_byte_count / 8;
    dma.TCD->CITER = buffer_byte_count / 8;

    dma.triggerAtHardwareEvent(DMAMUX_SOURCE_DAC0);
    dma.enable();

    SIM_SCGC6 |= SIM_SCGC6_PDB;         // enable PDB clock
    PDB0_SC |= PDB_SC_PDBEN;            // enable PDB
    PDB0_SC |= PDB_SC_TRGSEL(15);       // SW trigger
    PDB0_SC |= PDB_SC_CONT;             // run contiguous

    PDB0_SC |= PDB_SC_PRESCALER(0b111); // prescaler 128

    // Prescaler multipliers other than 1x don't work correctly.
    PDB0_SC |= PDB_SC_MULT(0b0);

    // Adjust for output frequency.
    // out frequency == F_BUS / PRESCALER / (PDB0_MOD + 1)
    uint16_t mod = 0xffff;
    PDB0_MOD = mod;

    // The hardware doesn't do what the manual claims. The DAC trigger counter is
    // not reset on PDB counter overflow. Things work correctly, if the PDB mod
    // is used.
    PDB0_DACINT0 = mod;                // trigger DAC once per PDB cycle
    PDB0_DACINTC0 = PDB_DACINTC_TOE;   // enable DAC interval trigger

    PDB0_SC |= PDB_SC_LDOK;            // sync buffered PDB registers
    PDB0_SC |= PDB_SC_SWTRIG;          // start PDB

    unsigned src_idx_prev = 0;
    unsigned dac_out_idx_prev = 0;

    while(true) {
        noInterrupts();
        unsigned src_idx_curr = ((uintptr_t) dma.sourceAddress() - (uintptr_t) buffer.data()) / 2;
        unsigned dest_idx = ((uintptr_t) dma.destinationAddress() - (uintptr_t) &DAC0_DAT0L) / 2;
        unsigned dac_out_idx_curr = DAC0_C2 >> 4;
        unsigned dac_val = ((aliased_uint16_vptr) &DAC0_DAT0L)[dac_out_idx_curr];
        interrupts();
        if(src_idx_prev == src_idx_curr && dac_out_idx_prev == dac_out_idx_curr) continue;
        src_idx_prev = src_idx_curr;
        dac_out_idx_prev = dac_out_idx_curr;
        Serial.printf("DMA src idx: %4u   DMA dest idx: %4u   DAC out idx: %4u   DAC value: %4u\n",
                      src_idx_curr, dest_idx, dac_out_idx_curr, dac_val);
    }
}

void loop() {}
