#include "mapped_encoder.h"

auto& serial = Serial;


// Use 4 counts per detent configuration with custom hardware settings.
struct Encoder2Config : public MappedEncoderConf4cpd {
    static const uint8_t pin_mode = INPUT_PULLDOWN;
    static const bool reverse = true;
    static const bool invert_A = true;
    static const bool invert_B = true;
};

// Channel A: pin 3
// Channel B: pin 4
// Encoder value type: int32_t
// Use default 2 counts per detent configuration.
using enc1 = MappedEncoder<3, 4, int32_t, MappedEncoderConf2cpd>;

// Channel A: pin 5
// Channel B: pin 6
// Encoder value type: int32_t
// Use custom 4 counts per detent configuration
using enc2 = MappedEncoder<5, 6, double, Encoder2Config>;

void setup() {
    serial.begin(115200);
    delay(2000);

    int32_t enc1_val = 0;
    double enc2_val = 3.0l;

    // Use value range of [-500, 500] with 10 step size, start value is enc1_val
    enc1::begin(enc1_val, -500, 500, 10);
    // Use value range of [1.0, 10.0] with 0.1 step size, start value is enc2_val
    enc2::begin(enc2_val, 1.0l, 10.0l, 0.1l);

    while(true) {
        if(enc1_val != enc1::read()) {
            enc1_val = enc1::read();
            serial.printf("[%u] Enc val [1]: %i\n", millis(), enc1_val);
        }
        if(enc2_val != enc2::read()) {
            enc2_val = enc2::read();
            serial.printf("[%u] Enc val [2]: %f\n", millis(), enc2_val);
        }
    }
}

void loop() {}
