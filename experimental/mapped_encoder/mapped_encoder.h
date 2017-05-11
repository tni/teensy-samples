#ifndef MappedEncoder_h_
#define MappedEncoder_h_

// Encoder class that clamps the encoder value between [min_val, max_val]. The value is 
// changed by 'step_size'.
//
// This class performs special filtering for mechanical encoders with 2 or 4 counts per
// detent to eliminate bounce issues. Intermediate counts are filtered out, so that
// there is one effective step per detent.
//
// Integer and floating point types are usable as 'value_type'.

#include <type_traits>

enum class MappedEncoderType {
    enc_1cpd, // 1 count per detent
    enc_2cpd, // 2 counts per detent
    enc_4cpd, // 4 counts per detent
};

// Base configuration for 1 step per detent encoders. No filtering is performed. Vulnerable
// to glitches.
struct MappedEncoderConf1cpd {
    static const MappedEncoderType enc_type = MappedEncoderType::enc_1cpd;
    static const bool reverse = false;
    static const uint8_t pin_mode = INPUT_PULLUP;
    static const bool invert_A = false;
    static const bool invert_B = false;
};

// Base configuration for 2 steps per detent encoders. Glitch filtering is performed.
// If the encoder does not work well, try swapping A and B channel, which changes the 
// filtering behavior.
struct MappedEncoderConf2cpd {
    static const MappedEncoderType enc_type = MappedEncoderType::enc_2cpd;
    static const bool reverse = false;
    static const uint8_t pin_mode = INPUT_PULLUP;
    static const bool invert_A = false;
    static const bool invert_B = false;
};

// Base configuration for 4 steps per detent encoders. Glitch filtering is performed.
// It is assumed that A and B channel are high at the detent. Invert the channels
// if that is not the case.
struct MappedEncoderConf4cpd {
    static const MappedEncoderType enc_type = MappedEncoderType::enc_4cpd;
    static const bool reverse = false;
    static const uint8_t pin_mode = INPUT_PULLUP;
    static const bool invert_A = false;
    static const bool invert_B = false;
};


template<uint8_t pin_A, uint8_t pin_B, typename value_type_, typename config> 
class MappedEncoder {
public:
    using value_type = value_type_;
    static constexpr MappedEncoderType enc_type = config::enc_type;

    static void begin(value_type start_val, value_type min_val, value_type max_val, value_type step_size) {
        detachIsr();
        d.min_val = min_val;
        d.max_val = max_val; 
        d.val = start_val;
        d.step_size = step_size;
        if(!d.pins_setup) {
            pinMode(pin_A, config::pin_mode);
            pinMode(pin_B, config::pin_mode);
            delay(1); // allow pins to settle
            d.pins_setup = true;
        }
        d.state_A = readA();
        d.state_B = readB();
        if(enc_type == MappedEncoderType::enc_2cpd) d.val_change_state_A = d.state_A;
        if(enc_type == MappedEncoderType::enc_4cpd) d.count_since_val_change = 0;
        attachIsr();
    }
    static void end() {
        detachIsr();
    }
    static value_type read() {
        if(!value_type_atomic) noInterrupts();
        value_type res = d.val;
        if(!value_type_atomic) interrupts();
        return res;
    }
 private:
    static constexpr bool value_type_atomic = sizeof(value_type) <= 4;
    static bool readA() { return digitalReadFast(pin_A) ^ config::invert_A; }
    static bool readB() { return digitalReadFast(pin_B) ^ config::invert_B; }
    static void detachIsr() { detachInterrupt(pin_A); detachInterrupt(pin_B); }
    static void attachIsr() { attachInterrupt(pin_A, isr, CHANGE); attachInterrupt(pin_B, isr, CHANGE); }

    static void isr() {
        bool new_A = readA();
        bool new_B = readB();
        
        if(new_A == d.state_A && new_B == d.state_B) return; // no pin change

        if(d.state_A != new_A && d.state_B != new_B) {
            // both A and B changed, should not happen, encoder glitch
            d.state_A = new_A;
            d.state_B = new_B;
            return;
        }
        
        value_type val = d.val;

        // [old channel A value] xor [new channel B value] allows us to calculate
        // the rotation direction
        bool direction = d.state_A ^ new_B ^ config::reverse;

        bool skip_value_update = false;
        if(enc_type == MappedEncoderType::enc_2cpd) {
            if(new_A != new_B || d.val_change_state_A == new_A) {
                // filter out steps where A == B
                // and steps where channel A didn't change from last value update
                skip_value_update = true;
            } else {
                // we have a valid step
                d.val_change_state_A = new_A;
            }
        } else if(enc_type == MappedEncoderType::enc_4cpd) {
            d.count_since_val_change += direction ? 1 : -1;

            // channel A and B are high at a detent
            if(new_A && new_B) {
                // if there is no encoder glitch, count_since_val_change is either -4 or 4
                if(abs(d.count_since_val_change) >= 3) {
                    // Explicitely set direction. In rare cases it may be different from 
                    // the direction implied by count_since_val_change due to encoder
                    // glitches.
                    //direction = d.count_since_val_change > 0;
                    d.count_since_val_change = 0;
                } else {
                    skip_value_update = true;
                }
            } else {
                skip_value_update = true;
            }
        }
        
        if(!skip_value_update) {
            if(direction) { 
                if(val <= d.max_val - d.step_size) val += d.step_size;
                else val = d.max_val;
            } else { 
                if(val >= d.min_val + d.step_size) val -= d.step_size;
                else val = d.min_val;
            }
        }
        d.state_A = new_A;
        d.state_B = new_B;
        d.val = val;
    }

    struct EncData {
        value_type step_size = {};
        value_type min_val = {};
        value_type max_val = {};
        volatile value_type val = {};
        bool state_A = false;
        bool state_B = false;
        bool pins_setup = false;
        union {
            // state of channel A at last value change, used for 2 cycles/detent encoders
            bool val_change_state_A = false;
            // encoder count since last value change, used for 4 cycles/detent encoders
            int8_t count_since_val_change;
        };
    };
    static EncData d;
};

template<uint8_t pin_A, uint8_t pin_B, typename value_type_, typename config>
typename MappedEncoder<pin_A, pin_B, value_type_, config>::EncData MappedEncoder<pin_A, pin_B, value_type_, config>::d;

#endif
