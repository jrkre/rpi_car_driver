#ifndef ENCODER_H
#define ENCODER_H

#include <iostream>

typedef void (*encoder_callback_t)(int);

typeder struct{
    volatile IO_REG_TYPE *pin;
    IO_REG_TYPE mask;
    uint8_t state;
    uint32_t position;
} Encoder_internal_state_t;

class Encoder
{
public:

    Encoder(int gpio, encoder_callback_t new_callback, int new_counts_per_revolution=20, bool invert=false);
    


private:
    int gpio_pin, level_pos, last_gpio;
    int counts_per_revolution;
    long prev_encoder_ticks;


    void _pulse(int gpio, int level, uint32_t tick);

    encoder_callback_t callback;

    //static for C library compatibility
    static void _pulse_c(int gpio, int level, uint32_t tick, void *user);

    Encoder_internal_state_t encoder;
};




#endif // ENCODER_H