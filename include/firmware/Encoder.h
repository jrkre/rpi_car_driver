#ifndef ENCODER_H
#define ENCODER_H

#include <iostream>

typedef void (*encoder_callback_t)(int);

typedef struct{
    volatile int16_t *pin;
    int16_t mask;
    uint8_t state;
    uint32_t position;
} Encoder_internal_state_t;

class Encoder
{
public:

    Encoder(int gpio, encoder_callback_t new_callback=update_position, int counts_per_revolution=20);
    Encoder(int gpio, encoder_callback_t new_callback, int new_counts_per_revolution, bool invert=false);
    ~Encoder();
    


private:
    int gpio_pin, level_pos, last_gpio;
    int counts_per_revolution;
    long prev_encoder_ticks;
    bool invert;


    void _pulse(int gpio, int level, uint32_t tick);

    encoder_callback_t callback;

    //static for C library compatibility
    static void _pulse_c(int gpio, int level, uint32_t tick, void *user);

    static void update_position(int dir);

    Encoder_internal_state_t encoder;
};




#endif // ENCODER_H