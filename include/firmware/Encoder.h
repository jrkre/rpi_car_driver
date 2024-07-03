#ifndef ENCODER_H
#define ENCODER_H

#include <iostream>



typedef struct{
    volatile int16_t *pin;
    int16_t mask;
    uint8_t state;
    uint32_t position;
    int16_t ticks;
} Encoder_internal_state_t;

typedef void (*encoder_callback_t)(int, Encoder_internal_state_t&);


class Encoder
{
public:

    Encoder(int pi,int gpio, encoder_callback_t new_callback=update_position, int counts_per_revolution=20);

    Encoder(int pi, int gpio, encoder_callback_t new_callback, int new_counts_per_revolution, bool invert);
    ~Encoder();

    int getEncoderPosition();
    int getCountsPerRevolution();
    bool resetEncoder();
    
    int setPosition(int state);
    int changePosition(int incriment);


private:
    int gpio_pin, level_pos, last_gpio;
    int pi;
    int counts_per_revolution;
    long prev_encoder_ticks;
    bool invert = false;


    void _pulse(int pi, int gpio, uint32_t tick);

    encoder_callback_t callback;

    //static for C library compatibility
    static void _pulse_c(int pi, unsigned int gpio, uint32_t tick, void *user);

    //dir == 1 or dir == -1
    static void update_position(int dir, Encoder_internal_state_t &encoder);

    Encoder_internal_state_t encoder;
public:

    int getPi(){ return pi; }
};




#endif // ENCODER_H