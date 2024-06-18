#include <pigpio.h>
#include <iostream>

#include "Encoder.h"

void _pulse(int gpio, int level, uint32_t tick)
{
    if(gpio == gpio_pin) level_pos = level;

    if(gpio != last_gpio) //debounce
    {
        last_gpio = gpio;

        if ((gpio == gpio_pin) && (level == 1))
        {
            if(level_pos == 1)
            {
                callback(1);
            }
            else
            {
                callback(-1);
            }
        }
    }
    std::cout << "Pulse:" << level << "\n" << "Callback:" << callback << std::endl;
}

Encoder::Encoder(int gpio, encoder_callback_t new_callback, int counts_per_revolution=20)
{
    this.gpio_pin = gpio;
    this.callback = new_callback;
    this.counts_per_revolution = counts_per_revolution;

    //set up the gpio pin for pull up/down
    gpioSetMode(gpio_pin, PI_INPUT);
    gpioSetPullUpDown(gpio_pin, PI_PUD_UP);
    gpioSetAlertFunc(gpio_pin, _pulse);

    //delay for r-c filter

    delayMicroseconds(1000);

    
    
    encoder.register = PIN_TO_BASEREG(gpio_pin);
    encoder.mask = PIN_TO_BITMASK(gpio_pin);
    encoder.position = 0;

    unint8_t s = 0;

    if (gpioRead(gpio_pin)) s |= 1;
    if (gpioRead(gpio_pin)) s |= 2;
    encoder.state = s;


}


void Encoder::_pulse_c(int gpio, int level, uint32_t tick, void *user)
{
    //static callback for C linking
    re_decoder *self = (re_decoder *)user;

    mySelf -> _pulse(gpio, level, tick);
}

