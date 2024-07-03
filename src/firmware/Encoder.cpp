#include "ros/ros.h"
#include <pigpio.h>
#include <iostream>

#include "Encoder.h"

Encoder::Encoder(int gpio, encoder_callback_t new_callback, int counts_per_revolution)
{
    this.gpio_pin = gpio;
    this.callback = new_callback;
    this.counts_per_revolution = counts_per_revolution;

    //set up the gpio pin for pull up/down
    gpioSetMode(gpio_pin, PI_INPUT);
    gpioSetPullUpDown(gpio_pin, PI_PUD_UP);
    gpioSetAlertFunc(gpio_pin, _pulse);

    //delay for onboard r-c filter

    delayMicroseconds(1000);
    
    encoder.register = PIN_TO_BASEREG(gpio_pin);
    encoder.mask = PIN_TO_BITMASK(gpio_pin);
    encoder.position = 0;

    unint8_t s = 0;

    if (gpioRead(gpio_pin)) s |= 1;
    if (gpioRead(gpio_pin)) s |= 2;
    encoder.state = s;
}

Encoder(int gpio, encoder_callback_t new_callback, int new_counts_per_revolution, bool invert)
{
    Encoder(gpio, new_callback, new_counts_per_revolution);
    Encoder::invert = invert;
}

void Encoder::_pulse(int gpio, int level, uint32_t tick)
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
    // std::cout << "Pulse:" << level << "\n" << "Callback:" << callback << std::endl;
}

void Encoder::update_position(int dir)
{
    encoder.position += dir;
    if(encoder.position >= counts_per_revolution) encoder.position = 0;
    if(encoder.position < 0) encoder.position = counts_per_revolution - 1;

    ROS_INFO("Position: %d", encoder.position);
    // std::cout << "Position:" << encoder.position << std::endl;
}




void Encoder::_pulse_c(int gpio, int level, uint32_t tick, void *user)
{
    //static callback for C linking
    re_decoder *self = (re_decoder *)user;

    mySelf -> _pulse(gpio, level, tick);
}



