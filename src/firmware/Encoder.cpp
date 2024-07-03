#include "ros/ros.h"

// #include <pigpio.h>
// #include <pigpiod_if.h>
#include "pigpiod_if2.h"

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#include "Encoder.h"

Encoder::Encoder(int pi, int gpio, encoder_callback_t new_callback, int counts_per_revolution)
{
    this->gpio_pin = gpio;
    this->callback = new_callback;
    this->counts_per_revolution = counts_per_revolution;
    this->encoder.ticks = counts_per_revolution;

    this->pi = pi;

    //set up the gpio pin for pull up/down
    set_mode(pi, gpio_pin, PI_INPUT);

    int result = get_mode(pi, gpio_pin);
    if (result == 0)
    {
        ROS_INFO("GPIO mode set");
    }
    else if (result == PI_BAD_GPIO)
    { 
        ROS_INFO("GPIO set mode failed, PI_BAD_GPIO");
    }
    else if (result == PI_BAD_MODE)
    {
        ROS_INFO("GPIO set mode failed, PI_BAD_MODE");
    }
    else if (result == PI_NOT_PERMITTED)
    {
        ROS_INFO("GPIO set mode failed, PI_NOT_PERMITTED");
    }
    else {
        ROS_INFO("Unexpected error: %i", result);
    }

    result = set_pull_up_down(pi, gpio_pin, PI_PUD_UP);
    if (result == 0)
    {
        ROS_INFO("GPIO pull up set");
    }
    else{ ROS_INFO("GPIO set pull up resistor failed"); }
    
    int callbackId = event_callback_ex(pi, gpio_pin, _pulse_c, this);
    ROS_INFO("GPIO callback ID%i", callbackId);

    //delay for onboard r-c filter

    usleep(1000);
    
    // this->encoder->register = PIN_TO_BASEREG(gpio_pin);
    //this->encoder.mask = PIN_TO_BITMASK(gpio_pin);
    // this->encoder.position = 0;

    u_int8_t s = 0;

    if (gpio_read(pi, gpio_pin)) s |= 1;
    if (gpio_read(pi, gpio_pin)) s |= 2;
    encoder.state = s;

    ROS_INFO("Encoder registered on pin %i", gpio);
}

Encoder::Encoder(int pi, int gpio, encoder_callback_t new_callback, int new_counts_per_revolution, bool invert)
{
    Encoder(pi, gpio, new_callback, new_counts_per_revolution);
    Encoder::invert = invert;
}

Encoder::~Encoder()
{
    
}

void Encoder::_pulse(int pi, int event, uint32_t tick)
{
    ROS_INFO("pulse function triggered:\n  pi %i, \n  event %i, \n  tick %i", pi, event, tick);
    if(pi == gpio_pin) level_pos;

    if(pi != last_gpio) //debounce
    {
        last_gpio = pi;

        if ((pi == gpio_pin)) //&& (level == 1))
        {
            if(level_pos == 1)
            {
                callback(1,this->encoder);
            }
            else
            {
                callback(-1,this->encoder);
            }
        }
    }
    
    // std::cout << "Pulse:" << level << "\n" << "Callback:" << callback << std::endl;
}

void Encoder::update_position(int dir, Encoder_internal_state_t &encoder)
{
    encoder.position += dir;
    if(encoder.position >= encoder.ticks) encoder.position= 0;
    if(encoder.position < 0)  encoder.position = (encoder.ticks - 1);

    ROS_INFO("Position: %i", encoder.position);
    // std::cout << "Position:" << encoder.position << std::endl;
}

int Encoder::getCountsPerRevolution()
{
    return this->counts_per_revolution;
}

int Encoder::getEncoderPosition()
{
    return this->encoder.position;
}

int Encoder::changePosition(int increment)
{
    if (increment > 1 || increment < -1)
    {
        return 0;
    }
    else
    {
        this->encoder.position += increment;
        return increment;
    }
}

int Encoder::setPosition(int state)
{
    if (state > 20 || state < -20)
    {
        return 0;
    }
    else
    {
        this->encoder.position;
        ROS_INFO("Encoder position set to %i", state);
        return state;
    }
}




void Encoder::_pulse_c(int pi, unsigned int event, uint32_t tick, void *user)
{
    //static callback for C linking
    Encoder *self = (Encoder *)user;

    self -> _pulse(pi, event, tick);
    ROS_INFO("PULSE");
}



