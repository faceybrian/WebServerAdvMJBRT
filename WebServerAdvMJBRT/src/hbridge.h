#pragma once
#include "global.h"

class H_Bridge
{
  public:
    H_Bridge() {};
  
    void set_pwm(int32_t value);
    
    void begin(int32_t pin_pwm, int32_t pin_ina, int32_t pin_inb, 
               int32_t freq_hz, int32_t resolution_bit, int32_t pwm_ch);

    void break_motor();

    void coast_motor();

  private:
    int32_t pin_pwm;
    int32_t pin_ina;
    int32_t pin_inb;   
    int32_t freq_hz;
    int32_t resolution_bit;
    int32_t pwm_ch;
};
