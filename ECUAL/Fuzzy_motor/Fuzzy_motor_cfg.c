#include "Fuzzy_motor.h"

Fuzzy_motor motor1 = 
{
    {GPIOB, GPIOB},
    {GPIO_PIN_4, GPIO_PIN_5},
    TIM2,
    TIM4,
    PWM_CHANNEL_1,
    374,
    20,
    499,
    250,
    15,
    1,
    1,
    1,
    0.0,
    0.0,
    0,
    0,
    0.0,
    0,
    0,
    0,
    0,
    0
};

Fuzzy_IO fuzzy_inputs;
Fuzzy_IO fuzzy_outputs;
Fuzzy_rule_base fuzzy_rules;