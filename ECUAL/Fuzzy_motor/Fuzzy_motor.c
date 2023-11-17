#include "Fuzzy_motor.h"
#include "Fuzzy_motor_cfg.h"

#include <math.h>
#include <string.h>

#define min(a, b) ((a < b) ? a : b)
#define max(a, b) ((a > b) ? a : b)

// Private function prototypes
static void motorInit(Fuzzy_motor* motor);
static void motorBrake(Fuzzy_motor* motor);
static uint32_t readEncoder(Fuzzy_motor* motor);
static void getErrorFeedback(Fuzzy_motor* motor);
static void dutyCycleUpdate(uint16_t duty_cycle, Fuzzy_motor* motor);
static void motorHardwareControl(Fuzzy_IO* fuzzy_outputs, Fuzzy_motor* motor);

static Fuzzy_MF* mfInit(char* name, float point1, float point2, float point3, float point4);
static Fuzzy_IO* ioInit(char* name, Fuzzy_MF* mf_list);
static Fuzzy_rule_element* ruleElementInit(Fuzzy_MF* mf);
static Fuzzy_rule_base* ruleBaseInit(Fuzzy_rule_element* if_side, Fuzzy_rule_element* then_side);

static void degreeofMembership(Fuzzy_MF* mf, float input);
static float mfArea(Fuzzy_MF* mf);
static void fuzzification(Fuzzy_IO* fuzzy_inputs);
static void fuzzyRuleInference(Fuzzy_rule_base* fuzzy_rules);
static void defuzzification(Fuzzy_IO* fuzzy_outputs);


// -------------------------------------------- Private functions hidden from users ---------------------------------------------------

// Function to initiate the motor GPIO pins
static void motorInit(Fuzzy_motor* motor)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    // Initiate the GPIO pins of the motor
    for(uint8_t i = 0; i < 2; ++i)
    {
        HAL_GPIO_WritePin(motor->motor_ports[i], motor->motor_pins[i], 0);
        if(motor->motor_ports[i] == GPIOA)
            __HAL_RCC_GPIOA_CLK_ENABLE();
        else if (motor->motor_ports[i] == GPIOB)
            __HAL_RCC_GPIOB_CLK_ENABLE();
        else if (motor->motor_ports[i] == GPIOC)
            __HAL_RCC_GPIOC_CLK_ENABLE();
        else if (motor->motor_ports[i] == GPIOD)
            __HAL_RCC_GPIOD_CLK_ENABLE();
        else if (motor->motor_ports[i] == GPIOE)
            __HAL_RCC_GPIOE_CLK_ENABLE();
        GPIO_InitStruct.Pin = motor->motor_pins[i];
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
        HAL_GPIO_Init(motor->motor_ports[i], &GPIO_InitStruct);
    }
    // Calculate the gains
    motor->error_gain = (uint16_t)(motor->MAX_INPUT_SPEED * motor->encoder_rev * motor->time_frame) / 60000.0;
    motor->derror_gain = motor->error_gain;
    motor->output_gain = motor->MAX_PWM;
}

// Function to update the PWM dutycycle of the motor
static void dutyCycleUpdate(uint16_t duty_cycle, Fuzzy_motor* motor)
{
    switch (motor->pwm_channel)
    {
        case PWM_CHANNEL_1:
            motor->pwm_tim->CCR1 = duty_cycle;
            break;
        case PWM_CHANNEL_2:
            motor->pwm_tim->CCR2 = duty_cycle;
            break;
        case PWM_CHANNEL_3:
            motor->pwm_tim->CCR3 = duty_cycle;
            break;
        case PWM_CHANNEL_4:
            motor->pwm_tim->CCR4 = duty_cycle;
            break;
    default:
        break;
    }
}

// Function to brake the motor
static void motorBrake(Fuzzy_motor* motor)
{
    // Update the PWM
    dutyCycleUpdate(0, motor);
    // Set all direction pins to LOW
    HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], 0);
    HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], 0);
}

// Function to read the encoder value of the motor
static uint32_t readEncoder(Fuzzy_motor* motor)
{
    return motor->encoder_tim->CNT / 4;
}

// Function to get the feedback of each control cycle
static void getErrorFeedback(Fuzzy_motor* motor)
{
    // Get number of the encoder pulse in the last time frame
    if(motor->direction == 0 && (motor->current_encoder < motor->prev_encoder) && (motor->prev_encoder - motor->current_encoder > 16000))
    {
        motor->real_speed = (65535 / 4) - motor->prev_encoder;
        motor->real_speed += motor->current_encoder;
    }
    else if(motor->direction == 1 && (motor->current_encoder > motor->prev_encoder) && (motor->current_encoder - motor->prev_encoder > 16000) )
    {
        motor->real_speed = motor->current_encoder - (65535 / 4);
        motor->real_speed -= motor->prev_encoder;
    }
    else
        motor->real_speed = motor->current_encoder - motor->prev_encoder;
    // Get the error of the number of encoder per time frame and nromalized the value
    motor->speed_error = (motor->targetPulsePerFrame - motor->real_speed) * 1.0 / motor->error_gain;
    // Get the differential of the system feeback and normalized the value
    motor->speed_derror = (motor->real_speed - motor->prev_speed_feedback) * 1.0 / motor->derror_gain;
    // Update the value of some specifications
    motor->prev_encoder = motor->current_encoder;
    motor->prev_speed_feedback = motor->real_speed;
}

// Function to control the hardware of the motor
static void motorHardwareControl(Fuzzy_IO* fuzzy_outputs, Fuzzy_motor* motor)
{
    // Get the output from the Fuzzy controller
    motor->output += (fuzzy_outputs->value) * motor->output_gain;
    
    // Saturation the output 
    if(motor->output > motor->MAX_PWM)
        motor->output = motor->MAX_PWM;
    else if(motor->output < 0)
        motor->output = 0;

    uint16_t pwm_dutycycle = abs(motor->output);
    if(pwm_dutycycle < motor->DEAD_BAND)
        pwm_dutycycle = 0;

    // Control the direction of the motor
    if(motor->direction == 0)
    {
        HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], 1);
        HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], 0);
    }
    else if(motor->direction == 1)
    {
        HAL_GPIO_WritePin(motor->motor_ports[0], motor->motor_pins[0], 0);
        HAL_GPIO_WritePin(motor->motor_ports[1], motor->motor_pins[1], 1);
    }

    // Feed the value of the PWM duty cycle
    dutyCycleUpdate(pwm_dutycycle, motor);

    if(pwm_dutycycle == 0)
        motorBrake(motor);
}

// Function to initiate a membership function
static Fuzzy_MF* mfInit(char* name, float point1, float point2, float point3, float point4)
{
    Fuzzy_MF* membership_function = (Fuzzy_MF*)malloc(sizeof(Fuzzy_MF));
    strcpy(membership_function->name, name);
    membership_function->point1 = point1;
    membership_function->point2 = point2;
    membership_function->point3 = point3;
    membership_function->point4 = point4;
    membership_function->output = 0;
    membership_function->next = NULL;
    return membership_function;
}

// Function to initiate the IO of the Fuzzy system
static Fuzzy_IO* ioInit(char* name, Fuzzy_MF* mf_head)
{
    Fuzzy_IO* fuzzy_io = (Fuzzy_IO*)malloc(sizeof(Fuzzy_IO));
    strcpy(fuzzy_io->name, name);
    fuzzy_io->value = 0;
    fuzzy_io->membership_functions = mf_head;
    fuzzy_io->next = NULL;
    return fuzzy_io;
}

// Function to intiate the rule base element
static Fuzzy_rule_element* ruleElementInit(Fuzzy_MF* mf)
{
    Fuzzy_rule_element* rule_element = (Fuzzy_rule_element*)malloc(sizeof(Fuzzy_rule_element));
    rule_element->value = &(mf->output);
    rule_element->next = NULL;
    return rule_element;
}

// Function to initiate the rule base of the Fuzzy logic
static Fuzzy_rule_base* ruleBaseInit(Fuzzy_rule_element* if_side, Fuzzy_rule_element* then_side)
{
    Fuzzy_rule_base* rule_base = (Fuzzy_rule_base*)malloc(sizeof(Fuzzy_rule_base));
    rule_base->if_side = if_side;
    rule_base->then_side = then_side;
    rule_base->next = NULL;
    return rule_base;
}

// Function to calculate the degree of membership 
static void degreeofMembership(Fuzzy_MF* mf, float input)
{
    // Case out of bound
    if(input < mf->point1 || input > mf->point4)
    {
        mf->output = 0;
        return;
    }
    // Case of right triangle to the left
    if(mf->point1 == mf->point2 && mf->point2 == mf->point3)
        mf->output = (mf->point4 - input) / (mf->point4 - mf->point3);
    // Case of right triangle to the right
    else if(mf->point2 == mf->point3 && mf->point3 == mf->point4)
        mf->output = (input - mf->point1) / (mf->point2 - mf->point1);
    // Case of right trapezoidal to the left
    else if(mf->point1 == mf->point2)
    {
        if(input < mf->point3)
            mf->output = 1;
        else
            mf->output = (mf->point4 - input) / (mf->point4 - mf->point3);
    }
    // Case of right trapezoidal to the right
    else if(mf->point3 == mf->point4)
    {
        if(input > mf->point2)
            mf->output = 1;
        else
            mf->output = (input - mf->point1) / (mf->point2 - mf->point1);
    }
    // Case of normal triangle and normal trapezoidal
    else
    {
        if(input < mf->point2)
            mf->output = (input - mf->point1) / (mf->point2 - mf->point1);
        else if(input > mf->point3)
            mf->output = (mf->point4 - input) / (mf->point4 - mf->point3);
        else
            mf->output = 1;
    }
}

// Function to perform the fuzzification
static void fuzzification(Fuzzy_IO* fuzzy_inputs)
{
    Fuzzy_IO* inputs;
    Fuzzy_MF* mfs;
    for(inputs = fuzzy_inputs; inputs != NULL; inputs = inputs->next)
    {
        for(mfs = inputs->membership_functions; mfs != NULL; mfs = mfs->next)
        {
            degreeofMembership(mfs, inputs->value);
        }
    }
}

// Function to perform rule inference
static void fuzzyRuleInference(Fuzzy_rule_base* fuzzy_rules)
{
    Fuzzy_rule_base* rules;
    Fuzzy_rule_element *rule_if, *rule_then;
    float strength;
    for(rules = fuzzy_rules; rules != NULL; rules = rules->next)
    {
        strength = 1;
        for(rule_if = rules->if_side; rule_if != NULL; rule_if = rule_if->next)
            strength = min(strength, *(rule_if->value));
        for(rule_then = rules->then_side; rule_then != NULL; rule_then = rule_then->next)
            *(rule_then->value) = max(strength, *(rule_then->value));
    }
}

// Function to calculate the area of the membership function
static float mfArea(Fuzzy_MF* mf)
{
    float base = mf->point4 - mf->point1;
    float top = mf->point3 - mf->point2;
    return mf->output * (base + top) / 2;
}

// Function to perform the defuzzication of the system
static void defuzzification(Fuzzy_IO* fuzzy_outputs)
{
    Fuzzy_IO* outputs;
    Fuzzy_MF* mfs;
    float sum_of_products, sum_of_area, area, centroid;
    for(outputs = fuzzy_outputs; outputs != NULL; outputs = outputs->next)
    {
        sum_of_products = 0;
        sum_of_area = 0;
        for(mfs = outputs->membership_functions; mfs != NULL; mfs = mfs->next)
        {
            area = mfArea(mfs);
            centroid = mfs->point1 + (mfs->point4 - mfs->point1) / 2;
            sum_of_area += area;
            sum_of_products += centroid * area;
        }
        outputs->value = sum_of_products / sum_of_area;         // Weighted average
    }
}

// ---------------------------------------------- Public functions used by users -------------------------------------------------------

// Function to get the input desired control speed of the motor
void inputSpeedHandling(Fuzzy_motor* motor, int speed)
{
    // Rescale the input rpm speed
    if(speed > motor->MAX_INPUT_SPEED)
        speed = motor->MAX_INPUT_SPEED;
    else if(speed < - motor->MAX_INPUT_SPEED)
        speed = - motor->MAX_INPUT_SPEED;

    // Get the direction of the movement (0: CCW, 1: CW)
    if(speed >= 0)
        motor->direction = 0;
    else
        motor->direction = 1;

    // Check whether the motor is already moving
    if(! motor->moving)
        motor->moving = 1;

    // Convert the desired speed to pulse per frame and input to the motor
    motor->targetPulsePerFrame = (speed * motor->encoder_rev * 1.0) * motor->time_frame / 60000.0;
}

// Function to initialize the Fuzzy logic system
void fuzzySystemInit(Fuzzy_motor* motor)
{
    // Declare Error membership functions
    Fuzzy_MF* error_NB = mfInit("NS", -1, -1, -0.6, -0.3);
    Fuzzy_MF* error_NS = mfInit("NB", -0.6, -0.3, -0.3, 0);
    Fuzzy_MF* error_ZE = mfInit("ZE", -0.3, 0, 0, 0.3);
    Fuzzy_MF* error_PS = mfInit("PS", 0, 0.3, 0.3, 0.6);
    Fuzzy_MF* error_PB = mfInit("PB", 0.3, 0.6, 1, 1);
    error_NB->next = error_NS;
    error_NS->next = error_ZE;
    error_ZE->next = error_PS;
    error_PS->next = error_PB;

    // Declare DError memebership functions
    Fuzzy_MF* derror_NB = mfInit("NS", -1, -1, -1, -0.5);
    Fuzzy_MF* derror_NS = mfInit("NB", -1, -0.5, -0.5, 0);
    Fuzzy_MF* derror_ZE = mfInit("ZE", -0.5, 0, 0, 0.5);
    Fuzzy_MF* derror_PS = mfInit("PS", 0, 0.5, 0.5, 1);
    Fuzzy_MF* derror_PB = mfInit("PB", 0.5, 1, 1, 1);
    derror_NB->next = derror_NS;
    derror_NS->next = derror_ZE;
    derror_ZE->next = derror_PS;
    derror_PS->next = derror_PB;

    // Declare speed output membership functions
    Fuzzy_MF* speed_NB = mfInit("NS", -1, -1, -1, -0.667);
    Fuzzy_MF* speed_NM = mfInit("NM", -1, -0.667, -0.667, 0.333);
    Fuzzy_MF* speed_NS = mfInit("NB", -0.667, -0.333, -0.333, 0);
    Fuzzy_MF* speed_ZE = mfInit("ZE", -0.333, 0, 0, 0.333);
    Fuzzy_MF* speed_PS = mfInit("PS", 0, 0.333, 0.333, 0.667);
    Fuzzy_MF* speed_PM = mfInit("PM", 0.333, 0.667, 0.667, 1);
    Fuzzy_MF* speed_PB = mfInit("PB", 0.667, 1, 1, 1);
    speed_NB->next = speed_NM;
    speed_NM->next = speed_NS;
    speed_NS->next = speed_ZE;
    speed_ZE->next = speed_PS;
    speed_PS->next = speed_PM;
    speed_PM->next = speed_PB;

    // Initiate the system inputs
    Fuzzy_IO* error_input = ioInit("Error", error_NB);
    Fuzzy_IO* derror_intput = ioInit("DError", derror_NB);
    error_input->next = derror_intput;
    fuzzy_inputs = *error_input;

    // Initiate the system output
    Fuzzy_IO* speed_output = ioInit("Speed", speed_NB);
    fuzzy_outputs = *speed_output;

    // Initiate the system rule
    Fuzzy_MF* rule_out[5][5] = {{speed_NB, speed_NB, speed_NM, speed_NS, speed_ZE},
                                {speed_NB, speed_NM, speed_NS, speed_ZE, speed_PS},
                                {speed_NM, speed_NS, speed_ZE, speed_PS, speed_PM},
                                {speed_NS, speed_ZE, speed_PS, speed_PM, speed_PB},
                                {speed_ZE, speed_PS, speed_PM, speed_PB, speed_PB}};

    uint8_t i, j;
    Fuzzy_MF *mfi, *mfj;
    Fuzzy_rule_base *head, *temp;
    for(mfi = derror_NB, i = 0; mfi != NULL; mfi = mfi->next, ++i)
    {
        for(mfj = error_NB, j = 0; mfj != NULL; mfj = mfj->next, j++)
        {
            Fuzzy_rule_element* rule_if = ruleElementInit(mfi);
            rule_if->next = ruleElementInit(mfj);
            Fuzzy_rule_element* rule_then = ruleElementInit(rule_out[i][j]);
            Fuzzy_rule_base* rule = ruleBaseInit(rule_if, rule_then);
            // Case of 1st rule
            if(i == 0 && j == 0)
            {
                head = rule;
                temp = head;
            }
            else
            {
                temp->next = rule;
                temp = temp->next;
            }
        }
    } 
    fuzzy_rules = *head;

    // Initiate the motor hardware
    motorInit(motor);
}

// Function to perform fuzzy control 
void fuzzySpeedControl(Fuzzy_motor* motor, Fuzzy_IO* fuzzy_inputs, Fuzzy_IO* fuzzy_outputs, Fuzzy_rule_base* fuzzy_rules)
{
    // Get the speed feedback input for the controller
    getErrorFeedback(motor);
    Fuzzy_IO* inputs = fuzzy_inputs;
    inputs->value = motor->speed_error;
    inputs = inputs->next;
    inputs->value = motor->speed_derror;

    // Fuzzification 
    fuzzification(fuzzy_inputs);
    // Rule inference
    fuzzyRuleInference(fuzzy_rules);
    // Defuzzication
    defuzzification(fuzzy_outputs);
    // Control the hardware with output from the controller
    motorHardwareControl(fuzzy_outputs, motor);
}