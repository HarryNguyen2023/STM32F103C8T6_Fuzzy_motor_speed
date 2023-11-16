#ifndef FUZZY_MOTOR_H
#define FUZZY_MOTOR_H

#include "stm32f1xx_hal.h"

#define MAX_NAME 10

typedef enum
{
    PWM_CHANNEL_1,
    PWM_CHANNEL_2,
    PWM_CHANNEL_3,
    PWM_CHANNEL_4
}PWM_CHANNEL;

// Structure for the membership function of Fuzzy logic
typedef struct Fuzzy_MF
{
    char name[3];
    float output;       // Degree of membership
    float point1;       // Left lower point on the x-axis of the mebership function
    float point2;       // Left upper point on the x-axis of the membership function
    float point3;       // Right upper point of the trapezoidal
    float point4;       // Right lower point of the trapezoidal
    struct Fuzzy_MF* next;     // Pointer to the next membership function in the linked list
}Fuzzy_MF;

// Structure of input or output of the Fuzzy logic
typedef struct Fuzzy_IO
{
    char name[MAX_NAME];               // Name of the input or output type
    float value;                       // IO value changing every scan cycle
    Fuzzy_MF* membership_functions;    // List of the membership functions of the specific I/O
    struct Fuzzy_IO* next;                    // Pointer to the next I/O in the system
}Fuzzy_IO;

// Structure for rule elements
typedef struct Fuzzy_rule_element
{
    float* value;               // Pointer to the output of the antecedent/output strength value;
    struct Fuzzy_rule_element* next;   // Pointer to the next element;
}Fuzzy_rule_element;

// Structure for the rule system of the Fuzzy logic
typedef struct Fuzzy_rule_base
{
    Fuzzy_rule_element* if_side;        // The if clause of the rule
    Fuzzy_rule_element* then_side;      // The then clause of the rule
    struct Fuzzy_rule_base* next;              // Next rule 
}Fuzzy_rule_base;

// Structure for the technical specifications of the motor
typedef struct 
{
     // Pin for control the motor directions
    GPIO_TypeDef* motor_ports[2];
    uint16_t motor_pins[2];

    // Timer module for controlling the encoder
    TIM_TypeDef* encoder_tim;
    TIM_TypeDef* pwm_tim;
    PWM_CHANNEL pwm_channel;
    
    // Encoder resolution of the motor
    uint16_t encoder_rev;
    uint16_t time_frame;

    // Motor speed specification
    uint16_t MAX_PWM;
    uint16_t MAX_INPUT_SPEED;
    uint8_t DEAD_BAND;

    // Input of the Fuzzy logic
    uint16_t error_gain;
    uint16_t derror_gain;
    uint16_t output_gain;
    float speed_error;
    float speed_derror;

    // Speed specifications
    uint8_t direction;
    uint8_t moving; 
    float targetPulsePerFrame;
    uint32_t current_encoder;
    uint32_t prev_encoder;
    int16_t real_speed;
    int16_t prev_speed_feedback; 
    int32_t output;
}Fuzzy_motor;

// Public function prototypes
void inputSpeedHandling(Fuzzy_motor* motor, int speed);

void fuzzySystemInit(Fuzzy_motor* motor);
void fuzzySpeedControl(Fuzzy_motor* motor, Fuzzy_IO* fuzzy_inputs, Fuzzy_IO* fuzzy_outputs, Fuzzy_rule_base* fuzzy_rules);

#endif
