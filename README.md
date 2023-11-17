# STM32F103C8T6_Fuzzy_motor_speed
## Author Harry Nguyen (11/17/2023)

### 1. Scope 

This project has been conducted to check the practical application of the Fuzzy logic controller in speed control of the brushed DC motor, which is mostly only be implemented in MATLAB for theoretical study.

### 2. Introduction

Fuzzy is an advanced controller, which utilized the vague languages of the human's logic  to control systems that are hard to be mathematically modelled. Fuzzy algorithm mainly includes 3 main steps:

- Fuzzification: converting the input value to the degree of membership functions.
- Rule inference: derive the output's degree of membership functions from the inputs used the Fuzzy rules and MIN-MAX operator.
- Defuzzification: compute the output of the controller from the output's degree of membership functions using the weighted average or centroid method.

### 3. Components:

- MCU STM32F103C8T6 (Blue Pill)
- Driver TB6612NG
- Brushed DC motor JGA25 with encoder (374 pulse per revolution)
- UART-TTL IC.

### 4.Result

I have successfully controlled the speed of the motor with Fuzzy logic controller, in which I have used concept of Linked List to create the data structure for the Fuzzy inputs, outputs and rules, refered to [1]. However, I have normalized all the inputs and outputs to the scale of [-1,1] in the Fuzzy controller and used the gains to interface them with the real hardware system's encoder and PWM. One more thing I have added is the Fuzzy reset function, which will reset all the membership function's value to 0 before a new scan cycle of the controller, as I noticed without reseting those values, the controller will be corrupted.

![image](https://github.com/HarryNguyen2023/STM32F103C8T6_Fuzzy_motor_speed/assets/136590151/25f3b36b-c3fa-4460-9bc3-11b462aa33c1)

Above is the diagram plotted in Jupyter Notebooks using the real speed feedback from motor via UART. As we can see, the controller parameters are:

- Settling time: 800 ms.
- Overshoot: 50%.
- Steady state error: 2%.

### 5. Future intention

Notice that this controller can be improved by tuing the gains, as well as changing the degree of membership functions of the IO and the Fuzzy rule by using some optimized algorithm such as Genetic Algorithm. In the future, I intend to design the Fuzzy-PID adaptive controller with can adjust the PID parameters automatically in the condition of load changes.

## Reference

[1]. Greg Viot, Fuzzy Logic in C, Dr. Dobb's Journal, February 1993.
[2]. Nguyễn Đức Hiển, Hoàng Đình Cơ, TÀI LIỆU HỌC TẬP ĐIỀU KHIỂN MỜ VÀ MẠNG NORON, Trường ĐH KTe-KThuat Công Nghiệp, Hà Nội, 2019.


