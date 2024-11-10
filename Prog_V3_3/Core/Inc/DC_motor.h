#ifndef INC_DC_MOTOR_H_
#define INC_DC_MOTOR_H_

#include <PID.h>
#include <main.h>

#define PI 3.14
#define V_MAX 7.5
#define V_MIN 1.0
#define GIRI_MIN_CONV 0.000503271

float DegreeSec2RPM(float);

float Voltage2Duty(float);
uint8_t Ref2Direction(float);
void set_PWM_and_dir(uint32_t, uint8_t);



#endif /* INC_DC_MOTOR_H_ */
