#ifndef _MOTOR_H_
#define _MOTOR_H_

typedef enum
{
    motor_forward,
    motor_back
}motor_path;

extern void motor_init(void);
void PIT2_IRQHandler(void);

#endif