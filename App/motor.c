#include "include.h"

#define Inverters 0

#define MOTOR_FTM   FTM0
#define MOTOR1_PWM  FTM_CH3
#define MOTOR2_PWM  FTM_CH4

#define MOTOR1_PWM_IO  FTM0_CH3
#define MOTOR2_PWM_IO  FTM0_CH4

#define MOTOR_HZ   10000//(20*FTM0_PRECISON0)
extern int16 AimSpeed;


void motor_init(void)
{
   ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,0);      //初始化 电机 PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,0);      //初始化 电机 PWM
 
    pit_init_ms(PIT2,30);                         //初始化PIT1，定时时间为： adc_period ms
    set_vector_handler(PIT2_VECTORn ,PIT2_IRQHandler);          //设置PIT2的中断服务函数为 PIT2_IRQHandler
    NVIC_SetPriority(PIT2_IRQn,2);                                 //E6A2   中断优先级也是2   待测   2017.4.22
   //enable_irq (PIT2_IRQn);                                     //使能PIT1中断
}

void PIT2_IRQHandler(void)
{
     AimSpeed=saidao_analyse();
   // Speed_control(AimSpeed);   //2017.4.30  电机pd  
  //  AimSpeed=300;
     if(  Carstop==1)
     {    
        SPeedAim(0);
     }
      
     else
      SPeedAim(AimSpeed);
    led (LED1,LED_ON);
}
