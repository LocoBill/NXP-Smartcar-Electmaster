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
   ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,0);      //��ʼ�� ��� PWM
    ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,0);      //��ʼ�� ��� PWM
 
    pit_init_ms(PIT2,30);                         //��ʼ��PIT1����ʱʱ��Ϊ�� adc_period ms
    set_vector_handler(PIT2_VECTORn ,PIT2_IRQHandler);          //����PIT2���жϷ�����Ϊ PIT2_IRQHandler
    NVIC_SetPriority(PIT2_IRQn,2);                                 //E6A2   �ж����ȼ�Ҳ��2   ����   2017.4.22
   //enable_irq (PIT2_IRQn);                                     //ʹ��PIT1�ж�
}

void PIT2_IRQHandler(void)
{
     AimSpeed=saidao_analyse();
   // Speed_control(AimSpeed);   //2017.4.30  ���pd  
  //  AimSpeed=300;
     if(  Carstop==1)
     {    
        SPeedAim(0);
     }
      
     else
      SPeedAim(AimSpeed);
    led (LED1,LED_ON);
}
