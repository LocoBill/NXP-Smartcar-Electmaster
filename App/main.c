 /*!
  *     COPYRIGHT NOTICE
  *     Copyright (c) 2013,ɽ��Ƽ�
  *     All rights reserved.
  *     �������ۣ�ɽ����̳ http://www.vcan123.com
  *
  *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
  *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
  *
  * @file       main.c
  * @brief      ɽ��K60 ƽ̨������
  * @author     ɽ��Ƽ�
  * @version    v5.0
  * @date       2013-08-28
  */
/*
  lanzhou ��ֲ�룬 2017.4.17   Loco.
    ��Ųɼ�ֲ�� ����ֻ��AB ��ͨ����Ч  �����  2017.4.22  Loco.

   ͨ�����������϶������ɼ����ֵ����������֤ ����ٶ�δ����  2017.4.24 Loco.
    ����ͷ���������   2017.4.24 Loco.

    ���������ǣ���ǲ���������н���(���Ƶ�ʣ�����)��2017.4.29 Loco.

    �����ֵ18500������2017.5.1Loco.
    speedpid   �������   2017.5.1Loco.
       //SpeedPID����ADC3��ADC5������   �����2017.5.2Loco.
  �ھŽ��ŵĵ��pid�޸�ֲ�룬�����ܣ�����̫������2017.5.2Loco.
     �Ӹɻɹ�   ��PTB6.7 �������   2017.5.3Loco.
    PTB,PTE�����ж�   ����� 2017.5.4 Loco.
    �����жϽ������ʼ�����ߣ� �ٶ�pid����  2017.5.5Loco.
    ��������   Juan.2017.5.6
      
//2017.5.6
int16 speedpid(uint16_t AmSpeed)  ����2017.5.6Loco.
 ײ�������꣬���ܣ���ʮ����ת�����ͨ��234����100  2017.5.6Loco.


��������    �����ʼ����������������  2017.5.9
    mos���� ������������������������������������������������������������������2017.5.12Loco

   
   speedpid����  32λ  ����ֹ�����
mos�ܣ�Ӳ�����⡣2017.5.13
      P 71    I  12
    k60 ż����λ����������   2017.5.14

     ��λ������̾���    ͣ����������·�� ��һ��������ͣ�������� ��moto.c  ���ж�ʹ�ܣ�   ��ص�ѹ7.3v ��������2017.5.15 Loco.
    
    �����   ��ͷ����2 017.5.16  Loco.   T . T

      jixie��е�ṹ��ת����  2017.5.17 
        
        �������ܣ�����ʮ����ײǽ����ת�����������ܣ����У�����ͻ��2017.5.20Loco.
       �����ֵ��λ19300��ԭʼƫ��   2017.5.20Loco.

        K4 ���� ��ʱδ���� ��ص�ѹӰ�� ΢��  //2017.5.21 1:00 Loco.

     ��ص�ѹ7.8���ϣ�17s  18s��K4���920δ���٣�
     ����޷�2500   ������ʲô��
       
       ���pid���´���  �޷�9900 P 12   D  2   2m 2017.5.23  ��ص�ѹ������
    ����ж�  5ms
      �ٶ�570 ΢��������2017.5.24 2:46
        K5����  2015.5.24 11:12
    
 */ 
    

#include "include.h"
    
int16 LeftAD,RightAD,LAD,RAD;
int16 AimSpeed=400;
int16_t TurnPWM;       //2017.4.26

//LED��ʼ��
void LED_init(void)
{
    led_init(LED0);
    led_init(LED1);
    led_init(LED2);
    led_init(LED3);
}

//GPIO��ʼ��
void GPIO_init(void)
{
   //���뿪�س�ʼ��
   uart_init (UART4 ,115200);   
   gpio_init (PTC7, GPI, HIGH) ;                
   gpio_init (PTC8, GPI, HIGH) ;               
   gpio_init (PTC9, GPI, HIGH) ;              
   gpio_init (PTC10, GPI, HIGH) ;               
   gpio_init (PTC11, GPI, HIGH) ;              
   gpio_init (PTC12, GPI, HIGH) ;
   
   //������ʼ��
   gpio_init (PTC16, GPI, HIGH) ;
   port_init_NoALT (PTC16, IRQ_FALLING | PF | PULLUP );    //��ʼ�� PTC16 �ܽţ��½��ش����жϣ�����Դ�˲���������ԭ�ȸ��ù��ܣ���������
   gpio_init (PTC17, GPI, HIGH) ;
   port_init_NoALT (PTC17, IRQ_FALLING | PF | PULLUP );   
   gpio_init (PTE26, GPI, HIGH) ;
   port_init_NoALT (PTE26, IRQ_FALLING | PF | PULLUP );   
   gpio_init (PTE27, GPI, HIGH) ;
   port_init_NoALT (PTE27, IRQ_FALLING | PF | PULLUP );   
  
   //�ɻɹ�
   gpio_init (PTB6, GPI, HIGH) ;
   port_init_NoALT (PTB6, IRQ_FALLING | PF | PULLUP );    
   gpio_init (PTB7, GPI, HIGH) ;
   port_init_NoALT (PTB7, IRQ_FALLING | PF | PULLUP );    

   set_vector_handler(PORTB_VECTORn , portb_handler);         //�ж�
   NVIC_SetPriority(PORTB_IRQn,1);                            //  �ж����ȼ�
   enable_irq (PORTB_IRQn);                                   //ʹ���ж�
   
   set_vector_handler(PORTC_VECTORn , portc_handler);         //�����ж�
   NVIC_SetPriority(PORTC_IRQn,0);                            //  �ж����ȼ�
   enable_irq (PORTC_IRQn);                                   //ʹ���ж�
   
   set_vector_handler(PORTE_VECTORn , porte_handler);         //�����ж�
   NVIC_SetPriority(PORTE_IRQn,0);                            //  �ж����ȼ�
   enable_irq (PORTE_IRQn);                                     //ʹ���ж�
   

}


void init(void)
{
    DisableInterrupts;
    
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);//�ж����ȼ���
    
    motor_init();     //���
    servo_init();     //���
    E6A2_init();      //������
    OLED_init();       //LCD��
    discern_init();   //ʶ��ģ��
    LED_init();
    GPIO_init();
    
    FTM_CnV_REG(FTMx[FTM1], FTM_CH1) = MINDTURN;
    Dianci_MAXMIN_calculate(Max_Min);                      //��������Сֵ��� 
    //  OLED_Fill(0x00);//���� 
    EnableInterrupts;
  
}

void main(void)
{
    init();
 
    while(1)
    {
  
     
//          FTM_CnV_REG(FTMx[FTM1], FTM_CH1) = turn;   
//            OLED_Print_Num(60,0,turn);  //�����ַ�����ʾ
//
//      
//     FTM_CnV_REG(FTMx[FTM1], FTM_CH1) = 19100;
//     DELAY_MS(2000);
   //  FTM_CnV_REG(FTMx[FTM1], FTM_CH1) =23550;
//     DELAY_MS(2000);
//     FTM_CnV_REG(FTMx[FTM1], FTM_CH1) = 19100;
//     DELAY_MS(2000);
//     FTM_CnV_REG(FTMx[FTM1], FTM_CH1) =14900;   //Steer    PTA13
//     DELAY_MS(2000);
//       FTM_CnV_REG(FTMx[FTM1], FTM_CH1) =18000;   //Steer    PTA13
//             DELAY_MS(2000);


//       
  //     FTM_CnV_REG(FTMx[FTM1], FTM_CH1) =22500;   //Steer    PTA13
 //      DELAY_MS(1000);
   //    FTM_CnV_REG(FTMx[FTM1], FTM_CH1) =18500;
   //    DELAY_MS(1000);
 //      FTM_CnV_REG(FTMx[FTM1], FTM_CH1) =14500;
  //     DELAY_MS(1000);
 //        DELAY_MS(2000);
//        ftm_pwm_duty(servo_FTM, servo_CH,9500);
//      DELAY_MS(2000);
//        ftm_pwm_duty(servo_FTM, servo_CH,8500);
//       DELAY_MS(2000);
//        ftm_pwm_duty(servo_FTM, servo_CH,2500);
//        DELAY_MS(2000);
//       ftm_pwm_duty(servo_FTM, servo_CH,5500);

       // control_result=discern();
      //  set_angle(control_result.angle);
//#if(motor_control==1)
//        set_speed(control_result.speed);
//        LCD_printf(0,90,"%5d",control_result.speed);
//#else
//        set_speed(200);
//#endif
        
    }
}
