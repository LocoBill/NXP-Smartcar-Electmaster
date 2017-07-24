/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       MK60_port.c
 * @brief      port�������ú͸��ֹ�������
 * @author     ɽ��Ƽ�
 * @version    v5.1
 * @date       2014-04-25
 */

#include "common.h"
#include "MK60_port.h"
#include  "VCAN_LED.H"   
#include  "MK60_FTM.H" 
#include "include.h"


extern int16 turn;
extern float Kp_L;                                            //������PI�������Ե�.
extern float Ki_L;
extern volatile struct FTM_MemMap *FTMx[3];
int stopflag=0;
extern int16 AimSpeed;
int Carstop=0;

PORT_MemMapPtr PORTX[PTX_MAX] = {PORTA_BASE_PTR, PORTB_BASE_PTR, PORTC_BASE_PTR, PORTD_BASE_PTR, PORTE_BASE_PTR};

/*!
 *  @brief      PORT��ʼ��
 *  @param      PTxn    �˿�
 *  @param      cfg     �˿��������ã��紥��ѡ�����������ѡ��
 *  @since      v5.0
 *  @note       ��port_init_NoALT��ͬ���ǣ��˺�����Ҫ���� MUX ���ù��ܣ�
                ���� MUX = ALT0
 *  Sample usage:       port_init (PTA8, IRQ_RISING | PF | ALT1 | PULLUP );    //��ʼ�� PTA8 �ܽţ������ش����жϣ�����Դ�˲��������ù���ΪGPIO ����������
 */
void  port_init(PTXn_e ptxn, uint32 cfg )
{
    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK << PTX(ptxn));                           //����PORTx�˿�

    PORT_ISFR_REG(PORTX_BASE(ptxn)) = (1<<PTn(ptxn));                           // ��ձ�־λ

    PORT_PCR_REG(PORTX_BASE(ptxn), PTn(ptxn)) = cfg;                            // ���ù��� , ȷ������ģʽ ,������������������
}

/*!
 *  @brief      PORT��ʼ��
 *  @param      PTxn    �˿�
 *  @param      cfg     �˿��������ã��紥��ѡ�����������ѡ��
 *  @since      v5.0
 *  @note       ��port_init��ͬ���ǣ��˺�������Ҫ���� MUX ���ù��ܣ���ʹ������Ҳ����Ч����
                MUX ���� Ϊԭ�ȼĴ������õ�ֵ
 *  Sample usage:       port_init_NoALT (PTA8, IRQ_RISING | PF | PULLUP );    //��ʼ�� PTA8 �ܽţ������ش����жϣ�����Դ�˲���������ԭ�ȸ��ù��ܣ���������
 */
void  port_init_NoALT(PTXn_e ptxn, uint32 cfg)
{
    SIM_SCGC5 |= (SIM_SCGC5_PORTA_MASK << PTX(ptxn));                           //����PORTx�˿�

    PORT_ISFR_REG(PORTX_BASE(ptxn)) = (1<<PTn(ptxn));                           // ��ձ�־λ

    //���cfg���MUX�����ؼĴ������MUX
    cfg &= ~PORT_PCR_MUX_MASK;                      //����MUX �ֶΣ�������Ҫ����ALT������ԭ����ALT��
    cfg |=  (PORT_PCR_REG(PORTX_BASE(ptxn), PTn(ptxn)) & PORT_PCR_MUX_MASK);    //��ȡ�Ĵ��������õ� MUX

    PORT_PCR_REG(PORTX_BASE(ptxn), PTn(ptxn)) = cfg;            // ���ù��� , ȷ������ģʽ ,������������������
}

/*!
 *  @brief      PORTA�Ĳο��жϷ�����
 *  @since      v5.0
 *  @warning    �˺�����Ҫ�û������Լ�������ɣ�����������ṩһ��ģ��
 *  Sample usage:       set_vector_handler(PORTA_VECTORn , porta_handler);    //�� porta_handler ������ӵ��ж�����������Ҫ�����ֶ�����
 */
void porta_handler(void)
{
    uint8  n = 0;    //���ź�

    //PTA6
    n = 6;
    if(PORTA_ISFR & (1 << n))           //PTA6�����ж�
    {
        PORTA_ISFR  = (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */


        /*  ����Ϊ�û�����  */
    }

    //���� PTA7 ��Ҫִ�е��û�������Ϊ func() �������ֱ�ӵ������º궨�壺
    //PORT_FUNC(A,7,func);
    //���������������� PTA6 ��Ч����һ���ģ�ֻ�����Ǽ����û�������
}


void portb_handler(void)
{
    uint8  n = 0,m=0;    //���ź�

    //PTC16
    n = 6;
    if(PORTB_ISFR & (1 << n))           //PTA6�����ж�
    {
        PORTB_ISFR  = (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */
    //  stopflag++;
     //      DELAY_MS(500);
  //      stopflag=0;
        
    //    disable_irq (PIT2_IRQn);                                     //ʹ��PIT1�ж�
    //    SPeedAim(0); 
     //   DELAY_MS(800);
        /*
        disable_irq (PIT2_IRQn); 
         SPeedAim(0);
         DELAY_MS(AimSpeed);
        ftm_pwm_duty(FTM0, FTM_CH3, 0);  
        ftm_pwm_duty(FTM0, FTM_CH4, 0); 
        led (LED0,LED_ON);
        led (LED1,LED_ON);       
        led (LED2,LED_ON);
        led (LED3,LED_ON); 
        */
        if(time_flag>500)        
        Carstop=1;
   /*
        while(1)
        {
          
           led (LED1,LED_OFF); 
           led (LED3,LED_OFF); 
            if(K6)
            {
              led (LED0,LED_OFF);
              led (LED1,LED_OFF);       
              led (LED2,LED_OFF);
              led (LED3,LED_OFF);
              enable_irq (PIT1_IRQn);                                     //ʹ��PIT1��
              break;
            }   
          
        }
  */ 


        /*  ����Ϊ�û�����  */
    }
    m=7;
          if(PORTB_ISFR & (1 << m))           //PTA6�����ж�
    {
        PORTB_ISFR  = (1 << m);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */
    //  stopflag++;
    //      DELAY_MS(500);

      //  stopflag=0;
    //    disable_irq (PIT2_IRQn);                                     //ʹ��PIT1�ж�
   //     SPeedAim(0);
      //  DELAY_MS(800);
        /*
        disable_irq (PIT2_IRQn); 
         SPeedAim(0);
         DELAY_MS(AimSpeed);
        ftm_pwm_duty(FTM0, FTM_CH3, 0);  
        ftm_pwm_duty(FTM0, FTM_CH4, 0); 
        led (LED0,LED_ON);
        led (LED1,LED_ON);       
        led (LED2,LED_ON);
        led (LED3,LED_ON);
        */
        if(time_flag>500)
          Carstop=1;
     /*   
        while(1)
        {
          
           led (LED1,LED_OFF); 
           led (LED3,LED_OFF); 
            if(K6)
            {
              led (LED0,LED_OFF);
              led (LED1,LED_OFF);       
              led (LED2,LED_OFF);
              led (LED3,LED_OFF);
              enable_irq (PIT1_IRQn);                                     //ʹ��PIT1��
              break;
            }   
          
        }
   */
        /*  ����Ϊ�û�����  */
    }

    //���� PTA7 ��Ҫִ�е��û�������Ϊ func() �������ֱ�ӵ������º궨�壺
    //PORT_FUNC(A,7,func);
    //���������������� PTA6 ��Ч����һ���ģ�ֻ�����Ǽ����û�������
}

void portc_handler(void)
{
    uint8  n = 0,m=0;    //���ź�

    //PTC16
    n = 16;
    if(PORTC_ISFR & (1 << n))           //PTA6�����ж�
    {
        PORTC_ISFR  = (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */
//turn=turn+100;
        if(K1)
          angle_p=angle_p+0.1;
        else if(K2)
          angle_p=angle_p+0.01;
        else
           angle_p=angle_p+1;
      // *K5_value1=*K5_value1+1;
        led (LED3,LED_ON);
        DELAY_MS(200);
        led (LED3,LED_OFF);

        /*  ����Ϊ�û�����  */
    }
    m=17;
          if(PORTC_ISFR & (1 << m))           //PTA6�����ж�
    {
        PORTC_ISFR  = (1 << m);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */
//turn=turn-100;
          if(K1)
          angle_p=angle_p-0.1;
         else if(K2)
          angle_p=angle_p-0.01;
         else
            angle_p=angle_p-1;
       // *K5_value1=*K5_value1-1;
        led (LED3,LED_ON);
        DELAY_MS(200);
        led (LED3,LED_OFF);

        /*  ����Ϊ�û�����  */
    }
    
 
    
    
}


void porte_handler(void)
{
    uint8  n = 0,m=0;    //���ź�

    //PTC16
    n = 26;
    if(PORTE_ISFR & (1 << n))           //PTA6�����ж�
    {
        PORTE_ISFR  = (1 << n);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */
//turn=turn+100;
          if(K1)
          angle_d=angle_d+0.1;
         else if(K2)
          angle_d=angle_d+0.01;
         else
            angle_d=angle_d+1;
      //  *K5_value2=*K5_value2+1;
        led (LED2,LED_ON);
        DELAY_MS(200);
        led (LED2,LED_OFF);

        /*  ����Ϊ�û�����  */
    }
    m=27;
          if(PORTE_ISFR & (1 << m))           //PTA6�����ж�
    {
        PORTE_ISFR  = (1 << m);        //д1���жϱ�־λ

        /*  ����Ϊ�û�����  */
//turn=turn-100;
         if(K1)
          angle_d=angle_d-0.1;
         else if(K2)
          angle_d=angle_d-0.01;
         else
            angle_d=angle_d-1;
      //  *K5_value2=*K5_value2-1;
        led (LED2,LED_ON);
        DELAY_MS(200);
        led (LED2,LED_OFF);

        /*  ����Ϊ�û�����  */
    }
    
}
