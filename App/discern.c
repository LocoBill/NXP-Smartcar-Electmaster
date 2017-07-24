#include "include.h"
#include "algorithm.h"
#include <string.h>

#define adc_period 5

#define K7   !gpio_get(PTC16)
#define K8   !gpio_get(PTC17)
#define K26   !gpio_get(PTE26)
#define K27   !gpio_get(PTE27)

extern float  SpeedKP     ;
extern int16 AimSpeed;
extern int16_t TurnPWM;
extern int16 LeftAD,RightAD,LAD,RAD;
uint8  TimeCount = 0 ,stop_flag=0,number_stop=450;
int16_t TurnPWM1;
//int16_t TurnPWM1;
int16 Number=0;


static void PIT1_IRQHandler(void)
{
 // 2017.4.22  Loco
  
    Data_analyse();           //��ʱ���жϴ������ݣ��������2017.4.17 Loco. 
    TurnPWM=Steer_control(); 
       // TurnPWM=Turn1();  
   // TurnPWM1=TurnPWM; 
    FTM_CnV_REG(FTMN[FTM1], FTM_CH1) =TurnPWM; 
 //  AimSpeed=saidao_analyse();   
 //  SPeedAim(AimSpeed);      //���ٴ򿪶�����󲻴��2017.5.1
   // Speed_control(AimSpeed);   //2017.4.30  ���pd  

    led (LED0,LED_ON);
    
    
    //   if(stopflag>=2)
   //   {
    
        
   //   }        
      
 /*  
      if(K8&&(!K7))
    {
      if(Number>=300)    //�ó���3����
       { 
          AimSpeed=0;//�������
           stop_flag=1;
       }
    }  
    else if(K8&&K7)
    {
      if(Number>=600)    //�ó���6����
       { 
          AimSpeed=0;//�������
           stop_flag=1;
       }
    }
//ͣ������  
    if(Number>=800) //8��֮�����ͣ�����
   {  Number=800 ;
     if(K26||K27)
       stop_flag=1;
      
   }
   
   if(stop_flag==1)
   
     AimSpeed=0;//�������
   
    SPeedAim(AimSpeed);// Ŀ���ٶ�ֵ  0X1A  �����ֵ���Ե������١�
  if(stop_flag==1)
  {
      if(number_stop>0)
     {  number_stop--;
        FTM_CnV_REG(FTMN[FTM0], FTM_CH1) = 400;
        FTM_CnV_REG(FTMN[FTM0], FTM_CH2) = 0;
    
      }
  }
   */
     
}

void discern_init(void)
{   
   Emcinit() ;       //��Ŵ�����AD��ʼ��
   pit_init_ms(PIT1, adc_period);                         //��ʼ��PIT1����ʱʱ��Ϊ�� adc_period ms
   set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);          //����PIT1���жϷ�����Ϊ PIT1_IRQHandler
   NVIC_SetPriority(PIT1_IRQn,2);                                 //E6A2   �ж����ȼ�Ҳ��2   ����   2017.4.22
   enable_irq (PIT1_IRQn);                                     //ʹ��PIT1�ж�
  
}

