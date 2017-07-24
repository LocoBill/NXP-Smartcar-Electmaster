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
  
    Data_analyse();           //定时器中断处理数据，待解决。2017.4.17 Loco. 
    TurnPWM=Steer_control(); 
       // TurnPWM=Turn1();  
   // TurnPWM1=TurnPWM; 
    FTM_CnV_REG(FTMN[FTM1], FTM_CH1) =TurnPWM; 
 //  AimSpeed=saidao_analyse();   
 //  SPeedAim(AimSpeed);      //车速打开舵机向左不打角2017.5.1
   // Speed_control(AimSpeed);   //2017.4.30  电机pd  

    led (LED0,LED_ON);
    
    
    //   if(stopflag>=2)
   //   {
    
        
   //   }        
      
 /*  
      if(K8&&(!K7))
    {
      if(Number>=300)    //让车跑3秒钟
       { 
          AimSpeed=0;//电机控制
           stop_flag=1;
       }
    }  
    else if(K8&&K7)
    {
      if(Number>=600)    //让车跑6秒钟
       { 
          AimSpeed=0;//电机控制
           stop_flag=1;
       }
    }
//停车控制  
    if(Number>=800) //8秒之后进行停车检测
   {  Number=800 ;
     if(K26||K27)
       stop_flag=1;
      
   }
   
   if(stop_flag==1)
   
     AimSpeed=0;//电机控制
   
    SPeedAim(AimSpeed);// 目标速度值  0X1A  改这个值可以调整车速。
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
   Emcinit() ;       //电磁传感器AD初始化
   pit_init_ms(PIT1, adc_period);                         //初始化PIT1，定时时间为： adc_period ms
   set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);          //设置PIT1的中断服务函数为 PIT1_IRQHandler
   NVIC_SetPriority(PIT1_IRQn,2);                                 //E6A2   中断优先级也是2   待测   2017.4.22
   enable_irq (PIT1_IRQn);                                     //使能PIT1中断
  
}

