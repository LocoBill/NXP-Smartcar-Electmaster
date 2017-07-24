#include "include.h"
#include <math.h>

extern int16_t TurnPWM;
extern int16 distence_store[30];
extern int16 steer_pwmdty;
extern int16_t speedCount ,SpeedPWM  ;
extern int16 AimSpeed;
extern int16 temp1;
extern int16 distence;
extern int16 Goal_speed;
extern float  AD_adjust_one[6]; //��һ�����ݴ������
int16 send[5];
extern int in_shi,out_shi;

extern int16 x1,x2,x3;     //Ϊ�����Ⱥ�,��һ����ƣ��ֱ�PI
extern float Kp_L;                                            //������PI�������Ե�.
extern float Ki_L;

int32 time_flag=0;
speed_t motor_speed=0;      //2017.4.26
static volatile speed_t goal_speed=0;

speed_t get_speed(void)
{
    return motor_speed;
}

static void PIT0_IRQHandler(void)
{
#if(sampling_period==20)
    motor_speed=-ftm_quad_get(quad_module);
#else
    //ת��Ϊ20ms�ڵı���ֵ
    motor_speed=-(speed_t)((double)ftm_quad_get(quad_module)/sampling_period*20);  //2017.5.5 �Ӹ���
#endif

    ftm_quad_clean(quad_module);
    PIT_Flag_Clear(PIT0);
  //  time_flag++;
   
    send[0] = motor_speed;
    send[1] = AimSpeed;
    send[2] = SpeedPWM;
    send[3] = Kp;
    send[4] = Kd;

    vcan_sendware(send, sizeof(send));
    
      OLED_Fill(0x00);//����
      //  OLED_Print_Num(2,0,TurnPWM);  //�����ַ�����ʾ
      //  OLED_Print_Num(15,0,steer_pwmdty);  //�����ַ�����ʾ
      OLED_Print_Num(5,0,AimSpeed);  //�����ַ�����ʾ
      OLED_Print_Num(40,0,motor_speed);  //�����ַ�����ʾ
      
      if(distence<0)
      {     
            OLED_Print_Str(75,0,"-");  //�����ַ�����ʾ
            OLED_Print_Num(80,0,65535-distence);  //�����ַ�����ʾ
      }
      else OLED_Print_Num(80,0,distence);  //�����ַ�����ʾ
    
      OLED_Print_Num(5,2,angle_p*100);  //�����ַ�����ʾ
      OLED_Print_Num(5,4,angle_d*100);  //�����ַ�����ʾ

     // OLED_Print_Num(5,2,*K5_value1);  //�����ַ�����ʾ
    //  OLED_Print_Num(5,4,*K5_value2);  //�����ַ�����ʾ
      
   //   OLED_Print_Str(30,2,"0,5:");  //�����ַ�����ʾ
   //   OLED_Print_Str(30,4,"1,4:");  //�����ַ�����ʾ
      OLED_Print_Str(5,6,"���2,3:");  //�����ַ�����ʾ
      OLED_Print_Num(70,2,(uint16)AD_adjust_one[0]);  //�����ַ�����ʾ       
         
      OLED_Print_Num(99,2,(uint16)AD_adjust_one[5]);  //�����ַ�����ʾ                
         
      OLED_Print_Num(70,4,(uint16)AD_adjust_one[1]);  //�����ַ�����ʾ   
         
      OLED_Print_Num(99,4,(uint16)AD_adjust_one[4]);  //�����ַ�����ʾ   
          
      OLED_Print_Num(70,6,(uint16)AD_adjust_one[2]);  //�����ַ�����ʾ     
      OLED_Print_Num(99,6,(uint16)AD_adjust_one[3]);  //�����ַ�����ʾ            
      DELAY_MS(1);
 
//    if(time_flag>=500)
//    {
//       
//       // gpio_init (PTB6, GPI, HIGH) ;
//       // gpio_init (PTB7, GPI, HIGH) ;
//        enable_irq (PORTB_IRQn); 
//    }
       if(K6)
    {
      led (LED0,LED_OFF);
      led (LED1,LED_OFF);       
      led (LED2,LED_OFF);
      led (LED3,LED_OFF);
      time_flag++;
      if((time_flag >= 180) && (time_flag <= 200))
      {
      //   enable_irq (PIT2_IRQn);  //���ʹ��  
        //   SPeedAim(1000);       //���ܼ��� 1000     
//         ftm_pwm_duty(FTM0, FTM_CH3, 2200);  
//         ftm_pwm_duty(FTM0, FTM_CH4, 0);
          // enable_irq (PORTB_IRQn);  //
         
      }
      else if (time_flag > 201)
      {
         enable_irq (PIT2_IRQn);  //���ʹ�� 
         if(time_flag >= 65500)
         {
           time_flag =201;
         }
      }
    }
    
    if(!K6)
    {
           Carstop=0;
    }
}

void E6A2_init(void)
{
    ftm_quad_init(quad_module);                                 //FTM2 ���������ʼ�������õĹܽſɲ� port_cfg.h ��
    pit_init_ms(PIT0, sampling_period);                         //��ʼ��PIT0����ʱʱ��Ϊ�� sampling_period ms
    set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);          //����PIT0���жϷ�����Ϊ PIT0_IRQHandler
    NVIC_SetPriority(PIT0_IRQn,1);                             
    enable_irq (PIT0_IRQn);                                     //ʹ��PIT0�ж�
}


