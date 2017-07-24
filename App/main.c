 /*!
  *     COPYRIGHT NOTICE
  *     Copyright (c) 2013,山外科技
  *     All rights reserved.
  *     技术讨论：山外论坛 http://www.vcan123.com
  *
  *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
  *     修改内容时必须保留山外科技的版权声明。
  *
  * @file       main.c
  * @brief      山外K60 平台主程序
  * @author     山外科技
  * @version    v5.0
  * @date       2013-08-28
  */
/*
  lanzhou 试植入， 2017.4.17   Loco.
    电磁采集植入 测试只有AB 两通道有效  待解决  2017.4.22  Loco.

   通过仿真理论上舵机能随采集电感值动作，待验证 电机速度未处理  2017.4.24 Loco.
    摄像头处理待屏蔽   2017.4.24 Loco.

    舵机初步打角，打角不够，舵机有叫声(舵机频率？？？)，2017.4.29 Loco.

    舵机中值18500，，，2017.5.1Loco.
    speedpid   与舵机打角   2017.5.1Loco.
       //SpeedPID导致ADC3与ADC5出问题   待解决2017.5.2Loco.
  第九届电磁的电机pid修改植入，车能跑，问题太多待解决2017.5.2Loco.
     加干簧管   （PTB6.7 待解决）   2017.5.3Loco.
    PTB,PTE不进中断   待解决 2017.5.4 Loco.
    按键中断解决（初始化拉高） 速度pid待定  2017.5.5Loco.
    程序整理   Juan.2017.5.6
      
//2017.5.6
int16 speedpid(uint16_t AmSpeed)  待定2017.5.6Loco.
 撞弯能跑完，反跑，入十字左转，电感通道234又显100  2017.5.6Loco.


电驱发烫    电机初始化？？？？？？？  2017.5.9
    mos发热 ！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！2017.5.12Loco

   
   speedpid函数  32位  （防止溢出）
mos管，硬件问题。2017.5.13
      P 71    I  12
    k60 偶尔复位？？？？？   2017.5.14

     复位问题底盘静电    停车距离两个路肩 第一次起跑线停车待屏蔽 （moto.c  关中断使能）   电池电压7.3v 低速能跑2017.5.15 Loco.
    
    换舵机   从头再来2 017.5.16  Loco.   T . T

      jixie机械结构右转不够  2017.5.17 
        
        中速能跑，正跑十字弯撞墙（左转不够），反跑（内切）？冲突，2017.5.20Loco.
       舵机中值该位19300（原始偏左）   2017.5.20Loco.

        K4 高速 计时未减， 电池电压影响 微调  //2017.5.21 1:00 Loco.

     电池电压7.8以上，17s  18s（K4冲刺920未加速）
     电机限幅2500   ？？？什么鬼
       
       电机pid重新处理  限幅9900 P 12   D  2   2m 2017.5.23  电池电压！！！
    舵机中断  5ms
      速度570 微调？？、2017.5.24 2:46
        K5保底  2015.5.24 11:12
    
 */ 
    

#include "include.h"
    
int16 LeftAD,RightAD,LAD,RAD;
int16 AimSpeed=400;
int16_t TurnPWM;       //2017.4.26

//LED初始化
void LED_init(void)
{
    led_init(LED0);
    led_init(LED1);
    led_init(LED2);
    led_init(LED3);
}

//GPIO初始化
void GPIO_init(void)
{
   //拨码开关初始化
   uart_init (UART4 ,115200);   
   gpio_init (PTC7, GPI, HIGH) ;                
   gpio_init (PTC8, GPI, HIGH) ;               
   gpio_init (PTC9, GPI, HIGH) ;              
   gpio_init (PTC10, GPI, HIGH) ;               
   gpio_init (PTC11, GPI, HIGH) ;              
   gpio_init (PTC12, GPI, HIGH) ;
   
   //按键初始化
   gpio_init (PTC16, GPI, HIGH) ;
   port_init_NoALT (PTC16, IRQ_FALLING | PF | PULLUP );    //初始化 PTC16 管脚，下降沿触发中断，带无源滤波器，保留原先复用功能，上拉电阻
   gpio_init (PTC17, GPI, HIGH) ;
   port_init_NoALT (PTC17, IRQ_FALLING | PF | PULLUP );   
   gpio_init (PTE26, GPI, HIGH) ;
   port_init_NoALT (PTE26, IRQ_FALLING | PF | PULLUP );   
   gpio_init (PTE27, GPI, HIGH) ;
   port_init_NoALT (PTE27, IRQ_FALLING | PF | PULLUP );   
  
   //干簧管
   gpio_init (PTB6, GPI, HIGH) ;
   port_init_NoALT (PTB6, IRQ_FALLING | PF | PULLUP );    
   gpio_init (PTB7, GPI, HIGH) ;
   port_init_NoALT (PTB7, IRQ_FALLING | PF | PULLUP );    

   set_vector_handler(PORTB_VECTORn , portb_handler);         //中断
   NVIC_SetPriority(PORTB_IRQn,1);                            //  中断优先级
   enable_irq (PORTB_IRQn);                                   //使能中断
   
   set_vector_handler(PORTC_VECTORn , portc_handler);         //按键中断
   NVIC_SetPriority(PORTC_IRQn,0);                            //  中断优先级
   enable_irq (PORTC_IRQn);                                   //使能中断
   
   set_vector_handler(PORTE_VECTORn , porte_handler);         //按键中断
   NVIC_SetPriority(PORTE_IRQn,0);                            //  中断优先级
   enable_irq (PORTE_IRQn);                                     //使能中断
   

}


void init(void)
{
    DisableInterrupts;
    
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_4);//中断优先级组
    
    motor_init();     //电机
    servo_init();     //舵机
    E6A2_init();      //编码器
    OLED_init();       //LCD屏
    discern_init();   //识别模块
    LED_init();
    GPIO_init();
    
    FTM_CnV_REG(FTMx[FTM1], FTM_CH1) = MINDTURN;
    Dianci_MAXMIN_calculate(Max_Min);                      //电磁最大最小值检测 
    //  OLED_Fill(0x00);//黑屏 
    EnableInterrupts;
  
}

void main(void)
{
    init();
 
    while(1)
    {
  
     
//          FTM_CnV_REG(FTMx[FTM1], FTM_CH1) = turn;   
//            OLED_Print_Num(60,0,turn);  //汉字字符串显示
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
