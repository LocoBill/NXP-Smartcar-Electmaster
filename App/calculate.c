#include "include.h"
#include "common.h"
  
//#include  "delay.h"
// 2014.02.18. 
//ok 

#define   NM    8
#define   T1    70
#define   T2    20
#define   T3    20


//拨码开关初始化


int16 *K5_value1=0,*K5_value2=0;
int in_shi=0,out_shi=0;


float Kp,Kd,angle_p=139.00,angle_d=13.00;
int16 m3=3;
int16 m4=9;
int16 m5=40;
int16 temp1;
int16 steer_pwmdty,steer_mid=19100;
int16 STEER_MAX=2200;

//lanzhou
volatile struct FTM_MemMap *FTMx[3] = {FTM0_BASE_PTR, FTM1_BASE_PTR, FTM2_BASE_PTR}; //定义三个指针数组保存 FTMn 的地址


float  AD_adjust_one[6]; //归一化数据存放数组
float ad_adjust_one[6];
float ad_averge[6];
int16  AD_valu[6],AD_V[6][NM];
int16  AD_sum[6];
int16  Temp[8];
int16 distence,distence_old;    
int16 distence_store[30];  //数组优化打脚,优化速度 
int16 distence_sum_weight; //前五加权  用于P

/////////速度调节器///////
 int32_t speedCount ,SpeedPWM  ;
uint8 allow_i_left_flag=1;      //允许积分
uint8 allow_i_right_flag=1;
int16 speed_old=0;        //上次实际速度
int16 now_speed=0,motor_speed_old=0;
int16 x1=0,x2=0,x3=0;     //为不分先后,在一起控制，分别PI
int16 y1=0,y2=0,y3=0;
float Kp_Speed[4];
float Ki_Speed[4];        //都必须大于等于0     学习次数取3    实际满足条件一直在学习
float speed_err[4]={0,0,0,0};
int8 q=0;                         //学习次数取3
float p1=10;                       //调整速率
float p2=10;
float i1=2;
float Kp_L=86;                                            //智能型PI，参数自调.
float Ki_L=0;    //左边初始化PI       
//#define P 5              //初始P  0.8    I  0.3
//#define I 0
//#define D 0
int16 P = 12;
int16 I = 2;
int16 D = 0;
#define Speed_A (P+I+D)
#define Speed_B (P+2*D)
#define Speed_C (D)
int32 result;

float  SpeedKP = 3.2 ;
float  SpeedKI = 1.0 ;
float  SpeedKD = 1.3 ;
float  Speedcontrol = 500;
int16  Max_Min[8];

int16_t speedcount=0;

int8 left_right_flag=2;      //左右判定        12/3
int8 left_right_flag_old;    
int8 chuandao_flag=0;

/////////速度变化表///////////
int8 stright_flag=1;             //直道，小s <=30
int8 stright_into_min_wan=0;     //直进小弯
int8 min_wan_into_mid_wan=0;     //小-中
int8 mid_wan_into_max_wan=0;     //中-大

int8 min_wan_flag=0;             //小弧     30-150
int8 mid_wan_flag=0;             //中弧     150-250
int8 max_wan_flag=0;             //大弧     >250

int8 max_wan_into_mid_wan=0;     //大-中
int8 mid_wan_into_min_wan=0;     //中-小
int8 min_wan_into_stright=0;     //小-直
int8 stright_into_zhijiao=0;     //直道进直角

int8 chuwan_flag=0;     //出十字弯道
int8 ruwan_flag=0;     //入十字弯道
int8 wan_in_flag=0;     //入十字弯道
int8 shun_ni_flag=2;     //顺逆时针
int8 shun_ni_flag_old=2;  
int8 speed_flag=0;
float wandao_beishu=0.6;
int8  out_stop_flag=0;     //冲出停车
int16 out_stop_time=0;     //冲出停车时间
int16 Goal_speed=0;
int16 speed_choose[5][11]=
{
  //速度      最大450
  //速度基底,直道加速，弯道加，出弯加,直进弯减           //都随偏差加减   标志间不会有冲突        //先加后减
// {100,10,8,5,2,0,0,0,0,0,0},           //保底速度         mode 0
// {300,10,8,5,2,0,0,0,0,0,0},           //中档             mode 1
// {400,10,8,5,2,0,0,0,0,0,0},           //高档             mode 2
// {500,10,8,5,2,0,0,0,0,0,0}            //冲刺             mode 3
 
 {120,10,8,5,2,0,0,0,0,0,0},           //保底速度         mode 0
 {280,10,8,5,2,0,0,0,0,0,0},           //中档             mode 1
 {520,10,8,5,2,0,0,0,0,0,0},           //高档             mode 2
 {520,10,8,5,2,0,0,0,0,0,0},           //冲刺             mode 3   //920
 {670,10,8,5,2,0,0,0,0,0,0}     //K5

// {600,60,50,30,10,0,0,0,0,0,0},           //保底速度         mode 0
// {1000,100,80,50,20,0,0,0,0,0,0},           //中档             mode 1
// {1200,120,100,60,30,0,0,0,0,0,0},           //高档             mode 2
// {1600,160,140,90,40,0,0,0,0,0,0}            //冲刺             mode 3

};


int16 speed_sta;    //基准       

int16 speed_add1;   //直道,小s中       //随标志加减    20       特定地方相当与规则查表  如下：
int16 speed_add2;   //小弯道中                     15
int16 speed_add3;   //中弯道中                     10
int16 speed_add4;   //大弯道中                     5

int16 speed_add5;   //       
int16 speed_add6;   //
int16 speed_add7;   //


void pwminit()//PWM初始化
{
  //gpio_Interrupt_init(PTA,19, GPI_DOWN, RING) ; //初始化编码器IO
 // gpio_Interrupt_init(PTC,5, GPI_DOWN, RING) ;  //初始化编码器IO 
  
  /*  
          gpio_Interrupt_init(PTA,19, GPI_DOWN, RING) ;为蓝宙函数（gpio.c）
         RING为中断模式  #define RING          0x09    什么鬼！！！！！！！！！
  ！！！！！！！！！！！！！！！！！！！！   
  ！！！！！！！！！！！！！！！！！！！！
  */
/*
//编码器
  //正交解码模块通道  端口          可选范围              建议
#define FTM1_QDPHA_PIN  PTA12       //PTA8、PTA12、PTB0
#define FTM1_QDPHB_PIN  PTA13       //PTA9、PTA13、PTB1

#define FTM2_QDPHA_PIN  PTA10       //PTA10、PTB18
#define FTM2_QDPHB_PIN  PTA11       //PTA11、PTB19     2017.4.11 Loco,

*/
 // gpio_init (PTA19, GPI,0);
 // gpio_init (PTC5, GPI,0);
  gpio_init (PTA10, GPI,0);
  gpio_init (PTA11, GPI,0);    //4.11
  //FTM_PWM_init(FTM1, CH0, 180, 23);   //舵机  MOD = 31250
  // 舵机参数最大值 31250
  ftm_pwm_init(FTM1, FTM_CH1, 300, 450);   //舵机  MOD = 31250
  // 舵机参数最大值 31250    PTA13
  
  //FTM_PWM_init(FTM0, CH0, 10000, 0);  //电机  MOD=2500
  ftm_pwm_init(FTM0, FTM_CH3, 10000, 0);  //电机参数最大值 2500 //PTA6        2017.4.5   Loco
  ftm_pwm_init(FTM0, FTM_CH4, 10000, 0);  //电机PWM初始化 频率10Khz 占空比0，占空比最大2500.PTA7
  //FTM_PWM_init(FTM0, CH3, 10000, 0);
  
}

void Emcinit()
{
  
  
//#define AMP1     ADC0_SE8        //PTB0
//#define AMP2     ADC0_SE9        //PTB1
//#define AMP3     ADC0_SE12       //PTB2
//#define AMP4     ADC0_SE13       //PTB3
//#define AMP5     ADC1_SE10       //PTB4
//#define AMP6     ADC1_SE11       //PTB5

    adc_init(ADC0,SE8);
    adc_init(ADC0,SE9);
    adc_init(ADC0,SE12);
    adc_init(ADC0,SE13);
    adc_init(ADC1,SE10);
    adc_init(ADC1,SE11);
    
//    adc_init(ADC1,SE6a);     //PE2
//    adc_init(ADC1,SE7a);     //PE3
//    adc_init(ADC1,SE10);     //PB4
//    adc_init(ADC1,SE11);     //PB5
//    adc_init(ADC1,SE12);     //PB6
//    adc_init(ADC1,SE13);     //PE2
//  
}
/*   关于程序注释中右弯道的定义  
|
|
|
|
|      右弯道
|
|

|——|   车前轮
| 
|   小车
|——|   车后轮   俯视图
上述为程序中注释右弯道的定义，左弯道与此相反，不在叙述
** ===================================================================
** Turn
输入：  
输出 ：Apwm  目标转向值 ；
** ===================================================================
*/

float  controlvalue1,controlvalue2,controlvalue3;
int16 Turn1()
{
  float TurnPWM,oldTurnPWM ,Apwm;
  static float Turn1_Err;
  float Turn2_Err,Turn_EC,Turn_store[10],Turn_sum;
  float leftvalueB, rightvalueD,leftvalueA , rightvalueE,leftvalueC,rightvalueF;
  float Turn_P_Value,Turn_D_Value;
  float subvalue1, addvalue1,controlvalue,subvalue2, addvalue2,subvalue3, addvalue3; //定义3个虚拟电感
  float   Kp=44,Kd=0,KP,KD;                        
  int16 i,m=0;
  Data_analyse();
   //Read_AD();
      leftvalueA   = AD_adjust_one[0]-7;
      leftvalueB   = AD_adjust_one[1]-10;
      leftvalueC   = AD_adjust_one[2];
      rightvalueD  = AD_adjust_one[3];
      rightvalueE  = AD_adjust_one[4]-10;
      rightvalueF  = AD_adjust_one[5]-10;
  if(leftvalueA<0)
    leftvalueA=0;
  if(leftvalueB<0)
    leftvalueB=0;
  if(rightvalueE<0)
    rightvalueE=0;
  if(rightvalueF<0)
    rightvalueF=0;
 //计算差比和---第一组-常用
      
    subvalue1=rightvalueF-leftvalueA ;  //只能用右边减去左边——求差
    addvalue1=leftvalueA+rightvalueF ; //求和
    
 //计算差比和---第二组-备用防丢线串道
      
    subvalue2=rightvalueE-leftvalueB ;  //只能用右边减去左边——求差
    addvalue2=leftvalueB+rightvalueE ; //求和
    
 //计算差比和---第三组-直角加十字
      
    subvalue3=rightvalueD-leftvalueC ;  //只能用右边减去左边——求差
    addvalue3=leftvalueC+rightvalueD ; //求和   
      
    if(addvalue1==0)//无电感值时防抖处理
    {
       TurnPWM      = 0;
       controlvalue1= 0;     
    }
    
     if(addvalue2==0)//无电感值时防抖处理
    {
       TurnPWM      = 0;
       controlvalue2= 0;
    }
    
  //差比和放大100倍用于计算偏差
    controlvalue1  = subvalue1/addvalue1 *100;
    controlvalue2  = subvalue2/addvalue2 *100; 
    controlvalue3  = subvalue3/addvalue3 *100; 
    
    //位置式PD调节舵机
    Turn2_Err    = Turn1_Err;
    Turn1_Err    = controlvalue1;
    Turn_EC      = Turn1_Err-Turn2_Err;
 
    Turn_P_Value  = Kp*Turn1_Err;
    
    Turn_D_Value  = Kd*Turn_EC;
   
    TurnPWM       =  (int16)(Turn_D_Value+Turn_P_Value);
  
 /////////逆时针特殊处理测试////////////////
  /*
  if(left_right_flag==1)
  { 
   
     left_right_flag=2;
    if(Abs_f(controlvalue1)>0&&Abs_f(controlvalue1)<15)
    
      TurnPWM=0;
    if(Abs_f(controlvalue1)>=15&&Abs_f(controlvalue1)<55)
      
      TurnPWM=-0.9*(Abs_f(controlvalue1)-15)*(Abs_f(controlvalue1)-95);
    
    else if(Abs_f(controlvalue1)>=55)
      TurnPWM=2650+0.6*(Abs_f(controlvalue1)-10)*(Abs_f(controlvalue1)-100);     
     
      TurnPWM=-TurnPWM*1.3;
     
  }
  
  if (left_right_flag==3)
  {
    
      left_right_flag=2;
    if(Abs_f(controlvalue1)>0&&Abs_f(controlvalue1)<10)
    
      TurnPWM=0;
    if(Abs_f(controlvalue1)>=10&&Abs_f(controlvalue1)<55)
      
      TurnPWM= -0.48*(Abs_f(controlvalue1)-10)*(Abs_f(controlvalue1)-100);
  
    else if(Abs_f(controlvalue1)>=55)
      TurnPWM=2800+0.9*(Abs_f(controlvalue1)-10)*(Abs_f(controlvalue1)-100);
    
    TurnPWM=TurnPWM*1.2;
    if(controlvalue1>=50&&controlvalue1>=60)
    TurnPWM=oldTurnPWM;
  }
  
 
*/
   
  
  /*******************************************/
  if(TurnPWM>=LEFTMAX)    TurnPWM = LEFTMAX;
  if(TurnPWM <= RIGHTMAX) TurnPWM = RIGHTMAX; 
  
  Apwm = MINDTURN + TurnPWM;        
   
    
   oldTurnPWM=TurnPWM;

  return   (int16)Apwm;  
  
}



/*
** ===================================================================
** SpeedPID
输入：speedCount采集车速，AmSpeed 目标车速；  
输出 ：SpeedPWMOUT  计算车速 ；
** ===================================================================
*/


//int16_t  SpeedPWM = 0 ;

int16_t SpeedPID(uint16_t speedCount,uint16_t AmSpeed)// 比较经典的增量式PID
{                                                 // 请参考相关PID的资料
  // 在此不做展开
 
 static float LastSpeedCut0,LastSpeedCut1,LastSpeedCut2 ,SpeedLastPWMK ;
 float  SpeedPWMKP ,SpeedPWMKI ,SpeedPWMKD,SpeedPWMK ;
 float  SpeedPWMOUT;    
 float  SpeedDifference0=0;
 float  speedDEARE1,speedDEARE2,DSpeed ;     
  //LastSpeedCut0 = (int16_t) speedCount ;
  DSpeed =(int16_t) AmSpeed ;
  
  //SpeedDifference0 = DSpeed - LastSpeedCut0  ;
  SpeedDifference0 = DSpeed - (int16_t) speedCount;
  LastSpeedCut0=  SpeedDifference0;
    
    
  speedDEARE1 = LastSpeedCut0 - LastSpeedCut1;
  speedDEARE2 = LastSpeedCut2+LastSpeedCut0-2*LastSpeedCut1;
  LastSpeedCut2  = LastSpeedCut1;
  LastSpeedCut1  = LastSpeedCut0;
  
  
  //SpeedPWMKP = SpeedKP*SpeedDifference0;
  SpeedPWMKP=SpeedKP*speedDEARE1;
  if(SpeedPWMKP>KPPLUSMAX)
  {
    SpeedPWMKP = KPPLUSMAX;
  }
  else if (SpeedPWMKP <KPNEGATIVEMAX)
  {
    SpeedPWMKP = KPNEGATIVEMAX;                       
  }
  
  
  //SpeedPWMKI = SpeedKI* speedDEARE1;
     SpeedPWMKI = SpeedKI* SpeedDifference0;
  if(SpeedPWMKI > KIPLUSMAX)
  {
    SpeedPWMKI = KIPLUSMAX;
  } 
  else if(SpeedPWMKI < KINEGATIVEMAX)
  {
    SpeedPWMKI = KINEGATIVEMAX;
  }
  
  SpeedPWMKD = SpeedKD* speedDEARE2;
  
  if(SpeedPWMKD > KDPLUSMAX)
  {         
    SpeedPWMKD = KDPLUSMAX;
  } 
  else if(SpeedPWMKD < KDNEGATIVEMAX)
  {
    SpeedPWMKD = KDNEGATIVEMAX;
  }
  
  SpeedPWMK = SpeedPWMKD+SpeedPWMKI+SpeedPWMKP ;
  
  if(SpeedPWMK > KWPLUSMAX)
  {
    SpeedPWMK = KWPLUSMAX;
  }
  else if(SpeedPWMK < KWNEGATIVEMAX)
  {
    SpeedPWMK = KWNEGATIVEMAX;          
  }
  
  SpeedPWMOUT = SpeedLastPWMK + SpeedPWMK;
  if(SpeedPWMOUT < 0 )
  {
    SpeedPWMOUT = 0 ;
  } 
  else if(SpeedPWMOUT > Speedcontrol)
  {
    SpeedPWMOUT = Speedcontrol ;
  }
  SpeedLastPWMK = SpeedPWMOUT ;
  //   uart_putchar(UART0,SpeedPWMOUT);
  //   uart_putchar(UART0,SpeedPWMK);        
  return  (int16)SpeedPWMOUT ;
  
}

/********************************************
  功能说明： 电机PI调节    反馈200，输出2000
  智能型PI调节器(采用位置式，学习型)
  积分分离+分段PI
  根据单片机及编码器特性，积分量化误差可以不计，具体根据Err_speed-Err_speed_old看
  原理根据电力拖动第3版，数字PI调节器

  目前：位置式，
  需双限幅
  参考电拖P109位置式数字PI调节器框图

**********************************************/
void Speed_control(uint16_t AmSpeed)       //分开控制，调一边对另一边无影响  
{  
   //初值确定，在车上测试，等系统学习一段时间可知道
  
    Kp_Speed[q]=Kp_L;
    Ki_Speed[q]=Ki_L;

    now_speed =(motor_speed+motor_speed_old)/2;   
    motor_speed_old = motor_speed;
    
     
  //  x1=AmSpeed -motor_speed;                           //转速偏差   
    x1=AmSpeed - now_speed;                           //转速偏差                          
    x2=x2+x1;                        //转速偏差积分            
    x3=speed_old-now_speed;            //实际转速变化率负值      

     //积分限幅     小点好
   /* 
     if(x2>=300)  x2=300;
     else if(x2<=-300)  x2=-300;
     
     if(y2>=300) y2=300;
     else if(y2<=-300)  y2=-300;   
  */  
    
    speed_err[0] = x1;  //2017.5.6
     
     //每次穿越积分清零，防退饱和超调
/*   
     //积分分离      左
     if(abs(x1)<=30)     // 0.1~0.2Err
     {   
       //分段PI      左
        if(x1*x3<0)         //
        {
          Kp_Speed[q+1]=Kp_Speed[q]-p1*0.01;
          if(x1*x2>0)
            Ki_Speed[q+1]=Ki_Speed[q]+i1*0.1;        
          else
            Ki_Speed[q+1]=Ki_Speed[q]-i1*0.1;
        } 
        else if(x1*x3>0)
        {
          Kp_Speed[q+1]=Kp_Speed[q]+p2*0.01;
          if(x1*x2>0)
            Ki_Speed[q+1]=Ki_Speed[q]+i1*0.1;        
          else
            Ki_Speed[q+1]=Ki_Speed[q]-i1*0.1;  
        }
         else if(x1*x3==0)
        {
          Kp_Speed[q+1]=Kp_Speed[q];
          Ki_Speed[q+1]=Ki_Speed[q];
        }  
       
        Kp_L=Kp_Speed[q+1];
        Ki_L=Ki_Speed[q+1];
        
        if(Kp_L<0) {Kp_L=0;}
        if(Ki_L<0) {Ki_L=0;}
    // if(Kp_L>13) {Kp_L=13;}
    // if(Ki_L>10) {Ki_L=10;}
     
     }
     else if(abs(x1)>30)    
     {
       Kp_L=800/abs(x1);
       x2=0;
     }  
    //应PI>=0
     */

   //双限幅程序
 
    
   SpeedPWM=(int16)(Kp_L*x1+ Ki_L*x2);  // 2017.5.2
  
   
        if(SpeedPWM>2500)           //输出限幅 左
        {SpeedPWM=2500;}
        else 
        if(SpeedPWM<-2500)
        {SpeedPWM=-2500;}  
        
  

     //赋值
    if(SpeedPWM>=0)                //正转
    {
      //ftm_pwm_duty(FTM0, FTM_CH3,SpeedPWM);
      // ftm_pwm_duty(FTM0, FTM_CH4,0);    //赋值左边
      FTM_CnV_REG(FTMx[FTM0], FTM_CH3) = SpeedPWM ;
      FTM_CnV_REG(FTMx[FTM0], FTM_CH4) = 0;
    }
    if(SpeedPWM<0)                 //反转
    {
      SpeedPWM = 0 - SpeedPWM ;
      //  ftm_pwm_duty(FTM0, FTM_CH3,0);
      // ftm_pwm_duty(FTM0, FTM_CH4,SpeedPWM);
      FTM_CnV_REG(FTMx[FTM0], FTM_CH3) = 0 ;
      FTM_CnV_REG(FTMx[FTM0], FTM_CH4) = SpeedPWM;
    }
 
     //偏差存储  x2存了
     //Err_speed_old_left=x1;
     //Err_speed_old_right=Err_speed_right;
   
    //实际速度存储
    speed_err[3]=speed_err[2];
    speed_err[2]=speed_err[1];
    speed_err[1]=speed_err[0];

    speed_old=now_speed;
    
    //学习3次   实际一直学
    q++;
    if(q>2)
    q=0;
}



/****速度pid****/
//2017.5.6
int32 speedpid(uint16_t AmSpeed)
{
  static speed_t speed_diff[3]={0,0,0};
    
    int16 _goal_speed=AmSpeed;
    int16 _motor_speed=motor_speed;
    
    speed_diff[2]=speed_diff[1];
    speed_diff[1]=speed_diff[0];
    speed_diff[0]=_goal_speed-_motor_speed;
    
//    if(abs(speed_diff[0])<5)
//    {
//        return;
//    }
    
    result=(int32)(Speed_A*speed_diff[0]+Speed_B*speed_diff[1]+Speed_C*speed_diff[2]);
    return result;
}


/*
** ===================================================================
** SPeedAim
输入：AIMSpeed 目标车速；  
输出 ： ；
** ===================================================================
*/
void SPeedAim(int16_t AIMSpeed )    //  
{ 
 // int16_t speedCount ,SpeedPWM  ;
 // extern int16_t LeftSpeedC ,RingtSpeedC ;
 // speedCount = LeftSpeedC  ;//（左轮车速+右轮车速）/2  为什么用>>1 这有利于程序执行效率，在此不做展开。
  speedCount = motor_speed  ;    //2017.4.24
  speedcount=speedCount;
  // speedCount =  (LeftSpeedC + RingtSpeedC ) ;
  /*  
  uart_putchar(UART0,0xaa) ;
  uart_putchar(UART0,LeftSpeedC>>8);
  uart_putchar(UART0,LeftSpeedC);
  uart_putchar(UART0,RingtSpeedC>>8);
  uart_putchar(UART0,RingtSpeedC);*/
  
 // LeftSpeedC = 0 ;
 // RingtSpeedC = 0 ;
  
  //速度PID 当前车速速度 ，目标速度
 //SpeedPWM =SpeedPID(speedCount ,AIMSpeed ) ;    //speedpid与舵机打角冲突？？？2017.5.1Loco.
  //SpeedPID导致ADC3与ADC5出问题   待解决2017.5.2Loco.
  SpeedPWM =speedpid(AIMSpeed ) ;  //2017.5.6
  
  /*
  uart_putchar(UART0,0xcc) ;
  uart_putchar(UART0,SpeedPWM>>8);
  uart_putchar(UART0,SpeedPWM);
  */
  //SpeedPWM =0 ;
  if(SpeedPWM > 9900)// 速度PWM 大于2500 就不在增加  2500为最大
  {SpeedPWM = 9900 ;}//赋值2500
  else if(SpeedPWM < -9900){// +2500 和-2500 的方向不一样，正反转
    SpeedPWM = -9900 ;
  }
  if(SpeedPWM > 0)// 速度PWM 大于零 为一转向。
  {
    //FTM_CnV_REG(FTMx[FTM0], CH0) = SpeedPWM;//参考 下面CH0 注释 只是转向不一样
    //   FTM_CnV_REG(FTMx[FTM0], CH0) = 300;
 //  FTM_CnV_REG(FTMx[FTM0], FTM_CH3) = SpeedPWM ;
  // FTM_CnV_REG(FTMx[FTM0], FTM_CH4) = 0;
    
   ftm_pwm_duty(FTM0, FTM_CH3, SpeedPWM);  
   ftm_pwm_duty(FTM0, FTM_CH4, 0);  
    //  FTM_CnV_REG(FTMx[FTM0], CH2) = 500;
    //FTM_CnV_REG(FTMx[FTM0], CH3) = 0 ;
  } 
  else                     //速度PWM 小于零 为大于零的反向。
  {
    SpeedPWM = -SpeedPWM ;
    //FTM_CnV_REG(FTMx[FTM0], CH0) = 0;        //ftm0 ch0通道 占空比0            -电
  //  FTM_CnV_REG(FTMx[FTM0], FTM_CH3) = 0 ;//ftm0 ch1通道 占空比当前速度PWM  -机
  //  FTM_CnV_REG(FTMx[FTM0], FTM_CH4) = SpeedPWM;        //ftm0 ch0通道 占空比0            -电
    
    
    ftm_pwm_duty(FTM0, FTM_CH3, 0);  
    ftm_pwm_duty(FTM0, FTM_CH4, SpeedPWM); 
    //FTM_CnV_REG(FTMx[FTM0], CH3) = SpeedPWM ;//ftm0 ch1通道 占空比当前速度PWM  -机
    //          /                     - 2  
  }
   
  
}


/*
** ===================================================================
** Dianci_MAXMIN_calculate
输入：  
输出：通过扫描赛道，检测6个电感值的最大最小值（把小车从赛道最左移到最右端，以此来检测） ；
** ===================================================================
*/
void Dianci_MAXMIN_calculate(int16 *max_min)
{  
   int16 AD[6];
   int16 i,j;
 /*   
   if(K3)
   {
      angle_p=139.00;
      angle_d=13.00;
   }
   else if(K4)
   {
      angle_p=190.00;
      angle_d=13.00;
   }
    else if(K5)
   {
      angle_p=139.00;
      angle_d=13.00;
   }
   */
   max_min[0]=max_min[1]=max_min[2]=max_min[3]=max_min[4]=max_min[5]=0;  //存储ABDCEF电感的最大值
   max_min[6]=max_min[7]=max_min[8]=max_min[9]=max_min[10]=max_min[11]=20;//存储ABDCEF电感的最小值
   for(i=0;i<2400;i++)
   {  
        AD[0] = ad_ave(ADC0, AD8, ADC_12bit, 5); //A左传感器采集信号采集5次均值滤波
        AD[1] = ad_ave(ADC0, AD9, ADC_12bit, 5) ; //B左传感器采集信号采集5次均值滤波
        AD[2] = ad_ave(ADC0, AD12, ADC_12bit, 5) ; //C左传感器采集信号采集5次均值滤波 
        AD[3] = ad_ave(ADC0, AD13,  ADC_12bit, 5) ; //D右传感器采集信号采集5次均值滤波
        AD[4] = ad_ave(ADC1, AD10,  ADC_12bit, 5) ; //E右传感器采集信号采集5次均值滤波 
        AD[5] = ad_ave(ADC1, AD11, ADC_12bit, 5); //F右传感器采集信号采集5次均值滤波
        
      
       for(j=0;j<6;j++)
       {
         if(AD[j]>max_min[j])        
            max_min[j]=AD[j];
         if(AD[j]<=max_min[j+6]) 
            max_min[j+6]=AD[j];
       }
      
       DELAY_MS(1);
         in_shi=0;
        out_shi=0;
   } 
     
   //    OLED_Fill(0x00);//黑屏 
//       OLED_Print_Num(5,0,max_min[0]);           //汉字字符串显示
//       OLED_Print_Num(40,0,max_min[6]);
//       OLED_Print_Num(80,0,max_min[11]);
}

uint16 Abs(int16 x)
{
    if(x<0)  return - x;
    else     return x;
}

float Abs_f(float x)
{
    if(x<0)  return - x;
    else     return x;
}

 
void Read_AD(void)
{
  float ad_value[6][5],ad_sum[6],temp;
  int16 i,j,k;
  for(i=0;i<5;i++)//四路电感各采集5次
  {
        ad_value[0][i] = ad_ave(ADC0, SE8, ADC_12bit, 5) ; //A左传感器采集信号采集5次均值滤波
        ad_value[1][i] = ad_ave(ADC0, SE9, ADC_12bit, 5); //B左传感器采集信号采集5次均值滤波
        ad_value[2][i] = ad_ave(ADC0, SE12, ADC_12bit, 5) ; //C左传感器采集信号采集5次均值滤波 
        ad_value[3][i] = ad_ave(ADC0, SE13,  ADC_12bit, 5) ; //D右传感器采集信号采集5次均值滤波
        ad_value[4][i] = ad_ave(ADC1, SE10,  ADC_12bit, 5); //E右传感器采集信号采集5次均值滤波 
        ad_value[5][i] = ad_ave(ADC1, SE11, ADC_12bit, 5); //F右传感器采集信号采集5次均值滤波
  }
  //冒泡排序
  for(i=0;i<6;i++)
  {
    for(j=0;j<4;j++)
    {
      
      for(k=0;k<4-j;k++)
      { 
         if( ad_value[i][k]>ad_value[i][k+1])
          {
              temp              = ad_value[i][k+1] ;
              ad_value[i][k+1]  = ad_value[i][k]   ;
              ad_value[i][k]    = temp             ;
          }
      } 
    }
  }
  //去头掐尾，中间三项求平均
  for(i=0;i<6;i++)
  {
     ad_sum[i]   =ad_value[i][1]+ad_value[i][2]+ad_value[i][3];
     ad_averge[i]=ad_sum[i]/3.0;
   }
  //滑动平均滤波
  
  for(i=0;i<NM-1;i++)
  {
     AD_V[0][i]=AD_V[0][i+1];
     AD_V[1][i]=AD_V[1][i+1];
     AD_V[2][i]=AD_V[2][i+1];
     AD_V[3][i]=AD_V[3][i+1];
     AD_V[4][i]=AD_V[4][i+1];
     AD_V[5][i]=AD_V[5][i+1];
  }
  for(i=0;i<6;i++)
  {
     AD_V[i][NM-1]=(int16)ad_averge[i];
    
  }
    
  for(i = 0;i < NM;i ++)
  {
       
         AD_sum[0] += AD_V[0][i];
         AD_sum[1] += AD_V[1][i];
         AD_sum[2] += AD_V[2][i];
         AD_sum[3] += AD_V[3][i];
         AD_sum[4] += AD_V[4][i];
         AD_sum[5] += AD_V[5][i];
        
  }
  
  
   for(i=0;i<6;i++)  //求平均
   {
        
         AD_valu[i] = AD_sum[i] / NM;
         AD_sum[i] = 0; 
   }  
  
  
 //归一化处理
  for(i=0;i<6;i++)  
   {
     ad_adjust_one[i]=(float)(AD_valu[i]-Max_Min[i+6])/(float)(Max_Min[i]-Max_Min[i+6]);
        if(ad_adjust_one[i]<=0.0)        
          ad_adjust_one[i]=0.001;
          
        if(ad_adjust_one[i]>1.0)
          ad_adjust_one[i]=1.0;
        
     AD_adjust_one[i]=(int16)(ad_adjust_one[i]*100);
   
   /*  //5电感不对称，打脚不够修正*/
    if(AD_adjust_one[5]>=80&&AD_adjust_one[5]<100) 
       AD_adjust_one[5]= AD_adjust_one[5]*0.98;
    else if(AD_adjust_one[5]>=70&&AD_adjust_one[5]<80)
       AD_adjust_one[5]= AD_adjust_one[5]*0.95;
    else if(AD_adjust_one[5]>=60&&AD_adjust_one[5]<70)
       AD_adjust_one[5]= AD_adjust_one[5]*0.85;
    else if(AD_adjust_one[5]>=50&&AD_adjust_one[5]<60)
       AD_adjust_one[5]= AD_adjust_one[5]*0.8; 
    else if(AD_adjust_one[5]>=45&&AD_adjust_one[5]<50)
       AD_adjust_one[5]= AD_adjust_one[5]*0.7; 
    else if(AD_adjust_one[5]>=40&&AD_adjust_one[5]<45)
       AD_adjust_one[5]= AD_adjust_one[5]*0.6; 
    else if(AD_adjust_one[5]>=35&&AD_adjust_one[5]<40)
       AD_adjust_one[5]= AD_adjust_one[5]*0.5; 
    else if(AD_adjust_one[5]>=30&&AD_adjust_one[5]<35)
       AD_adjust_one[5]= AD_adjust_one[5]*0.4; 
    else if(AD_adjust_one[5]>=25&&AD_adjust_one[5]<30)
       AD_adjust_one[5]= AD_adjust_one[5]*0.3; 
    else if(AD_adjust_one[5]>=20&&AD_adjust_one[5]<25)
       AD_adjust_one[5]= AD_adjust_one[5]*0.2; 
   }  
  
}


/************************************************
功能描述：电感值数据分析——各种标志位置位
函数名：Data_analyse
输入：无
输出：无
************************************************/

void Data_analyse(void)
{
    int16_t i=0,temp2;  
    Read_AD();
  
  
  //水平方向预处理 
    if(AD_adjust_one[0]>=T1 ||AD_adjust_one[5]>=T1)
    {
    if(Abs_f((int16)(AD_adjust_one[0]-AD_adjust_one[5]))<=T2)
    {
      left_right_flag=2;//车子居中
    }
    else
    {
      if(AD_adjust_one[0]>AD_adjust_one[5])
        left_right_flag=3;//车子居右
      else if(AD_adjust_one[0]<AD_adjust_one[5])
        left_right_flag=1;//车子居左
    }
  }

//2017.5.5 Loco   十字弯
  else if(AD_adjust_one[0]<20 && AD_adjust_one[5]<20)        //直角十字方向修正，根据斜电感，垂直电感，水平电感，差，和，值综合处理      
  {
    if(AD_adjust_one[2]>95)
      left_right_flag=1;
    if(AD_adjust_one[2]<80)
      left_right_flag=3; 
  }
  
//  if(AD_adjust_one[1]>95 && AD_adjust_one[4]>95 && AD_adjust_one[2]>95 && AD_adjust_one[3]>95&&in_shi==0&&out_shi==0)
//  {
//    in_shi=1;out_shi=1;
//  }
//  else if(AD_adjust_one[1]<95 || AD_adjust_one[4]<95 || AD_adjust_one[2]<95 || AD_adjust_one[3]<95)
//    out_shi=0;
// else if(AD_adjust_one[1]>95 && AD_adjust_one[4]>95 && AD_adjust_one[2]>95 && AD_adjust_one[3]>95&&in_shi==1&&out_shi==0)
//    in_shi=0;out_shi=1;
   
/*
  else if(AD_adjust_one[0]<T1 && AD_adjust_one[5]<T1)        //当前排的电感出线了用后排的判断方向  
  {
   
    if((AD_adjust_one[0]<AD_adjust_one[5])&&(AD_adjust_one[1]>AD_adjust_one[4]))//顺时针弯道处理
   {
     temp2=AD_adjust_one[1];
     AD_adjust_one[1]=AD_adjust_one[4];
     AD_adjust_one[4]=temp2;
   }
   if(AD_adjust_one[1]>AD_adjust_one[4])
      left_right_flag=3;//车子居右
   if(AD_adjust_one[1]<AD_adjust_one[4])
      left_right_flag=1;//车子居左
   
  if((AD_adjust_one[0]>AD_adjust_one[5])&&(ad_averge[2]>ad_averge[3])) //逆时针弯道误判处理 已测试
   {
     if(AD_adjust_one[1]<=AD_adjust_one[4])
       left_right_flag=left_right_flag_old;

   }
  
  
  }
*/    //2017.5.2
  
  if(left_right_flag==1&&left_right_flag_old==3)
  {
    if(AD_adjust_one[0]<=T3||AD_adjust_one[5]<=T3)             //小于多少不变,可实际调 ,值太小效果不好，值太大，折角抓线有问题  ，故下面对distence直接处理防串道
      left_right_flag=left_right_flag_old;
  }
  if(left_right_flag==3&&left_right_flag_old==1)
  {
    if(AD_adjust_one[0]<=T3||AD_adjust_one[5]<=T3)                //也可直接作用于舵机控制,但现为取distence最大
      left_right_flag=left_right_flag_old;
  }
  left_right_flag_old=left_right_flag;
  
////////////计算偏差////////////////////////////              斜率一致原则             
  if(left_right_flag==2)                         
  {
     distence=-(int16)((AD_adjust_one[5]-AD_adjust_one[0])*3/2);          
  } 
   
  if(left_right_flag==1)                                                    
  {
     if(AD_adjust_one[0]>=70)
        distence=-30;
     else if(AD_adjust_one[0]>10 &&AD_adjust_one[0]<70)
     {
       distence=-30-(int16)(12000/(50+AD_adjust_one[0])-100)*3; 
     }
     else if(AD_adjust_one[0]<=10)
     distence=-330;                          
   }
  if(left_right_flag==3)
  {
      if(AD_adjust_one[5]>=70)
        distence=30;
      if(AD_adjust_one[5]>10 &&AD_adjust_one[5]<70)
      {
        distence=30+(int16)(12000/(50+AD_adjust_one[5])-100)*3;  
      }
      else if(AD_adjust_one[5]<=10)
      distence=330;
  }
   
   //中间段防跳动对D的影响      30调
 /*  
   if((abs(distence)<40 && abs(distence_old)>40)||(abs(distence)>40 && abs(distence_old)<40))
   {
    if(distence-distence_old>10)           //5ms/20ms,5次
    {
      distence=distence_old+10;
    }
    if(distence-distence_old<-10)
    {
      distence=distence_old-10;
    }
   }
  */
//   2017.5.2
   
    //解决串道 2017.5.5
   if((distence>0 && distence_old<-250)||( distence<0 && distence_old>250) )
   {  distence=distence_old;}
  
   
  distence_old=distence;

  for(i=28;i>=0;i--)                 //必须是递减的   程序单步到此舵机打角？？？2017.5.1
  {
    distence_store[i+1]=distence_store[i];
  } 
  distence_store[0]=distence_old;
  
  distence_sum_weight=(distence_store[0]*10+distence_store[1]*4+distence_store[2]*3+distence_store[3]*2+distence_store[4])/20;

   /**/
}

/********************************
功能描述：舵机打脚计算
函数名： Steer_control
输入：无
输出：无
*********************************/

int16 Steer_control(void)
{
//  if(speed_flag==1)
//  { 
//    angle_p=85;
//  } 
  /*
  if(K2)
  {
    angle_p=145;
    angle_d=12;
    if(K3)
      {
        angle_p=139;
        angle_d=13;
      }
  }
  */
 
  Kp=(int16)(angle_p+(float)(Abs_f(distence_sum_weight)*m3/100));
  
  
  Kd=(int16)(angle_d+(float)(Abs_f(distence_store[0]-distence_store[4])*m4/100)+30*m5/(Kp+10));
  temp1=(int16)((distence_sum_weight*Kp/10+(distence_store[0]-distence_store[4])* Kd/10));
//
 // if(temp1>0)
  //  temp1=(int16)(temp1*0.8);   //左转内切，0.78，，2017.5.1
//  
//   if(temp1<0)
//    temp1=(int16)(temp1*1.65);   //右转不够，，，2017.5.13
//  
//   if(AD_adjust_one[1]>95 && AD_adjust_one[4]>95 && AD_adjust_one[2]>95 && AD_adjust_one[3]>95)  //十字处理2047.5.20
//   {
//      if(left_right_flag !=2)
//        temp1=(int16)(temp1*1.5); 
//   }
//   
//    if(Abs(distence)>300)     //大弯2017.5.20
//    {
//        temp1=(int16)(temp1*1.2); 
//    }
//   
  
 
  
  if(K3)     //million speed
  {
    
//     if(temp1>0)     //左转
//     {
//         if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //反跑 180‘弯内切  2017.5.20
//           temp1=(int16)(temp1*0.2);         //0.2    
//     }
     
   //   if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //反跑 180‘弯内切  2017.5.20
   //        temp1=(int16)(temp1*0.2);         //0.2     
   // /*
     temp1=(int16)(temp1*1.05);      //外切  P141.2 141.3  D 13.4   2017.5.24 1:53
    
      if(temp1>0)     //左转
      {
        temp1=(int16)(temp1*0.92); 
        if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //反跑 180‘弯内切  2017.5.20
           temp1=(int16)(temp1*0.18);         //0.2     
//       if(Abs(distence)>300)      
//        temp1=(int16)(temp1*1.2); 
//      else  temp1=(int16)(temp1*0.75); 
//       if(out_shi==1)
//          temp1=(int16)(temp1*1.2); 
//        else  temp1=(int16)(temp1*0.8); 
      }
       //  if((temp1<0) && (Abs(distence)>300))
       //    temp1=(int16)(temp1*0.9);   //右转，2017.5.13    
  //  */
  }
  
  
  if(K4)    //high speed
  {
     temp1=(int16)(temp1*1.05);      //外切  P141.2 141.3  D 13.4   2017.5.24 1:53
     if(temp1>0)     //左转
     {
         if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //反跑 180‘弯内切  2017.5.20
           temp1=(int16)(temp1*0.2);         //0.2    
     }
    
    /*
      if(temp1>0)     //左转
      {
        temp1=(int16)(temp1*1.05); 
        if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //反跑 180‘弯内切  2017.5.20
           temp1=(int16)(temp1*0.2);         //0.2     
      }
         if((temp1<0) && (Abs(distence)>300))
           temp1=(int16)(temp1*0.9);   //右转，2017.5.13  
    */
  }
  
 
    else if(K5)    //
    { 
      if(temp1>0)     //左转
      {
        temp1=(int16)(temp1*1.05); 
        if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //反跑 180‘弯内切  2017.5.20
           temp1=(int16)(temp1*0.19);         //0.2  
      }
      /*
      if(temp1>0)     //左转
      {
        temp1=(int16)(temp1*1.05); 
        if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //反跑 180‘弯内切  2017.5.20
           temp1=(int16)(temp1*0.2);         //0.2     
      }
         if((temp1<0) && (Abs(distence)>300))
           temp1=(int16)(temp1*0.9);   //右转，2017.5.13  
      */
    }
 
  
  
  if(temp1>=LEFTMAX)    temp1 = LEFTMAX;
  if(temp1 <= RIGHTMAX) temp1 = RIGHTMAX; 
 
  steer_pwmdty = steer_mid -temp1;   //Kd可以乘以斜电感差 //Kd为原始变化，防抖
   
  if((ad_averge[2]>2000)&&(ad_averge[3]>900))
    steer_pwmdty=steer_mid;       //屏蔽舵机打角   2017.4.29
  
  
  return (int16)steer_pwmdty; 
}


/***************
函数名：saidao_analyse
函数功能描述：赛道标志设定
输入：无
输出:目标速度
******************/

int16 saidao_analyse(void)
{
  int i=0; 
  int32 distence_Abs_sum[4]={0,0,0,0};   //绝对之和，前10，中10,后10         每次清零故在内部定义
  int32 distence_alg_sum[4]={0,0,0,0};   //代数和，  
   //速度策略
 
 //1
    //30个偏差代数和
    for(i=0;i<30;i++)
    {
      distence_alg_sum[0]=distence_alg_sum[0]+distence_store[i];
    }
    distence_alg_sum[0]=distence_alg_sum[0]/30;
    //30个偏差绝对值和
    for(i=0;i<30;i++)
    {
      distence_Abs_sum[0]=distence_Abs_sum[0]+Abs(distence_store[i]);
    }
    distence_Abs_sum[0]=distence_Abs_sum[0]/30;
 //2 
    //前10代数和
    for(i=0;i<10;i++)
    {
      distence_alg_sum[1]=distence_alg_sum[1]+distence_store[i];
    }
    distence_alg_sum[1]=distence_alg_sum[1]/10;
 
    //前10绝对值和
    for(i=0;i<10;i++)
    {
      distence_Abs_sum[1]=distence_Abs_sum[1]+Abs(distence_store[i]);
    }
    distence_Abs_sum[1]=distence_Abs_sum[1]/10;
    
 //3
    //中10代数和
    for(i=10;i<20;i++)
    {
      distence_alg_sum[2]=distence_alg_sum[2]+distence_store[i];
    }
    distence_alg_sum[2]=distence_alg_sum[2]/10;
 
    //中10绝对值和
    for(i=10;i<20;i++)
    {
      distence_Abs_sum[2]=distence_Abs_sum[2]+Abs(distence_store[i]);
    }
    distence_Abs_sum[2]=distence_Abs_sum[2]/10;
//4    
   //后10代数和
    for(i=20;i<30;i++)
    {
      distence_alg_sum[3]=distence_alg_sum[3]+distence_store[i];
    }
    distence_alg_sum[3]=distence_alg_sum[3]/10;
 
    //后10绝对值和
    for(i=20;i<30;i++)
    {
      distence_Abs_sum[3]=distence_Abs_sum[3]+Abs(distence_store[i]);
    }
    distence_Abs_sum[3]=distence_Abs_sum[3]/10; 
 
    for(i=0;i<4;i++)  
    {
      Temp[i]=distence_Abs_sum[i];
    }
    for(i=0;i<4;i++)  
    {
      Temp[i+4]=distence_alg_sum[i];
    }
    /**/
//因数组有一定滞后性故用来辅助打脚，即用来控制电机速度，和差速。而把电机速度转成辅助PD控制舵机
//无论什么路况，偏差连续，只是急缓               进出根据首尾判断，弯中根据首尾和中间判断    
    //代数和很小，直道
    if(Abs(distence_alg_sum[0])<30)
    {
      stright_flag=1; 
    }
    else
      stright_flag=0; 
    
    //直-小弯 
    if(distence_Abs_sum[3]<50 && distence_Abs_sum[1]>100 )         //3级间依次标志有效，则速度减的平滑，若从小突到大，则标志同时有效，减的急
    {
      stright_into_min_wan=1;
    }
    else
      stright_into_min_wan=0;
    
    //小-中
    if(distence_Abs_sum[3]<100 && distence_Abs_sum[1]>200)
    {
      min_wan_into_mid_wan=1;  
    }
    else
      min_wan_into_mid_wan=0;
    
    //中-大
    if(distence_Abs_sum[3]<200 && distence_Abs_sum[1]>250 )
    {
      mid_wan_into_max_wan=1;
    }
    else
      mid_wan_into_max_wan=0;
    
    //小。。。
    if(Abs(distence_Abs_sum[1]-distence_Abs_sum[3])<50 && distence_Abs_sum[2]>30 &&distence_Abs_sum[2]<150)
    {
      min_wan_flag=1;
    }
    else
      min_wan_flag=0;
    //中。。
    if(Abs(distence_Abs_sum[1]-distence_Abs_sum[3])<50 && distence_Abs_sum[2]>150 &&distence_Abs_sum[2]<250)
    {
      mid_wan_flag=1;
    }
    else
      mid_wan_flag=0;
    //大。。
    if(Abs(distence_Abs_sum[1]-distence_Abs_sum[3])<50 && distence_Abs_sum[2]>250)
    {
      max_wan_flag=1;
    }
    else
      max_wan_flag=0;
    
    //大-中
     if(distence_Abs_sum[1]<200 && distence_Abs_sum[3]>250 )        //
    {
      max_wan_into_mid_wan=1;
    }
    else
      max_wan_into_mid_wan=0;
     
    //中-小
    if(distence_Abs_sum[1]<100 && distence_Abs_sum[3]>200 )
    {
      mid_wan_into_min_wan=1;
    }
    else
      mid_wan_into_min_wan=0; 
    //小-直
    if(distence_Abs_sum[1]<50 && distence_Abs_sum[3]>100 )
    {
      min_wan_into_stright=1;
    }
    else
      min_wan_into_stright=0;
      
    shun_ni_flag_old=shun_ni_flag;  
    
    ////////////////////////////////////////////////
   if((distence_Abs_sum[1]*distence_alg_sum[1])>0)//逆时针
         shun_ni_flag=1;
   else if((distence_Abs_sum[1]*distence_alg_sum[1])<0)//顺时针
         shun_ni_flag=3;
   else if((distence_Abs_sum[1]*distence_alg_sum[1])==0)
            shun_ni_flag=2;
    
   
   if (shun_ni_flag==1)//顺时针
   {
      if(distence_Abs_sum[1]>=280 && distence_Abs_sum[1]<=300 ) //出弯入弯速度处理
        chuwan_flag=1;
   }
   if (shun_ni_flag==2)//逆时针
   {
      if(distence_Abs_sum[1]>=250 && distence_Abs_sum[1]<=330 ) //出弯入弯速度处理
        chuwan_flag=1;
   }
    ////////////////////////////////////////////////
  
   if(distence_Abs_sum[1]>=100) 
      ruwan_flag=1;
   else   ruwan_flag=0;
    
   
   //拨码开关K5选参数
  /* 
   if(K5)
   {
     K5_value1 = &P;
     K5_value2 = &I;
   }
   
   if(!K5)
   {
     K5_value1 = &angle_p;   //float 冲突
     K5_value2 = &angle_d;
   }
     
*/
   
   //拨码开关调速度档位 
//   if(K1)
//   {   
         Speedcontrol=550;   
         speed_sta=speed_choose[0][0] ;
         speed_add1=speed_choose[0][1];
         speed_add2=speed_choose[0][2];
         speed_add3=speed_choose[0][3];
         speed_add4=speed_choose[0][4];
         speed_add5=speed_choose[0][5];
         speed_add6=speed_choose[0][6];
         speed_add7=speed_choose[0][7];
                                       
     /*    if(K2)                   
        { 
          speed_flag=1;
          Speedcontrol=650; 
          speed_sta=speed_choose[1][0] ;
          speed_add1=speed_choose[1][1];
         speed_add2=speed_choose[1][2];
         speed_add3=speed_choose[1][3];
         speed_add4=speed_choose[1][4];
         speed_add5=speed_choose[1][5];
         speed_add6=speed_choose[1][6];
         speed_add7=speed_choose[1][7];
      
        }
         */
                         
        if(K3)                   
        {
          Speedcontrol=670; 
          speed_sta=speed_choose[2][0] ;
        }   
       
        if(K4)                   
        {
          Speedcontrol=670; 
          speed_sta=speed_choose[2][0] ;
        //wandao_beishu=0.5 ;
        }
        
          if(K5)           //保底        
        {
          Speedcontrol=670; 
          speed_sta=speed_choose[2][0] ;
        //wandao_beishu=0.5 ;
        }
           
                         
  // }                   
                         
   //车速度=设定-偏差减（10）+直道加+小弯中+中弯中+大弯中+大出中+中出小+小出直-直进小-小进中-中进大             //后期微改直道猛加在微减，到能减下来的水平
    
//   Goal_speed=speed_sta-Abs(distence)/33+speed_add1*stright_flag+ 
//              speed_add2*min_wan_flag+speed_add3*mid_wan_flag+speed_add4*max_wan_flag-
//              min_wan_into_mid_wan*speed_add5-stright_into_min_wan*speed_add6;  
   
      Goal_speed=speed_sta-Abs(distence)/33+speed_add1*stright_flag+ 
              speed_add2*min_wan_flag+speed_add3*mid_wan_flag+speed_add4*max_wan_flag-
              min_wan_into_mid_wan*speed_add5-stright_into_min_wan*speed_add6;               
   
   //保护程序
   if((AD_adjust_one[0]<10 && AD_adjust_one[5]<17 && AD_adjust_one[1]<10 &&AD_adjust_one[4]<10 ))     //冲出,停车 
   {
      out_stop_time++;
   }
   else
    out_stop_time=0;
   if(out_stop_time>10)  //测电机用，平时给小点
   {
    out_stop_flag=1;
    out_stop_time=0;
   } 
   if(out_stop_flag==1)
    Goal_speed=0;
   //已测试效果还可以，出弯不抖了，但加长了时间，先准备测试入弯减速2015.5.24。
  
   if(chuwan_flag==1)//出弯减速处理十字出弯都动，测试中。
     {
       chuwan_flag=0;
   
       Goal_speed=Goal_speed*wandao_beishu;   // 5017.5.5 放出
    
     }
  
  if(stright_flag==1)
    Goal_speed=speed_sta-Abs(distence)/33+speed_add1*stright_flag;    
//   if(distence<20 || distence>-20)
//     Goal_speed = (int16)(1.2*Goal_speed);    //2017.5.6自加
   if(motor_speed>250 && left_right_flag != 2 )   //直道入弯 2017.5.7
   {
       Goal_speed=(int16)(0.85*Goal_speed);
   }
   
   
  if( ruwan_flag==1)   
  {
   
          //入弯全局减速  0.88  2017.4.15
//       if(Abs(distence)>300)                      //  大弯再减   0.8   2017.4.15
//          Goal_speed=(int16)(0.8*Goal_speed);   //2017.5.14
       
    if(K2)      
    {
         Goal_speed=(int16)(0.85*Goal_speed);
      // if(speed_flag==1)       
       if((left_right_flag == 1) && (Abs(distence)>300))    //车偏左 右转   2017.5.15
        {
         //  Goal_speed=(int16)(0.9*Goal_speed);              
       //    temp1=(int16)(temp1*0.5);                      //右转加强  2017.5.15
                      
        }       
   
        if((left_right_flag == 3) && (Abs(distence)>220))           //左转内切  2017.5.15
        {
           //   Goal_speed=(int16)(1.1*Goal_speed); 
       //      temp1=(int16)(temp1*0.5);   
        }
            
        //以上低速         
    }
      
     else if(K3)    // million speed
     {
          Goal_speed=(int16)(0.85*Goal_speed); 
//           if(Abs(distence)>315)                      //  大弯再减   0.8   2017.4.15
//              Goal_speed=(int16)(0.98*Goal_speed);   //2017.5.14    0.8
       /*
      //  Goal_speed=(int16)(0.9*Goal_speed);
        if(Abs(distence)>300)                      //  大弯再减   0.8   2017.4.15
          Goal_speed=(int16)(0.78*Goal_speed);   //2017.5.14    0.8
        if((left_right_flag == 1) && (Abs(distence)>220))    // 右转   2017.5.15
        {
           // Goal_speed=(int16)(0.85*Goal_speed);                       //右转  2017.5.15
        }      
        
        if((left_right_flag == 3) && (Abs(distence)>220))    //zuo转   
        {
            Goal_speed=(int16)(1.2*Goal_speed);                       //zuo转  2017.5.15
        }  */
       
     }
    
     if(K4)        //high speed
    {
      //  Goal_speed=(int16)(0.85*Goal_speed);  
        Goal_speed=(int16)(0.91*Goal_speed);
//         if(Abs(distence)>315)                      //  大弯再减   0.8   2017.4.15
//              Goal_speed=(int16)(0.98*Goal_speed);   //2017.5.14    0.8
//         if(motor_speed>250 && left_right_flag != 2 )   //直道入弯 2017.5.7
//   {
//       Goal_speed=(int16)(0.2*Goal_speed);
//   }
      /*
          Goal_speed=(int16)(0.95*Goal_speed);
        if(Abs(distence)>300)                      //  大弯再减   0.8   2017.4.15
          Goal_speed=(int16)(0.6*Goal_speed);   //2017.5.14    0.8
        if((left_right_flag == 1) && (Abs(distence)>220))    // 右转   2017.5.15
        {
            Goal_speed=(int16)(0.9*Goal_speed);             //0.85  3.23放出          //右转  2017.5.15
        }      
        
        if((left_right_flag == 3) && (Abs(distence)>220))    //zuo转   
        {
             Goal_speed=(int16)(1.2*Goal_speed);                       //zuo转  2017.5.15
             if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //反跑 180‘弯内切  2017.5.20
               Goal_speed=(int16)(1.5*Goal_speed);        //2号电池7.4V *1.2  
            
        }   
      */
      
    }
    
     else if(K5)        //
    {
      
      Goal_speed=(int16)(0.85*Goal_speed);  
      Goal_speed=(int16)(0.95*Goal_speed); // 5.24 9:33
      /*
         
        if(Abs(distence)>300)                      //  大弯再减   0.8   2017.4.15
          Goal_speed=(int16)(0.6*Goal_speed);   //2017.5.14    0.8
        if((left_right_flag == 1) && (Abs(distence)>220))    // 右转   2017.5.15
        {
            Goal_speed=(int16)(0.9*Goal_speed);             //0.85  3.23放出          //右转  2017.5.15
        }      
        
        if((left_right_flag == 3) && (Abs(distence)>220))    //zuo转   
        {
             Goal_speed=(int16)(1.2*Goal_speed);                       //zuo转  2017.5.15
             if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //反跑 180‘弯内切  2017.5.20
               Goal_speed=(int16)(1.5*Goal_speed);        //2号电池7.4V *1.2  
            
        }  */ 
    }
        
  }
    
 
   return  Goal_speed;                
}
