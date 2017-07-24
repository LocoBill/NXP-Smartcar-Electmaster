#include "include.h"
#include "common.h"
  
//#include  "delay.h"
// 2014.02.18. 
//ok 

#define   NM    8
#define   T1    70
#define   T2    20
#define   T3    20


//���뿪�س�ʼ��


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
volatile struct FTM_MemMap *FTMx[3] = {FTM0_BASE_PTR, FTM1_BASE_PTR, FTM2_BASE_PTR}; //��������ָ�����鱣�� FTMn �ĵ�ַ


float  AD_adjust_one[6]; //��һ�����ݴ������
float ad_adjust_one[6];
float ad_averge[6];
int16  AD_valu[6],AD_V[6][NM];
int16  AD_sum[6];
int16  Temp[8];
int16 distence,distence_old;    
int16 distence_store[30];  //�����Ż����,�Ż��ٶ� 
int16 distence_sum_weight; //ǰ���Ȩ  ����P

/////////�ٶȵ�����///////
 int32_t speedCount ,SpeedPWM  ;
uint8 allow_i_left_flag=1;      //�������
uint8 allow_i_right_flag=1;
int16 speed_old=0;        //�ϴ�ʵ���ٶ�
int16 now_speed=0,motor_speed_old=0;
int16 x1=0,x2=0,x3=0;     //Ϊ�����Ⱥ�,��һ����ƣ��ֱ�PI
int16 y1=0,y2=0,y3=0;
float Kp_Speed[4];
float Ki_Speed[4];        //��������ڵ���0     ѧϰ����ȡ3    ʵ����������һֱ��ѧϰ
float speed_err[4]={0,0,0,0};
int8 q=0;                         //ѧϰ����ȡ3
float p1=10;                       //��������
float p2=10;
float i1=2;
float Kp_L=86;                                            //������PI�������Ե�.
float Ki_L=0;    //��߳�ʼ��PI       
//#define P 5              //��ʼP  0.8    I  0.3
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

int8 left_right_flag=2;      //�����ж�        12/3
int8 left_right_flag_old;    
int8 chuandao_flag=0;

/////////�ٶȱ仯��///////////
int8 stright_flag=1;             //ֱ����Сs <=30
int8 stright_into_min_wan=0;     //ֱ��С��
int8 min_wan_into_mid_wan=0;     //С-��
int8 mid_wan_into_max_wan=0;     //��-��

int8 min_wan_flag=0;             //С��     30-150
int8 mid_wan_flag=0;             //�л�     150-250
int8 max_wan_flag=0;             //��     >250

int8 max_wan_into_mid_wan=0;     //��-��
int8 mid_wan_into_min_wan=0;     //��-С
int8 min_wan_into_stright=0;     //С-ֱ
int8 stright_into_zhijiao=0;     //ֱ����ֱ��

int8 chuwan_flag=0;     //��ʮ�����
int8 ruwan_flag=0;     //��ʮ�����
int8 wan_in_flag=0;     //��ʮ�����
int8 shun_ni_flag=2;     //˳��ʱ��
int8 shun_ni_flag_old=2;  
int8 speed_flag=0;
float wandao_beishu=0.6;
int8  out_stop_flag=0;     //���ͣ��
int16 out_stop_time=0;     //���ͣ��ʱ��
int16 Goal_speed=0;
int16 speed_choose[5][11]=
{
  //�ٶ�      ���450
  //�ٶȻ���,ֱ�����٣�����ӣ������,ֱ�����           //����ƫ��Ӽ�   ��־�䲻���г�ͻ        //�ȼӺ��
// {100,10,8,5,2,0,0,0,0,0,0},           //�����ٶ�         mode 0
// {300,10,8,5,2,0,0,0,0,0,0},           //�е�             mode 1
// {400,10,8,5,2,0,0,0,0,0,0},           //�ߵ�             mode 2
// {500,10,8,5,2,0,0,0,0,0,0}            //���             mode 3
 
 {120,10,8,5,2,0,0,0,0,0,0},           //�����ٶ�         mode 0
 {280,10,8,5,2,0,0,0,0,0,0},           //�е�             mode 1
 {520,10,8,5,2,0,0,0,0,0,0},           //�ߵ�             mode 2
 {520,10,8,5,2,0,0,0,0,0,0},           //���             mode 3   //920
 {670,10,8,5,2,0,0,0,0,0,0}     //K5

// {600,60,50,30,10,0,0,0,0,0,0},           //�����ٶ�         mode 0
// {1000,100,80,50,20,0,0,0,0,0,0},           //�е�             mode 1
// {1200,120,100,60,30,0,0,0,0,0,0},           //�ߵ�             mode 2
// {1600,160,140,90,40,0,0,0,0,0,0}            //���             mode 3

};


int16 speed_sta;    //��׼       

int16 speed_add1;   //ֱ��,Сs��       //���־�Ӽ�    20       �ض��ط��൱�������  ���£�
int16 speed_add2;   //С�����                     15
int16 speed_add3;   //�������                     10
int16 speed_add4;   //�������                     5

int16 speed_add5;   //       
int16 speed_add6;   //
int16 speed_add7;   //


void pwminit()//PWM��ʼ��
{
  //gpio_Interrupt_init(PTA,19, GPI_DOWN, RING) ; //��ʼ��������IO
 // gpio_Interrupt_init(PTC,5, GPI_DOWN, RING) ;  //��ʼ��������IO 
  
  /*  
          gpio_Interrupt_init(PTA,19, GPI_DOWN, RING) ;Ϊ���溯����gpio.c��
         RINGΪ�ж�ģʽ  #define RING          0x09    ʲô������������������
  ����������������������������������������   
  ����������������������������������������
  */
/*
//������
  //��������ģ��ͨ��  �˿�          ��ѡ��Χ              ����
#define FTM1_QDPHA_PIN  PTA12       //PTA8��PTA12��PTB0
#define FTM1_QDPHB_PIN  PTA13       //PTA9��PTA13��PTB1

#define FTM2_QDPHA_PIN  PTA10       //PTA10��PTB18
#define FTM2_QDPHB_PIN  PTA11       //PTA11��PTB19     2017.4.11 Loco,

*/
 // gpio_init (PTA19, GPI,0);
 // gpio_init (PTC5, GPI,0);
  gpio_init (PTA10, GPI,0);
  gpio_init (PTA11, GPI,0);    //4.11
  //FTM_PWM_init(FTM1, CH0, 180, 23);   //���  MOD = 31250
  // ����������ֵ 31250
  ftm_pwm_init(FTM1, FTM_CH1, 300, 450);   //���  MOD = 31250
  // ����������ֵ 31250    PTA13
  
  //FTM_PWM_init(FTM0, CH0, 10000, 0);  //���  MOD=2500
  ftm_pwm_init(FTM0, FTM_CH3, 10000, 0);  //����������ֵ 2500 //PTA6        2017.4.5   Loco
  ftm_pwm_init(FTM0, FTM_CH4, 10000, 0);  //���PWM��ʼ�� Ƶ��10Khz ռ�ձ�0��ռ�ձ����2500.PTA7
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
/*   ���ڳ���ע����������Ķ���  
|
|
|
|
|      �����
|
|

|����|   ��ǰ��
| 
|   С��
|����|   ������   ����ͼ
����Ϊ������ע��������Ķ��壬���������෴����������
** ===================================================================
** Turn
���룺  
��� ��Apwm  Ŀ��ת��ֵ ��
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
  float subvalue1, addvalue1,controlvalue,subvalue2, addvalue2,subvalue3, addvalue3; //����3��������
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
 //�����Ⱥ�---��һ��-����
      
    subvalue1=rightvalueF-leftvalueA ;  //ֻ�����ұ߼�ȥ��ߡ������
    addvalue1=leftvalueA+rightvalueF ; //���
    
 //�����Ⱥ�---�ڶ���-���÷����ߴ���
      
    subvalue2=rightvalueE-leftvalueB ;  //ֻ�����ұ߼�ȥ��ߡ������
    addvalue2=leftvalueB+rightvalueE ; //���
    
 //�����Ⱥ�---������-ֱ�Ǽ�ʮ��
      
    subvalue3=rightvalueD-leftvalueC ;  //ֻ�����ұ߼�ȥ��ߡ������
    addvalue3=leftvalueC+rightvalueD ; //���   
      
    if(addvalue1==0)//�޵��ֵʱ��������
    {
       TurnPWM      = 0;
       controlvalue1= 0;     
    }
    
     if(addvalue2==0)//�޵��ֵʱ��������
    {
       TurnPWM      = 0;
       controlvalue2= 0;
    }
    
  //��ȺͷŴ�100�����ڼ���ƫ��
    controlvalue1  = subvalue1/addvalue1 *100;
    controlvalue2  = subvalue2/addvalue2 *100; 
    controlvalue3  = subvalue3/addvalue3 *100; 
    
    //λ��ʽPD���ڶ��
    Turn2_Err    = Turn1_Err;
    Turn1_Err    = controlvalue1;
    Turn_EC      = Turn1_Err-Turn2_Err;
 
    Turn_P_Value  = Kp*Turn1_Err;
    
    Turn_D_Value  = Kd*Turn_EC;
   
    TurnPWM       =  (int16)(Turn_D_Value+Turn_P_Value);
  
 /////////��ʱ�����⴦�����////////////////
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
���룺speedCount�ɼ����٣�AmSpeed Ŀ�공�٣�  
��� ��SpeedPWMOUT  ���㳵�� ��
** ===================================================================
*/


//int16_t  SpeedPWM = 0 ;

int16_t SpeedPID(uint16_t speedCount,uint16_t AmSpeed)// �ȽϾ��������ʽPID
{                                                 // ��ο����PID������
  // �ڴ˲���չ��
 
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
  ����˵���� ���PI����    ����200�����2000
  ������PI������(����λ��ʽ��ѧϰ��)
  ���ַ���+�ֶ�PI
  ���ݵ�Ƭ�������������ԣ��������������Բ��ƣ��������Err_speed-Err_speed_old��
  ԭ����ݵ����϶���3�棬����PI������

  Ŀǰ��λ��ʽ��
  ��˫�޷�
  �ο�����P109λ��ʽ����PI��������ͼ

**********************************************/
void Speed_control(uint16_t AmSpeed)       //�ֿ����ƣ���һ�߶���һ����Ӱ��  
{  
   //��ֵȷ�����ڳ��ϲ��ԣ���ϵͳѧϰһ��ʱ���֪��
  
    Kp_Speed[q]=Kp_L;
    Ki_Speed[q]=Ki_L;

    now_speed =(motor_speed+motor_speed_old)/2;   
    motor_speed_old = motor_speed;
    
     
  //  x1=AmSpeed -motor_speed;                           //ת��ƫ��   
    x1=AmSpeed - now_speed;                           //ת��ƫ��                          
    x2=x2+x1;                        //ת��ƫ�����            
    x3=speed_old-now_speed;            //ʵ��ת�ٱ仯�ʸ�ֵ      

     //�����޷�     С���
   /* 
     if(x2>=300)  x2=300;
     else if(x2<=-300)  x2=-300;
     
     if(y2>=300) y2=300;
     else if(y2<=-300)  y2=-300;   
  */  
    
    speed_err[0] = x1;  //2017.5.6
     
     //ÿ�δ�Խ�������㣬���˱��ͳ���
/*   
     //���ַ���      ��
     if(abs(x1)<=30)     // 0.1~0.2Err
     {   
       //�ֶ�PI      ��
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
    //ӦPI>=0
     */

   //˫�޷�����
 
    
   SpeedPWM=(int16)(Kp_L*x1+ Ki_L*x2);  // 2017.5.2
  
   
        if(SpeedPWM>2500)           //����޷� ��
        {SpeedPWM=2500;}
        else 
        if(SpeedPWM<-2500)
        {SpeedPWM=-2500;}  
        
  

     //��ֵ
    if(SpeedPWM>=0)                //��ת
    {
      //ftm_pwm_duty(FTM0, FTM_CH3,SpeedPWM);
      // ftm_pwm_duty(FTM0, FTM_CH4,0);    //��ֵ���
      FTM_CnV_REG(FTMx[FTM0], FTM_CH3) = SpeedPWM ;
      FTM_CnV_REG(FTMx[FTM0], FTM_CH4) = 0;
    }
    if(SpeedPWM<0)                 //��ת
    {
      SpeedPWM = 0 - SpeedPWM ;
      //  ftm_pwm_duty(FTM0, FTM_CH3,0);
      // ftm_pwm_duty(FTM0, FTM_CH4,SpeedPWM);
      FTM_CnV_REG(FTMx[FTM0], FTM_CH3) = 0 ;
      FTM_CnV_REG(FTMx[FTM0], FTM_CH4) = SpeedPWM;
    }
 
     //ƫ��洢  x2����
     //Err_speed_old_left=x1;
     //Err_speed_old_right=Err_speed_right;
   
    //ʵ���ٶȴ洢
    speed_err[3]=speed_err[2];
    speed_err[2]=speed_err[1];
    speed_err[1]=speed_err[0];

    speed_old=now_speed;
    
    //ѧϰ3��   ʵ��һֱѧ
    q++;
    if(q>2)
    q=0;
}



/****�ٶ�pid****/
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
���룺AIMSpeed Ŀ�공�٣�  
��� �� ��
** ===================================================================
*/
void SPeedAim(int16_t AIMSpeed )    //  
{ 
 // int16_t speedCount ,SpeedPWM  ;
 // extern int16_t LeftSpeedC ,RingtSpeedC ;
 // speedCount = LeftSpeedC  ;//�����ֳ���+���ֳ��٣�/2  Ϊʲô��>>1 �������ڳ���ִ��Ч�ʣ��ڴ˲���չ����
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
  
  //�ٶ�PID ��ǰ�����ٶ� ��Ŀ���ٶ�
 //SpeedPWM =SpeedPID(speedCount ,AIMSpeed ) ;    //speedpid������ǳ�ͻ������2017.5.1Loco.
  //SpeedPID����ADC3��ADC5������   �����2017.5.2Loco.
  SpeedPWM =speedpid(AIMSpeed ) ;  //2017.5.6
  
  /*
  uart_putchar(UART0,0xcc) ;
  uart_putchar(UART0,SpeedPWM>>8);
  uart_putchar(UART0,SpeedPWM);
  */
  //SpeedPWM =0 ;
  if(SpeedPWM > 9900)// �ٶ�PWM ����2500 �Ͳ�������  2500Ϊ���
  {SpeedPWM = 9900 ;}//��ֵ2500
  else if(SpeedPWM < -9900){// +2500 ��-2500 �ķ���һ��������ת
    SpeedPWM = -9900 ;
  }
  if(SpeedPWM > 0)// �ٶ�PWM ������ Ϊһת��
  {
    //FTM_CnV_REG(FTMx[FTM0], CH0) = SpeedPWM;//�ο� ����CH0 ע�� ֻ��ת��һ��
    //   FTM_CnV_REG(FTMx[FTM0], CH0) = 300;
 //  FTM_CnV_REG(FTMx[FTM0], FTM_CH3) = SpeedPWM ;
  // FTM_CnV_REG(FTMx[FTM0], FTM_CH4) = 0;
    
   ftm_pwm_duty(FTM0, FTM_CH3, SpeedPWM);  
   ftm_pwm_duty(FTM0, FTM_CH4, 0);  
    //  FTM_CnV_REG(FTMx[FTM0], CH2) = 500;
    //FTM_CnV_REG(FTMx[FTM0], CH3) = 0 ;
  } 
  else                     //�ٶ�PWM С���� Ϊ������ķ���
  {
    SpeedPWM = -SpeedPWM ;
    //FTM_CnV_REG(FTMx[FTM0], CH0) = 0;        //ftm0 ch0ͨ�� ռ�ձ�0            -��
  //  FTM_CnV_REG(FTMx[FTM0], FTM_CH3) = 0 ;//ftm0 ch1ͨ�� ռ�ձȵ�ǰ�ٶ�PWM  -��
  //  FTM_CnV_REG(FTMx[FTM0], FTM_CH4) = SpeedPWM;        //ftm0 ch0ͨ�� ռ�ձ�0            -��
    
    
    ftm_pwm_duty(FTM0, FTM_CH3, 0);  
    ftm_pwm_duty(FTM0, FTM_CH4, SpeedPWM); 
    //FTM_CnV_REG(FTMx[FTM0], CH3) = SpeedPWM ;//ftm0 ch1ͨ�� ռ�ձȵ�ǰ�ٶ�PWM  -��
    //          /                     - 2  
  }
   
  
}


/*
** ===================================================================
** Dianci_MAXMIN_calculate
���룺  
�����ͨ��ɨ�����������6�����ֵ�������Сֵ����С�������������Ƶ����Ҷˣ��Դ�����⣩ ��
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
   max_min[0]=max_min[1]=max_min[2]=max_min[3]=max_min[4]=max_min[5]=0;  //�洢ABDCEF��е����ֵ
   max_min[6]=max_min[7]=max_min[8]=max_min[9]=max_min[10]=max_min[11]=20;//�洢ABDCEF��е���Сֵ
   for(i=0;i<2400;i++)
   {  
        AD[0] = ad_ave(ADC0, AD8, ADC_12bit, 5); //A�󴫸����ɼ��źŲɼ�5�ξ�ֵ�˲�
        AD[1] = ad_ave(ADC0, AD9, ADC_12bit, 5) ; //B�󴫸����ɼ��źŲɼ�5�ξ�ֵ�˲�
        AD[2] = ad_ave(ADC0, AD12, ADC_12bit, 5) ; //C�󴫸����ɼ��źŲɼ�5�ξ�ֵ�˲� 
        AD[3] = ad_ave(ADC0, AD13,  ADC_12bit, 5) ; //D�Ҵ������ɼ��źŲɼ�5�ξ�ֵ�˲�
        AD[4] = ad_ave(ADC1, AD10,  ADC_12bit, 5) ; //E�Ҵ������ɼ��źŲɼ�5�ξ�ֵ�˲� 
        AD[5] = ad_ave(ADC1, AD11, ADC_12bit, 5); //F�Ҵ������ɼ��źŲɼ�5�ξ�ֵ�˲�
        
      
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
     
   //    OLED_Fill(0x00);//���� 
//       OLED_Print_Num(5,0,max_min[0]);           //�����ַ�����ʾ
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
  for(i=0;i<5;i++)//��·��и��ɼ�5��
  {
        ad_value[0][i] = ad_ave(ADC0, SE8, ADC_12bit, 5) ; //A�󴫸����ɼ��źŲɼ�5�ξ�ֵ�˲�
        ad_value[1][i] = ad_ave(ADC0, SE9, ADC_12bit, 5); //B�󴫸����ɼ��źŲɼ�5�ξ�ֵ�˲�
        ad_value[2][i] = ad_ave(ADC0, SE12, ADC_12bit, 5) ; //C�󴫸����ɼ��źŲɼ�5�ξ�ֵ�˲� 
        ad_value[3][i] = ad_ave(ADC0, SE13,  ADC_12bit, 5) ; //D�Ҵ������ɼ��źŲɼ�5�ξ�ֵ�˲�
        ad_value[4][i] = ad_ave(ADC1, SE10,  ADC_12bit, 5); //E�Ҵ������ɼ��źŲɼ�5�ξ�ֵ�˲� 
        ad_value[5][i] = ad_ave(ADC1, SE11, ADC_12bit, 5); //F�Ҵ������ɼ��źŲɼ�5�ξ�ֵ�˲�
  }
  //ð������
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
  //ȥͷ��β���м�������ƽ��
  for(i=0;i<6;i++)
  {
     ad_sum[i]   =ad_value[i][1]+ad_value[i][2]+ad_value[i][3];
     ad_averge[i]=ad_sum[i]/3.0;
   }
  //����ƽ���˲�
  
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
  
  
   for(i=0;i<6;i++)  //��ƽ��
   {
        
         AD_valu[i] = AD_sum[i] / NM;
         AD_sum[i] = 0; 
   }  
  
  
 //��һ������
  for(i=0;i<6;i++)  
   {
     ad_adjust_one[i]=(float)(AD_valu[i]-Max_Min[i+6])/(float)(Max_Min[i]-Max_Min[i+6]);
        if(ad_adjust_one[i]<=0.0)        
          ad_adjust_one[i]=0.001;
          
        if(ad_adjust_one[i]>1.0)
          ad_adjust_one[i]=1.0;
        
     AD_adjust_one[i]=(int16)(ad_adjust_one[i]*100);
   
   /*  //5��в��Գƣ���Ų�������*/
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
�������������ֵ���ݷ����������ֱ�־λ��λ
��������Data_analyse
���룺��
�������
************************************************/

void Data_analyse(void)
{
    int16_t i=0,temp2;  
    Read_AD();
  
  
  //ˮƽ����Ԥ���� 
    if(AD_adjust_one[0]>=T1 ||AD_adjust_one[5]>=T1)
    {
    if(Abs_f((int16)(AD_adjust_one[0]-AD_adjust_one[5]))<=T2)
    {
      left_right_flag=2;//���Ӿ���
    }
    else
    {
      if(AD_adjust_one[0]>AD_adjust_one[5])
        left_right_flag=3;//���Ӿ���
      else if(AD_adjust_one[0]<AD_adjust_one[5])
        left_right_flag=1;//���Ӿ���
    }
  }

//2017.5.5 Loco   ʮ����
  else if(AD_adjust_one[0]<20 && AD_adjust_one[5]<20)        //ֱ��ʮ�ַ�������������б��У���ֱ��У�ˮƽ��У���ͣ�ֵ�ۺϴ���      
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
  else if(AD_adjust_one[0]<T1 && AD_adjust_one[5]<T1)        //��ǰ�ŵĵ�г������ú��ŵ��жϷ���  
  {
   
    if((AD_adjust_one[0]<AD_adjust_one[5])&&(AD_adjust_one[1]>AD_adjust_one[4]))//˳ʱ���������
   {
     temp2=AD_adjust_one[1];
     AD_adjust_one[1]=AD_adjust_one[4];
     AD_adjust_one[4]=temp2;
   }
   if(AD_adjust_one[1]>AD_adjust_one[4])
      left_right_flag=3;//���Ӿ���
   if(AD_adjust_one[1]<AD_adjust_one[4])
      left_right_flag=1;//���Ӿ���
   
  if((AD_adjust_one[0]>AD_adjust_one[5])&&(ad_averge[2]>ad_averge[3])) //��ʱ��������д��� �Ѳ���
   {
     if(AD_adjust_one[1]<=AD_adjust_one[4])
       left_right_flag=left_right_flag_old;

   }
  
  
  }
*/    //2017.5.2
  
  if(left_right_flag==1&&left_right_flag_old==3)
  {
    if(AD_adjust_one[0]<=T3||AD_adjust_one[5]<=T3)             //С�ڶ��ٲ���,��ʵ�ʵ� ,ֵ̫СЧ�����ã�ֵ̫���۽�ץ��������  ���������distenceֱ�Ӵ��������
      left_right_flag=left_right_flag_old;
  }
  if(left_right_flag==3&&left_right_flag_old==1)
  {
    if(AD_adjust_one[0]<=T3||AD_adjust_one[5]<=T3)                //Ҳ��ֱ�������ڶ������,����Ϊȡdistence���
      left_right_flag=left_right_flag_old;
  }
  left_right_flag_old=left_right_flag;
  
////////////����ƫ��////////////////////////////              б��һ��ԭ��             
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
   
   //�м�η�������D��Ӱ��      30��
 /*  
   if((abs(distence)<40 && abs(distence_old)>40)||(abs(distence)>40 && abs(distence_old)<40))
   {
    if(distence-distence_old>10)           //5ms/20ms,5��
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
   
    //������� 2017.5.5
   if((distence>0 && distence_old<-250)||( distence<0 && distence_old>250) )
   {  distence=distence_old;}
  
   
  distence_old=distence;

  for(i=28;i>=0;i--)                 //�����ǵݼ���   ���򵥲����˶����ǣ�����2017.5.1
  {
    distence_store[i+1]=distence_store[i];
  } 
  distence_store[0]=distence_old;
  
  distence_sum_weight=(distence_store[0]*10+distence_store[1]*4+distence_store[2]*3+distence_store[3]*2+distence_store[4])/20;

   /**/
}

/********************************
���������������ż���
�������� Steer_control
���룺��
�������
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
  //  temp1=(int16)(temp1*0.8);   //��ת���У�0.78����2017.5.1
//  
//   if(temp1<0)
//    temp1=(int16)(temp1*1.65);   //��ת����������2017.5.13
//  
//   if(AD_adjust_one[1]>95 && AD_adjust_one[4]>95 && AD_adjust_one[2]>95 && AD_adjust_one[3]>95)  //ʮ�ִ���2047.5.20
//   {
//      if(left_right_flag !=2)
//        temp1=(int16)(temp1*1.5); 
//   }
//   
//    if(Abs(distence)>300)     //����2017.5.20
//    {
//        temp1=(int16)(temp1*1.2); 
//    }
//   
  
 
  
  if(K3)     //million speed
  {
    
//     if(temp1>0)     //��ת
//     {
//         if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //���� 180��������  2017.5.20
//           temp1=(int16)(temp1*0.2);         //0.2    
//     }
     
   //   if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //���� 180��������  2017.5.20
   //        temp1=(int16)(temp1*0.2);         //0.2     
   // /*
     temp1=(int16)(temp1*1.05);      //����  P141.2 141.3  D 13.4   2017.5.24 1:53
    
      if(temp1>0)     //��ת
      {
        temp1=(int16)(temp1*0.92); 
        if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //���� 180��������  2017.5.20
           temp1=(int16)(temp1*0.18);         //0.2     
//       if(Abs(distence)>300)      
//        temp1=(int16)(temp1*1.2); 
//      else  temp1=(int16)(temp1*0.75); 
//       if(out_shi==1)
//          temp1=(int16)(temp1*1.2); 
//        else  temp1=(int16)(temp1*0.8); 
      }
       //  if((temp1<0) && (Abs(distence)>300))
       //    temp1=(int16)(temp1*0.9);   //��ת��2017.5.13    
  //  */
  }
  
  
  if(K4)    //high speed
  {
     temp1=(int16)(temp1*1.05);      //����  P141.2 141.3  D 13.4   2017.5.24 1:53
     if(temp1>0)     //��ת
     {
         if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //���� 180��������  2017.5.20
           temp1=(int16)(temp1*0.2);         //0.2    
     }
    
    /*
      if(temp1>0)     //��ת
      {
        temp1=(int16)(temp1*1.05); 
        if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //���� 180��������  2017.5.20
           temp1=(int16)(temp1*0.2);         //0.2     
      }
         if((temp1<0) && (Abs(distence)>300))
           temp1=(int16)(temp1*0.9);   //��ת��2017.5.13  
    */
  }
  
 
    else if(K5)    //
    { 
      if(temp1>0)     //��ת
      {
        temp1=(int16)(temp1*1.05); 
        if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //���� 180��������  2017.5.20
           temp1=(int16)(temp1*0.19);         //0.2  
      }
      /*
      if(temp1>0)     //��ת
      {
        temp1=(int16)(temp1*1.05); 
        if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //���� 180��������  2017.5.20
           temp1=(int16)(temp1*0.2);         //0.2     
      }
         if((temp1<0) && (Abs(distence)>300))
           temp1=(int16)(temp1*0.9);   //��ת��2017.5.13  
      */
    }
 
  
  
  if(temp1>=LEFTMAX)    temp1 = LEFTMAX;
  if(temp1 <= RIGHTMAX) temp1 = RIGHTMAX; 
 
  steer_pwmdty = steer_mid -temp1;   //Kd���Գ���б��в� //KdΪԭʼ�仯������
   
  if((ad_averge[2]>2000)&&(ad_averge[3]>900))
    steer_pwmdty=steer_mid;       //���ζ�����   2017.4.29
  
  
  return (int16)steer_pwmdty; 
}


/***************
��������saidao_analyse
��������������������־�趨
���룺��
���:Ŀ���ٶ�
******************/

int16 saidao_analyse(void)
{
  int i=0; 
  int32 distence_Abs_sum[4]={0,0,0,0};   //����֮�ͣ�ǰ10����10,��10         ÿ����������ڲ�����
  int32 distence_alg_sum[4]={0,0,0,0};   //�����ͣ�  
   //�ٶȲ���
 
 //1
    //30��ƫ�������
    for(i=0;i<30;i++)
    {
      distence_alg_sum[0]=distence_alg_sum[0]+distence_store[i];
    }
    distence_alg_sum[0]=distence_alg_sum[0]/30;
    //30��ƫ�����ֵ��
    for(i=0;i<30;i++)
    {
      distence_Abs_sum[0]=distence_Abs_sum[0]+Abs(distence_store[i]);
    }
    distence_Abs_sum[0]=distence_Abs_sum[0]/30;
 //2 
    //ǰ10������
    for(i=0;i<10;i++)
    {
      distence_alg_sum[1]=distence_alg_sum[1]+distence_store[i];
    }
    distence_alg_sum[1]=distence_alg_sum[1]/10;
 
    //ǰ10����ֵ��
    for(i=0;i<10;i++)
    {
      distence_Abs_sum[1]=distence_Abs_sum[1]+Abs(distence_store[i]);
    }
    distence_Abs_sum[1]=distence_Abs_sum[1]/10;
    
 //3
    //��10������
    for(i=10;i<20;i++)
    {
      distence_alg_sum[2]=distence_alg_sum[2]+distence_store[i];
    }
    distence_alg_sum[2]=distence_alg_sum[2]/10;
 
    //��10����ֵ��
    for(i=10;i<20;i++)
    {
      distence_Abs_sum[2]=distence_Abs_sum[2]+Abs(distence_store[i]);
    }
    distence_Abs_sum[2]=distence_Abs_sum[2]/10;
//4    
   //��10������
    for(i=20;i<30;i++)
    {
      distence_alg_sum[3]=distence_alg_sum[3]+distence_store[i];
    }
    distence_alg_sum[3]=distence_alg_sum[3]/10;
 
    //��10����ֵ��
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
//��������һ���ͺ��Թ�����������ţ����������Ƶ���ٶȣ��Ͳ��١����ѵ���ٶ�ת�ɸ���PD���ƶ��
//����ʲô·����ƫ��������ֻ�Ǽ���               ����������β�жϣ����и�����β���м��ж�    
    //�����ͺ�С��ֱ��
    if(Abs(distence_alg_sum[0])<30)
    {
      stright_flag=1; 
    }
    else
      stright_flag=0; 
    
    //ֱ-С�� 
    if(distence_Abs_sum[3]<50 && distence_Abs_sum[1]>100 )         //3�������α�־��Ч�����ٶȼ���ƽ��������Сͻ�������־ͬʱ��Ч�����ļ�
    {
      stright_into_min_wan=1;
    }
    else
      stright_into_min_wan=0;
    
    //С-��
    if(distence_Abs_sum[3]<100 && distence_Abs_sum[1]>200)
    {
      min_wan_into_mid_wan=1;  
    }
    else
      min_wan_into_mid_wan=0;
    
    //��-��
    if(distence_Abs_sum[3]<200 && distence_Abs_sum[1]>250 )
    {
      mid_wan_into_max_wan=1;
    }
    else
      mid_wan_into_max_wan=0;
    
    //С������
    if(Abs(distence_Abs_sum[1]-distence_Abs_sum[3])<50 && distence_Abs_sum[2]>30 &&distence_Abs_sum[2]<150)
    {
      min_wan_flag=1;
    }
    else
      min_wan_flag=0;
    //�С���
    if(Abs(distence_Abs_sum[1]-distence_Abs_sum[3])<50 && distence_Abs_sum[2]>150 &&distence_Abs_sum[2]<250)
    {
      mid_wan_flag=1;
    }
    else
      mid_wan_flag=0;
    //�󡣡�
    if(Abs(distence_Abs_sum[1]-distence_Abs_sum[3])<50 && distence_Abs_sum[2]>250)
    {
      max_wan_flag=1;
    }
    else
      max_wan_flag=0;
    
    //��-��
     if(distence_Abs_sum[1]<200 && distence_Abs_sum[3]>250 )        //
    {
      max_wan_into_mid_wan=1;
    }
    else
      max_wan_into_mid_wan=0;
     
    //��-С
    if(distence_Abs_sum[1]<100 && distence_Abs_sum[3]>200 )
    {
      mid_wan_into_min_wan=1;
    }
    else
      mid_wan_into_min_wan=0; 
    //С-ֱ
    if(distence_Abs_sum[1]<50 && distence_Abs_sum[3]>100 )
    {
      min_wan_into_stright=1;
    }
    else
      min_wan_into_stright=0;
      
    shun_ni_flag_old=shun_ni_flag;  
    
    ////////////////////////////////////////////////
   if((distence_Abs_sum[1]*distence_alg_sum[1])>0)//��ʱ��
         shun_ni_flag=1;
   else if((distence_Abs_sum[1]*distence_alg_sum[1])<0)//˳ʱ��
         shun_ni_flag=3;
   else if((distence_Abs_sum[1]*distence_alg_sum[1])==0)
            shun_ni_flag=2;
    
   
   if (shun_ni_flag==1)//˳ʱ��
   {
      if(distence_Abs_sum[1]>=280 && distence_Abs_sum[1]<=300 ) //���������ٶȴ���
        chuwan_flag=1;
   }
   if (shun_ni_flag==2)//��ʱ��
   {
      if(distence_Abs_sum[1]>=250 && distence_Abs_sum[1]<=330 ) //���������ٶȴ���
        chuwan_flag=1;
   }
    ////////////////////////////////////////////////
  
   if(distence_Abs_sum[1]>=100) 
      ruwan_flag=1;
   else   ruwan_flag=0;
    
   
   //���뿪��K5ѡ����
  /* 
   if(K5)
   {
     K5_value1 = &P;
     K5_value2 = &I;
   }
   
   if(!K5)
   {
     K5_value1 = &angle_p;   //float ��ͻ
     K5_value2 = &angle_d;
   }
     
*/
   
   //���뿪�ص��ٶȵ�λ 
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
        
          if(K5)           //����        
        {
          Speedcontrol=670; 
          speed_sta=speed_choose[2][0] ;
        //wandao_beishu=0.5 ;
        }
           
                         
  // }                   
                         
   //���ٶ�=�趨-ƫ�����10��+ֱ����+С����+������+������+�����+�г�С+С��ֱ-ֱ��С-С����-�н���             //����΢��ֱ���ͼ���΢�������ܼ�������ˮƽ
    
//   Goal_speed=speed_sta-Abs(distence)/33+speed_add1*stright_flag+ 
//              speed_add2*min_wan_flag+speed_add3*mid_wan_flag+speed_add4*max_wan_flag-
//              min_wan_into_mid_wan*speed_add5-stright_into_min_wan*speed_add6;  
   
      Goal_speed=speed_sta-Abs(distence)/33+speed_add1*stright_flag+ 
              speed_add2*min_wan_flag+speed_add3*mid_wan_flag+speed_add4*max_wan_flag-
              min_wan_into_mid_wan*speed_add5-stright_into_min_wan*speed_add6;               
   
   //��������
   if((AD_adjust_one[0]<10 && AD_adjust_one[5]<17 && AD_adjust_one[1]<10 &&AD_adjust_one[4]<10 ))     //���,ͣ�� 
   {
      out_stop_time++;
   }
   else
    out_stop_time=0;
   if(out_stop_time>10)  //�����ã�ƽʱ��С��
   {
    out_stop_flag=1;
    out_stop_time=0;
   } 
   if(out_stop_flag==1)
    Goal_speed=0;
   //�Ѳ���Ч�������ԣ����䲻���ˣ����ӳ���ʱ�䣬��׼�������������2015.5.24��
  
   if(chuwan_flag==1)//������ٴ���ʮ�ֳ��䶼���������С�
     {
       chuwan_flag=0;
   
       Goal_speed=Goal_speed*wandao_beishu;   // 5017.5.5 �ų�
    
     }
  
  if(stright_flag==1)
    Goal_speed=speed_sta-Abs(distence)/33+speed_add1*stright_flag;    
//   if(distence<20 || distence>-20)
//     Goal_speed = (int16)(1.2*Goal_speed);    //2017.5.6�Լ�
   if(motor_speed>250 && left_right_flag != 2 )   //ֱ������ 2017.5.7
   {
       Goal_speed=(int16)(0.85*Goal_speed);
   }
   
   
  if( ruwan_flag==1)   
  {
   
          //����ȫ�ּ���  0.88  2017.4.15
//       if(Abs(distence)>300)                      //  �����ټ�   0.8   2017.4.15
//          Goal_speed=(int16)(0.8*Goal_speed);   //2017.5.14
       
    if(K2)      
    {
         Goal_speed=(int16)(0.85*Goal_speed);
      // if(speed_flag==1)       
       if((left_right_flag == 1) && (Abs(distence)>300))    //��ƫ�� ��ת   2017.5.15
        {
         //  Goal_speed=(int16)(0.9*Goal_speed);              
       //    temp1=(int16)(temp1*0.5);                      //��ת��ǿ  2017.5.15
                      
        }       
   
        if((left_right_flag == 3) && (Abs(distence)>220))           //��ת����  2017.5.15
        {
           //   Goal_speed=(int16)(1.1*Goal_speed); 
       //      temp1=(int16)(temp1*0.5);   
        }
            
        //���ϵ���         
    }
      
     else if(K3)    // million speed
     {
          Goal_speed=(int16)(0.85*Goal_speed); 
//           if(Abs(distence)>315)                      //  �����ټ�   0.8   2017.4.15
//              Goal_speed=(int16)(0.98*Goal_speed);   //2017.5.14    0.8
       /*
      //  Goal_speed=(int16)(0.9*Goal_speed);
        if(Abs(distence)>300)                      //  �����ټ�   0.8   2017.4.15
          Goal_speed=(int16)(0.78*Goal_speed);   //2017.5.14    0.8
        if((left_right_flag == 1) && (Abs(distence)>220))    // ��ת   2017.5.15
        {
           // Goal_speed=(int16)(0.85*Goal_speed);                       //��ת  2017.5.15
        }      
        
        if((left_right_flag == 3) && (Abs(distence)>220))    //zuoת   
        {
            Goal_speed=(int16)(1.2*Goal_speed);                       //zuoת  2017.5.15
        }  */
       
     }
    
     if(K4)        //high speed
    {
      //  Goal_speed=(int16)(0.85*Goal_speed);  
        Goal_speed=(int16)(0.91*Goal_speed);
//         if(Abs(distence)>315)                      //  �����ټ�   0.8   2017.4.15
//              Goal_speed=(int16)(0.98*Goal_speed);   //2017.5.14    0.8
//         if(motor_speed>250 && left_right_flag != 2 )   //ֱ������ 2017.5.7
//   {
//       Goal_speed=(int16)(0.2*Goal_speed);
//   }
      /*
          Goal_speed=(int16)(0.95*Goal_speed);
        if(Abs(distence)>300)                      //  �����ټ�   0.8   2017.4.15
          Goal_speed=(int16)(0.6*Goal_speed);   //2017.5.14    0.8
        if((left_right_flag == 1) && (Abs(distence)>220))    // ��ת   2017.5.15
        {
            Goal_speed=(int16)(0.9*Goal_speed);             //0.85  3.23�ų�          //��ת  2017.5.15
        }      
        
        if((left_right_flag == 3) && (Abs(distence)>220))    //zuoת   
        {
             Goal_speed=(int16)(1.2*Goal_speed);                       //zuoת  2017.5.15
             if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //���� 180��������  2017.5.20
               Goal_speed=(int16)(1.5*Goal_speed);        //2�ŵ��7.4V *1.2  
            
        }   
      */
      
    }
    
     else if(K5)        //
    {
      
      Goal_speed=(int16)(0.85*Goal_speed);  
      Goal_speed=(int16)(0.95*Goal_speed); // 5.24 9:33
      /*
         
        if(Abs(distence)>300)                      //  �����ټ�   0.8   2017.4.15
          Goal_speed=(int16)(0.6*Goal_speed);   //2017.5.14    0.8
        if((left_right_flag == 1) && (Abs(distence)>220))    // ��ת   2017.5.15
        {
            Goal_speed=(int16)(0.9*Goal_speed);             //0.85  3.23�ų�          //��ת  2017.5.15
        }      
        
        if((left_right_flag == 3) && (Abs(distence)>220))    //zuoת   
        {
             Goal_speed=(int16)(1.2*Goal_speed);                       //zuoת  2017.5.15
             if(AD_adjust_one[1]>98 && AD_adjust_one[2]>98 )     //���� 180��������  2017.5.20
               Goal_speed=(int16)(1.5*Goal_speed);        //2�ŵ��7.4V *1.2  
            
        }  */ 
    }
        
  }
    
 
   return  Goal_speed;                
}
