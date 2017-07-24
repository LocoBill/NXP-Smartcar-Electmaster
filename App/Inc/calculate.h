#ifndef __CALCULATE_H__
#define __CALCULATE_H__

#define K1   !gpio_get(PTC7)
#define K2   !gpio_get(PTC8)
#define K3   !gpio_get(PTC9)
#define K4   !gpio_get(PTC10)
#define K5   !gpio_get(PTC11)
#define K6   !gpio_get(PTC12)


//#define MINDTURN 19000//9375//参数 详细 参考 .c文件 有详细注释
//#define LEFTMAX 4200       //   右转打角困难（机械结构)
//#define LEFTMIN 30
//#define RIGHTMAX -4450//
//#define RINGTMIN 30


#define MINDTURN 19300//9375//参数 详细 参考 .c文件 有详细注释
#define LEFTMAX 4800       //   右转打角困难（机械结构)
#define LEFTMIN 30
#define RIGHTMAX -4800//
#define RINGTMIN 30

#define LEFTAD_OFFE 0    //静态下左边的采集的AD值
#define RINGT_OFFE  0     //静态下右边的采集的AD值
#define LEFTAD_COUNT  3     //
#define RINGT_COUNT  3     //
#define LOST_LINE  2     //
#define TURNMAX 600
#define TURNMIN 470


 /**********************
 * PID 参数 详细 参考 .c文件 有详细注释
 *********************/
#define  KPPLUSMAX      (500)
#define  KPNEGATIVEMAX  (-500)
#define  KIPLUSMAX      (500)
#define  KINEGATIVEMAX  (-500)
#define  KDPLUSMAX      (500)
#define  KDNEGATIVEMAX  (-500)
#define  KWPLUSMAX      (500)
#define  KWNEGATIVEMAX  (-500)
#define  KOUPLUSMAX     (800)

#define LIN_COUT     100
 
//lanzhou   
extern volatile struct FTM_MemMap *FTMx[3];
extern float  AD_adjust_one[6]; //归一化数据存放数组
extern int16  Max_Min[8];
extern int16 *K5_value1,*K5_value2;
extern int stopflag;
extern int Carstop;
extern int32 time_flag;

void pwminit();//程序详细 参考 .c文件
void Emcinit();
int16 Turn( int16 *leftAD, int16 *rightAD );
int16 Turn1();
int16_t SpeedPID(uint16_t speedCount,uint16_t AmSpeed) ;    //计算速度PWM
void Speed_control(uint16_t AmSpeed);       //分开控制，调一边对另一边无影响  
int32 speedpid(uint16_t AmSpeed);

void SPeedAim(int16_t AIMSpeed ) ;
void Dianci_MAXMIN_calculate(int16 *max_min);
uint16 Abs(int16 x);
float Abs_f(float x);
void Read_AD(void);
void Data_analyse(void);
int16 Steer_control(void);
int16 saidao_analyse(void);
#endif 
