#ifndef __CALCULATE_H__
#define __CALCULATE_H__

#define K1   !gpio_get(PTC7)
#define K2   !gpio_get(PTC8)
#define K3   !gpio_get(PTC9)
#define K4   !gpio_get(PTC10)
#define K5   !gpio_get(PTC11)
#define K6   !gpio_get(PTC12)


//#define MINDTURN 19000//9375//���� ��ϸ �ο� .c�ļ� ����ϸע��
//#define LEFTMAX 4200       //   ��ת������ѣ���е�ṹ)
//#define LEFTMIN 30
//#define RIGHTMAX -4450//
//#define RINGTMIN 30


#define MINDTURN 19300//9375//���� ��ϸ �ο� .c�ļ� ����ϸע��
#define LEFTMAX 4800       //   ��ת������ѣ���е�ṹ)
#define LEFTMIN 30
#define RIGHTMAX -4800//
#define RINGTMIN 30

#define LEFTAD_OFFE 0    //��̬����ߵĲɼ���ADֵ
#define RINGT_OFFE  0     //��̬���ұߵĲɼ���ADֵ
#define LEFTAD_COUNT  3     //
#define RINGT_COUNT  3     //
#define LOST_LINE  2     //
#define TURNMAX 600
#define TURNMIN 470


 /**********************
 * PID ���� ��ϸ �ο� .c�ļ� ����ϸע��
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
extern float  AD_adjust_one[6]; //��һ�����ݴ������
extern int16  Max_Min[8];
extern int16 *K5_value1,*K5_value2;
extern int stopflag;
extern int Carstop;
extern int32 time_flag;

void pwminit();//������ϸ �ο� .c�ļ�
void Emcinit();
int16 Turn( int16 *leftAD, int16 *rightAD );
int16 Turn1();
int16_t SpeedPID(uint16_t speedCount,uint16_t AmSpeed) ;    //�����ٶ�PWM
void Speed_control(uint16_t AmSpeed);       //�ֿ����ƣ���һ�߶���һ����Ӱ��  
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
