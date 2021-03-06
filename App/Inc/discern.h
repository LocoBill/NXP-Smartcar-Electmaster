#ifndef _DISCERN_H_
#define _DISCERN_H_

typedef struct
{
    angle_t angle;
    speed_t speed;
}discern_result_t;

typedef enum
{
    curve,      //弯道
    beeline,    //直线
    crossing,   //十字
    obstacle    //障碍
}traffic_t;
    

extern float Kp,Kd,angle_p,angle_d;    //舵机参数
     



extern void discern_init(void);
//extern discern_result_t discern(void);

#endif