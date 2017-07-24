#ifndef _E6A2_H_
#define _E6A2_H_
#define quad_module     FTM2
#define sampling_period 20

extern uint8 time_100ms;
extern speed_t motor_speed;

extern void  E6A2_init(void);

extern int16 P ;
extern int16 I;
extern int16 D;



#endif