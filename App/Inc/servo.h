#ifndef _SERVO_H_
#define _SERVO_H_

typedef enum
{
    servo_left,
    servo_right
}servo_path;

#define servo_FTM   FTM1
#define servo_CH    FTM_CH1
#define servo_HZ    (300)
#define median      (5500)





extern void servo_init(void);


#endif
