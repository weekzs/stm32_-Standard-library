#include "stm32f10x.h"                  // Device header

float Err=0,last_err=0,next_err=0,pwm= 0,add=0,p=0.1,i=0,d=0;
float Err2=0,last_err2=0,next_err2=0,pwm2=0,add2=0,p2=0.1,i2=0,d2=0;
int16_t myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

void pwm_control()
{
    if(pwm>360)
        pwm=300;
    if(pwm<0)
        pwm=0;
}

void pwm2_control()
{
    if(pwm2>360)
        pwm2=300;
    if(pwm2<0)
        pwm2=0;
}

int16_t pid1(float speed1,float tar1)
{
    speed1=myabs(speed1);
    Err=tar1-speed1;
    add=p*(Err-last_err)+i*(Err)+d*(Err+next_err-2*last_err);
    pwm+=add;
    pwm_control();
    next_err=last_err;
    last_err=Err;
    return pwm;
}

int16_t pid2(float speed2,float tar2)
{
    speed2=myabs(speed2);
    Err2=tar2-speed2;
    add2=p2*(Err2-last_err2)+i2*(Err2)+d2*(Err2+next_err2-2*last_err2);
    pwm2+=add2;
    pwm2_control();
    next_err2=last_err2;
    last_err2=Err2;
    return pwm2;
}
