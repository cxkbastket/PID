#ifndef __PID_H
#define __PID_H
#include "sys.h"


#define pwmout PBout(7)	

typedef struct 
{
	signed int  rt;			
	signed int  yt;			
	
	signed int  Kp;			
	signed int  T;			
	signed int  Ti;			
	signed int  Td;			
	
	signed int  e;		
	signed int  e_1; 
	signed int  Se;  
	
	signed int  OUT0;	
	
  signed int  OUT;
	
	u16 C10ms;
	
	u16 pwmcycle;
	
}PID;
extern  signed int Ttem;	
extern PID pid;
void PID_Init(void);
void TIM4_10ms(void);
void PID_Calc(void);
void Init_Timer3(void);
void PIDOUT_Init(void);  
void PID_out(void);

#endif 
