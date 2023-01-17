#include "pid.h"
#include "ds18b20.h"
#include "led.h"


PID pid;
signed int Ttem=0;			


void read_temper()		
{
	pid.yt = dS18b20_Get_Temp();
}

void PID_Init()
{
	pid.rt =  370;		
	pid.Kp = 1.85;
	pid.T = 500;   
	pid.Ti = 370;  
	pid.Td = 0.09;  
	pid.pwmcycle = 200;
	pid.OUT0 = 1;
	pid.e_1 = 0;
	pid.Se = 0;
}

void PID_Calc()
{
	signed int ec;
	signed int Iout,Pout,Dout;
	long out;
	
	if(pid.C10ms<pid.T/10)  
	{
		return ;
	}
	LED1=!LED1;
	pid.e = (pid.rt - pid.yt);   
	Pout = pid.e*pid.Kp/10;				
	
	pid.Se += pid.e;          
	
	ec = pid.e - pid.e_1;		
			
	Iout = pid.Se/(pid.Ti*10);			
	
	Dout = pid.Td*ec/10;				
 	
	out = Pout+Iout+Dout+pid.OUT0;   


	if(out>pid.pwmcycle)
	{
		pid.OUT = pid.pwmcycle;
	}
	else if(out<0)
	{
		pid.OUT = 1;
	}
	else 
		pid.OUT = out;

	pid.e_1 = pid.e;  
	
	pid.C10ms = 0;
	
}

void PIDOUT_Init()  
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

}

void PID_out()
{
	static u16 pw;
	pw++;
	if(pw>=pid.pwmcycle)
	{
		pw = 0;
	}

	
	if(pw<pid.OUT)
	{
		pwmout = 1;		
	}
	else
	{
		pwmout = 0;		
	}
}





void TIM4_10ms(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInit1;
	NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOG, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);			
	
	TIM_TimeBaseInit1.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit1.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit1.TIM_Period = 99;
	TIM_TimeBaseInit1.TIM_Prescaler = 8399;
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInit1);

	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(TIM4,ENABLE); 
}

void TIM4_IRQHandler(void)
{

	if(TIM_GetITStatus(TIM4,TIM_IT_Update)) 
	{
			LED0=!LED0;
			Ttem++;
			pid.C10ms++;
		
	TIM_ClearFlag(TIM4,TIM_IT_Update); 
	}
}



void Init_Timer3()
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseTimer3;
	NVIC_InitTypeDef NVIC_InitTimer3;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseTimer3.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseTimer3.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseTimer3.TIM_Prescaler=8399;		
	TIM_TimeBaseTimer3.TIM_Period=9;			
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseTimer3);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	
	NVIC_InitTimer3.NVIC_IRQChannel=TIM3_IRQn;
	NVIC_InitTimer3.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitTimer3.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitTimer3.NVIC_IRQChannelSubPriority=3;
	NVIC_Init(&NVIC_InitTimer3);
	
	TIM_Cmd(TIM3,ENABLE);
	
}	

void TIM3_IRQHandler(void)		
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update))
	{
		PID_out();
		TIM_ClearFlag(TIM3,TIM_IT_Update);
	}
}
