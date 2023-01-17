#include "fuzzyPID.h"
#define NL   -3
#define NM	 -2
#define NS	 -1
#define ZE	 0
#define PS	 1
#define PM	 2
#define PL	 3


#define  MAXE (10)
#define  MINE (-MAXE)

#define  MAXEC (10)
#define  MINEC (-MAXEC)

#define KE   3/MAXE
#define KEC  3/MAXEC


#define  KUP   2.0		
#define  KUI   0.05
#define  KUD   0.05

#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define range(x, a, b)		(min(max(x, a), b))
#define myabs(x)			((x<0)? -x:x)


static const float fuzzyRuleKp[7][7]={
	PL,	PL,	PM,	PL,	PS,	PM,	ZE,
	PL,	PM,	PM,	PM,	PS,	PM,	ZE,
	PM,	PS,	PS,	PS,	PS,	PS,	PM,
	PM,	PS,	ZE,	ZE,	ZE,	PS,	PM,
	PS,	PS,	PS,	PS,	PS,	PM,	PM,
	PM,	PM,	PM,	PM,	PL,	PL,	PL,
	PM,	PL,	PL,	PL,	PL,	PL,	PL
};
 
static const float fuzzyRuleKi[7][7]={
	NL,	NL,	NL,	NL,	NM,	NL,	NL,
	NL,	NL,	NM,	NM,	NM,	NL,	NL,
	NM,	NM,	NS,	NS,	NS,	NM,	NM,
	NM,	NS,	ZE,	ZE,	ZE,	NS,	NM,
	NM,	NS,	NS,	NS,	NS,	NM,	NM,
	NM,	NM,	NS,	NM,	NM,	NL,	NL,
	NM,	NL,	NM,	NL,	NL,	NL,	NL
};
 
static const float fuzzyRuleKd[7][7]={
	PS,	PS,	ZE,	ZE,	ZE,	PL,	PL,
	NS,	NS,	NS,	NS,	ZE,	NS,	PM,
	NL,	NL,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NS,	NS,	NS,	ZE,	PS,	PS,
	PS,	ZE,	ZE,	ZE,	ZE,	PL,	PL
};
 

void fuzzy(float e,float ec,FUZZY_PID_t *fuzzy_PID)
{
 
     float etemp,ectemp;					
     float eLefttemp,ecLefttemp;		
     float eRighttemp ,ecRighttemp;
 
     int eLeftIndex,ecLeftIndex;		
     int eRightIndex,ecRightIndex;
		 e = RANGE(e,MINE,MAXE);
     ec = RANGE(ec,MINEC,MAXEC);
		 e = e*KE;
		 ec = ec * KEC;

     etemp = e > 3.0 ? 0.0 : (e < - 3.0 ? 0.0 : (e >= 0.0 ? (e >= 2.0 ? 2.5: (e >= 1.0 ? 1.5 : 0.5)) : (e >= -1.0 ? -0.5 : (e >= -2.0 ? -1.5 : (e >= -3.0 ? -2.5 : 0.0) ))));
     eLeftIndex = (int)((etemp-0.5) + 3);        //[-3,3] -> [0,6]
     eRightIndex = (int)((etemp+0.5) + 3);
     eLefttemp =etemp == 0.0 ? 0.0:((etemp+0.5)-e); 			//
     eRighttemp=etemp == 0.0 ? 0.0:( e-(etemp-0.5));
     ectemp = ec > 3.0 ? 0.0 : (ec < - 3.0 ? 0.0 : (ec >= 0.0 ? (ec >= 2.0 ? 2.5: (ec >= 1.0 ? 1.5 : 0.5)) : (ec >= -1.0 ? -0.5 : (ec >= -2.0 ? -1.5 : (ec >= -3.0 ? -2.5 : 0.0) ))));
     ecLeftIndex = (int)((ectemp-0.5) + 3);        //[-3,3] -> [0,6]
     ecRightIndex = (int)((ectemp+0.5) + 3);

     ecLefttemp =ectemp == 0.0 ? 0.0:((ectemp+0.5)-ec);
     ecRighttemp=ectemp == 0.0 ? 0.0:( ec-(ectemp-0.5));


	fuzzy_PID->Kp = (eLefttemp * ecLefttemp * fuzzyRuleKp[eLeftIndex][ecLeftIndex] + eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex] + eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);
 
	fuzzy_PID->Ki = (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex] + eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex] + eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);
 
	fuzzy_PID->Kd = (eLefttemp * ecLefttemp *  fuzzyRuleKd[eLeftIndex][ecLeftIndex] + eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex]
					+ eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex] + eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);


	fuzzy_PID->Kp = fuzzy_PID->Kp * KUP;
	fuzzy_PID->Ki = fuzzy_PID->Ki * KUI;
  fuzzy_PID->Kd = fuzzy_PID->Kd * KUD;
 
}

void dc_pid_init(struct dc_pid *pid,
			dc_t kp,dc_t ki,dc_t kd,
			dc_t out_min,dc_t out_max)
{
	pid->max_limit = out_max;
	pid->min_limit = out_min;
	pid->rt = 0;
	pid->yt = 0;
	pid->out = 0;
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->e_0 = 0;
	pid->e_1 = 0;
	pid->e_2 = 0;
}

dc_t dc_pid_calc(struct dc_pid *pid,dc_t rt,dc_t yt)
{
	dc_t ep, ei, ed;
	FUZZY_PID_t fuzzy_pid;
	pid->rt = rt;
	pid->yt = yt;
	pid->e_0 = pid->rt - pid->yt;
	ep = pid->e_0  - pid->e_1;
	ei = pid->e_0;
	ed = pid->e_0 - 2*pid->e_1 + pid->e_2;
	fuzzy(ei,ep,&fuzzy_pid);
	pid->out = (pid->kp + fuzzy_pid.Kp)*ep + (pid->ki + fuzzy_pid.Ki)*ei + (pid->kd + fuzzy_pid.Kd)*ed;
	pid->out = range(pid->out, pid->min_limit, pid->max_limit);
	pid->e_2 = pid->e_1;
	pid->e_1 = pid->e_0;
	return pid->out;
}
