#ifndef _fuzzyPID_h_
#ifdef DIGITALCTRL_USING_DOUBLE
typedef double dc_t;
#else
typedef float dc_t;
#endif
#define max(a, b)			(a>b? a:b)
#define min(a, b)			(a<b? a:b)
#define RANGE(x, a, b)		(min(max(x, a), b))
typedef struct{
	float Kp;
	float Ki;
	float Kd;
}FUZZY_PID_t;
void fuzzy(float e,float ec,FUZZY_PID_t *fuzzy_PID);			
#endif

struct dc_pid {
	dc_t max_limit;		
	dc_t min_limit;			
	dc_t rt;		
	dc_t yt;		
	dc_t out;
	dc_t kp;
	dc_t ki;
	dc_t kd;
	dc_t e_0;			
	dc_t e_1;			
	dc_t e_2;			
};
void dc_pid_init(struct dc_pid *pid,dc_t kp,dc_t ki,dc_t kd,dc_t out_min,dc_t out_max);
dc_t dc_pid_calc(struct dc_pid *pid,dc_t target,dc_t feedback);
#endif
