#ifndef _PID_H
#define _PID_H

void PID_Init_inner(PID_Struct *pid,float p,float i,float d,float maxI,float maxOut,float sample_freq, float cutoff_freq);
void PID_Init_outer(PID_Struct *pid,float p,float i,float d,float maxI,float maxOut);

void PID_Calc(PID_Struct *pid,float reference,float feedback);
void PID_Calc_filter(PID_Struct *pid,float reference,float feedback);
void PID_CascadeCalc(CascadePID_Struct *pid,float outerRef,float outerFdb,float innerFdb);


#endif


