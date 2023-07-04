//#include "stm32f10x.h"                  // Device header
////首先定义PID结构体用于存放一个PID的数据
//typedef struct
//{
//   	float kp,ki,kd;//三个系数
//    float error,lastError;//误差、上次误差
//    float integral,maxIntegral;//积分、积分限幅
//    float output,maxOutput;//输出、输出限幅
//}PID;
// 
////用于初始化pid参数的函数
//void PID_Init(PID *pid,float p,float i,float d,float maxI,float maxOut)
//{
//    pid->kp=p;
//    pid->ki=i;
//    pid->kd=d;
//    pid->maxIntegral=maxI;
//    pid->maxOutput=maxOut;
//}
// 
////进行一次pid计算
////参数为(pid结构体,目标值,反馈值)，计算结果放在pid结构体的output成员中
//void PID_Calc(PID *pid,float reference,float feedback)
//{
// 	//更新数据
//    pid->lastError=pid->error;//将旧error存起来
//    pid->error=reference-feedback;//计算新error
//    //计算微分
//    float dout=(pid->error-pid->lastError)*pid->kd;
//    //计算比例
//    float pout=pid->error*pid->kp;
//    //计算积分
//    pid->integral+=pid->error*pid->ki;
//    //积分限幅
//    if(pid->integral > pid->maxIntegral) pid->integral=pid->maxIntegral;
//    else if(pid->integral < -pid->maxIntegral) pid->integral=-pid->maxIntegral;
//    //计算输出
//    pid->output=pout+dout+pid->integral;
//    //输出限幅
//    if(pid->output > pid->maxOutput) pid->output=pid->maxOutput;
//    else if(pid->output < -pid->maxOutput) pid->output=-pid->maxOutput;
//}
// 
//PID mypid;//创建一个PID结构体变量
// 
//int main()
//{
//    //...这里有些其他初始化代码
//    PID_Init(&mypid,10,1,5,800,1000);//初始化PID参数
//    while(1)//进入循环运行
//    {
//        float feedbackValue=...;//这里获取到被控对象的反馈值
//        float targetValue=...;//这里获取到目标值
//        PID_Calc(&mypid,targetValue,feedbackValue);//进行PID计算，结果在output成员变量中
//        设定执行器输出大小(mypid.output);
//        delay(10);//等待一定时间再开始下一次循环
//    }
//}

////此处需要插入上面的单级PID相关代码
// 
////串级PID的结构体，包含两个单级PID
//typedef struct
//{
//    PID inner;//内环
//    PID outer;//外环
//    float output;//串级输出，等于inner.output
//}CascadePID;
// 
////串级PID的计算函数
////参数(PID结构体,外环目标值,外环反馈值,内环反馈值)
//void PID_CascadeCalc(CascadePID *pid,float outerRef,float outerFdb,float innerFdb)
//{
//    PID_Calc(&pid->outer,outerRef,outerFdb);//计算外环
//    PID_Calc(&pid->inner,pid->outer.output,innerFdb);//计算内环
//    pid->output=pid->inner.output;//内环输出就是串级PID的输出
//}
// 
//CascadePID mypid;//创建串级PID结构体变量
// 
//int main()
//{
//    //...其他初始化代码
//    PID_Init(&mypid.inner,10,0,0,0,1000);//初始化内环参数
//    PID_Init(&mypid.outer,5,0,5,0,100);//初始化外环参数
//    while(1)//进入循环运行
//    {
//        float outerTarget=...;//获取外环目标值
//        float outerFeedback=...;//获取外环反馈值
//        float innerFeedback=...;//获取内环反馈值
//        PID_CascadeCalc(&mypid,outerTarget,outerFeedback,innerFeedback);//进行PID计算
//        设定执行机构输出大小(mypid.output);
//        delay(10);//延时一段时间
//    }
//}