#include "stm32f10x.h"                  // Device header
#include "Struct.h"
Remote_Control_Struct Remote_Control_Structure;
void IC_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);	
	
	TIM_InternalClockConfig(TIM1);
	TIM_InternalClockConfig(TIM4);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;//濾波器時鐘選擇
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;//計數器模式
	TIM_TimeBaseInitStruct.TIM_Period=65535-1;//ARR計數
	TIM_TimeBaseInitStruct.TIM_Prescaler=72-1;//PSC預分頻器
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter=0;//重複計數器
	TIM_TimeBaseInit(TIM1,&TIM_TimeBaseInitStruct);
	
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;//輸入通道
	TIM_ICInitStructure.TIM_ICFilter=0xF;//輸入濾波強度選擇
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//上升源觸發
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;//輸入分頻選擇選擇
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;//輸入是否交差
	TIM_ICInit(TIM1,&TIM_ICInitStructure);

	TIM_PWMIConfig(TIM1,&TIM_ICInitStructure);//PWM1模式

	TIM_ICInitStructure.TIM_Channel=TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Falling;//上升源觸發
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_IndirectTI;//輸入是否交差
	TIM_ICInit(TIM1,&TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_4;//輸入通道
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//上升源觸發
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;//輸入是否交差
	TIM_ICInit(TIM1,&TIM_ICInitStructure);	

	
	TIM_SelectInputTrigger(TIM1,TIM_TS_TI1FP1);//選擇從模式通道
	TIM_SelectSlaveMode(TIM1,TIM_SlaveMode_Reset);//從模式選擇清零
	
	

	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStruct);
	

	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;//輸入通道
	TIM_ICInitStructure.TIM_ICFilter=0xF;//輸入濾波強度選擇
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//上升源觸發
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;//輸入分頻選擇選擇
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;//輸入是否交差
	TIM_ICInit(TIM4,&TIM_ICInitStructure);

	TIM_PWMIConfig(TIM4,&TIM_ICInitStructure);//PWM1模式

	TIM_ICInitStructure.TIM_Channel=TIM_Channel_3;
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;//上升源觸發
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;//輸入是否交差
	TIM_ICInit(TIM4,&TIM_ICInitStructure);
	
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_4;//輸入通道
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Falling;//上升源觸發
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_IndirectTI;//輸入是否交差
	TIM_ICInit(TIM4,&TIM_ICInitStructure);	
	
	TIM_SelectInputTrigger(TIM4,TIM_TS_TI1FP1);//選擇從模式通道
	TIM_SelectSlaveMode(TIM4,TIM_SlaveMode_Reset);//從模式選擇清零
	
	TIM_Cmd(TIM1,ENABLE);
	TIM_Cmd(TIM4,ENABLE);	
}

void Get_Remote_Control(void)
{	
	Remote_Control_Structure.THROTTLE=(TIM_GetCapture2(TIM1)+1);
	Remote_Control_Structure.YAW=(TIM_GetCapture3(TIM1)+1)-(TIM_GetCapture4(TIM1)+1);
	Remote_Control_Structure.PITCH=(TIM_GetCapture2(TIM4)+1);
	Remote_Control_Structure.ROLL=(TIM_GetCapture4(TIM4)+1)-(TIM_GetCapture3(TIM4)+1);
}	
