#include "stm32f10x.h"                  // Device header
void Timer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_InternalClockConfig(TIM3);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//濾波器時鐘選擇
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//計數器模式
	TIM_TimeBaseInitStructure.TIM_Period=1000-1;//ARR計數
	TIM_TimeBaseInitStructure.TIM_Prescaler=720-1;//預分頻器
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//重複計數器
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ClearFlag(TIM3, TIM_FLAG_Update);//因為Timebase會觸發中斷,清除中斷
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);//使能中斷

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM3, ENABLE);

}

/*
void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}
*/
