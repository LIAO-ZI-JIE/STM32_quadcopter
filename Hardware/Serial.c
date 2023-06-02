#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
uint8_t Serial_RxData;
uint8_t Serial_RxFlag;
void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	 
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;	 
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=115200;//�j�v
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//�y��
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx ;//��f�Ҧ�(��J�ο�X)
	USART_InitStructure.USART_Parity=USART_Parity_No;//�ƾڸ����
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//��������
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//�ƾڪ���
	USART_Init(USART1,&USART_InitStructure);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//�}�ұ������_
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
	
	USART_Cmd(USART1,ENABLE);
}
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1,Byte);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);
}

void Serial_SendArray(uint8_t *Array,uint16_t Length)
{
	uint16_t i;
	
	for(i=0;i<Length;i++)
	{
		Serial_SendByte(Array[i]);
	}
}
void Serial_SendString(char *String)
{
	uint8_t i;
	for(i=0;String[i] != '\0';i++)
	{
		Serial_SendByte(String[i]);
	}
}
uint32_t Pow(uint32_t X,uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}
void Serial_SendNum(uint32_t Number,uint8_t Length)
{
	uint8_t i;
	for(i=0;i<Length;i++)
	{
	Serial_SendByte(Number/Pow(10,Length-i-1)%10+'0');
	}
}
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}
uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

//uint8_t Serial_GetRxData(void)
//{
//	return Serial_RxData;
//}
//void usart1_SendByte(u8 data)
//{
//    /* 发送字节数据到UART */
//    USART_SendData(USART1,data);
// 
//    /*等待发送寄存器为空 */
//    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
//}
// 
///****************** 发送八位数组************************/
//void usart1_SendArray(vu8 *array, uint16_t num)
//{
//    vu8 i;
// 
//  for(i=0; i<num; i++)
//  {
//        /* ·发送一个字节数据到USART */
//     usart1_SendByte(array[i]);
//  }
//    /* 等待发送完成 */
//  while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET);
//}
// 
// 
//vu8 testdatatosend[50];
///*波形显示函数*/
//void Test_Send_User(uint16_t data1, uint16_t data2, uint16_t data3,uint16_t data4,uint16_t data5,uint16_t data6,uint16_t data7,uint16_t data8,uint16_t data9,uint16_t data10)
//{
//    vu8 _cnt=0;
//    vu8 sum=0;
//    vu8 i=0;
// 
//    testdatatosend[_cnt++]=0x88;
//    testdatatosend[_cnt++]=0xA1;
//    testdatatosend[_cnt++]=0;
// 
//    testdatatosend[_cnt++]=data1>>8;     
//    testdatatosend[_cnt++]=data1&0xff;   
// 
//    testdatatosend[_cnt++]=data2>>8;
//    testdatatosend[_cnt++]=data2&0xff;
// 
//    testdatatosend[_cnt++]=data3>>8;
//    testdatatosend[_cnt++]=data3&0xff;
// 
//    testdatatosend[_cnt++]=data4>>8;
//    testdatatosend[_cnt++]=data4&0xff;
// 
//    testdatatosend[_cnt++]=data5>>8;
//    testdatatosend[_cnt++]=data5&0xff;
// 
//    testdatatosend[_cnt++]=data6>>8;
//    testdatatosend[_cnt++]=data6&0xff;
// 
//    testdatatosend[_cnt++]=data7>>8;
//    testdatatosend[_cnt++]=data7&0xff;
// 
//    testdatatosend[_cnt++]=data8>>8;
//    testdatatosend[_cnt++]=data8&0xff;
// 
//    testdatatosend[_cnt++]=data9>>8;
//    testdatatosend[_cnt++]=data9&0xff;
// 
//    testdatatosend[_cnt++]=data10>>8;
//    testdatatosend[_cnt++]=data10&0xff;
// 
// 
// 
//    testdatatosend[2] = _cnt-3;
// 
//    for(i=0;i<_cnt;i++)
//        sum += testdatatosend[i];
// 
//    testdatatosend[_cnt++]=sum;
// 
//    usart1_SendArray(testdatatosend,_cnt);
//}
// 
// 
// 
// 
// 
///*****************************************************************************************
//*下面的函数主要用于无人机调试，也可用于其他pid调试
//*
//***************************************************************************************/
// 
///*发送陀螺仪数据给上位机*/
//void Test_Send_User1(int16_t acc_x_,int16_t acc_y_,int16_t acc_z_,int16_t gyro_x_,int16_t gyro_y_,int16_t gyro_z_,int16_t roll_,int16_t pitch_,int16_t yaw_)
//{
//	 vu8 _cnt=0;
//    vu8 sum=0;
//    vu8 i=0;
// 
//    testdatatosend[_cnt++]=0x88;
//    testdatatosend[_cnt++]=0xAF;
//    testdatatosend[_cnt++]=0x1C;
// 
//    testdatatosend[_cnt++]=acc_x_>>8;     //
//    testdatatosend[_cnt++]=acc_x_&0xff;   //
// 
//    testdatatosend[_cnt++]=acc_y_>>8;
//    testdatatosend[_cnt++]=acc_y_&0xff;
// 
//    testdatatosend[_cnt++]=acc_z_>>8;
//    testdatatosend[_cnt++]=acc_z_&0xff;
// 
//    testdatatosend[_cnt++]=gyro_x_>>8;
//    testdatatosend[_cnt++]=gyro_x_&0xff;
// 
//    testdatatosend[_cnt++]=gyro_y_>>8;
//    testdatatosend[_cnt++]=gyro_y_&0xff;
// 
//    testdatatosend[_cnt++]=gyro_z_>>8;
//    testdatatosend[_cnt++]=gyro_z_&0xff;
//		
//		testdatatosend[_cnt++]=0;
//		testdatatosend[_cnt++]=0;
//		
//		testdatatosend[_cnt++]=0;
//		testdatatosend[_cnt++]=0;
//		
//		testdatatosend[_cnt++]=0;
//		testdatatosend[_cnt++]=0;
// 
//    testdatatosend[_cnt++]=roll_>>8;
//    testdatatosend[_cnt++]=roll_&0xff;
// 
//    testdatatosend[_cnt++]=pitch_>>8;
//    testdatatosend[_cnt++]=pitch_&0xff;
// 
//    testdatatosend[_cnt++]=yaw_>>8;
//    testdatatosend[_cnt++]=yaw_&0xff;
// 
//    testdatatosend[_cnt++]=0x00;
//    testdatatosend[_cnt++]=0x00;
//		testdatatosend[_cnt++]=0x00;
//    testdatatosend[_cnt++]=0x00;
// 
//    testdatatosend[2] = _cnt-3;
// 
//    for(i=0;i<_cnt;i++)
//        sum += testdatatosend[i];
// 
//    testdatatosend[_cnt++]=sum;
// 
//    usart1_SendArray(testdatatosend,_cnt);
//	
//}
// 
// 
// 
// 
///*发送电机pwm和飞控电池电压给上位机*/
//void Test_Send_User2(uint16_t throt,uint16_t yaw_2,uint16_t roll_2,uint16_t pitch_2,uint16_t aux_1,uint16_t aux_2,uint16_t aux_3,uint16_t aux_4,uint16_t aux_5,uint16_t pwm1,uint16_t pwm2,uint16_t pwm3,uint16_t pwm4,uint16_t votage)
//{
//	
//	vu8 _cnt=0;
//    vu8 sum=0;
//    vu8 i=0;
// 
//    testdatatosend[_cnt++]=0x88;
//    testdatatosend[_cnt++]=0xAE;
//    testdatatosend[_cnt++]=0x12;
//	
//	  testdatatosend[_cnt++]=throt>>8;
//    testdatatosend[_cnt++]=throt&0xff;
// 
//    testdatatosend[_cnt++]=yaw_2>>8;     //
//    testdatatosend[_cnt++]=yaw_2&0xff;   //
// 
//    testdatatosend[_cnt++]=roll_2>>8;
//    testdatatosend[_cnt++]=roll_2&0xff;
// 
//    testdatatosend[_cnt++]=pitch_2>>8;
//    testdatatosend[_cnt++]=pitch_2&0xff;
// 
//    testdatatosend[_cnt++]=aux_1>>8;
//    testdatatosend[_cnt++]=aux_1&0xff;
// 
//    testdatatosend[_cnt++]=aux_2>>8;
//    testdatatosend[_cnt++]=aux_2&0xff;
// 
//    testdatatosend[_cnt++]=aux_3>>8;
//    testdatatosend[_cnt++]=aux_3&0xff;
// 
//    testdatatosend[_cnt++]=aux_4>>8;
//    testdatatosend[_cnt++]=aux_4&0xff;
// 
//    testdatatosend[_cnt++]=aux_5>>8;
//    testdatatosend[_cnt++]=aux_5&0xff;
// 
//    testdatatosend[_cnt++]=pwm1>>8;
//		testdatatosend[_cnt++]=pwm1&0xff;
// 
//    testdatatosend[_cnt++]=pwm2>>8;
//		testdatatosend[_cnt++]=pwm2&0xff;
//		
//    testdatatosend[_cnt++]=pwm3>>8;
//		testdatatosend[_cnt++]=pwm3&0xff;
// 
//    testdatatosend[_cnt++]=pwm4>>8;
//		testdatatosend[_cnt++]=pwm4&0xff;
//		
//		testdatatosend[_cnt++]=votage>>8;
//    testdatatosend[_cnt++]=votage&0xff;
// 
//    testdatatosend[2] = _cnt-3;
// 
//    for(i=0;i<_cnt;i++)
//        sum += testdatatosend[i];
// 
//    testdatatosend[_cnt++]=sum;
// 
//    usart1_SendArray(testdatatosend,_cnt);
//	
//}
// 
// 
// 
///*发送OFFSET给上位机*/
//void Test_Send_User3(int16_t acc_x_3,int16_t acc_y_3,int16_t acc_z_3,int16_t gyro_x_3,int16_t gyro_y_3,int16_t gyro_z_3)
//{
//	vu8 _cnt=0;
//    vu8 sum=0;
//    vu8 i=0;
// 
//    testdatatosend[_cnt++]=0x88;
//    testdatatosend[_cnt++]=0xAC;
//    testdatatosend[_cnt++]=0x1C;
//	  testdatatosend[_cnt++]=0xAC;
//	
//	  testdatatosend[_cnt++]=acc_x_3>>8;
//    testdatatosend[_cnt++]=acc_x_3&0xff;
// 
//    testdatatosend[_cnt++]=acc_y_3>>8;     //
//    testdatatosend[_cnt++]=acc_y_3&0xff;   //
// 
//    testdatatosend[_cnt++]=acc_z_3>>8;
//    testdatatosend[_cnt++]=acc_z_3&0xff;
// 
//    testdatatosend[_cnt++]=gyro_x_3>>8;
//    testdatatosend[_cnt++]=gyro_x_3&0xff;
// 
//    testdatatosend[_cnt++]=gyro_y_3>>8;
//    testdatatosend[_cnt++]=gyro_y_3&0xff;
// 
//    testdatatosend[_cnt++]=gyro_z_3>>8;
//    testdatatosend[_cnt++]=gyro_z_3&0xff;
// 
//    //testdatatosend[2] = _cnt-3;
// 
//    for(i=0;i<_cnt;i++)
//        sum += testdatatosend[i];
// 
//    testdatatosend[31]=sum;
// 
//    usart1_SendArray(testdatatosend,32);
//	
//	
//}
// 
// 
////给上位机发送PID参数*/
//void Test_Send_User4(uint16_t rol_p,uint16_t rol_i,uint16_t rol_d,uint16_t pit_p,uint16_t pit_i,uint16_t pit_d,uint16_t yaw_p,uint16_t yaw_i,uint16_t yaw_d)
//{
//	vu8 _cnt=0;
//    vu8 sum=0;
//    vu8 i=0;
// 
//    testdatatosend[_cnt++]=0x88;
//    testdatatosend[_cnt++]=0xAC;
//    testdatatosend[_cnt++]=0x1C;
//	  testdatatosend[_cnt++]=0xAD;
//	
//	  testdatatosend[_cnt++]=rol_p>>8;
//    testdatatosend[_cnt++]=rol_p&0xff;
// 
//    testdatatosend[_cnt++]=rol_i>>8;     //
//    testdatatosend[_cnt++]=rol_i&0xff;   //
// 
//    testdatatosend[_cnt++]=rol_d>>8;
//    testdatatosend[_cnt++]=rol_d&0xff;
// 
//    testdatatosend[_cnt++]=pit_p>>8;
//    testdatatosend[_cnt++]=pit_p&0xff;
// 
//    testdatatosend[_cnt++]=pit_i>>8;
//    testdatatosend[_cnt++]=pit_i&0xff;
// 
//    testdatatosend[_cnt++]=pit_d>>8;
//    testdatatosend[_cnt++]=pit_d&0xff;
// 
//    testdatatosend[_cnt++]=yaw_p>>8;
//    testdatatosend[_cnt++]=yaw_p&0xff;
// 
//    testdatatosend[_cnt++]=yaw_i>>8;
//    testdatatosend[_cnt++]=yaw_i&0xff;
// 
//    testdatatosend[_cnt++]=yaw_d>>8;
//    testdatatosend[_cnt++]=yaw_d&0xff;
// 
//   // testdatatosend[2] = _cnt-3;
// 
//    for(i=0;i<_cnt;i++)
//        sum += testdatatosend[i];
// 
//    testdatatosend[31]=sum;
// 
//    usart1_SendArray(testdatatosend,32);
//	
//	
//}
// 
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET	)
	{
		Serial_RxData=USART_ReceiveData(USART1);
		Serial_RxFlag=1;
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);	
	}
}