#include "stm32f10x.h"                  // Device header
#include "MPU9250_Reg.h"
#include <stdlib.h>
#include <Math.h>
#include "Struct.h"
#include "Matrix.h"
#include "Delay.h"
#include "Struct.h"
#include "LED.h"
#include "oled.h"
#define MPU9250_ADDRESS		0xD0
XYZ_Struct Acc_result;
uint8_t x_axis,y_axis,z_axis; 
uint8_t MPU9250_Calibrate_flag=1;
Calibrate_Struct Calibrate_Structure_Acc;
void MPU9250_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_EVENT)
{
	uint32_t Timeout;
	Timeout = 10000;
	while (I2C_CheckEvent(I2Cx, I2C_EVENT) != SUCCESS)
	{
		Timeout --;
		if (Timeout == 0)
		{
			break;
		}
	}
}

void MPU9250_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU9250_ADDRESS, I2C_Direction_Transmitter);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2, RegAddress);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C2, Data);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2C2, ENABLE);
}



void AK8963_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, AK8963_ADDR, I2C_Direction_Transmitter);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2, RegAddress);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING);
	
	I2C_SendData(I2C2, Data);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTOP(I2C2, ENABLE);
}

uint8_t AK8963_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, AK8963_ADDR, I2C_Direction_Transmitter);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2, RegAddress);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, AK8963_ADDR, I2C_Direction_Receiver);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);
	
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data = I2C_ReceiveData(I2C2);
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	
	return Data;
}

uint8_t MPU9250_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU9250_ADDRESS, I2C_Direction_Transmitter);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2, RegAddress);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU9250_ADDRESS, I2C_Direction_Receiver);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);
	
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data = I2C_ReceiveData(I2C2);
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);
	
	return Data;
}

void MPU9250_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 400000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9 ;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_Init(I2C2, &I2C_InitStructure);
	
	I2C_Cmd(I2C2, ENABLE);	
	MPU9250_WriteReg(PWR_MGMT_1, 0x80);	//重設
	Delay_ms(100);//等待100ms

	/**********************Init SLV0 i2c**********************************/	
	MPU9250_WriteReg(INT_PIN_CFG ,0x02);// MPU 可直接访问MPU6050辅助I2C
	///*******************Init GYRO and ACCEL******************************/	
	MPU9250_WriteReg(CONFIG, 0x07);      //???????????0x07(3600Hz)???????Internal_Sample_Rate==8K
	MPU9250_WriteReg(SMPLRT_DIV, 0x07);  //???????????0x07(1kHz) (SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV) )
	MPU9250_WriteReg(GYRO_CONFIG, 0x18); //???????????????0x18(????2000deg/s)
	MPU9250_WriteReg(ACCEL_CONFIG_2, 0x08);//????????? ??? ?0x08  ?1.13kHz?	
	MPU9250_WriteReg(ACCEL_CONFIG, 0x10);//??????????????????????0x00/+-2g. 0x08/+-4g. 0x10/+-8g. 0x18(????16G)
	
  /**********************Init MAG **********************************/
	AK8963_WriteReg(AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
	Delay_ms(50);
	AK8963_WriteReg(AK8963_CNTL1_REG,0x0F);
	Delay_ms(50);

	x_axis=AK8963_ReadReg(AK8963_ASAX);// X???????
	x_axis=((((float)x_axis-128)/256)+1);
	y_axis=AK8963_ReadReg(AK8963_ASAY);
	y_axis=((((float)y_axis-128)/256)+1);
	z_axis=AK8963_ReadReg(AK8963_ASAZ);
	z_axis=((((float)z_axis-128)/256)+1);
	AK8963_WriteReg(AK8963_CNTL1_REG,0x00);
	Delay_ms(50);
	AK8963_WriteReg(AK8963_CNTL1_REG,0x16); // use i2c to set AK8963 working on Continuous measurement mode2 & 16-bit output	
    Delay_ms(50);

}

//void MPU9250_Init(void)
//{
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//	
//	GPIO_InitTypeDef GPIO_InitStructure;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	
//	I2C_InitTypeDef I2C_InitStructure;
//	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//	I2C_InitStructure.I2C_ClockSpeed = 400000;
//	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9 ;
//	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
//	I2C_Init(I2C2, &I2C_InitStructure);
//	
//	I2C_Cmd(I2C2, ENABLE);
//	MPU9250_WriteReg(MPU9250_PWR_MGMT_1,0X80);//复位MPU9250
//	Delay_ms(100);	
//	MPU9250_WriteReg(MPU9250_PWR_MGMT_1, 0x01);
//	MPU9250_WriteReg(MPU9250_PWR_MGMT_2, 0x00);
//	MPU9250_WriteReg(MPU9250_SMPLRT_DIV, 0x09);
//	MPU9250_WriteReg(MPU9250_CONFIG, 0x06);
//	MPU9250_WriteReg(MPU9250_GYRO_CONFIG, 0x18);
//	MPU9250_WriteReg(MPU9250_ACCEL_CONFIG, 0x18);



////	MPU9250_WriteReg(MPU9250_RA_PWR_MGMT_1,0X80);//复位MPU9250
////	Delay_ms(100);
////	MPU9250_WriteReg(MPU9250_RA_PWR_MGMT_1,0X01);//唤醒MPU9250,并选择PLL为时钟源
////	MPU9250_WriteReg(MPU9250_RA_PWR_MGMT_2,0X00);//设置输出三轴陀螺仪与加速度仪数据
////	MPU9250_WriteReg(MPU9250_RA_INT_ENABLE,0X00);//禁止中断
////	MPU9250_WriteReg(MPU9250_RA_SMPLRT_DIV,0X00);//采样分频（采样频率=陀螺仪输出频率/(1+DIV),采样频率1000Hz）	
////	MPU9250_WriteReg(MPU9250_RA_GYRO_CONFIG,0X18);//陀螺仪满量程+-2000°/s	(最低分辨率=2^16/4000=16.4LSB/°/s)
////	MPU9250_WriteReg(MPU9250_RA_ACCEL_CONFIG,0X08);//加速度满量程+-4g  (最低分辨率=2^16/8g=8196LSB/g)
////	MPU9250_WriteReg(MPU9250_RA_ACCEL_CONFIG_2,0X04);//设置加速度滤波为DLPF=20Hz
////	MPU9250_WriteReg(MPU9250_RA_CONFIG,MPU9250_DLPF_BW_98);//设置陀螺仪的输出为1KHz，DLPF=98HZ
////	MPU9250_WriteReg(MPU9250_RA_INT_PIN_CFG,0X02);//MPU 可直接访问MPU9250辅助IIC



//}

uint8_t MPU9250_GetID(void)
{
	return MPU9250_ReadReg(WHO_AM_I);
}

uint8_t AK8963_GetID(void)
{
	return AK8963_ReadReg(AK8963_WHOAMI_REG);
}


void MPU9250_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;
	
	DataH = MPU9250_ReadReg(ACCEL_XOUT_H);
	DataL = MPU9250_ReadReg(ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL;
	
	DataH = MPU9250_ReadReg(ACCEL_YOUT_H);
	DataL = MPU9250_ReadReg(ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;
	
	DataH = MPU9250_ReadReg(ACCEL_ZOUT_H);
	DataL = MPU9250_ReadReg(ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;
	
	DataH = MPU9250_ReadReg(GYRO_XOUT_H);
	DataL = MPU9250_ReadReg(GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL;
	
	DataH = MPU9250_ReadReg(GYRO_YOUT_H);
	DataL = MPU9250_ReadReg(GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL;
	
	DataH = MPU9250_ReadReg(GYRO_ZOUT_H);
	DataL = MPU9250_ReadReg(GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;
}


void MPU9250_GetData_continuous(IMU_Struct *IMU_Structure)
{
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU9250_ADDRESS, I2C_Direction_Transmitter);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2, ACCEL_XOUT_H);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, MPU9250_ADDRESS, I2C_Direction_Receiver);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->AccX = ((I2C_ReceiveData(I2C2))<<8);	

	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->AccX |= (I2C_ReceiveData(I2C2));	
	
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->AccY = ((I2C_ReceiveData(I2C2))<<8);	

	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->AccY |=  I2C_ReceiveData(I2C2);	

	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->AccZ = ((I2C_ReceiveData(I2C2))<<8);	

	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->AccZ |=  I2C_ReceiveData(I2C2);	
	
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->Temp = ((I2C_ReceiveData(I2C2))<<8);	

	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->Temp |= (I2C_ReceiveData(I2C2));	
	
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->GyroX = ((I2C_ReceiveData(I2C2))<<8);	

	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->GyroX |= (I2C_ReceiveData(I2C2));
	
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->GyroY = ((I2C_ReceiveData(I2C2))<<8);	

	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->GyroY |= (I2C_ReceiveData(I2C2));	

	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->GyroZ = ((I2C_ReceiveData(I2C2))<<8);	

	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);
	
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	IMU_Structure->GyroZ |= (I2C_ReceiveData(I2C2));
	
	I2C_AcknowledgeConfig(I2C2, ENABLE);
}


//void READ_MPU9250_MAG(IMU_Struct *IMU_Structure)
//{
//	uint8_t Data[8];
//	uint8_t c;
//	I2C_GenerateSTART(I2C2, ENABLE);
//	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
//	
//	I2C_Send7bitAddress(I2C2, AK8963_ADDR, I2C_Direction_Transmitter);
//	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
//	
//	I2C_SendData(I2C2, MAG_XOUT_L);
//	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
//	
//	I2C_GenerateSTART(I2C2, ENABLE);
//	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
//	
//	I2C_Send7bitAddress(I2C2, AK8963_ADDR, I2C_Direction_Receiver);
//	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
//	
//	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
//	Data[0] = I2C_ReceiveData(I2C2);	

//	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
//	Data[1] = I2C_ReceiveData(I2C2);	
//	
//	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
//	Data[2] = I2C_ReceiveData(I2C2);	

//	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
//	Data[3] = I2C_ReceiveData(I2C2);	

//	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
//	Data[4] = I2C_ReceiveData(I2C2);	

//	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
//	Data[5]=  I2C_ReceiveData(I2C2);	
//	


//	
//	
//		

//	I2C_AcknowledgeConfig(I2C2, DISABLE);
//	I2C_GenerateSTOP(I2C2, ENABLE);
//	
//	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
//	Data[6]=  I2C_ReceiveData(I2C2);
//	
//	I2C_AcknowledgeConfig(I2C2, ENABLE);
//	c = Data[6];
////	LED(1);
//	if(!(c & 0x08)) 
//	{ // Check if magnetic sensor overflow set, if not then report data
//	LED(1);
//	IMU_Structure->MagX = (((int16_t)Data[1] << 8) | Data[0])*(((x_axis-128)>>8)+1) ;  // Turn the MSB and LSB into a signed 16-bit value
//	IMU_Structure->MagY = (((int16_t)Data[3] << 8) | Data[2])*(((y_axis-128)>>8)+1) ;  // Data stored as little Endian
//	IMU_Structure->MagZ = (((int16_t)Data[5] << 8) | Data[4])*(((z_axis-128)>>8)+1) ; 
//	OLED_ShowSignedNum(2, 1, Data[6], 5);
//    OLED_ShowSignedNum(3, 1, (((int16_t)Data[3] << 8) | Data[2]), 5);
//    OLED_ShowSignedNum(4, 1, ((Data[5]<<8)|Data[4]), 5);	
//	}
//}
//void READ_MPU9250_MAG(IMU_Struct *IMU_Structure)
//{ 	
//	uint16_t BUF[6];	



//	
//	if((AK8963_ReadReg(AK8963_ST1_REG)&AK8963_ST1_DOR)==0)//data ready
//	{
//		
//			//????X???
//		 BUF[0]=AK8963_ReadReg(MAG_XOUT_L); //Low data	
//		 if((AK8963_ReadReg(AK8963_ST2_REG)&AK8963_ST2_HOFL)==1)// data reading end register & check Magnetic sensor overflow occurred 
//		 {
//			 BUF[0]=AK8963_ReadReg(MAG_XOUT_L);//reload data
//		 } 
//		 BUF[1]=AK8963_ReadReg(MAG_XOUT_H); //High data	
//		 if((AK8963_ReadReg(AK8963_ST2_REG)&AK8963_ST2_HOFL)==1)// data reading end register
//		 {
//			 BUF[1]=AK8963_ReadReg(MAG_XOUT_H);
//		 }
//		 IMU_Structure->MagX=((BUF[1]<<8)|BUF[0])*(((x_axis-128)>>8)+1);		//????? ???/RM-MPU-9250A-00 PDF/ 5.13	
//		 
//		//????Y???
//		BUF[2]=AK8963_ReadReg(MAG_YOUT_L); //Low data	
//		 if((AK8963_ReadReg(AK8963_ST2_REG)&AK8963_ST2_HOFL)==1)// data reading end register
//		 {
//			 BUF[2]=AK8963_ReadReg(MAG_YOUT_L);
//		 }		 
//		 BUF[3]=AK8963_ReadReg(MAG_YOUT_H); //High data	
//		 if((AK8963_ReadReg(AK8963_ST2_REG)&AK8963_ST2_HOFL)==1)// data reading end register
//		 {
//			 BUF[3]=AK8963_ReadReg(MAG_YOUT_H);
//		 }
//		 IMU_Structure->MagY=((BUF[3]<<8)|BUF[2])*(((y_axis-128)>>8)+1);	
//		 
//		//????Z???
//		 BUF[4]=AK8963_ReadReg(MAG_ZOUT_L); //Low data	
//		 if((AK8963_ReadReg(AK8963_ST2_REG)&AK8963_ST2_HOFL)==1)// data reading end register
//		 {
//			 BUF[4]=AK8963_ReadReg(MAG_ZOUT_L);
//		 }	 
//		 BUF[5]=AK8963_ReadReg(MAG_ZOUT_H); //High data	
//		 if((AK8963_ReadReg(AK8963_ST2_REG)&AK8963_ST2_HOFL)==1)// data reading end register
//		 {
//			 BUF[5]=AK8963_ReadReg(MAG_ZOUT_H);
//		 }
//		 OLED_ShowHexNum(1, 8, BUF[3] ,5);
//		 IMU_Structure->MagZ=((BUF[5]<<8)|BUF[4])*(((z_axis-128)>>8)+1);	
//		 OLED_ShowSignedNum(2, 1, ((BUF[1]<<8)|BUF[0]), 5);
//		 OLED_ShowSignedNum(3, 1, y_axis, 5);
//		 OLED_ShowSignedNum(4, 1, ((BUF[5]<<8)|BUF[4]), 5);	
//		 
//	}
//}

void READ_MPU9250_MAG(IMU_Struct *IMU_Structure)
{ 		

	if(AK8963_ReadReg(AK8963_ST1_REG) & 0x01)//data ready
	{
		
	uint8_t Data[8];
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, AK8963_ADDR, I2C_Direction_Transmitter);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	
	I2C_SendData(I2C2, MAG_XOUT_L);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	
	I2C_GenerateSTART(I2C2, ENABLE);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT);
	
	I2C_Send7bitAddress(I2C2, AK8963_ADDR, I2C_Direction_Receiver);
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data[0] = I2C_ReceiveData(I2C2);	

	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data[1] = I2C_ReceiveData(I2C2);	
	
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data[2] = I2C_ReceiveData(I2C2);	

	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data[3] = I2C_ReceiveData(I2C2);	

	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data[4] = I2C_ReceiveData(I2C2);	

	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data[5]=  I2C_ReceiveData(I2C2);	

	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);
	
	MPU9250_WaitEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED);
	Data[6]=  I2C_ReceiveData(I2C2);
//	OLED_ShowSignedNum(2, 1, Data[6], 5);
//	Data[0]=AK8963_ReadReg(0x03);
//	Data[1]=AK8963_ReadReg(0x04);
//	Data[2]=AK8963_ReadReg(0x05);
//	Data[3]=AK8963_ReadReg(0x06);
//	Data[4]=AK8963_ReadReg(0x07);
//	Data[5]=AK8963_ReadReg(0x08);
//	Data[6]=AK8963_ReadReg(0x09);

	if(!(Data[6] & 0x08)) 
	{ // Check if magnetic sensor overflow set, if not then report data

	IMU_Structure->MagX = (((int16_t)Data[1] << 8) | Data[0])*x_axis ;  // Turn the MSB and LSB into a signed 16-bit value
	IMU_Structure->MagY = (((int16_t)Data[3] << 8) | Data[2])*y_axis ;  // Data stored as little Endian
	IMU_Structure->MagZ = (((int16_t)Data[5] << 8) | Data[4])*z_axis ; 

	}
	}
}

void MPU9250_Calibrate_Calculate(Calibrate_Struct* Calibrate_Structure,float input[6][3])
{
	
	float** K = MakeMat(6);
	float* matrix6_1=Make6_1Matrix();
	// Matrix_input(K,matrix6_1,101.337205935947,131.172918165603,502.676396433907);
	// Matrix_input(K,matrix6_1,159.509326414305,251.156295969072,269.250000017474);
	// Matrix_input(K,matrix6_1,74.0641529086098,265.804486765913,265.804486765913);	
	// Matrix_input(K,matrix6_1,265.804486765913,181.950505227651,414.54463162164);
	// Matrix_input(K,matrix6_1,29.8372324206984,-35.7615518330834,342.856728256145);
	// Matrix_input(K,matrix6_1,129.494077118401,89.4477295320249,478.638676558308);

	// Matrix_input(K,matrix6_1,87, -52, -4454);
	// Matrix_input(K,matrix6_1,-3805, -24, -390);
	// Matrix_input(K,matrix6_1,4389, 6, -228);	
	// Matrix_input(K,matrix6_1,-1963, -13, -3797);
	// Matrix_input(K,matrix6_1,4389, 6, -228);
	// Matrix_input(K,matrix6_1,327, -2047, -3880);
	
	Matrix_input(K,matrix6_1,input);

	
	// Matrix_input(K,matrix6_1,1, 0, 0);
	// Matrix_input(K,matrix6_1,-1, 0, 0);
	// Matrix_input(K,matrix6_1,0, 1, 0);	
	// Matrix_input(K,matrix6_1,0, -1, 0);
	// Matrix_input(K,matrix6_1,0, 0, 1);
	// Matrix_input(K,matrix6_1,0,0, -1);
	
	float** Kt = Matrix_Transpose(K);
	float** KtK=Matrix_Multiply(Kt,K);
	free_Matrix(K);
	float** KtK_inverse = Matrix_inverse(KtK);
	free_Matrix(KtK);
	float** KtK_inverseKt=Matrix_Multiply(KtK_inverse,Kt);
	free_Matrix(Kt);
	free_Matrix(KtK_inverse);
	float* matrix6_1_result=Matrix6_1_Multiply(KtK_inverseKt,matrix6_1);
	free_Matrix(KtK_inverseKt);
	
	Calibrate_Structure->Ox=-matrix6_1_result[2]/2;
	Calibrate_Structure->Oy=-matrix6_1_result[3]/(2*matrix6_1_result[0]);
	Calibrate_Structure->Oz=-matrix6_1_result[4]/(2*matrix6_1_result[1]);
	Calibrate_Structure->Rx=sqrt(Calibrate_Structure->Ox*Calibrate_Structure->Ox+matrix6_1_result[0]*Calibrate_Structure->Oy*Calibrate_Structure->Oy+matrix6_1_result[1]*Calibrate_Structure->Oz*Calibrate_Structure->Oz-matrix6_1_result[5]);
	Calibrate_Structure->Ry=sqrt(Calibrate_Structure->Rx*Calibrate_Structure->Rx/matrix6_1_result[0]);
	Calibrate_Structure->Rz=sqrt(Calibrate_Structure->Rx*Calibrate_Structure->Rx/matrix6_1_result[1]);

	// printf("\nY\n");
	// printf("%f  ",matrix6_1[0]);
	// printf("%f  ",matrix6_1[1]);
	// printf("%f  ",matrix6_1[2]);
	// printf("%f  ",matrix6_1[3]);
	// printf("%f  ",matrix6_1[4]);
	// printf("%f  ",matrix6_1[5]);

	// printf("\nabc\n");
	// printf("%f  ",matrix6_1_result[0]);
	// printf("%f  ",matrix6_1_result[1]);
	// printf("%f  ",matrix6_1_result[2]);
	// printf("%f  ",matrix6_1_result[3]);
	// printf("%f  ",matrix6_1_result[4]);
	// printf("%f  ",matrix6_1_result[5]);

	//	printf("\noR\n");
	//	printf("%f  ",Ox);
	//	printf("%f  ",Oy);
	//	printf("%f  ",Oz);
	//	printf("%f  ",Rx);
	//	printf("%f  ",Ry);
	//	printf("%f  ",Rz);

	// printf("\n原矩阵:>");
	// print(K);
	// printf("轉置矩阵:>");
	// print(Kt);
	// printf("矩陣相乘:>");
	// print(KtK);	
	// printf("逆矩阵:>");
	// print(KtK_inverse);
	// printf("\n逆矩阵*Kt:>");
	// print(KtK_inverseKt);	
	free(matrix6_1);
	free(matrix6_1_result);
	
}

void MPU9250_Calibrate_PrepareData(void)
{
	static int time;
	static int flag=0;
	static float MPU9250_Acc_Data[6][3]={	
											{120.23,0,0.1},
											{-80.32,0.123,-0.1},
											{0.1,100.43,-0.2},
											{0.11,-100.43,-0.1253},
											{0.11,0.114,100.54},
											{0.2,-0.07,-100.34}
										};
	time++;
	if(time>=50)
	{
//		LED(1);
		if(flag==6)
		{
			
			MPU9250_Calibrate_Calculate(&Calibrate_Structure_Acc,MPU9250_Acc_Data);
			MPU9250_Calibrate_flag=0;
			flag=0;
		}
		MPU9250_Acc_Data[flag][0]=IMU_Structure.AccX;
		MPU9250_Acc_Data[flag][1]=IMU_Structure.AccY;
		MPU9250_Acc_Data[flag][2]=IMU_Structure.AccZ;
//		LED(0);
		OLED_ShowSignedNum(1, 1, flag, 4);
		flag++;
		time=0;
	}
}

void MPU9250_Calibrate(void)
{
	if(MPU9250_Calibrate_flag==1)
	{
		MPU9250_Calibrate_PrepareData();
		return;
	}
	Acc_result.X=(float)((IMU_Structure.AccX+Calibrate_Structure_Acc.Ox)/Calibrate_Structure_Acc.Rx);
	Acc_result.Y=(float)((IMU_Structure.AccY+Calibrate_Structure_Acc.Oy)/Calibrate_Structure_Acc.Ry);
	Acc_result.Z=(float)((IMU_Structure.AccZ+Calibrate_Structure_Acc.Oz)/Calibrate_Structure_Acc.Rz);
	
}
