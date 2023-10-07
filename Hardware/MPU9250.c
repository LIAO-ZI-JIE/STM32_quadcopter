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
#include "Serial.h"


#define I2C_MODE

#define MPU9250_ADDRESS		0xD0
#define Acc_Conversion  0.00119651f  //1/8196*9.80665
#define Gyro_Conversion 0.00026631f //3.141592653589793/180*0.0152587890625
Result_Struct Result_Structure;
IMU_Struct IMU_Structure;
Offset_Struct Offset_Structure={
	11,
	10,
	-642,
	651,
	-684,
	-36
};
uint8_t x_axis,y_axis,z_axis; 
uint8_t MPU9250_Mag_Calibrate_flag=0;
uint8_t MPU9250_Acc_Gryo_Calibrate_flag=0;
Calibrate_Struct Calibrate_Structure_Mag={
	109.5749,
	157.5716,
	-149.5112,
	257.2241,
	267.5712,
	244.7495
};



#if defined (I2C_MODE)
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

//void MPU9250_GetData(IMU_Struct* IMU_Structure)
//{
//	uint8_t DataH, DataL;
//	
//	DataH = MPU9250_ReadReg(ACCEL_XOUT_H);
//	DataL = MPU9250_ReadReg(ACCEL_XOUT_L);
//	IMU_Structure->AccX = (DataH << 8) | DataL;
//	
//	DataH = MPU9250_ReadReg(ACCEL_YOUT_H);
//	DataL = MPU9250_ReadReg(ACCEL_YOUT_L);
//	IMU_Structure->AccY = (DataH << 8) | DataL;
//	
//	DataH = MPU9250_ReadReg(ACCEL_ZOUT_H);
//	DataL = MPU9250_ReadReg(ACCEL_ZOUT_L);
//	IMU_Structure->AccZ = (DataH << 8) | DataL;
//	
//	DataH = MPU9250_ReadReg(GYRO_XOUT_H);
//	DataL = MPU9250_ReadReg(GYRO_XOUT_L);
//	IMU_Structure->GyroX = (DataH << 8) | DataL;
//	
//	DataH = MPU9250_ReadReg(GYRO_YOUT_H);
//	DataL = MPU9250_ReadReg(GYRO_YOUT_L);
//	IMU_Structure->GyroY = (DataH << 8) | DataL;
//	
//	DataH = MPU9250_ReadReg(GYRO_ZOUT_H);
//	DataL = MPU9250_ReadReg(GYRO_ZOUT_L);
//	IMU_Structure->GyroZ= (DataH << 8) | DataL;
//}
void MPU9250_GetData(IMU_Struct* IMU_Structure)
{ 
 	 uint16_t BUF[12];
   BUF[0]=MPU9250_ReadReg(GYRO_XOUT_L); 
   BUF[1]=MPU9250_ReadReg(GYRO_XOUT_H);
   IMU_Structure->GyroX=	(BUF[1]<<8)|BUF[0];

   BUF[2]=MPU9250_ReadReg(GYRO_YOUT_L);
   BUF[3]=MPU9250_ReadReg(GYRO_YOUT_H);
   IMU_Structure->GyroY=	(BUF[3]<<8)|BUF[2];

   BUF[4]=MPU9250_ReadReg(GYRO_ZOUT_L);
   BUF[5]=MPU9250_ReadReg(GYRO_ZOUT_H);
   IMU_Structure->GyroZ=	(BUF[5]<<8)|BUF[4];

	
	 BUF[6]=MPU9250_ReadReg(ACCEL_XOUT_L); 
   BUF[7]=MPU9250_ReadReg(ACCEL_XOUT_H);
   IMU_Structure->AccX=	(BUF[7]<<8)|BUF[6];

   BUF[8]=MPU9250_ReadReg(ACCEL_YOUT_L);
   BUF[9]=MPU9250_ReadReg(ACCEL_YOUT_H);
   IMU_Structure->AccY=	(BUF[9]<<8)|BUF[8];

   BUF[10]=MPU9250_ReadReg(ACCEL_ZOUT_L); 
   BUF[11]=MPU9250_ReadReg(ACCEL_ZOUT_H);
   IMU_Structure->AccZ=  (BUF[11]<<8)|BUF[10];

}


void MPU9250_GetData_continuous(IMU_Struct *IMU_Structure)
{
//	if(MPU9250_GetID() & 0x00) return;
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

		if(!(Data[6] & 0x08)) 
		{ // Check if magnetic sensor overflow set, if not then report data

			IMU_Structure->MagX = (((int16_t)Data[1] << 8) | Data[0])*x_axis ;  // Turn the MSB and LSB into a signed 16-bit value
			IMU_Structure->MagY = (((int16_t)Data[3] << 8) | Data[2])*y_axis ;  // Data stored as little Endian
			IMU_Structure->MagZ = (((int16_t)Data[5] << 8) | Data[4])*z_axis ; 

		}
	}
}
#endif

#if defined (I2C_SOFT_MODE)
MPU9250_WriteReg

#endif

#if defined (SPI_MODE)




#endif

void MPU9250_Init(void)
{

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

#if defined (I2C_MODE) 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);	
	I2C_InitTypeDef I2C_InitStructure;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 400000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9 ;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_Init(I2C2, &I2C_InitStructure);
	
	I2C_Cmd(I2C2, ENABLE);	
#endif
	MPU9250_WriteReg(PWR_MGMT_1, 0x80);	//重設
	Delay_ms(100);//等待100ms

	/**********************Init SLV0 i2c**********************************/	
	
	
	#if defined (I2C_MODE)
	MPU9250_WriteReg(INT_PIN_CFG ,0x02);// MPU 可直接访问MPU6050辅助I2C
	#elif defined (SPI_MODE)
	MPU9250_WriteReg(INT_PIN_CFG ,0x02);
	#endif
	
	///*******************Init GYRO and ACCEL******************************/
	MPU9250_WriteReg(MPU9250_RA_PWR_MGMT_1, 0x01);		//選擇x軸陀螺儀時鐘
	MPU9250_WriteReg(MPU9250_RA_PWR_MGMT_2, 0x00);//設置六軸全輸出
	MPU9250_WriteReg(MPU9250_RA_INT_ENABLE, 0x00);//禁止中斷
//	MPU9250_WriteReg(CONFIG, 0x03);      //輸出頻率1000hz濾波參數92  53
	MPU9250_WriteReg(SMPLRT_DIV, 0x00);  //輸出頻率不分頻(1kHz) 
	MPU9250_WriteReg(GYRO_CONFIG, 0x08); //500deg/s
	//	MPU9250_WriteReg(ACCEL_CONFIG_2, 0x04);//濾波20    用軟件濾波
	MPU9250_WriteReg(ACCEL_CONFIG, 0x08);//0x00/+-2g. 0x08/+-4g. 0x10/+-8g. 0x18(????16G)
	
  /**********************Init MAG **********************************/
	AK8963_WriteReg(AK8963_CNTL2_REG,AK8963_CNTL2_SRST); // Reset AK8963
	Delay_ms(50);
	AK8963_WriteReg(AK8963_CNTL1_REG,0x0F);//切換至讀取模式
	Delay_ms(50);

	x_axis=AK8963_ReadReg(AK8963_ASAX);
	x_axis=((((float)x_axis-128)/256)+1);
	y_axis=AK8963_ReadReg(AK8963_ASAY);
	y_axis=((((float)y_axis-128)/256)+1);
	z_axis=AK8963_ReadReg(AK8963_ASAZ);
	z_axis=((((float)z_axis-128)/256)+1);
	AK8963_WriteReg(AK8963_CNTL1_REG,0x00);
	Delay_ms(50);
	AK8963_WriteReg(AK8963_CNTL1_REG,0x16); // 工作於Continuous measurement mode2 & 16-bit output	
  Delay_ms(50);

}



uint8_t MPU9250_GetID(void)
{
	return MPU9250_ReadReg(WHO_AM_I);
}

uint8_t AK8963_GetID(void)
{
	return AK8963_ReadReg(AK8963_WHOAMI_REG);
}


void MPU9250_Calibrate_Calculate(Calibrate_Struct* Calibrate_Structure,float input[6][3])
{
	
	float** K = MakeMat(6);
	float* matrix6_1=Make6_1Matrix();
	
	Matrix_input(K,matrix6_1,input);

	
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

	free(matrix6_1);
	free(matrix6_1_result);
	
}

void MPU9250_Mag_Calibrate_PrepareData(void)
{
	static int8_t time;
	static int8_t flag=0;
	static float MPU9250_Mag_Data[6][3];
	time++;
	if(time>=50)
	{
		MPU9250_Mag_Data[flag][0]=IMU_Structure.MagX;
		MPU9250_Mag_Data[flag][1]=IMU_Structure.MagY;
		MPU9250_Mag_Data[flag][2]=IMU_Structure.MagZ;
		if(flag==5)
		{
			
			MPU9250_Calibrate_Calculate(&Calibrate_Structure_Mag,MPU9250_Mag_Data);
			for(int8_t i=0; i<6; i++)
			{
				for(int8_t j=0; j<3; j++)
				{
					printf("%f ",MPU9250_Mag_Data[i][j]);
				
				}
				printf("\r\n");
				
			}
//			printf("Ox:%f Oy:%f Oz:%f Rx:%f Ry:%f Rz:%f   \r\n",Calibrate_Structure_Mag.Ox,Calibrate_Structure_Mag.Oy,Calibrate_Structure_Mag.Oz,Calibrate_Structure_Mag.Rx,Calibrate_Structure_Mag.Ry,Calibrate_Structure_Mag.Rz);
			MPU9250_Mag_Calibrate_flag=0;
			flag=0;
		}
		OLED_ShowSignedNum(1, 1, flag, 4);
		flag++;
		time=0;
	}
}
uint8_t MPU9250_Offset_Calibrate(int16_t Sensivity)
{
	static int32_t tempGx=0,tempGy=0,tempGz=0,tempAx=0,tempAy=0,tempAz=0; 
	static uint16_t cnt=0;
	cnt++;
	tempAx+=IMU_Structure.AccX;
	tempAy+=IMU_Structure.AccY;
	tempAz+=IMU_Structure.AccZ-Sensivity;
	tempGx+=IMU_Structure.GyroX;
	tempGy+=IMU_Structure.GyroY;
	tempGz+=IMU_Structure.GyroZ;
	if(cnt==200)      
	{
		Offset_Structure.Acc_Offset_X=tempAx/cnt;
		Offset_Structure.Acc_Offset_Y=tempAy/cnt;
		Offset_Structure.Acc_Offset_Z=tempAz/cnt;
		Offset_Structure.Gyro_Offset_X=tempGx/cnt;
		Offset_Structure.Gyro_Offset_Y=tempGy/cnt;
		Offset_Structure.Gyro_Offset_Z=tempGz/cnt;
		cnt = 0;
		tempAx=0;
		tempAy=0;
		tempAz=0;
		tempGx=0;
		tempGy=0;
		tempGz=0;
		return 1;
	}
	return 0;
}
void MPU9250_Calibrate(void)
{
	if(MPU9250_Mag_Calibrate_flag==1)
	{
		MPU9250_Mag_Calibrate_PrepareData();
		return;
	}
	if(MPU9250_Acc_Gryo_Calibrate_flag==1)
	{
		if(MPU9250_Offset_Calibrate(8196))
		{
			MPU9250_Acc_Gryo_Calibrate_flag=0;
		}
		return;
	}		
//	printf("AX:%d  AY:%d AZ:%d GX:%d  GY:%d GZ:%d\r\n",Offset_Structure.Acc_Offset_X,Offset_Structure.Acc_Offset_Y,Offset_Structure.Acc_Offset_Z,Offset_Structure.Gyro_Offset_X,Offset_Structure.Gyro_Offset_Y,Offset_Structure.Gyro_Offset_Z);
	Result_Structure.Mag.X=(float)((IMU_Structure.MagX+Calibrate_Structure_Mag.Ox)/Calibrate_Structure_Mag.Rx);
	Result_Structure.Mag.Y=(float)((IMU_Structure.MagY+Calibrate_Structure_Mag.Oy)/Calibrate_Structure_Mag.Ry);
	Result_Structure.Mag.Z=(float)((IMU_Structure.MagZ+Calibrate_Structure_Mag.Oz)/Calibrate_Structure_Mag.Rz);
	Result_Structure.Acc.X=(float)(IMU_Structure.AccX-Offset_Structure.Acc_Offset_X)*Acc_Conversion;
	Result_Structure.Acc.Y=(float)(IMU_Structure.AccY-Offset_Structure.Acc_Offset_Y)*Acc_Conversion;
	Result_Structure.Acc.Z=(float)(IMU_Structure.AccZ-Offset_Structure.Acc_Offset_Z)*Acc_Conversion;
	Result_Structure.Gyro.X=(float)(IMU_Structure.GyroX-Offset_Structure.Gyro_Offset_X)*Gyro_Conversion;
	Result_Structure.Gyro.Y=(float)(IMU_Structure.GyroY-Offset_Structure.Gyro_Offset_Y)*Gyro_Conversion;
	Result_Structure.Gyro.Z=(float)(IMU_Structure.GyroZ-Offset_Structure.Gyro_Offset_Z)*Gyro_Conversion;
//	printf("X:%f  Y:%f Z:%f \r\n",Mag_result.X,Mag_result.Y,Mag_result.Z);
//	printf("Ax:%f  Ay:%f  Az:%f  Gx:%f  Gy:%f   Gz:%f  Mx:%f  My:%f  Mz:%f \r\n",Result_Structure.Acc.X,Result_Structure.Acc.Y,Result_Structure.Acc.Z,Result_Structure.Gyro.X,Result_Structure.Gyro.Y,Result_Structure.Gyro.Z,Result_Structure.Mag.X,Result_Structure.Mag.Y,Result_Structure.Mag.Z);
}
