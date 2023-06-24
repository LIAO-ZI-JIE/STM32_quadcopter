#ifndef __MPU9250_H
#define __MPU9250_H

void MPU9250_WriteReg(uint8_t RegAddress, uint8_t Data);
uint8_t MPU9250_ReadReg(uint8_t RegAddress);

void MPU9250_Init(void);
uint8_t MPU9250_GetID(void);
uint8_t AK8963_GetID(void);
void MPU9250_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ);
void MPU9250_ReadReg_continuous(uint8_t RegAddress,IMU_Struct *IMU_Structure);
void i2c_Mag_write(u8 reg,u8 value);
void AK8963_WriteReg(uint8_t RegAddress, uint8_t Data);

#endif
