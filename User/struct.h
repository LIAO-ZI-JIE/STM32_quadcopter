#ifndef _STRUCTURE_H
#define _STRUCTURE_H
typedef struct
{
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
	int16_t Temp;
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;
	int16_t MagX;
	int16_t MagY;
	int16_t MagZ;
}IMU_Struct;
typedef struct
{
	int16_t X;
	int16_t Y;
	int16_t Z;
}int16_Struct;


typedef struct
{
	float Roll;
	float Pitch;
	float Yaw;
}Attitude_Struct;

typedef struct PID
{
  float P;         //参数
  float I;
  float D;
  float Error;     //比例项
  float Integral;  //积分项
  float Differ;    //微分项
  float PreError;
  float PrePreError;
  float Ilimit; 
  float Irang;
  float Pout;
  float Iout;
  float Dout;
  float OutPut;   
  uint8_t Ilimit_flag;    //积分分离	
}PID_TYPE; 

typedef struct
{
	float Ox;
	float Oy;
	float Oz;
	float Rx;
	float Ry;
	float Rz;
}Calibrate_Struct;

typedef struct
{
	float X;
	float Y;
	float Z;
}XYZ_Struct;

typedef struct
{	
	int16_t Acc_Offset_X;
	int16_t Acc_Offset_Y;
	int16_t Acc_Offset_Z;
	int16_t Gyro_Offset_X;
	int16_t Gyro_Offset_Y;
	int16_t Gyro_Offset_Z;

}Offset_Struct;
typedef struct
{
	XYZ_Struct Acc;
	XYZ_Struct Gyro;
	XYZ_Struct Mag;
}Result_Struct;
extern IMU_Struct IMU_Structure;
extern Attitude_Struct Attitude_Structure;
extern Calibrate_Struct Calibrate_Structure_Acc;
extern XYZ_Struct Acc_result;
#endif


