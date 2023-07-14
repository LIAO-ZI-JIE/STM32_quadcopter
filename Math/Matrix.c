#include "stm32f10x.h"                  // Device header
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "LED.h"
#include "oled.h"
/**
  * @brief  將矩陣釋放
  * @param  需釋放的矩陣地址
  * @retval 無 
  */
void free_Matrix(float** src)
{
    int i;
    for(i = 0;i < 6; i++)
    {
        free(src[i]);
    }
    free(src);
}
/**
  * 代碼來源:  http://t.csdn.cn/OQoUy
  * @brief  將矩陣inverse
  * @param  需要inverse的矩陣地址
  * @retval inverse好的矩陣地址 
  */
float** Matrix_inverse(float** src)
{
	//step 1
	//判断指针是否为空
	int i, j, k, row, col, n,principal;
	float** res, ** res2,tmp;//res为增广矩阵，res为输出的逆矩阵
    float Max;
	//判断矩阵维数
	row = 6;
	col = 6;
	//step 2
	res = (float**)malloc(sizeof(float*) * row);
	res2 = (float**)malloc(sizeof(float*) * row);
	n = 2 * row;
	for (i = 0; i < row; i++)
	{
		res[i] = (float*)malloc(sizeof(float) * n);
		res2[i] = (float*)malloc(sizeof(float) * col);
		memset(res[i], 0, sizeof(res[0][0]) * n);//初始化
		memset(res2[i], 0, sizeof(res2[0][0]) * col);
	}
	//step 3
	//进行数据拷贝
	for (i = 0; i < row; i++)
	{
    //此处源代码中的n已改为col，当然方阵的row和col相等，这里用col为了整体逻辑更加清晰
		memcpy(res[i], src[i], sizeof(res[0][0]) * col);
	}
	//将增广矩阵右侧变为单位阵
	for (i = 0; i < row; i++)
	{
		for (j = col; j < n; j++)
		{
			if (i == (j - row))
				res[i][j] = 1.0;
		}
	}
	
	for (j = 0; j < col; j++)
	{
       //step 4
	   //整理增广矩阵，选主元
		principal = j;
        Max = fabs(res[principal][j]); // 用绝对值比较
        // 默认第一行的数最大
        // 主元只选主对角线下方
        for (i = j; i < row; i++)
        {
            if (fabs(res[i][j]) > Max)
            {
                principal = i;
                Max = fabs(res[i][j]);
            }
        }
        if (j != principal)
        {
            for (k = 0; k < n; k++)
            {
                tmp = res[principal][k];
                res[principal][k] = res[j][k];
                res[j][k] = tmp;
            }
        }
        //step 5
		//将每列其他元素化0
		for (i = 0; i < row; i++)
		{
			if (i == j || res[i][j] == 0)continue;
			float b = res[i][j] / res[j][j];
			for (k = 0; k < n; k++)
			{
				res[i][k] += b * res[j][k] * (-1);
			}
		}
		//阶梯处化成1
		float a = 1.0 / res[j][j];
		for (i = 0; i < n; i++)
		{
			res[j][i] *= a;
		}
	}
	//step 6
	//将逆矩阵部分拷贝到res2中
	for (i = 0; i < row; i++)
	{
		memcpy(res2[i], res[i] + row, sizeof(res[0][0]) * row);
	}
	//必须释放res内存！
	free_Matrix(res);
	return res2;
}

/**
  * @brief  製作一個n*n的方陣
  * @param  方陣的大小
  * @retval 製作好的方陣地址 
  */
float** MakeMat(int n)
{
	int i = 0;
	float** res = (float**)malloc(sizeof(float*) * n);

	for (i = 0; i < n; i++)
	{
		res[i] = (float*)malloc(sizeof(float) * n);
	}
	return res;
}
/**
   * @brief  將數據填入6*6和6*1矩陣
   * @param	 要填入數據的6*6矩陣地址
   * @param  要填入數據的6*1矩陣地址
   * @param  數據資料6,3陣列
   * @retval 無
   */ 
void Matrix_input(float** input1_6,float* input6_1,float input[6][3])
{
	uint8_t i;
	for(i=0;i<6;i++)
	{
		
		input1_6[i][0]=(input[i][1])*(input[i][1]);
		input1_6[i][1]=(input[i][2])*(input[i][2]);
		input1_6[i][2]=(input[i][0]);
		input1_6[i][3]=(input[i][1]);
		input1_6[i][4]=(input[i][2]);
		input1_6[i][5]=1;
		input6_1[i]=-(input[i][0])*(input[i][0]);	
	
	}	

}
 /**
  * @brief  將矩陣相乘
  * @param  要相乘的左矩陣的地址
  * @param  要相乘的右矩陣的地址
  * @retval 輸出結果的矩陣的地址
  */
float** Matrix_Multiply(float ** Matrix_left,float ** Matrix_right)
{
    int i,j,k;
	float **Matrix_result=MakeMat(6);
	for (i = 0; i < 6; i++)
	{
		for (j = 0; j < 6; j++)
		{
            Matrix_result[i][j]=0.0;
			for (k = 0; k < 6; k++)
			{
				Matrix_result[i][j] += Matrix_left[i][k] * Matrix_right[k][j];
			}
		}
    }
	return Matrix_result;
}

/**
  * @brief  將傳入的6*6矩陣轉置
  * @param  要轉置的矩陣地址 
  * @retval 轉置好的矩陣地址
  */
float** Matrix_Transpose(float** input)
{
	int i,j;
	float** output=MakeMat(6);
	for(i=0;i<6;i++){
		for(j=0;j<6;j++){
			output[i][j]=input[j][i];		//转换
		}
	}

	return output;
}
/**
  * @brief  製作一個6*1的矩陣
  * @param  無 
  * @retval 製作好的矩陣的地址
  */
float* Make6_1Matrix(void)
{
	float* res = (float*)malloc(sizeof(float) * 6);
	return res;
}
/**
  * @brief  將傳入的6*1矩陣相乘並回傳相成的矩陣
  * @param  左矩陣地址 
  * @param  6*1的右矩陣
  * @retval 6*1的矩陣
  */
float* Matrix6_1_Multiply(float ** Matrix_left,float * Matrix_right)
{
    int i,k;
	float *Matrix_result=Make6_1Matrix();
	for (i = 0; i < 6; i++)
	{
            Matrix_result[i]=0.0;
			for (k = 0; k < 6; k++)
			{
				Matrix_result[i] += Matrix_left[i][k] * Matrix_right[k];
			}
    }
	return Matrix_result;
}

