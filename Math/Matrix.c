#include <stdlib.h>
#include <math.h>
#include <string.h>
void free_Matrix(float** src)
{
    int i;
    for(i = 0;i < 6; i++)
    {
        free(src[i]);
    }
    free(src);
}

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
 
void Matrix_input(float** input1_6,float* input6_1,float input[6][3])
{
	int i;
	for(i=0;i<7;i++)
	{
		input1_6[i][0]=input[i][1]*input[i][1];
		input1_6[i][1]=input[i][2]*input[i][2];
		input1_6[i][2]=input[i][0];
		input1_6[i][3]=input[i][1];
		input1_6[i][4]=input[i][2];
		input1_6[i][5]=1;
		input6_1[i]=-input[i][0]*input[i][0];	
	
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
float* Make6_1Matrix(void)
{
	float* res = (float*)malloc(sizeof(float) * 6);
	return res;
}

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

