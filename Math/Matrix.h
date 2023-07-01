#ifndef _MATH_H
#define _MATH_H

void free_Matrix(float** src);
float** Matrix_inverse(float** src);
float** MakeMat(int n);
void Matrix_input(float** input1_6,float* input6_1,float input[6][3]);
float** Matrix_Multiply(float ** Matrix_left,float ** Matrix_right);
float** Matrix_Transpose(float** input);
float* Make6_1Matrix(void);
float* Matrix6_1_Multiply(float ** Matrix_left,float * Matrix_right);

#endif
