#include "matrix_comp.h"

float determinant_2x2(float arr[2][2])
{
        return arr[0][0] * arr[1][1] - arr[0][1] * arr[1][0]; 
}

float determinant_3x3(float arr[3][3])
{
        float cofactor_0 = arr[1][1] * arr[2][2] - arr[1][2] * arr[2][1];
        float cofactor_1 = arr[1][0] * arr[2][2] - arr[1][2] * arr[2][0];
        float cofactor_2 = arr[1][0] * arr[2][1] - arr[1][1] * arr[2][0];
        return arr[0][0] * cofactor_0 - arr[0][1] * cofactor_1 + arr[0][2] * cofactor_2;
}

void scalar_multiplication_3x3(float res[3][3], float arr[3][3], float k)
{
        for (int i = 0; i < 3; i++)
        {
                for(int j = 0; j < 3; j++)
                {
                        res[i][j] = k * arr[i][j];
                }
        }
}

void matrix_vector_multiplication_3x3(float res[3], float arr[3][3], float vec[3])
{
        for(int i = 0; i < 3; i++)
        {
                res[i] = 0;
                for(int j = 0; j < 3; j++)
                {
                        res[i] += arr[i][j] * vec[j];
                }
        }
}

