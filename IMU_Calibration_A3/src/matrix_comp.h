/*
        calculates the determinant of a 2x2 matrix 
*/
float determinant_2x2(float arr[2][2]);

/*
        calculates the determinant of a 3x3 matrix
*/
float determinant_3x3(float arr[3][3]);

/*
        multiply the matrix by some scalar k
*/
void scalar_multiplication_3x3(float res[3][3], float arr[3][3], float k); 

/*
        multiply a 3x3 matrix by a 3x1 vector
*/
void matrix_vector_multiplication_3x3(float res[3], float arr[3][3], float vec[3]);