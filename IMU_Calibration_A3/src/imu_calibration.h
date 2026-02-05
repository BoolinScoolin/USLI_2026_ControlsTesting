#include <String>
#include <stdint.h>
#include "Measurements.h"
#include "Sensors.h"
#include "SD.h"
//#include <assignment of drdy pin 

// tumble calibration from https://www.st.com/resource/en/design_tip/dm00253745-6point-tumble-sensor-calibration-stmicroelectronics.pdf

// stores the direction and magnitude of gravity in g's expected 
//float trueGravityVector[6][3]; 

// order of axis rotation for tumble calibration
enum axis {
        pos_X, 
        neg_X, 
        pos_Y, 
        neg_Y, 
        pos_Z, 
        neg_Z
};

// IMU data takes the form 

/*
        See the linked documentation to see how these arrays relate to the 3-axis IMU formula 
*/
struct TUMBLE_Data {
        int alignmentTimeout; // the 
        int calibrationPeriod; // the length of a tumble iteration in units of seconds
        float measuredAcceleration[3][6]; // stores the average acceleration recorded during the calibration period of each tumble position; each row corresponds to measurements along one axis
        float gainMatrix[3][3]; //stores the gains and cross gains calculated during the tumble calibration
        float invGainMatrix[3][3]; // stores the inverted gain matrix for computation
        float axisOffset[3]; // stores the offsets calculated along each axis during the tumble calibration
        float estimatedTrueAcceleration[3]; // stores the last estimate of true accleration
};

/*
        Given the alignment timeout period and calibration period, initialize and return the tumble data with these values and empty matrices  
*/
TUMBLE_Data tumble_data_init(int align_T, int calib_T);

/*
        runs a 6-pt tumble calibration that informs the user when to rotate the IMU
        along its 6 axes. Acts as helper function that periodically calls run_1pt_tumble_calibration 
*/
void run_6pt_tumble_calibration(IMU_Measurements* imu_data, TUMBLE_Data* tumbl_d);

/*
        Averages IMU readings together. Assumes the imu_data is updated in an interrupt callback. 
*/
void run_1pt_tumble_calibration(IMU_Measurements* imu_data, TUMBLE_Data* tumbl_d, int iteration);

/*
        calculate the offsets of the IMU from the tumble calibration data
*/
void calculate_offsets(TUMBLE_Data* tumbl_d);

/*
        calculate the gain matrix consisting of gains and cross-axis gains 
*/
void calculate_gains(TUMBLE_Data* tumbl_d);

/*
        prints outputs of the tumble data structure
*/
void print_tumble_outputs(TUMBLE_Data* tumbl_d);

/*
        writes outputs of the tumble data structure to a .txt on the SD card
*/
void write_tumble_outputs(TUMBLE_Data* tumbl_d, File& calibrationFile);


/*
        given some imu_data, update the estimated true accleration after computing it using matrix math 
*/
void estimate_true_accel(TUMBLE_Data* tumbl_d, IMU_Measurements* imu_data);

/*
        An unstable method of computing the inverse of the gain matrix using the adjoint-determinant method.
        Updates the inverse gain matrix in the TUMBLE_data structure
*/      
void invert_gain_matrix(TUMBLE_Data* tumbl_d); 