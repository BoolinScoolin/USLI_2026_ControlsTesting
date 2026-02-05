#include "imu_calibration.h"
#include "matrix_comp.h"
#include "main.h"
#include <Arduino.h>

#define TRUE_G_MPS2 9.80665

String printAxis[6] = {"+X", "-X", "+Y", "-Y", "+Z", "-Z"};


TUMBLE_Data tumble_data_init(int align_T, int calib_T)
{
        TUMBLE_Data tumbl_d;
        tumbl_d.alignmentTimeout = align_T;
        tumbl_d.calibrationPeriod = calib_T;

        // initialize all vectors/matrices to 0 
        for (int i = 0; i < 3; i++)
        {
                tumbl_d.axisOffset[i] = 0;
                tumbl_d.estimatedTrueAcceleration[i] = 0;
                for(int j = 0; j < 6; j++)
                {
                        tumbl_d.measuredAcceleration[i][j] = 0;
                        if(j < 3)
                        {
                                tumbl_d.gainMatrix[i][j] = 0;
                                tumbl_d.invGainMatrix[i][j] = 1;
                        }
                }
        }
        return tumbl_d;
}

void invert_gain_matrix(TUMBLE_Data* tumbl_d)
{
        //copy gain matrix to reduce length of scalar equations 
        float K[3][3]; 
        for(int i = 0; i < 3; i++)
        {
                for(int j = 0; j < 3; j++)
                {
                        K[i][j] = tumbl_d->gainMatrix[i][j];
                }
        }
        float adjointMatrix[3][3];
 
        // can defintely make this an iterative algorithm with for loops
        adjointMatrix[0][0] =   K[1][1] * K[2][2] - K[1][2] * K[2][1];
        adjointMatrix[0][1] = -(K[0][1] * K[2][2] - K[0][2] * K[2][1]);
        adjointMatrix[0][2] =   K[0][1] * K[1][2] - K[0][2] * K[1][1];
        adjointMatrix[1][0] = -(K[1][0] * K[2][2] - K[1][2] * K[2][0]);
        adjointMatrix[1][1] =   K[0][0] * K[2][2] - K[0][2] * K[2][0];
        adjointMatrix[1][2] = -(K[0][0] * K[1][2] - K[1][0] * K[0][2]);
        adjointMatrix[2][0] =   K[1][0] * K[2][1] - K[1][1] * K[2][0];
        adjointMatrix[2][1] = -(K[0][0] * K[2][1] - K[0][1] * K[2][0]);
        adjointMatrix[2][2] =   K[0][0] * K[1][1] - K[0][1] * K[1][0];

        // compute the determinant of the 3x3 matrix
        float det = determinant_3x3(K);
        Serial.print("Determinant: ");
        Serial.println(det);
        if (det < 1e-6)
        {
                Serial.println("Matrix is signular (i.e., determinant is zero and inverse does not exist).\n");
        }
        // if matrix is invertible, assign the appropriate values 
        scalar_multiplication_3x3(tumbl_d->invGainMatrix, adjointMatrix, 1/det);
}


void run_6pt_tumble_calibration(IMU_Measurements* imu_data, TUMBLE_Data* tumbl_d)
{
        for (int i = 0; i < 6; i++)
        {
                Serial.print("Align the IMU's ");
                Serial.print(printAxis[i]);
                Serial.print(" axis with the direction of gravity within the next ");
                Serial.print(tumbl_d->alignmentTimeout);
                Serial.println(" seconds.");

                unsigned long align_T_ms = (unsigned long)tumbl_d->alignmentTimeout * 1000;
                int counter = tumbl_d->alignmentTimeout;
                unsigned long startTime = millis();
                while(millis() - startTime < align_T_ms)
                {
                        Serial.print(counter);
                        Serial.println(" seconds left.");
                        counter--;
                        delay(1000);
                }       
                Serial.println("IMU should be aligned. If not, restart the procedure.");
                Serial.print("Pre-Calibration Reading: ");
                Serial.print(imu_data->accelX);
                Serial.print(" ");
                Serial.print(imu_data->accelY);
                Serial.print(" ");
                Serial.println(imu_data->accelZ);   
                Serial.println("Starting acceleration averages in current position. Do not move the IMU.");

                run_1pt_tumble_calibration(imu_data, tumbl_d, i);
        }
        Serial.println("Calibration procedures are complete. Now doing final calculations.");
        calculate_offsets(tumbl_d);
        calculate_gains(tumbl_d);
        Serial.println("Final calculations are complete. Now printing results");
        print_tumble_outputs(tumbl_d);
}

void run_1pt_tumble_calibration(IMU_Measurements* imu_data, TUMBLE_Data* tumbl_d, int iteration)
{
        // run an average over all acceleration values collected during the calibration period
        float accelXSum = 0;
        float accelYSum = 0;
        float accelZSum = 0;
        uint64_t count = 0; 
        // perform delay calculation
        unsigned long calib_T_ms = (unsigned long)(tumbl_d->calibrationPeriod * 1000);
        Serial.println(calib_T_ms);
        unsigned long startTime = millis();
        while(millis() - startTime < calib_T_ms)
        {
                // check if drdy changes from hi to lo; i.e., if previous drdy was high and now it is low
                
                /*
                bool prev_drdy = false; 
                bool new_imu_reading = false; 
                if(prev_drdy && !digitalRead(drdy))
                {
                        new_imu_reading = true;
                }*/
                //Serial.println("Reading IMU");
                // simplest if new_imu_reading is a global variable assigned after reading the IMU data in the ISR                
                if(imu_reading_new)
                {
                        imu_reading_new = false;
                        accelXSum += imu_data->accelX;
                        accelYSum += imu_data->accelY;
                        accelZSum += imu_data->accelZ;
                        count++;
                }
        }
        tumbl_d->measuredAcceleration[0][iteration] = accelXSum / count;
        tumbl_d->measuredAcceleration[1][iteration] = accelYSum / count;
        tumbl_d->measuredAcceleration[2][iteration] = accelZSum / count;
        Serial.println("Measurements along this axis are complete.");
        Serial.print("Average reading: ");
        Serial.print(tumbl_d -> measuredAcceleration[0][iteration]);
        Serial.print(" ");
        Serial.print(tumbl_d -> measuredAcceleration[1][iteration]);
        Serial.print(" ");
        Serial.print(tumbl_d -> measuredAcceleration[2][iteration]);
        Serial.print("   ");
        Serial.print("The sample count was: "); 
        Serial.println(count);
        Serial.println("Starting next alignment process.");

}

void calculate_offsets(TUMBLE_Data* tumbl_d)
{
        // average all measured accelerations along the axis during calibration to get the offset along that axis
        for(int i = 0; i < 3; i++)
        {
                for (int j = 0; j < 6; j++)
                {
                        tumbl_d->axisOffset[i] += tumbl_d->measuredAcceleration[i][j];
                }
                tumbl_d->axisOffset[i] /= 6;
        }
}

void calculate_gains(TUMBLE_Data* tumbl_d)
{
        // average gain calculations for axis and cross-axis gains 
        // using axis offsets and measured accelerations 
        for(int i = 0; i < 3; i++)
        {
                for (int j = 0; j < 3; j++)
                {
                        tumbl_d->gainMatrix[i][j] = (tumbl_d->measuredAcceleration[i][2*j]
                                                   - tumbl_d->axisOffset[i]
                                                   - tumbl_d->measuredAcceleration[i][2*j+1]
                                                   + tumbl_d->axisOffset[i])/(2*TRUE_G_MPS2);
                }
        }

        // Compute inverted gain matrix
        invert_gain_matrix(tumbl_d);
}

void print_tumble_outputs(TUMBLE_Data* tumbl_d)
{
        Serial.println("Axis offset vector:");
        for(int i = 0; i < 3; i++)
        {
                Serial.print(printAxis[2*i]);
                Serial.print("off: ");
                Serial.println(tumbl_d->axisOffset[i]);
        }
        Serial.print("Copy-form: ");
        for(int i = 0; i < 3; i++)
        {
                Serial.print(tumbl_d->axisOffset[i], 6);
                if (i != 3) {
                        Serial.print(",");
                }
        }

        Serial.println("\nGain matrix:");
        for(int i = 0; i < 3; i++)
        {
                for(int j = 0; j < 3; j++)
                {
                        Serial.print(tumbl_d->gainMatrix[i][j], 6);
                        Serial.print("  ");
                }
                Serial.print("\n");
        }
        Serial.print("Copy-form: ");
        for(int i = 0; i < 3; i++)
        {
                for(int j = 0; j < 3; j++)
                {
                        Serial.print(tumbl_d->gainMatrix[i][j], 6);
                        if (i != 3 && j != 3) {
                                Serial.print(",");
                        }
                }
        }

        Serial.println("\nInverted Gain matrix:");
        for(int i = 0; i < 3; i++)
        {
                for(int j = 0; j < 3; j++)
                {
                        Serial.print(tumbl_d->invGainMatrix[i][j], 6);
                        Serial.print("  ");
                }
                Serial.print("\n");
        }
        Serial.print("Copy-form: ");
        for(int i = 0; i < 3; i++)
        {
                for(int j = 0; j < 3; j++)
                {
                        Serial.print(tumbl_d->invGainMatrix[i][j], 6);
                        if (i != 3 && j != 3) {
                                Serial.print(",");
                        }
                }
        }
}

void write_tumble_outputs(TUMBLE_Data* tumbl_d, File& calibrationFile) {
        
        // Store calibration data
        calibrationFile.print("gainMatrix,");
        calibrationFile.print(tumbl_d->gainMatrix[0][0], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->gainMatrix[0][1], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->gainMatrix[0][2], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->gainMatrix[1][0], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->gainMatrix[1][1], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->gainMatrix[1][2], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->gainMatrix[2][0], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->gainMatrix[2][1], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->gainMatrix[2][2], 6);

        calibrationFile.print("\ninvGainMatrix,");
        calibrationFile.print(tumbl_d->invGainMatrix[0][0], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->invGainMatrix[0][1], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->invGainMatrix[0][2], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->invGainMatrix[1][0], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->invGainMatrix[1][1], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->invGainMatrix[1][2], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->invGainMatrix[2][0], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->invGainMatrix[2][1], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->invGainMatrix[2][2], 6);

        calibrationFile.print("\naxisOffset,");
        calibrationFile.print(tumbl_d->axisOffset[0], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->axisOffset[1], 6);
        calibrationFile.print(",");
        calibrationFile.print(tumbl_d->axisOffset[2], 6);

        calibrationFile.flush();
}

void estimate_true_accel(TUMBLE_Data* tumbl_d, IMU_Measurements* imu_data)
{
        float unbiasedAcceleration[3];
        unbiasedAcceleration[0] = imu_data->accelX - tumbl_d->axisOffset[0];
        unbiasedAcceleration[1] = imu_data->accelY - tumbl_d->axisOffset[1];
        unbiasedAcceleration[2] = imu_data->accelZ - tumbl_d->axisOffset[2];

        // assume inverted gain matrix has already been calculated
        matrix_vector_multiplication_3x3(tumbl_d->estimatedTrueAcceleration, tumbl_d->invGainMatrix, unbiasedAcceleration);
}