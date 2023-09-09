//=====================================================================================================
// ahrs.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef AHRS_H
#define AHRS_H

//---------------------------------------------------------------------------------------------------
// Function declarations

void ahrs_init(float sampleFreqDef, float betaDef);
void ahrs_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void ahrs_update_imu(float gx, float gy, float gz, float ax, float ay, float az);
void ahrs_get_euler_in_degrees(float *heading, float *pitch, float *roll);

#endif // AHRS_H
//=====================================================================================================
// End of file
//=====================================================================================================
