//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

#include <Arduino.h>

#include <vector.h>

#define float32vector TVector<float>

#define sampleFreq    119.0f            // sample frequency in Hz
#define twoKpDef    (2.0f * 0.5f)    // 2 * proportional gain
#define twoKiDef    (2.0f * 0.3f)    // 2 * integral gain

class MahonyAHRS
{
protected:
//----------------------------------------------------------------------------------------------------
// Variable declaration

    volatile float twoKp = twoKpDef;			// 2 * proportional gain (Kp)
    volatile float twoKi = twoKiDef;			// 2 * integral gain (Ki)
    volatile float q0 = 1;
    volatile float q1 = 0;
    volatile float q2 = 0;
    volatile float q3 = 0;
    
    volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    // integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations
public:
    
    float32vector Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    float32vector UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    float32vector UpdateGyro(float gx, float gy, float gz);
    float32vector CorrectAccel(float ax, float ay, float az);
    float32vector RPY(void);
    float32vector Reset(void) {q0 = 1; q1 = q2 = q3 = 0; return RPY();}
};
#endif
//=====================================================================================================
// End of file
//=====================================================================================================
