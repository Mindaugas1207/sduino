//=============================================================================================
// SensorFusion.h
//=============================================================================================
//
// Madgwick's implementation of Mahony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
// 23/11/2017   Aster			Optimised time handling and melted in one library
//
//=============================================================================================
#ifndef SensorFusion_h
#define SensorFusion_h

#include <math.h>
#include <tuple>
//--------------------------------------------------------------------------------------------
// Variable declaration

class SF {
//-------------------------------------------------------------------------------------------
// Function declarations

public:

	void init();
	
	void MahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltat);
	void MahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float deltat);
	
	void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltat);
    void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float deltat);
	
	void MadgwickUpdate_kriswiner(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

	// find initial Quaternios
	// it is good practice to provide mean values from multiple measurements
    bool initQuat(float ax, float ay, float az, float mx, float my, float mz);

	std::tuple<float, float, float> computeAngles();
	std::tuple<float, float, float, float> getQuaternion();

private:
	float beta;				//Madgwick: 2 * proportional gain
	float twoKp;			//Mahony: 2 * proportional gain (Kp)
	float twoKi;			//Mahony: 2 * integral gain (Ki)
	float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
	float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
	static float invSqrt(float x);
	void vectorCross(float A[3], float B[3], float cross[3]);
};

#endif
