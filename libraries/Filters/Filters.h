/*
KalmanFilter.h KalmanFilter.cpp - Library for BST-Balance car code.
Created by SKY ZHU&ROOMS LUO, OCTOBER 2, 2016.
JUST FOR THE Company of Technology of yahboom.
In order to  avoid Infringement Act,this core is not for the commerce except being authorized by the writer.
*/

#ifndef Filters_h
#define Filters_h

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

constexpr float raw_to_gyro(int16_t raw){
	return - raw / 131;
}

constexpr float raw_to_accel(int16_t raw){
	return raw / 16.4;
}

/////////////////// Band Limited Filter //////////////////////
template <typename T>
T band_limited_filter(T const &current, T const &measurment, T const &add_gain){
  return current * add_gain + measurment * (1-add_gain); 
}

class Filters
{
public:
	void complimentary_filter(float angle_m, float gyro_m,float dt,float const K1);
	void kalman_filter(double angle_m, double gyro_m,float dt,float const Q_angle,float const Q_gyro,float const R_angle,float const C_0);
	void angle_test(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float const Q_angle,float const Q_gyro,
									float const R_angle,float const C_0,float const K1);
  float gyro_x,gyro_y,gyro_z;
  float accel_z = 0;
  float angle;
  float angle6;
private:
	float angle_err,q_bias;
	float Pdot[4] = { 0, 0, 0, 0};
	float P[2][2] = {{ 1, 0 }, { 0, 1 }};
	float  PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
	float angle_dot;                               
	
};
#endif
//
// END OF FILE
//