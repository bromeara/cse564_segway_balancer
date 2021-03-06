/*
KalmanFilter.h KalmanFilter.cpp - Library for BST-Balance car code.
Created by SKY ZHU&ROOMS LUO, OCTOBER 2, 2016.
JUST FOR THE Company of Technology of yahboom.
In order to  avoid Infringement Act,this core is not for the commerce except
being authorized by the writer.
*/

#include "Filters.h"

/////////////////// Complimentary Filter///////////////
void Filters::complimentary_filter(float angle_m, float gyro_m, float dt,
                                        float const K1) {
  // This is a complementary filter
  angle6 = K1 * angle_m + (1 - K1) * (angle6 + gyro_m * dt);
  // return angle6;
}

////////////////////////Kalman Filter/////////////////////////

void Filters::kalman_filter(double angle_m, double gyro_m, float dt,
                                 float const Q_angle, float const Q_gyro,
                                 float const R_angle, float const C_0) {
  angle += (gyro_m - q_bias) * dt;  // equation 5
  angle_err = angle_m - angle;      // setup for angle portion of equation 5
  // update the probabilities
  Pdot[0] = Q_angle - P[0][1] - P[1][0];  
  Pdot[1] = -P[1][1];
  Pdot[2] = -P[1][1];
  Pdot[3] = Q_gyro;
  P[0][0] += Pdot[0] * dt;
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  PCt_0 = C_0 * P[0][0];  // H is just an identity matrix so  use the variance
  PCt_1 = C_0 * P[1][0];
  E = R_angle + C_0 * PCt_0;  // The inverse term of equation 7
  K_0 = PCt_0 / E;            // equation 7
  K_1 = PCt_1 / E;
  t_0 = PCt_0;
  t_1 = C_0 * P[0][1];
  P[0][0] -= K_0 * t_0;  // Update the probability
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  angle += K_0 * angle_err;     // This is the state estimation x_hat k|k
  q_bias += K_1 * angle_err;    // This is the covariance estimation P k|k
  angle_dot = gyro_m - q_bias;  // This value is never used
}

///////////////////////////// Angle Test/////////////////////////////////
void Filters::angle_test(int16_t ax, int16_t ay, int16_t az, int16_t gx,
                             int16_t gy, int16_t gz, float dt,
                             float const Q_angle, float const Q_gyro,
                             float const R_angle, float const C_0,
                             float const K1) {
  float Angle = atan2(ay, az) * 180 /
                PI;  // Angle measured between current position and upright.
  gyro_x = raw_to_gyro(-gx + 128.1);  // Rotation around the x axis. (Offset for maybe
                                // an error rate or margin of some kind?)
  kalman_filter(Angle, gyro_x, dt, Q_angle, Q_gyro, R_angle, C_0);
  if (gz > 32768)
    gz -= 65536;  // If the value is outside the 16bit range loop it around to
                  // the negative2g  1g
  gyro_z = raw_to_gyro(gz);
  accel_z = raw_to_accel(az);

  float angleAx = atan2(ax, az) * 180 /
                  PI;     // Angle of orientation or spin around the z axis
  gyro_y = raw_to_gyro(gy);  // Convert from raw IMU encoded data to float
                          // gyroscope data in deg/s
  complimentary_filter(angleAx, gyro_y, dt, K1);  // A heavy Copy
}
