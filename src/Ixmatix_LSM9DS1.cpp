#include "Ixmatix_LSM9DS1.h"

bool Ixmatix_LSM9DS1::isLSM9DS1Ready() {
  uint8_t xg_response, m_response;
  
  readI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_WHO_AM_I, 1, &xg_response);
  readI2C(LSM9DS1_ADDR_M, LSM9DS1_R_M_WHO_AM_I, 1, &m_response);

  return (xg_response == LSM9DS1_RESP_XG_WHO_AM_I) &&
          (m_response == LSM9DS1_RESP_M_WHO_AM_I);
}

void Ixmatix_LSM9DS1::setConfig() {
  /*
   * Sensor configuration
   */
  // output data rate for gyroscope, accelerometer and magnetometer
  odr_g         = LSM9DS1_ODR_G_238_Hz;
  odr_xl        = LSM9DS1_ODR_XL_238_Hz;
  odr_m         = LSM9DS1_DO_M_10_Hz;
  // bandwidth for gyroscope and accelerometer
  bw_g          = LSM9DS_BW_G_01;
  bw_xl         = LSM9DS1_BW_XL_50_Hz;
  // Full scale for gyroscope, accelerometer and magnetometer
  fs_g          = LSM9DS1_FS_G_245DPS;
  fs_xl         = LSM9DS1_FS_XL_2G;
  fs_m          = LSM9DS1_FS_M_4_gauss;
  // Mode
  om            = LSM9DS1_OM_M_High;
  md            = LSM9DS1_MD_M_ContinuousConvesion;
  
  signX_G       = 0; // Pitch  gyroscope axis  - Positive sign
  signY_G       = 0; // Roll   gyroscope axis  - Positive sign
  signZ_G       = 0; // Yaw    gyroscope axis  - Positive sign

  bw_scal_odr   = 1; // 0: determined by ODR - 1: determined by BW_XL
  
  bdu_xlg       = 1; // 0: continuous update; 1: output registers not updated until MSB and LSB read)
  h_lactive     = 0; // 0: interrupt output pins active high; 1: interrupt output pins active low
  pp_od         = 0; // 0: push-pull mode; 1: open-drain mode
  if_add_inc    = 1; // 0: disabled; 1: enabled; // increment during a multiple byte access
  ble           = 0; // 0: daa LSB @ lower address; 1: MSB @ lower address
  sw_reset      = 0; // 0: normal mode; 1: reset device

  temp_comp     = 1; // temperature compensation
  omz           = LSM9DS1_OM_M_High;
  bdu_m         = 1; // Block data update for magnetic data
  /*
   * Algorithms configuration
   */
  q[0] = 1.0f;
  q[1] = q[2] = q[3] = 0.f;
  measErrorDeg  = 10.f;
  GyroMeasError = PI * (measErrorDeg / 180.0f);     // gyroscope measurement error in rads/s (start at 10 deg/s)
  GyroMeasDrift = PI * (0.0f  / 180.0f);    // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
  beta          = sqrt(3.0f / 4.0f) * GyroMeasError;
  zeta          = sqrt(3.0f / 4.0f) * GyroMeasDrift;

  Kp = (2.0f * 6.0f);           // 2 * proportional gain
  Ki = (2.0f * 0.0f);           // 2 * integral gain  
  eInt[0] = eInt[1] = eInt[2] = 0.f;

}

void Ixmatix_LSM9DS1::initLSM9DS1() {
  /*
   * CTRL_REG1_G
   */
  uint8_t ctrl_reg1_g = (odr_g << 5) | (fs_g << 3) | bw_g;
  setRegister(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG1_G, 
    ctrl_reg1_g, "CTRL_REG1_G");

  /*
   * ORIENT_CFG_G
   */
  // Angular rate sensor sign and orientation register
  uint8_t orient_cfg_g = (signX_G << 5) | (signY_G << 4) | (signZ_G << 3);
  setRegister(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_ORIENT_CFG_G, 
    orient_cfg_g, "ORIENT_CFG_G");

  /*
   * CTRL_REG4
   */
  // Giroscope 3 axis enabled, latched deactivated
  uint8_t ctrl_reg4 = 0b00111000;
  setRegister(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG4, 
      ctrl_reg4, "CTRL_REG4");

  /*
   * CTRL_REG5_XL
   */
  // Linear acceleration
  // No decimation of acceleration data and 3 axis activated
  uint8_t ctrl_reg5_xl = 0b00111000;
  setRegister(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG5_XL, 
      ctrl_reg5_xl, "CTRL_REG5_XL");

  /*
   * CTRL_REG6_XL
   */
  uint8_t ctrl_reg6_xl  = (odr_xl << 5) | (fs_xl << 3) | (bw_scal_odr << 2) | bw_xl;
  setRegister(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG6_XL, 
      ctrl_reg6_xl, "CTRL_REG6_XL");

  /*
   * CTRL_REG8
   */
  uint8_t ctrl_reg8 = bdu_xlg << 6 | h_lactive << 5 | pp_od << 4 | if_add_inc << 2 | ble << 1 | sw_reset;
  setRegister(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG8, 
    ctrl_reg8, "CTRL_REG8");
    
  /*
   * CTRL_REG1_M
   */
  uint8_t ctr_reg1_m = (temp_comp << 7) | (om << 5) | (odr_m << 2);
  setRegister(LSM9DS1_ADDR_M, LSM9DS1_R_M_CTRL_REG1_M, 
   ctr_reg1_m, "CTRL_REG1_M");
 
  /*
   * CTRL_REG2_M
   */
  uint8_t ctrl_reg2_m = (fs_m << 5);
  setRegister(LSM9DS1_ADDR_M, LSM9DS1_R_M_CTRL_REG2_M, 
   ctrl_reg2_m, "CTRL_REG2_M");

  /*
   * CTRL_REG3_M
   */
  uint8_t ctrl_reg3 = md;
  setRegister(LSM9DS1_ADDR_M, LSM9DS1_R_M_CTRL_REG3_M, 
   ctrl_reg3, "CTRL_REG3_M");

  /*
   * CTRL_REG4_M
   */

  uint8_t ctrl_reg4_m = (omz << 2);
  setRegister(LSM9DS1_ADDR_M, LSM9DS1_R_M_CTRL_REG4_M, 
   ctrl_reg4_m, "CTRL_REG4_M");

  /*
   * CTRL_REG5_M
   */

  uint8_t ctrl_reg5_m = (bdu_m << 6);
  setRegister(LSM9DS1_ADDR_M, LSM9DS1_R_M_CTRL_REG5_M, 
    ctrl_reg5_m, "CTRL_REG5_M");

  /*
   * Magnetic offsets
   */
  /*
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_L_M, (int16_t) mag_offsets[0]  & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_H_M, ((int16_t)mag_offsets[0] >> 8) & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_L_M, (int16_t) mag_offsets[1] & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_H_M, ((int16_t)mag_offsets[1] >> 8) & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_L_M, (int16_t) mag_offsets[2] & 0xFF);
  writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_H_M, ((int16_t)mag_offsets[2] >> 8) & 0xFF);
  */
}

void Ixmatix_LSM9DS1::setRegister(uint8_t addrs, uint8_t subaddrs, uint8_t ctrl_reg, char ctrl_name[]) {
  uint8_t ctrl_buff;
  char buff[30];
  writeI2C(addrs, subaddrs, ctrl_reg);
  #ifndef LSM9DS1_DEBUG
  readI2C(addrs, subaddrs, 1, &ctrl_buff);
  sprintf(buff, "\t%s\t", ctrl_name);
  Serial.print(buff);
  Serial.print(ctrl_buff, HEX);
  Serial.print("\t");
  Serial.println(ctrl_buff, BIN);
  #endif
}

void Ixmatix_LSM9DS1::readSensorData(uint8_t addrs, uint8_t subaddrs, int16_t * values) {
  uint8_t buff[6]; // Register Values
  readI2C(addrs, subaddrs, 6, &buff[0]);  // Read the six raw data registers into data array
  values[0] = ((int16_t)buff[1] << 8) | buff[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  values[1] = ((int16_t)buff[3] << 8) | buff[2] ;  
  values[2] = ((int16_t)buff[5] << 8) | buff[4] ; 
}


void Ixmatix_LSM9DS1::getRes(float *accel, float *gyro, float *mag) {
  switch (fs_xl) {
    case LSM9DS1_FS_XL_2G:
      (*accel) = LSM9DS1_LA_So_2g;
      break;
    case LSM9DS1_FS_XL_16G:
      (*accel) = LSM9DS1_LA_So_16g;
      break;
    case LSM9DS1_FS_XL_4G:
      (*accel) = LSM9DS1_LA_So_4g;
      break;
    case LSM9DS1_FS_XL_8G:
      (*accel) = LSM9DS1_LA_So_8g;
      break;
  }

  switch(fs_g){
    case LSM9DS1_FS_G_245DPS:
      (*gyro) = LSM9DS1_G_So_245_dps;
      break;
    case LSM9DS1_FS_G_500DPS:
      (*gyro) = LSM9DS1_G_So_245_dps;
      break;
    case LSM9DS1_FS_G_2000DPS:
      (*gyro) = LSM9DS1_G_So_245_dps;
      break;
  }

  switch(fs_m) {
    case LSM9DS1_FS_M_4_gauss:
      (*mag) = LSM9DS1_M_GN_4_gauss;
      break;
    case LSM9DS1_FS_M_8_gauss:
      (*mag) = LSM9DS1_M_GN_8_gauss;
      break;
    case LSM9DS1_FS_M_12_gauss:
      (*mag) = LSM9DS1_M_GN_12_gauss;
      break;
    case LSM9DS1_FS_M_16_gauss:
      (*mag) = LSM9DS1_M_GN_16_gauss;
      break;
  }

}

/*
 * I2C communication methods
 */

void Ixmatix_LSM9DS1::readI2C(uint8_t address, uint8_t subAddress, uint8_t size_buff, uint8_t * buff) {  
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  // It's assumed there'll be just one master
  Wire.endTransmission(false);
  Wire.requestFrom(address, (uint8_t) size_buff);
  for(uint8_t i=0; i<size_buff; i++)
    buff[i] = Wire.read();
}

void Ixmatix_LSM9DS1::readsI2C(uint8_t address, uint8_t subAddress, uint8_t size_buff, uint8_t * buff) {  
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.endTransmission(false);
  uint8_t i = 0;
  Wire.requestFrom(address, (size_t) size_buff);
  while (Wire.available()) {
        buff[i++] = Wire.read(); }
}

// LSM9DS1 doesn't need to write a stream of bytes
// so writeI2C sends only 1 byte
void Ixmatix_LSM9DS1::writeI2C(uint8_t address, uint8_t subAddress, uint8_t data) {
  Wire.beginTransmission(address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

/*
 * Filter algorithms
 */

//=====================================================================================================
// MadgwickAHRS
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date      Author          Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
// 19/02/2012 SOH Madgwick  Magnetometer measurement is normalised
//
//=====================================================================================================


//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update


void Ixmatix_LSM9DS1::updateMadgwickQuaternion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
  float recipNorm;
  float s0, s1, s2, s3;
  float qDot1, qDot2, qDot3, qDot4;
  float hx, hy;
  float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;


  // Convert gyroscope degrees/sec to radians/sec
  //gx *= 0.0174533f;
  //gy *= 0.0174533f;
  //gz *= 0.0174533f;

  // Rate of change of quaternion from gyroscope
  qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    _2q0mx = 2.0f * q0 * mx;
    _2q0my = 2.0f * q0 * my;
    _2q0mz = 2.0f * q0 * mz;
    _2q1mx = 2.0f * q1 * mx;
    _2q0 = 2.0f * q0;
    _2q1 = 2.0f * q1;
    _2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
    _2q0q2 = 2.0f * q0 * q2;
    _2q2q3 = 2.0f * q2 * q3;
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;

    // Reference direction of Earth's magnetic field
    hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    // Apply feedback step
    qDot1 -= beta * s0;
    qDot2 -= beta * s1;
    qDot3 -= beta * s2;
    qDot4 -= beta * s3;
  }

  // Integrate rate of change of quaternion to yield quaternion
  q0 += qDot1 * deltat;
  q1 += qDot2 * deltat;
  q2 += qDot3 * deltat;
  q3 += qDot4 * deltat;

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
  // anglesComputed = 0;

  q[0] = q0;
  q[1] = q1;
  q[2] = q2;
  q[3] = q3;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root


float Ixmatix_LSM9DS1::invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}



//=============================================================================================
// MahonyAHRS
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date      Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick  Optimised for reduced CPU load
//
// Algorithm paper:
// http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=4608934&url=http%3A%2F%2Fieeexplore.ieee.org%2Fstamp%2Fstamp.jsp%3Ftp%3D%26arnumber%3D4608934
//
//=============================================================================================

void Ixmatix_LSM9DS1::updateMahonyQuaternion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
  float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float pa, pb, pc;
  
  // Auxiliary variables to avoid repeated arithmetic
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q1q4 = q1 * q4;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q2q4 = q2 * q4;
  float q3q3 = q3 * q3;
  float q3q4 = q3 * q4;
  float q4q4 = q4 * q4;   
  
  // Normalise accelerometer measurement
  norm = sqrt(ax * ax + ay * ay + az * az);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  ax *= norm;
  ay *= norm;
  az *= norm;
  
  // Normalise magnetometer measurement
  norm = sqrt(mx * mx + my * my + mz * mz);
  if (norm == 0.0f) return; // handle NaN
  norm = 1.0f / norm;        // use reciprocal for division
  mx *= norm;
  my *= norm;
  mz *= norm;
  
  // Reference direction of Earth's magnetic field
  hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
  hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);
  
  // Estimated direction of gravity and magnetic field
  vx = 2.0f * (q2q4 - q1q3);
  vy = 2.0f * (q1q2 + q3q4);
  vz = q1q1 - q2q2 - q3q3 + q4q4;
  wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
  wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
  wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);  
  
  // Error is cross product between estimated direction and measured direction of gravity
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
  if (Ki > 0.0f)
  {
      eInt[0] += ex;      // accumulate integral error
      eInt[1] += ey;
      eInt[2] += ez;
  }
  else
  {
      eInt[0] = 0.0f;     // prevent integral wind up
      eInt[1] = 0.0f;
      eInt[2] = 0.0f;
  }
  
  // Apply feedback terms
  gx = gx + Kp * ex + Ki * eInt[0];
  gy = gy + Kp * ey + Ki * eInt[1];
  gz = gz + Kp * ez + Ki * eInt[2];
  
  // Integrate rate of change of quaternion
  pa = q2;
  pb = q3;
  pc = q4;
  q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
  q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
  q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
  q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);
  
  // Normalise quaternion
  norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
  norm = 1.0f / norm;
  q[0] = q1 * norm;
  q[1] = q2 * norm;
  q[2] = q3 * norm;
  q[3] = q4 * norm;

}

//====================================================================================================
// END OF CODE
//====================================================================================================
