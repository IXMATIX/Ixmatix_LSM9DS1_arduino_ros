/*
    LSM9DS1 accelerometer and gyroscope calibration

    Keep your sensor static and start to record data
    to get offsets for accelerometer and gyroscope.
    Once done, copy and paste those offsets into your
    code.  See "LSM9DS1 sensor communication with ROS" example.
    
    Accel-Gyro calibration Highly Based! on Kris Winer's 
    Code. More info: https://github.com/kriswiner/LSM9DS1    
*/

#include <Ixmatix_LSM9DS1.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

Ixmatix_LSM9DS1 imu;

/*
 * Calibration variables
 */
float xl_offsets[3] = { 0.f, 0.f, 0. };
float g_offsets[3]  = { 0.f, 0.f, 0. };

/*
 * LSM9DS1 variables
 */
float aRes;
float gRes;
float mRes;

float ax, ay, az;
float gx, gy, gz;
float mx, my, mz;

uint8_t statusXL_G;
uint8_t statusM;
uint8_t newXLData;
uint8_t newGData;
uint8_t newMData;

int16_t sensorValues[3];


void setup() {
  Serial.begin(115200);
  Serial.println("Waiting for LSM9DS1");
  
  // Wait for sensor
  Wire.begin();
  while (!imu.isLSM9DS1Ready());
  Serial.println("LSM9DS1: Ready...");
  Serial.println("LSM9DS1: Setting Gyro-Accel Ctrl Registers");

  // sensor is ready
  imu.setConfig();
  imu.initLSM9DS1();                // Setting sensor registers to get data
  imu.getRes(&aRes, &gRes, &mRes);  // Get resolution to convert raw data to real data
  Serial.print(aRes);
  Serial.println(" accelerometer resolution");
  Serial.print(gRes);
  Serial.println(" gyroscope resolution");

  accelgyrocalLSM9DS1(xl_offsets, g_offsets);
  
  Serial.println("");
  Serial.print("float xl_offsets[3] = { ");
  Serial.print(xl_offsets[0]);
  Serial.print(", ");
  Serial.print(xl_offsets[1]);
  Serial.print(", ");
  Serial.print(xl_offsets[2]);
  Serial.print(" };");
  Serial.println("");
  Serial.print("float g_offsets[3] = { ");
  Serial.print(g_offsets[0]);
  Serial.print(", ");
  Serial.print(g_offsets[1]);
  Serial.print(", ");
  Serial.print(g_offsets[2]);
  Serial.print(" };");
  Serial.println("");
}


/*
 * Extracted from Kriswiner code
 */

void accelgyrocalLSM9DS1(float * dest1, float * dest2)
{  
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  uint16_t samples, ii;

   // enable the 3-axes of the gyroscope
   imu.writeI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG4, 0x38);
   // configure the gyroscope
   imu.writeI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG1_G, imu.odr_g << 5 | imu.fs_g << 3 | imu.bw_g);
   delay(200);
   // enable the three axes of the accelerometer 
   imu.writeI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG5_XL, 0x38);
   // configure the accelerometer-specify bandwidth selection with Abw
   imu.writeI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG6_XL, imu.odr_xl << 5 | imu.fs_xl << 3 | 0x04 | imu.bw_xl);
   delay(200);
   // enable block data update, allow auto-increment during multiple byte read
   imu.writeI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG8, 0x44);
  
  // First get gyro bias
  byte c;
  imu.readI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG9, 1, &c);
  imu.writeI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG9, c | 0x02);     // Enable gyro FIFO  
  delay(50);                                                       // Wait for change to take effect
  imu.writeI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

  uint8_t samples_;
  imu.readI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_FIFO_SRC, 1, &samples_);
  samples = (uint16_t) samples_;
  samples &= 0x2F;
  
  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
    int16_t gyro_temp[3] = {0, 0, 0};
    imu.readsI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_OUT_X_L_G, 6, &data[0]);
    gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    gyro_bias[0] += (int32_t) gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    gyro_bias[1] += (int32_t) gyro_temp[1]; 
    gyro_bias[2] += (int32_t) gyro_temp[2]; 
  }  

  gyro_bias[0] /= samples; // average the data
  gyro_bias[1] /= samples; 
  gyro_bias[2] /= samples; 
  
  dest1[0] = (float)gyro_bias[0]*gRes;  // Properly scale the data to get deg/s
  dest1[1] = (float)gyro_bias[1]*gRes;
  dest1[2] = (float)gyro_bias[2]*gRes;

  imu.readI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG9, 1, &c);
  imu.writeI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG9, c & ~0x02);   //Disable gyro FIFO  
  delay(50);
  imu.writeI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_FIFO_CTRL, 0x00);  // Enable gyro bypass mode
 
  // now get the accelerometer bias
  imu.readI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG9, 1, &c);
  imu.writeI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG9, c | 0x02);     // Enable accel FIFO  
  delay(50);                                                       // Wait for change to take effect
  imu.writeI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable accel FIFO stream mode and set watermark at 32 samples
  delay(1000);  // delay 1000 milliseconds to collect FIFO samples

  imu.readI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_FIFO_SRC, 1, &samples_);
  samples = (uint16_t) samples_;
  samples &= 0x2F;
  
  for(ii = 0; ii < samples ; ii++) {            // Read the accel data stored in the FIFO
    int16_t accel_temp[3] = {0, 0, 0};
    imu.readsI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_OUT_X_L_XL, 6, &data[0]);
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1]; 
    accel_bias[2] += (int32_t) accel_temp[2]; 
  }  

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples; 
  accel_bias[2] /= samples; 
  
  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) (1.0/aRes);}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) (1.0/aRes);}
  
  dest2[0] = (float)accel_bias[0]*aRes;  // Properly scale the data to get g
  dest2[1] = (float)accel_bias[1]*aRes;
  dest2[2] = (float)accel_bias[2]*aRes;
  
  imu.readI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG9, 1, &c);
  imu.writeI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_CTRL_REG9, c & ~0x02);   //Disable accel FIFO  
  delay(50);
  imu.writeI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_FIFO_CTRL, 0x00);  // Enable accel bypass mode
}


void loop() {
  // 
}
