/*
    LSM9DS1 magnetometer calibration
    
    Use Motion Sensor Calibration Tool to calibrate magnetometer
    link - https://www.pjrc.com/store/prop_shield.html
    
    Write your magnetic offset value divided by 1000
    and keep magnetic mapping  with no changes in your
    code. See "LSM9DS1 sensor communication with ROS" example.

    Accelerometer and gyroscope need to be calibrated to get
    accurate data for magnetometer calibration
*/

#define LSM9DS1_DEBUG

#include <Ixmatix_LSM9DS1.h>

Ixmatix_LSM9DS1 imu;

/*
 * Calibration variables
 */
// xl = accelerometer ----- g  = gyroscope
float xl_offsets[3] = { +0.04f, -0.10f, -0.01f  };
float g_offsets[3]  = { -0.05f, +0.35f, -0.73f  };

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

  // sensor is ready
  Serial.println("LSM9DS1: Ready...");
  Serial.println("LSM9DS1: Setting Gyro-Accel Ctrl Registers");

  
  imu.setConfig();
  imu.initLSM9DS1();                // Setting sensor registers to get data
  imu.getRes(&aRes, &gRes, &mRes);  // Get resolution to convert raw data to real data
  Serial.print(aRes);
  Serial.println(" accelerometer resolution");
  Serial.print(gRes);
  Serial.println(" gyroscope resolution");
  Serial.print(mRes);
  Serial.println(" magnetometer resolution");
  Serial.println("");
}


void loop() {
  // Read accelerometer and gyroscope register status from sensor
  imu.readI2C(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_STATUS_REG, 1, &statusXL_G);
  // Get a boolean that indicates if there's new data available from status register
  newXLData = statusXL_G & 0x01;
  newGData  = statusXL_G & 0x02;
  // Read magnetometer register status from sensor
  imu.readI2C(LSM9DS1_ADDR_M, LSM9DS1_R_M_STATUS_REG_M, 1, &statusM);
  // Get a boolean that indicates if there's new data available from status register
  newMData  = statusM & 0x08;

  // is accelerometer data available?
  if (newXLData) {
    imu.readSensorData(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_OUT_X_L_XL, sensorValues); // get sensor values

    // The data that is returned from the sensor is
    // raw data obtained by the ADC (Analog to Digital Converter) as LSB units
    // we need to multiply this value to its resolution converter
    // to obtain real data. Check page 12 from datasheet
    ax = (float)sensorValues[0] * aRes - xl_offsets[0]; // offset or bias is subtracted to get calibrated data
    ay = (float)sensorValues[1] * aRes - xl_offsets[1];
    az = (float)sensorValues[2] * aRes - xl_offsets[2];
  }
  
  // is gyoscope data available?
  if (newGData) {
    imu.readSensorData(LSM9DS1_ADDR_XG, LSM9DS1_R_XG_OUT_X_L_G, sensorValues); // get sensor values

    gx = (float)sensorValues[0] * gRes - g_offsets[0];  // offset or bias is subtracted to get calibrated data
    gy = (float)sensorValues[1] * gRes - g_offsets[1];
    gz = (float)sensorValues[2] * gRes - g_offsets[2];
  }
  
  // is accelerometer data available?
  if (newMData) {
    imu.readSensorData(LSM9DS1_ADDR_M, LSM9DS1_R_M_OUT_X_L_M, sensorValues); // get sensor values

    mx = (float)sensorValues[0] * mRes; // magnetometer data not calibrated
    my = (float)sensorValues[1] * mRes;
    mz = (float)sensorValues[2] * mRes;
  }
  

  // Motion Sensor Calibration Tool doesn't allow float data
  //   NOTE:  Raw data not used, calibration tool doesnt generate
  //          a perfect sphere. Real data x1000 used instead.
  Serial.print("Raw:");
  Serial.print((int) (ax*1000));
  Serial.print(',');
  Serial.print((int) (ay*1000));
  Serial.print(',');
  Serial.print((int) (az*1000));
  Serial.print(',');
  Serial.print((int) (gx*1000));
  Serial.print(',');
  Serial.print((int) (gy*1000));
  Serial.print(',');
  Serial.print((int) (gz*1000));
  Serial.print(',');
  Serial.print((int) (mx*1000));
  Serial.print(',');
  Serial.print((int) (my*1000));
  Serial.print(',');
  Serial.print((int) (mz*1000));
  Serial.println();
  
  delay(300);
}
