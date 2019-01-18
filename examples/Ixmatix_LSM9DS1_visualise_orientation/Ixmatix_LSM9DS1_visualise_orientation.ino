/*
    LSM9DS1 Orientation Visualiser

    Use the OrientationVisualiser code from arduino
    https://github.com/arduino-libraries/MadgwickAHRS/blob/master/extras/Visualizer/Visualizer.pde
    with Processing 3. (https://processing.org/)
*/

#include <Ixmatix_LSM9DS1.h>

// Visualization Message Interval
const uint32_t INTERVAL_PRINT = (uint32_t) 1.f/30.f*1000; // 30Hz
unsigned long lastRefreshTime = 0;


Ixmatix_LSM9DS1 imu;

/*
 * Algorithm filter variables
 */
// deltat variables
int32_t lastUpdate = 0; 
uint32_t now = 0;


/*
 * Calibration variables
 */

// xl = accelerometer ----- g  = gyroscope
float xl_offsets[3] = { +0.04f, -0.10f, -0.01f  };
float g_offsets[3]  = { -0.05f, +0.35f, -0.73f  };

// Magnetic calibration data - obtaied from Motion Sensor Calibration Tool
// link - https://www.pjrc.com/store/prop_shield.html
// Magnetic offset
float mag_offsets[3]            = { -.00563f, .01678f, -.0068f };

// Magnetic Mapping
// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { +0.990, -0.002, -0.005 },
                                    { -0.002, +0.989, -0.075 },
                                    { -0.005, -0.075, +1.027 }
                                  };


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

    // Compensating for Hard-Iron Effects
    float mx_temp = (float)sensorValues[0] * mRes - mag_offsets[0];  // offset or bias is subtracted to get calibrated data
    float my_temp = (float)sensorValues[1] * mRes - mag_offsets[1];
    float mz_temp = (float)sensorValues[2] * mRes - mag_offsets[2];
    
    // Compensating for Soft-Iron Effects
    mx = mx_temp * mag_softiron_matrix[0][0] + my_temp * mag_softiron_matrix[0][1] + mz_temp * mag_softiron_matrix[0][2];
    my = mx_temp * mag_softiron_matrix[1][0] + my_temp * mag_softiron_matrix[1][1] + mz_temp * mag_softiron_matrix[1][2];
    mz = mx_temp * mag_softiron_matrix[2][0] + my_temp * mag_softiron_matrix[2][1] + mz_temp * mag_softiron_matrix[2][2];
  }
  
  // Get dt expressed in sec for madgwick/mahony algorithms
  now = micros();
  imu.deltat = ((now - lastUpdate) / 1000000.0f);
  lastUpdate = now;

  // Algorithm filter
  // mx is sent as the negative of mx due to axis coordinate difference
  // check datasheet page 10
  // y-axis is sent as the negative of y-axis to correct quaternion orientation
  // Madgwick and Mahony algorithms require rad/seg for gyroscope values
  // 1 rad/seg = 1 deg * (PI/ 180)
  //                          (                 |                   |                   |   |    |   |    |    |   )
  imu.updateMadgwickQuaternion(gx * PI / 180.0f , -gy * PI / 180.0f , gz * PI / 180.0f  , ax, -ay, az, -mx, -my, mz);
  //imu.updateMahonyQuaternion(  gx*PI/180.0f     , -gy*PI/180.0f     , gz*PI/180.0f      , ax, -ay, az, -mx, -my, mz);
  
  // is it time to send data to OrientationVisualiser?
  if (millis() - lastRefreshTime >= INTERVAL_PRINT) {
    lastRefreshTime += INTERVAL_PRINT;

    float heading   = 
      atan2(2.0f * (imu.q[1] * imu.q[2] + imu.q[0] * imu.q[3]), imu.q[0] * imu.q[0] + imu.q[1] * imu.q[1] - imu.q[2] * imu.q[2] - imu.q[3] * imu.q[3]);   
    float pitch     = 
      -asin(2.0f * (imu.q[1] * imu.q[3] - imu.q[0] * imu.q[2]));
    float roll  = 
      atan2(2.0f * (imu.q[0] * imu.q[1] + imu.q[2] * imu.q[3]), imu.q[0] * imu.q[0] - imu.q[1] * imu.q[1] - imu.q[2] * imu.q[2] + imu.q[3] * imu.q[3]);
    pitch *= 180.0f / PI;
    heading   *= 180.0f / PI; 
    // heading   -= 5.0f; // magnetic inclination
    roll  *= 180.0f / PI;
  
    Serial.print("Orientation: ");
    Serial.print(-heading);
    Serial.print(" ");
    Serial.print(-pitch);
    Serial.print(" ");
    Serial.println(-roll);
  
  }
}
