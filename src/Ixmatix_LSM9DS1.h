#ifndef __IxmatixLSM9DS1_R__H__
#define __IxmatixLSM9DS1_R__H__

#include "Arduino.h"
#include "Wire.h"

// Datasheet at https://www.st.com/resource/en/datasheet/DM00103319.pdf
// Accelerometer and gyroscope register address
#define LSM9DS1_R_XG_ACT_THS           0x04
#define LSM9DS1_R_XG_ACT_DUR           0x05
#define LSM9DS1_R_XG_INT_GEN_CFG_XL    0x06
#define LSM9DS1_R_XG_INT_GEN_THS_X_XL  0x07
#define LSM9DS1_R_XG_INT_GEN_THS_Y_XL  0x08
#define LSM9DS1_R_XG_INT_GEN_THS_Z_XL  0x09
#define LSM9DS1_R_XG_INT_GEN_DUR_XL    0x0A
#define LSM9DS1_R_XG_REFERENCE_G       0x0B
#define LSM9DS1_R_XG_INT1_CTRL         0x0C
#define LSM9DS1_R_XG_INT2_CTRL         0x0D
#define LSM9DS1_R_XG_WHO_AM_I          0x0F
#define LSM9DS1_R_XG_CTRL_REG1_G       0x10
#define LSM9DS1_R_XG_CTRL_REG2_G       0x11
#define LSM9DS1_R_XG_CTRL_REG3_G       0x12
#define LSM9DS1_R_XG_ORIENT_CFG_G      0x13
#define LSM9DS1_R_XG_INT_GEN_SRC_G     0x14
#define LSM9DS1_R_XG_OUT_TEMP_L        0x15
#define LSM9DS1_R_XG_OUT_TEMP_H        0x16
#define LSM9DS1_R_XG_STATUS_REG        0x17
#define LSM9DS1_R_XG_OUT_X_L_G         0x18
#define LSM9DS1_R_XG_OUT_X_H_G         0x19
#define LSM9DS1_R_XG_OUT_Y_L_G         0x1A
#define LSM9DS1_R_XG_OUT_Y_H_G         0x1B
#define LSM9DS1_R_XG_OUT_Z_L_G         0x1C
#define LSM9DS1_R_XG_OUT_Z_H_G         0x1D
#define LSM9DS1_R_XG_CTRL_REG4         0x1E
#define LSM9DS1_R_XG_CTRL_REG5_XL      0x1F
#define LSM9DS1_R_XG_CTRL_REG6_XL      0x20
#define LSM9DS1_R_XG_CTRL_REG7_XL      0x21
#define LSM9DS1_R_XG_CTRL_REG8         0x22
#define LSM9DS1_R_XG_CTRL_REG9         0x23
#define LSM9DS1_R_XG_CTRL_REG10        0x24
#define LSM9DS1_R_XG_INT_GEN_SRC_XL    0x26
// #define LSM9DS1_R_XG_STATUS_REG        0x27
#define LSM9DS1_R_XG_OUT_X_L_XL        0x28
#define LSM9DS1_R_XG_OUT_X_H_XL        0x29
#define LSM9DS1_R_XG_OUT_Y_L_XL        0x2A
#define LSM9DS1_R_XG_OUT_Y_H_XL        0x2B
#define LSM9DS1_R_XG_OUT_Z_L_XL        0x2C
#define LSM9DS1_R_XG_OUT_Z_H_XL        0x2D
#define LSM9DS1_R_XG_FIFO_CTRL         0x2E
#define LSM9DS1_R_XG_FIFO_SRC          0x2F
#define LSM9DS1_R_XG_INT_GEN_CFG_G     0x30
#define LSM9DS1_R_XG_INT_GEN_THS_XH_G  0x31
#define LSM9DS1_R_XG_INT_GEN_THS_XL_G  0x32
#define LSM9DS1_R_XG_INT_GEN_THS_YH_G  0x33
#define LSM9DS1_R_XG_INT_GEN_THS_YL_G  0x34
#define LSM9DS1_R_XG_INT_GEN_THS_ZH_G  0x35
#define LSM9DS1_R_XG_INT_GEN_THS_ZL_G  0x36
#define LSM9DS1_R_XG_INT_GEN_DUR_G     0x37
// Magnetic sensor register address
#define LSM9DS1_R_M_OFFSET_X_REG_L_M   0x05
#define LSM9DS1_R_M_OFFSET_X_REG_H_M   0x06
#define LSM9DS1_R_M_OFFSET_Y_REG_L_M   0x07
#define LSM9DS1_R_M_OFFSET_Y_REG_H_M   0x08
#define LSM9DS1_R_M_OFFSET_Z_REG_L_M   0x09
#define LSM9DS1_R_M_OFFSET_Z_REG_H_M   0x0A
#define LSM9DS1_R_M_WHO_AM_I           0x0F
#define LSM9DS1_R_M_CTRL_REG1_M        0x20
#define LSM9DS1_R_M_CTRL_REG2_M        0x21
#define LSM9DS1_R_M_CTRL_REG3_M        0x22
#define LSM9DS1_R_M_CTRL_REG4_M        0x23
#define LSM9DS1_R_M_CTRL_REG5_M        0x24
#define LSM9DS1_R_M_STATUS_REG_M       0x27
#define LSM9DS1_R_M_OUT_X_L_M          0x28
#define LSM9DS1_R_M_OUT_X_H_M          0x29
#define LSM9DS1_R_M_OUT_Y_L_M          0x2A
#define LSM9DS1_R_M_OUT_Y_H_M          0x2B
#define LSM9DS1_R_M_OUT_Z_L_M          0x2C
#define LSM9DS1_R_M_OUT_Z_H_M          0x2D
#define LSM9DS1_R_M_INT_CFG_M          0x30
#define LSM9DS1_R_M_INT_SRC_M          0x31
#define LSM9DS1_R_M_INT_THS_L_M        0x32
#define LSM9DS1_R_M_INT_THS_H_M        0x33

// The Slave Address for magnetometer and accel/gyro
// can be changed by setting the pin 5 (SDO_A/G for accel/gyro)
// or pin 6 (SDO_M for magnetometer) to High or Low

// This is useful if you have more I2C devices
// with the same address
#define LSM9DS1_SAD_0 1
#if LSM9DS1_SAD_0
  #define LSM9DS1_ADDR_XG 0x6B
  #define LSM9DS1_ADDR_M  0x1E
#else
  #define LSM9DS1_ADDR_XG 0x6A
  #define LSM9DS1_ADDR_M  0x1Cw
#endif

// Who am I Response
#define LSM9DS1_RESP_XG_WHO_AM_I  0x68 // XG        Who I am ID 01101000
#define LSM9DS1_RESP_M_WHO_AM_I   0x3D // Magnetic  Who I am ID 00111101

#define LSM9DS1_LA_So_2g  0.000061F
#define LSM9DS1_LA_So_4g  0.000122F
#define LSM9DS1_LA_So_8g  0.000244F
#define LSM9DS1_LA_So_16g 0.000732F

#define LSM9DS1_M_GN_4_gauss  0.00014F
#define LSM9DS1_M_GN_8_gauss  0.00029F
#define LSM9DS1_M_GN_12_gauss 0.00043F
#define LSM9DS1_M_GN_16_gauss 0.00058F

#define LSM9DS1_G_So_245_dps  .00875F
#define LSM9DS1_G_So_500_dps  .01750F
#define LSM9DS1_G_So_2000_dps .070F

// #define LSM9DS1_DEBUG

enum {
  LSM9DS1_ODR_G_PowerDown,
  LSM9DS1_ODR_G_14_9_Hz,
  LSM9DS1_ODR_G_59_5_Hz,
  LSM9DS1_ODR_G_119_Hz,
  LSM9DS1_ODR_G_238_Hz,
  LSM9DS1_ODR_G_476_Hz,
  LSM9DS1_ODR_G_952_Hz,
} ;

enum {
  LSM9DS1_FS_G_245DPS,
  LSM9DS1_FS_G_500DPS,
  LSM9DS1_FS_G_NOTAVBL,
  LSM9DS1_FS_G_2000DPS
};

enum {
  LSM9DS_BW_G_00,
  LSM9DS_BW_G_01,
  LSM9DS_BW_G_10,
  LSM9DS_BW_G_11
};

enum {
  LSM9DS1_ODR_XL_PowerDown,
  LSM9DS1_ODR_XL_10_Hz,
  LSM9DS1_ODR_XL_50_Hz,
  LSM9DS1_ODR_XL_119_Hz,
  LSM9DS1_ODR_XL_238_Hz,
  LSM9DS1_ODR_XL_476_Hz,
  LSM9DS1_ODR_XL_952_Hz
};

enum {
  LSM9DS1_FS_XL_2G,
  LSM9DS1_FS_XL_16G,
  LSM9DS1_FS_XL_4G,
  LSM9DS1_FS_XL_8G
};

enum {
  LSM9DS1_BW_XL_408_Hz,
  LSM9DS1_BW_XL_211_Hz,
  LSM9DS1_BW_XL_105_Hz,
  LSM9DS1_BW_XL_50_Hz
};

enum {
  LSM9DS1_OM_M_LowPower,
  LSM9DS1_OM_M_Medium,
  LSM9DS1_OM_M_High,
  LSM9DS1_OM_M_UltraHigh
};

enum {
  LSM9DS1_DO_M_0_625_Hz,
  LSM9DS1_DO_M_1_25_Hz,
  LSM9DS1_DO_M_2_5_Hz,
  LSM9DS1_DO_M_5_Hz,
  LSM9DS1_DO_M_10_Hz,
  LSM9DS1_DO_M_20_Hz,
  LSM9DS1_DO_M_40_Hz,
  LSM9DS1_DO_M_80_Hz
};

enum {
  LSM9DS1_FS_M_4_gauss,
  LSM9DS1_FS_M_8_gauss,
  LSM9DS1_FS_M_12_gauss,
  LSM9DS1_FS_M_16_gauss
};

enum {
  LSM9DS1_MD_M_ContinuousConvesion,
  LSM9DS1_MD_M_SingleConvertion,
  LSM9DS1_MD_M_PowerDown
};

class Ixmatix_LSM9DS1{
  public:
    // Sensor variables
    uint8_t odr_g;  // output data rate
    uint8_t odr_xl;
    uint8_t odr_m;

    uint8_t bw_g;   // bandwidth
    uint8_t bw_xl;

    uint8_t fs_g;   // Full scale
    uint8_t fs_xl;
    uint8_t fs_m;

    uint8_t om;     // Mode
    uint8_t md;

    bool signX_G;     // Pitch  gyroscope axis  - Positive sign
    bool signY_G;     // Roll   gyroscope axis  - Positive sign
    bool signZ_G;     // Yaw    gyroscope axis  - Positive sign
    bool bw_scal_odr; // 0: determined by ODR - 1: determined by BW_XL

    bool bdu_xlg;     // 0: continuous update; 1: output registers not updated until MSB and LSB read)
    bool h_lactive;   // 0: interrupt output pins active high; 1: interrupt output pins active low
    bool pp_od;       // 0: push-pull mode; 1: open-drain mode
    bool if_add_inc;  // 0: disabled; 1: enabled; // increment during a multiple byte access
    bool ble;         // 0: daa LSB @ lower address; 1: MSB @ lower address
    bool sw_reset;    // 0: normal mode; 1: reset device

    bool temp_comp;   // temperature compensation
    uint8_t omz;
    bool bdu_m;       // Block data update for magnetic data

    // Madgwick variables
    float q[4];
    float measErrorDeg;
    // Mahony variables
    float Kp;
    float Ki;
    float eInt[3];
    // filters variables
    float deltat;

    // Public methods
    Ixmatix_LSM9DS1() { setConfig(); };
    void setConfig();
    void initLSM9DS1();
    bool isLSM9DS1Ready();
    void setRegister(uint8_t addrs, uint8_t subaddrs, uint8_t ctrl_reg, char ctrl_name[]);
    void readSensorData(uint8_t addrs, uint8_t subaddrs, int16_t * values);
    void updateBeta(float measErrorDeg);
    void getRes(float *accel, float *gyro, float *mag);
    void readI2C(uint8_t address, uint8_t subAddress, uint8_t size_buff, uint8_t * buff);
    void readsI2C(uint8_t address, uint8_t subAddress, uint8_t size_buff, uint8_t * buff);
    void writeI2C(uint8_t address, uint8_t subAddress, uint8_t data);
    void updateMadgwickQuaternion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void updateMahonyQuaternion(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    float invSqrt(float x);
  private:
    float GyroMeasError;    // gyroscope measurement error in rads/s (start at 10 deg/s)
    float GyroMeasDrift;    // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float beta;
    float zeta;

};



#endif
