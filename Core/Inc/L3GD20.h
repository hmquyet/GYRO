#ifndef L3GD20_H
#define L3GD20_H

#include <stdint.h>

// L3GD20 Register Definitions
#define L3GD20_REG_WHO_AM_I    0x0F
#define L3GD20_REG_CTRL1       0x20
#define L3GD20_REG_CTRL2       0x21
#define L3GD20_REG_CTRL3       0x22
#define L3GD20_REG_CTRL4       0x23
#define L3GD20_REG_CTRL5       0x24

#define L3GD20_REG_OUT_X_L     0x28
#define L3GD20_REG_OUT_X_H     0x29
#define L3GD20_REG_OUT_Y_L     0x2A
#define L3GD20_REG_OUT_Y_H     0x2B
#define L3GD20_REG_OUT_Z_L     0x2C
#define L3GD20_REG_OUT_Z_H     0x2D

// Sensitivity
#define L3GD20_SENSITIVITY_250DPS   0.00875f
#define L3GD20_SENSITIVITY_500DPS   0.0175f
#define L3GD20_SENSITIVITY_2000DPS  0.07f

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} L3GD20_RawData_t;

typedef struct {
    float x;
    float y;
    float z;
} L3GD20_AngleRate_t;

void L3GD20_Init(void);
void L3GD20_ReadRawData(L3GD20_RawData_t *rawData);
void L3GD20_CalculateAngleRate(const L3GD20_RawData_t *rawData, L3GD20_AngleRate_t *angleRate);

#endif