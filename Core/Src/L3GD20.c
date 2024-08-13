#include "l3gd20.h"
#include "main.h"
#include "spi.h" 

static uint8_t spiTxBuf[2];
static uint8_t spiRxBuf[7];

void L3GD20_Init(void) {
	
    spiTxBuf[0] = L3GD20_REG_CTRL1;
    spiTxBuf[1] = 0xFF; 
    SPI_Transmit(spiTxBuf, 2);
		
    spiTxBuf[0] = L3GD20_REG_CTRL2;
    spiTxBuf[1] = 0x00; 
    SPI_Transmit(spiTxBuf, 2);
	
		spiTxBuf[0] = L3GD20_REG_CTRL3;
    spiTxBuf[1] = 0x00; 
    SPI_Transmit(spiTxBuf, 2);
	
		spiTxBuf[0] = L3GD20_REG_CTRL4;
    spiTxBuf[1] = 0x20; 
    SPI_Transmit(spiTxBuf, 2);
	
		spiTxBuf[0] = L3GD20_REG_CTRL5;
    spiTxBuf[1] = 0x10; 
    SPI_Transmit(spiTxBuf, 2);
}

void L3GD20_ReadRawData(L3GD20_RawData_t *rawData) {
   spiTxBuf[0] = L3GD20_REG_OUT_X_L | 0x80; 
    SPI_TransmitReceive(&spiTxBuf[0], &spiRxBuf[1], 1);
	 spiTxBuf[0] = L3GD20_REG_OUT_X_H | 0x80; 
    SPI_TransmitReceive(&spiTxBuf[0], &spiRxBuf[2], 1);
	 spiTxBuf[0] = L3GD20_REG_OUT_Y_L | 0x80; 
    SPI_TransmitReceive(&spiTxBuf[0], &spiRxBuf[3], 1);
	 spiTxBuf[0] = L3GD20_REG_OUT_X_H | 0x80; 
    SPI_TransmitReceive(&spiTxBuf[0], &spiRxBuf[4], 1);
	 spiTxBuf[0] = L3GD20_REG_OUT_Z_L | 0x80; 
    SPI_TransmitReceive(&spiTxBuf[0], &spiRxBuf[5], 1);
	 spiTxBuf[0] = L3GD20_REG_OUT_Z_H | 0x80; 
    SPI_TransmitReceive(&spiTxBuf[0], &spiRxBuf[6], 1);
	
	
    rawData->x = (spiRxBuf[2] << 8) | spiRxBuf[1];
    rawData->y = (spiRxBuf[4] << 8) | spiRxBuf[3];
    rawData->z = (spiRxBuf[6] << 8) | spiRxBuf[5];
}

void L3GD20_CalculateAngleRate(const L3GD20_RawData_t *rawData, L3GD20_AngleRate_t *angleRate) {
    angleRate->x = rawData->x * L3GD20_SENSITIVITY_2000DPS;
    angleRate->y = rawData->y * L3GD20_SENSITIVITY_2000DPS;
    angleRate->z = rawData->z * L3GD20_SENSITIVITY_2000DPS;
}