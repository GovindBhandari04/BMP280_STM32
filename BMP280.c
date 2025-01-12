  /*
 * BMP280.c
 *
 *  Created on: Jan 7, 2025
 *      Author: Govind Bhandari
 */

#include "main.h"
#include "stdio.h"
#include "BMP280.h"

HAL_StatusTypeDef  Status;

typedef uint32_t  BMP280_S32_t;
BMP280_S32_t    t_fine;
uint16_t dig_T1,dig_P1;
int16_t dig_T2,dig_T3,dig_P2,dig_P3,dig_P4,dig_P5,dig_P6,dig_P7,dig_P8,dig_P9;
double var1,var2;
float T,P;
uint32_t raw_pressure = 0.0,raw_temperature = 0.0;

uint8_t device_id = 0;
int i;

void i2c1_start()
{
	HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port,I2C1_SDA_Pin,GPIO_PIN_SET);
}

void i2c1_stop()
{
	HAL_GPIO_WritePin(I2C1_SDA_GPIO_Port,I2C1_SDA_Pin,GPIO_PIN_RESET);
}

void i2c_find_device()
{
	for(i=0;i<128;i++)
	{
		if(HAL_I2C_IsDeviceReady(BMP280,(uint8_t)(i<<1),1,10) == HAL_OK)
		{
			device_id = i;
			printf("BMP280 device ID is : %x02x\r\n",device_id);
			break;
		}

		else
		{
			device_id = 128;
			printf("BMP280 device ID is not found : %x02x\r\n",device_id);
		}
	}
}

void i2c1_write(uint8_t reg,uint8_t data)
{
	uint8_t w_buffer[2] = {reg,data};

	HAL_I2C_Master_Transmit(BMP280,BMP280_dev_addr << 1,w_buffer,2,HAL_MAX_DELAY);
}

uint8_t i2c1_read(uint8_t reg)
{
	uint8_t r_data;

	HAL_I2C_Master_Transmit(BMP280,BMP280_dev_addr << 1,&reg,sizeof(reg),HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(BMP280,BMP280_dev_addr << 1,&r_data,sizeof(r_data),HAL_MAX_DELAY);

	return r_data;
}

void Reset()
{
	i2c1_write(reset,0xB6);
	HAL_Delay(10);
}

void sleep_mode()
{
	i2c1_write(ctrl_meas,0x00);
}

void Normal_mode()
{
	i2c1_write(ctrl_meas,0x03);
}

void BMP280_init()
{
	Reset();
	i2c1_write(ctrl_meas,0x27);
	i2c1_write(config,0xA0);
}

void BMP280_calibration()
{
	uint8_t calib_address = dig_T1_lsb;
	uint8_t data[24];

	HAL_I2C_Master_Transmit(BMP280,BMP280_dev_addr << 1,&calib_address,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(BMP280,BMP280_dev_addr << 1,data,24,HAL_MAX_DELAY);

    dig_T1 = (uint16_t)(data[0] | (data[1] << 8));
    dig_T2 = (int16_t)(data[2] | (data[3] << 8));
    dig_T3 = (int16_t)(data[4] | (data[5] << 8));
    dig_P1 = (uint16_t)(data[6] | (data[7] << 8));
    dig_P2 = (int16_t)(data[8] | (data[9] << 8));
    dig_P3 = (int16_t)(data[10] | (data[11] << 8));
    dig_P4 = (int16_t)(data[12] | (data[13] << 8));
    dig_P5 = (int16_t)(data[14] | (data[15] << 8));
    dig_P6 = (int16_t)(data[16] | (data[17] << 8));
    dig_P7 = (int16_t)(data[18] | (data[19] << 8));
    dig_P8 = (int16_t)(data[20] | (data[21] << 8));
    dig_P9 = (int16_t)(data[22] | (data[23] << 8));

}

void BMP280_calibration_data()
{
	uint8_t tx_data = press_msb;
	uint8_t rx_data[6];
	HAL_I2C_Master_Transmit(BMP280,BMP280_dev_addr << 1,&tx_data,1,HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(BMP280,BMP280_dev_addr << 1,rx_data,6,HAL_MAX_DELAY);

	raw_pressure = (uint32_t)(rx_data[0] << 12 | rx_data[1] << 4 | rx_data[2] >> 4);
	raw_temperature = (uint32_t)(rx_data[3] << 12 | rx_data[4] << 4 | rx_data[5] >> 4);
}

float BMP280_Temperature_compensate()
{
var1 = (((double)raw_temperature)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
var2 = ((((double)raw_temperature)/131072.0 - ((double)dig_T1)/8192.0)*(((double)raw_temperature)/131072.0 - ((double)dig_T1)/8192.0))*((double)dig_T3);

t_fine = (BMP280_S32_t)(var1+var2);
T = (var1+var2)/5120.0;

return T;
}

float BMP280_Pressure_compensate()
{
	var1 = ((double)t_fine/2.0)-64000.0;
	var2 = var1*var1*((double)dig_P6)/32768.0;
	var2 = var2+var1*((double)dig_P5)*2.0;
	var2 = (var2/4.0)+(((double)dig_P4)*65536.0);
	var1 = (((double)dig_P3)*var1*var1/524288.0+((double)dig_P2)*var1)/524288.0;
	var1 = (1.0*var1/32768.0)*((double)dig_P1);
	P = 1048576.0-(double)raw_pressure;
	P = (P-(var2/4096.0))*6250.0/var1;
	var1 = ((double)dig_P3)*P*P/2147483648.0;
	var2 = P*((double)dig_P8)/32768.0;
	P = P+(var1+var2+((double)dig_P7))/16.0;

	return P;
}
