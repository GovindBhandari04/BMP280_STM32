/*
 * BMP280.h
 *
 *  Created on: Jan 7, 2025
 *      Author: Govind Bhandari
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "main.h"

extern I2C_HandleTypeDef    hi2c1;
#define BMP280              &hi2c1

#define BMP280_dev_addr     0x76
#define BMP280_write        0xEC
#define BMP280_read         0xED

//BMP280 Memory map
#define temp_xlsb     0xFC
#define temp_lsb      0xFB
#define temp_msb      0xFA
#define press_xlsb    0xF9
#define press_lsb     0xF8
#define press_msb     0xF7
#define config        0xF5
#define ctrl_meas     0xF4
#define status        0xF3
#define reset         0xE0
#define id            0xD0
#define dig_T1_lsb    0x88
#define dig_T1_msb    0x89
#define dig_T2_lsb    0x8A
#define dig_T2_msb    0x8B
#define dig_T3_lsb    0x8C
#define dig_T3_msb    0x8D
#define dig_P1_lsb    0x8E
#define dig_P1_msb    0x8F
#define dig_P2_lsb    0x90
#define dig_P2_msb    0x91
#define dig_P3_lsb    0x92
#define dig_P3_msb    0x93
#define dig_P4_lsb    0x94
#define dig_P4_msb    0x95
#define dig_P5_lsb    0x96
#define dig_P5_msb    0x97
#define dig_P6_lsb    0x98
#define dig_P6_msb    0x99
#define dig_P7_lsb    0x9A
#define dig_P7_msb    0x9B
#define dig_P8_lsb    0x9C
#define dig_P8_msb    0x9D
#define dig_P9_lsb    0x9E
#define dig_P9_msb    0x9F
#define reserved_lsb  0xA0
#define reserved_msb  0xA1

void i2c1_start();
void i2c1_stop();
void i2c_find_device();
void i2c1_write(uint8_t reg,uint8_t data);
uint8_t i2c1_read(uint8_t reg);
void Reset();
void sleep_mode();
void forced_mode();
void Normal_mode();
void BMP280_init();
void BMP280_calibration();
void BMP280_calibration_data();
float BMP280_Temperature_compensate();
float BMP280_Pressure_compensate();

#endif /* INC_BMP280_H_ */
