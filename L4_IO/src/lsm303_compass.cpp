/*
 * lsm303_compass.cpp
 *
 *  Created on: Mar 31, 2016
 *      Author: ankit
 */

#include <stdint.h>

#include "io.hpp" // All IO Class definitions

bool lsm303_compass::init()
{
    const unsigned char datarate15Hz = (0x04 << 2 | 1<<7); 	// data output rate = 15Hz
    const unsigned char gainConfig81 = 0x07 << 5;
    const unsigned char continousConvert = 0;
    const unsigned char accelEnable = 0x27;     			/* Normal mode 10Hz  */

    //printf("Initializing LSM303 sensor\n");
    writeReg(CTRL_REG1_A,accelEnable);
    writeReg(CTRL_REG4_A,0x28);
    writeReg(CRA_REG_M,0x94);
    //writeReg(CRB_REG_M,0x80);
    writeReg(MR_REG_M,0x00);

    return true;
}
int16_t lsm303_compass::getX()
{
    return (int16_t)get16BitRegister(OUT_X_H_A);
}
int16_t lsm303_compass::getY()
{
    return (int16_t)get16BitRegister(OUT_Y_H_A);
}
int16_t lsm303_compass::getZ()
{
    return (int16_t)get16BitRegister(OUT_Z_H_A);
}

uint8_t lsm303_compass::getXAngleMsb()
{
    return (uint8_t)readReg(OUT_X_H_M);
}
uint8_t lsm303_compass::getXAngleLsb()
{
    return (uint8_t)readReg(OUT_X_L_M);
}
uint8_t lsm303_compass::getYAngleMsb()
{
    return (uint8_t)readReg(OUT_Y_H_M);
}
uint8_t lsm303_compass::getYAngleLsb()
{
    return (uint8_t)readReg(OUT_Y_L_M);
}
uint8_t lsm303_compass::getZAngleMsb()
{
    return (uint8_t)readReg(OUT_Z_H_M);
}
uint8_t lsm303_compass::getZAngleLsb()
{
    return (uint8_t)readReg(OUT_Z_L_M);
}

uint8_t lsm303_compass::getStatus()
{
    return (uint8_t)readReg(SR_REG_M);
}

uint8_t lsm303_compass::getTemperatureMsb()
{
    return (uint8_t)readReg(TEMP_OUT_H_M);
}
uint8_t lsm303_compass::getTemperatureLsb()
{
    return (uint8_t)readReg(TEMP_OUT_L_M);
}
