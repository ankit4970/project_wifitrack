/*
 * lsm303_compass.hpp
 *
 *  Created on: Mar 31, 2016
 *      Author: ankit
 */

#ifndef L4_IO_LSM303_COMPASS_HPP_
#define L4_IO_LSM303_COMPASS_HPP_

#include <stdint.h>
#include "i2c2_device.hpp"  // I2C Device Base Class

/**
 * Acceleration Sensor class used to get acceleration data reading from the on-board sensor.
 * Acceleration data reading can provide absolute tilt of the board (if under no movement),
 * and it can also provide the movement activity of the board.
 *
 * @ingroup BoardIO
 */
class lsm303_compass : private i2c2_device, public SingletonTemplate<lsm303_compass>
{
    public:
        bool init();   ///< Initializes the sensor

        int16_t getX();  ///< @returns X-Axis value
        int16_t getY();  ///< @returns Y-Axis value
        int16_t getZ();  ///< @returns Z-Axis value
        int8_t getPosition();	///< @returns Position
        uint8_t getXAngleMsb();	///< @returns X-Axis Angle Value
        uint8_t getXAngleLsb();	///< @returns X-Axis Angle Value
        uint8_t getYAngleMsb();	///< @returns Y-Axis Angle Value
        uint8_t getYAngleLsb();	///< @returns Y-Axis Angle Value
        uint8_t getZAngleMsb();	///< @returns Z-Axis Angle Value
        uint8_t getZAngleLsb();	///< @returns Z-Axis Angle Value
        uint8_t getTemperatureMsb(); ///< @returns Temperature Value
        uint8_t getTemperatureLsb(); ///< @returns Temperature Value
        uint8_t getStatus();

    private:
        /// Private constructor of this Singleton class
        lsm303_compass() : i2c2_device(I2CAddr_Magnetometer)
        {
        }
        friend class SingletonTemplate<lsm303_compass>;  ///< Friend class used for Singleton Template

        typedef enum {
        	CRA_REG_M =	0x00,
			CRB_REG_M =	0x01,
			MR_REG_M = 	0x02,
			OUT_X_H_M = 	0x03,
			OUT_X_L_M =		0x04,
			OUT_Z_H_M =		0x05,
			OUT_Z_L_M =		0x06,
			OUT_Y_H_M =		0x07,
			OUT_Y_L_M =		0x08,
 			SR_REG_M =		0x09,
			CTRL_REG1_A =	0x20,	/*	control reg 1		*/
			CTRL_REG2_A	=	0x21,	/*	control reg 2		*/
			CTRL_REG3_A	=	0x22,	/*	control reg 3		*/
			CTRL_REG4_A	=	0x23,	/*	control reg 4		*/
			CTRL_REG5_A	=	0x24,	/*	control reg 5		*/
			CTRL_REG6_A	=	0x25,	/*	control reg 6		*/
			OUT_X_L_A  =    0x28,
			OUT_X_H_A  =    0x29,
			OUT_Y_L_A  =    0x2A,
			OUT_Y_H_A  =    0x2B,
			OUT_Z_L_A  =    0x2C,
			OUT_Z_H_A  =    0x2D,
			TEMP_OUT_H_M =	0x31,
			TEMP_OUT_L_M =	0x32
                } __attribute__ ((packed)) RegisterMap;
};



#endif /* L4_IO_LSM303_COMPASS_HPP_ */
