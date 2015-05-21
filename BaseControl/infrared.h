/**
 * \file infrared.h
 *
 * \ingroup PIC24-I2C Driver
 *
 * \brief Methods to initialize the DFRobot Position Sensor and receive
 * data.  Follows closely the method used by kako http://www.kako.com for 
 * the arduino on http://www.dfrobot.com/wiki/index.php/Positioning_ir_camera
 *
 * \author $Author: gellings and superjax $
 *
 * \version $Revision: 1.0 $
 *
 * \date $Date: 04/26/2015 14:16:20 $
 *
 * Contact: superjax08@gmail.com
 *
 * Created on: Mar 25, 2015
 *
 * \license
 *
 * Copyright (c) 2015, James Jackson, Gary Ellingson, Emily Lazglade, Nolan Crook
 *
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this 
 * list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this 
 * list of conditions and the following disclaimer in the documentation and/or other 
 * materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may 
 * be used to endorse or promote products derived from this software without specific 
 * prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INFRARED_H
#define	INFRARED_H

#include "i2c1_helper.h"
#include "i2c2_helper.h"

char IR1sensorAddress = 0xB0;
char IR2sensorAddress = 0xB0;


/**
 * ir1_init a method which writes a series of registers which prepares the 1st IR
 * camera to produce output
 */
void ir1_init()
{
    I2C1write2bytes(IR1sensorAddress, 0x30, 0x01); DelayuSec(10000);
    I2C1write2bytes(IR1sensorAddress, 0x30, 0x08); DelayuSec(10000);
    I2C1write2bytes(IR1sensorAddress, 0x06, 0x90); DelayuSec(10000);
    I2C1write2bytes(IR1sensorAddress, 0x08, 0xC0); DelayuSec(10000);
    I2C1write2bytes(IR1sensorAddress, 0x1A, 0x40); DelayuSec(10000);
    I2C1write2bytes(IR1sensorAddress, 0x33, 0x33); DelayuSec(10000);
}

/**
 * ir1_request a method which requests data from the IR sensor and puts it into
 * the pointers supplied
 * @param xptr pointer to the x-dimension data
 * @param yptr pointer to the y-dimension data
 */
void ir1_request(char* xptr, char* yptr)
{
    I2C1write(IR1sensorAddress, 0x36);
    char data[4] = {0, 0, 0, 0};
    I2C1requestFrom(IR1sensorAddress, 4, &data);

    /* Right here, the sensor returns a 10-bit value for each x and y 
     * dimension.  due to the limitations of i2c, only 8 bits (1 byte) can 
     * be sent at a time.  The remaining two bytes are sent  on the byte 
     * between the two main bytes, and are shown bitshifting into position 
     * on the other values.
     */

    /* We also found perhaps a bug in the Windows version of the XC16 compiler.
     * The bits stack backwards when in Windows
     * When Compiled on Windows, this code works
     */
    xptr[0] = data[1];
    yptr[0] = data[2];
    char s = data[3];
    xptr[1] = (s & 0x30) >> 4;
    yptr[1] = (s & 0xC0) >> 6;


    /* However, when compiled on Linux, this code works
     *
     * xptr[1] = data[1];
     * yptr[1] = data[2];
     * char s = data[3];
     * xptr[0] = (s & 0x30) >> 4;
     * yptr[0] = (s & 0xC0) >> 6;
     */

}

/**
 * ir2_request exactly the same as ir1_init, except for the second camera
 */
void ir2_init()
{
    I2C2write2bytes(IR2sensorAddress, 0x30, 0x01); DelayuSec(10000);
    I2C2write2bytes(IR2sensorAddress, 0x30, 0x08); DelayuSec(10000);
    I2C2write2bytes(IR2sensorAddress, 0x06, 0x90); DelayuSec(10000);
    I2C2write2bytes(IR2sensorAddress, 0x08, 0xC0); DelayuSec(10000);
    I2C2write2bytes(IR2sensorAddress, 0x1A, 0x40); DelayuSec(10000);
    I2C2write2bytes(IR2sensorAddress, 0x33, 0x33); DelayuSec(10000);
}


/**
 * ir2_request exactly the same as ir1_request, but for the second camera
 * @param xptr pointer to the x-dimension data
 * @param yptr pointer to the y-dimension data
 */
void ir2_request(char* xptr, char* yptr)
{
    I2C2write(IR1sensorAddress, 0x36);
    char data[4] = {0, 0, 0, 0};
    I2C2requestFrom(IR2sensorAddress, 4, &data);

    // I changed this in my code - James
    // It appears that my compiler reads bits weird.
    // Previously Gary was putting the bits into [1] first, then [0]
    xptr[0] = data[1];
    yptr[0] = data[2];
    char s = data[3];
    xptr[1] = (s & 0x30) >> 4;
    yptr[1] = (s & 0xC0) >> 6;
}

#endif	/* INFRARED_H */

