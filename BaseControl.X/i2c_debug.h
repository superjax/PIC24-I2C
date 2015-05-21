/**
 * \file i2c1_helper
 *
 * \ingroup PIC24-I2C Driver
 *
 * \brief Tools to send debug information to an arduino running the included
 * ArduinoDebug.ino sketch and connected on the I2C1 line.
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

#ifndef I2C_DEBUG_H
#define	I2C_DEBUG_H

#include "i2c2_helper.h"


// Assumes that the arduino is address 0x04
// The bitshift is to tell the arduino that we are
// writing data.
char arduinoAddress = 0x04<<1;


/**
 * debug_2_ints a function which writes 2 ints to the arduino
 * @param x1 first int to send
 * @param x2 second int to send
 */
void debug_2_ints(int x1, int y1)
{

    int val1 = x1;
    int val2 = y1;
    char* ptr1 = &val1;
    char* ptr2 = &val2;

    I2C1write4bytes(arduinoAddress, ptr1[1], ptr1[0], ptr2[1], ptr2[0]);
}


/**
 * debug_float a function which can write a float (4 bytes) to the arduino
 * @param val float to send
 */
void debug_float(float val)
{
    char* ptr1 = &val;
    I2C1write4bytes(arduinoAddress, ptr1[3], ptr1[2], ptr1[1], ptr1[0]);
}

#endif	/* I2C_DEBUG_H */

