/**
 * \file delay.h
 *
 * \ingroup PIC24-I2C Driver
 *
 * \brief Delay functions to help with i2c communication
 *
 * \author $Author: gellings, emros and superjax $
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

#ifndef DELAY_H
#define	DELAY_H

/* this function delays s number of clock cycles that it receives in
 as input.  It assumes use of the 8 MHz RC occilator*/
void DelayuSec(unsigned int s)
{
    int k = 0;
    while(k < s)
        k++;
}

/* This function delays s number of milliseconds.  It assumes the use of the 8MHz RC
 * Oscillator
 */
void Delay(int s)
{
    int k = 0;
    while(k < s)
    {
        DelayuSec(667);
        k++;  // 667 is equivalent to 0.001 seconds
    }
}

#endif	/* DELAY_H */

