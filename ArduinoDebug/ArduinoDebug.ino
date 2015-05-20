/**
 * \file ArduinoDebug.ino
 *
 * \ingroup PIC24-I2C Driver
 *
 * \brief Used to recieve int and float debut information from the PIC
 * microcontroller attached via I2C using the associated debug statements
 * found in the associated code
 *
 * \note Careful attention should be made to the order of information being passed
 * through I2C.  This code expects to receive exactly 4 ints and 2 floats in that order
 * every time it receives anything.  If you wanted to change that, you need to make sure that
 * the arduino is expecting the kind of output from the PIC.
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
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



#include <Wire.h> 
#include <math.h>
#define X1_OFFSET 567 // These are the values when the robot is looking directly at 
#define X2_OFFSET 514 // a light in the corner from the center of the ring
#define Y1_OFFSET 458
#define Y2_OFFSET 514


int camera_number; // a counter to order incoming data
int x1, x2, y1, y2; // data placeholder for returned position data
float r_rec, theta_rec; // data placeholder for either returned range data, or calculated data
float r1, r2; // data placeholder used in the calculation of range data
byte flag; // a flag used to throttle serial output only when new data is received

void setup() 
{ 
  flag = 1;
  r_rec = 5.5;
  theta_rec = 0;
  Wire.begin(0x04);                // join i2c bus with address #2 
  camera_number = 1;
  Wire.onRequest(requestEvent); // register event 
  Wire.onReceive(receivefloatEvent);
  Serial.begin(115200);           // start serial for output 
} 


/**
 * The idea here is to loop through, and check if new data has been recieved.
 * The i2C communication is bound to an interrupt, which is run whenever new
 * data comes in on the lines, so the main loop can run independently of the i2c
 * recieve lines.  It will simply output the new data if the flag is set by the i2c
 * recieve functions.
 */

void loop() 
{ 
  float r = 0;
  float theta = 0;
  float phi1 = 0.0794; // found manually from measuring the robot
  float phi2 = 0.0794;
  float d = 5; // found manually from measuring the robot
  if(flag)
  {

      if(x1 >= 1023 || x2>=1023) // only able to see with one eye
      {
        mono_vision(d, phi1, x1, phi2, x2, &theta, &r);
      }
      else
      {
        stereo_vision(d, phi1, x1, phi2, x2, &theta, &r);
      }
      
      
      Serial.print("x1 = ");
      Serial.print(x1,DEC);
      Serial.print("\t  y1 = ");
      Serial.print(y1,DEC);
      Serial.print("\t  x2 = ");
      Serial.print(x2,DEC);
      Serial.print("\t  y2 = ");
      Serial.print(y2,DEC);
      Serial.print("\t  r_rec =  ");
      Serial.print(r_rec,DEC);
      Serial.print("\t  theta_rec = ");
      Serial.println(theta_rec,DEC);
//      Serial.print("\t  R_calc =  ");
//      Serial.print(r,DEC);
//      Serial.print("\t  theta_calc = ");
//      Serial.println(theta*180.0/PI,DEC);
      flag = 0;
    } 
}

/**
 * This function is run as a callback for whenever new data is 
 * on the i2c line
 * It currently is unused, but could be useful in the future
 * Its functionality is wrapped up in the receivefloatEvent method
 */

void receive2intsEvent(int n)
{
      int int1, int2;
      int1 = Wire.read();
      int1 = (int1 | Wire.read() << 8) & 0b1111111111;
      int2 = Wire.read();
      int2 = (y1 | Wire.read() << 8) & 0b1111111111;
      
    Serial.print("int1 = ");
    Serial.print(int1);
    Serial.print("\t int1 = ");
    Serial.println(int1);
}

/**
 * This function does the actual reading of the i2c line, using the Wire library
 * The arduino will save off the first 10 bits of the the first 2 bytes (the data we are expecting is
 * at most 1023) for the position ints and the first 4 bytes of data for the floats.
 * 
 * camera_number was originally used to determine which camera the two ints came from,
 * but was used to decide when floats were coming down the pipe as well, so
 * the name camera_number probably could be changed to something less confusing.
 * We don't have 4 cameras, just 4 things that we need to grab from the i2c line
 */

void receivefloatEvent(int n)
{
  flag = 1;
  char* floatptr1 = (char*)&r_rec;
  char* floatptr2 = (char*)&theta_rec;
  switch(camera_number) {
    
    case 1:
      x1 = (Wire.read()<<8 | Wire.read()) & 0b1111111111;
      y1 = (Wire.read()<<8 | Wire.read()) & 0b1111111111;
      camera_number = 2;
      break;
      
    case 2:
      x2 = (Wire.read()<<8 | Wire.read()) & 0b1111111111;
      y2 = (Wire.read()<<8 | Wire.read()) & 0b1111111111;
      camera_number =3;
      break;
      
    case 3: 
      floatptr1[3] = Wire.read();
      floatptr1[2] = Wire.read();
      floatptr1[1] = Wire.read();
      floatptr1[0] = Wire.read();
      camera_number = 4;
      break;
      
    case 4: 
      floatptr2[3] = Wire.read();
      floatptr2[2] = Wire.read();
      floatptr2[1] = Wire.read();
      floatptr2[0] = Wire.read();
      camera_number = 1;
      break;
      
  } 
}

/* function that executes whenever data is received from master 
 * this function is registered as another callback, and was used
 * to make sure that communication was happening on the i2c line.
 */
void requestEvent() 
{
 Wire.write(0x03); 
}

/**
 *  If this is confusing, you can read the wiki on how this works
 */
void mono_vision(float d, float phi1, float x1, float phi2, float x2, float* theta, float* r)
{
   float beta, x, y;
   if(x1 >= 1023) // can't see with left eye
   {
     x = x2;
     y = y2;
     beta = (y2-Y2_OFFSET);
     *r = 2.4*tan(beta);
     *theta = (x2-X2_OFFSET)/1023.0 * 33.0/360.0*2.0*PI;
     r1 = 0;
     r2 = 0;  
     
   }
   else if(x2 >= 1023){ // can't see with right eye
     x = x1;
     y = y1;
     beta = (y1-Y1_OFFSET);
     *r = 2.4*tan(beta);
     *theta = (x1-X1_OFFSET)/1023.0 * 33.0/360.0*2.0*PI; 
     r1 = 0;
     r2 = 0;    
   }
   else
   {
     return; // don't do anything, you can see with both eyes
   }
   
}

void stereo_vision(float d, float phi1, float x1, float phi2, float x2, float* theta, float* r)
{
  // linearly interpolate gamma1 and gamma2
  float gamma1, gamma2;
  gamma1 = (x1-X1_OFFSET)/1023.0 * 33.0/360.0*2.0*PI;
  gamma2 = (x2-X2_OFFSET)/1023.0 * 33.0/360.0*2.0*PI;

  // calculate distance from camera to features r1 and r2
  r1 = sin(PI/2.0-phi2-gamma2)/(sin(phi1+gamma1+phi2+gamma2)/d);
  r2 = sin(PI/2.0-(phi1+gamma1))/(sin(phi1+gamma1+phi2+gamma2)/d);
  
  // find r and theta
  *r = pow(pow(r1,2.0)+pow(d/2.0,2.0)-2.0*r1*(d/2.0)*cos(PI/2.0-phi1-gamma1),1/2.0);
  float rad = *r;
  *theta = asin(r1*(sin(PI/2.0-phi1-gamma1)/rad)) - PI/2.0; //stuck always positive, due to range of asin()
  if(r1>r2)
  {
    float th = *theta;
    *theta = -th;
  }
  
}
