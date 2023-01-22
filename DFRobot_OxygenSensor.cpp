/*

  MIT License

  Copyright (C) <2019> <@DFRobot ZhiXinLiu>


  Permission is hereby granted, free of charge, to any person obtaining a copy of this

  software and associated documentation files (the "Software"), to deal in the Software

  without restriction, including without limitation the rights to use, copy, modify,

  merge, publish, distribute, sublicense, and/or sell copies of the Software, and to

  permit persons to whom the Software is furnished to do so.



  The above copyright notice and this permission notice shall be included in all copies or

  substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,

  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR

  PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE

  FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,

  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */
 
 
#include <Arduino.h>
#include <Wire.h>
#include "DFRobot_OxygenSensor.h"

DFRobot_OxygenSensor::DFRobot_OxygenSensor()
{
}
DFRobot_OxygenSensor::~DFRobot_OxygenSensor()
{
}

/* join i2c bus (address optional for master) */
bool DFRobot_OxygenSensor::begin(uint8_t addr)
{
  this->_addr = addr;              // Set the host address 
  Wire.begin();                     // connecting the i2c bus 
  Wire.beginTransmission(_addr);
  if(Wire.endTransmission() == 0) {
    return true;
  }
  return false;
}

void DFRobot_OxygenSensor::ReadFlash()
{
  uint8_t value = 0;
  Wire.beginTransmission(_addr);
  Wire.write(GET_KEY_REGISTER);
  Wire.endTransmission();
  delay(50);
  Wire.requestFrom(_addr, (uint8_t)1);
    while (Wire.available())
      value = Wire.read();
  if(value == 0) {
    this->_Key = 20.9 / 120.0;
  }else {
    this->_Key = (float)value / 1000.0;
  }
}

/* Write data to the i2c register  */
void DFRobot_OxygenSensor::i2cWrite(uint8_t Reg , uint8_t pdata)
{
  Wire.beginTransmission(_addr);
  Wire.write(Reg);
  Wire.write(pdata);
  Wire.endTransmission();
}

/* Set Key value */
void DFRobot_OxygenSensor::Calibrate(float vol, float mv) {
  uint8_t keyValue = vol * 10;
  if(mv < 0.000001 && mv > (-0.000001) ) {
    i2cWrite(USER_SET_REGISTER , keyValue);
  }else {
    keyValue = (vol / mv) * 1000;
    i2cWrite(AUTUAL_SET_REGISTER , keyValue);
  }
}

/* Reading oxygen concentration */
float DFRobot_OxygenSensor::ReadOxygenData(uint8_t CollectNum) 
{
  uint8_t rxbuf[10]={0}, k = 0;
  static uint8_t i = 0 ,j = 0;
  ReadFlash();
  if(CollectNum > 0) {
    for(j = CollectNum - 1;  j > 0; j--) {  OxygenData[j] = OxygenData[j-1]; } 
    Wire.beginTransmission(_addr);
    Wire.write(OXYGEN_DATA_REGISTER);
    Wire.endTransmission();
    delay(100);
    Wire.requestFrom(_addr, (uint8_t)3);
      while (Wire.available())
        rxbuf[k++] = Wire.read();
    OxygenData[0] = ((_Key) * (((float)rxbuf[0]) + ((float)rxbuf[1] / 10.0) + ((float)rxbuf[2] / 100.0)));
    if(i < CollectNum) i++;
    return getAverageNum(OxygenData, i);
  }else {
    return -1.0;
  }
}

/* Get the average data */
float DFRobot_OxygenSensor::getAverageNum(float bArray[], uint8_t iFilterLen) 
{
  uint8_t i = 0;
  double bTemp = 0;
  for(i = 0; i < iFilterLen; i++) {
    bTemp += bArray[i];
  }
  return bTemp / (float)iFilterLen;
}
