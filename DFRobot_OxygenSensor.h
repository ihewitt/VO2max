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
  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 */
#ifndef __DFRobot_OxygenSensor_H__
#define __DFRobot_OxygenSensor_H__

#define           ADDRESS_0                 0x70           // iic slave Address
#define           ADDRESS_1                 0x71
#define           ADDRESS_2                 0x72
#define           ADDRESS_3                 0x73

#define           OCOUNT                    100            // Oxygen Count Value
/* Oxygen register address */
#define           OXYGEN_DATA_REGISTER      0x03           // Oxygen data register
#define           USER_SET_REGISTER         0x08           // user set key value
#define           AUTUAL_SET_REGISTER       0x09           // autual set key value
#define           GET_KEY_REGISTER          0x0A           // get key value

class DFRobot_OxygenSensor{  
public:
  DFRobot_OxygenSensor();
  ~DFRobot_OxygenSensor();
  bool     begin(uint8_t addr = ADDRESS_0);
  void     Calibrate(float vol, float mv = 0);
  float    ReadOxygenData(uint8_t CollectNum);
  
private:
  void     ReadFlash();
  void     i2cWrite(uint8_t Reg , uint8_t pdata);
  uint8_t  _addr;                               // IIC Slave number
  float    _Key = 0.0;                          // oxygen key value
  float    OxygenData[OCOUNT] = {0.00};
  float    getAverageNum(float bArray[], uint8_t iFilterLen);
};

#endif
