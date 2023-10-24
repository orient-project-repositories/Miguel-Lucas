#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL1);

#define ID 1

void setup() {
  // put your setup code here, to run once:
  //Dxl.begin(1); //Initialize to 1000000 baud rate
  delay(10000);
}

void loop() 
{
  int i, j, val, result; 
  for(i=0;i<256;i++) 
  {
    for(j=0;j<9;j++)
    {
    Dxl.begin(j);
    delay(100);
    val = Dxl.readByte(i, 0);
    delay(100);
    result = Dxl.getResult();
    SerialUSB.print(" ID: ");
    SerialUSB.print(i);
    SerialUSB.print(" BAUD : ");
    SerialUSB.print(j);
    SerialUSB.print(" VAL : ");
    SerialUSB.print(val);
    SerialUSB.print(" ERROR CODE : ");
    SerialUSB.println(result);
    }
  }
  delay(10000);
}

