#include <stdio.h>
#include <string.h>

/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL1);

char buffer[128]; // buffer for string assembly before transmitting

void setup() {
  int CWPosLim;
  int CCWPosLim;
  // Initializes Dynamixel BUS
  // Assumes all dynamixels are configures with baud rate 1Mbaud
  // In Protocol 2.0, 1Mbaud corresponds to configuration 3
  Dxl.begin(3);
  //Initialize serial USB to communicate with PC
  SerialUSB.begin();
  delay(5000); //give time to open SerialMonitor
  
  // Query limits for motor ID 1 - read right motor
  CWPosLim = Dxl.readWord(1, 6);
  CCWPosLim = Dxl.readWord(1, 8);
  sprintf(buffer, "Motor 1: MaxPosLim=%d, MinPosLim=%d\n", CWPosLim, CCWPosLim);
  SerialUSB.println(buffer);
  // Query limits for motor ID 2 - front motor
  CWPosLim = Dxl.readWord(2, 6);
  CCWPosLim = Dxl.readWord(2, 8);
  sprintf(buffer, "Motor 2: MaxPosLim=%d, MinPosLim=%d\n", CWPosLim, CCWPosLim);
  SerialUSB.println(buffer);
  // Query limits for motor ID 3 - read left motor
  CWPosLim = Dxl.readWord(3, 6);
  CCWPosLim = Dxl.readWord(3, 8);
  sprintf(buffer, "Motor 3: MaxPosLim=%d, MinPosLim=%d\n", CWPosLim, CCWPosLim);
  SerialUSB.println(buffer);
}

void loop() {
  delay(5000);
  SerialUSB.println("Done!");
}
<<<<<<< HEAD

=======

>>>>>>> 7b96e2bc900f57a118e78d7f09cb20a7cd9fd116
