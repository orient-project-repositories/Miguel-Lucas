#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL1);
unsigned long sampling_time = 10;
unsigned long time;
unsigned long delta;
unsigned long sample_counter = 0;
int remaining;
char buffer[100];
int curpos1, curpos2, curpos3;

void setup() {
  // put your setup code here, to run once:
  SerialUSB.begin();
  Dxl.begin(1); //Initialize to 1000000 baud rate
  delay(1000);
  time = millis();
}

void loop() {
  // put your main code here, to run repeatedly: 
  //SerialUSB.println("Hello World");
  //SerialUSB.print("X\n");
  curpos1 = Dxl.readByte(1,3);
  int result = Dxl.getResult( );

 if( result == COMM_TXSUCCESS )
 {
 SerialUSB.println("COMM_TXSUCCESS");
 }
 else if( result == COMM_RXSUCCESS )
 {
   SerialUSB.println("COMM_RXSUCCESS");
 }
 else if( result == COMM_TXFAIL )
 {
   SerialUSB.println("COMM_TXFAIL");
 }
 else if( result == COMM_RXFAIL)
 {
SerialUSB.println("COMM_RXFAIL");
 }
 else if( result == COMM_TXERROR )
 {
 SerialUSB.println("COMM_TXERROR");
 }
 else if( result == COMM_RXWAITING )
 {
   SerialUSB.println("COMM_RXWAITING");
 }
 else 
  SerialUSB.print("UNKNOWN RESULT");
  
  delta = millis()-time;
  remaining = sampling_time-delta;
  if(remaining < 0)
  {
    SerialUSB.print("Warning - Sample Time Exceeded:");
    SerialUSB.println(-remaining);
  }
  else
  {
    delay(sampling_time - delta);
  }
  delta = millis()-time;
  time = millis();
  sample_counter = sample_counter+1;
  //SerialUSB.println(sample_counter);
  sprintf(buffer,"Pos1: %d Pos2: %d Pos3:%d Remaining: %d Delta: %d Time: %d \n",curpos1, curpos2, curpos3, remaining, delta, time);
  //SerialUSB.print(remaining);
  //SerialUSB.print("asdkhflkasjdhfgjkasldhgljkafshgjklsdhfjklsdhljfkhasjdkh \n");
  SerialUSB.print(buffer);
  //SerialUSB.print("\n");
  //SerialUSB.write(10);
  //SerialUSB.println(time);
}

