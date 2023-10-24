#include <stdio.h>
#include <string.h>

/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1 1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2 2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3 3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

Dynamixel Dxl(DXL_BUS_SERIAL1);

unsigned long sampling_time = 10; //10ms sampling time. Should do for reading positions.

int setpoint1 = 512 + 70;
int setpoint2 = 512;
int setpoint3 = 512 - 70;

char buffer[128];   // buffer for string assembly before transmitting
int flag_go;    // indicates if loop should run. If initialization fails, loop reports failure.
unsigned long time; 

void setup() {
  int CWPosLim;
  int CCWPosLim;
  // Initializes Dynamixel BUS
  // Assumes all dynamixels are configures with baud rate 1Mbaud
  // In Protocol 2.0, 1Mbaud corresponds to configuration 3
  Dxl.begin(3);
  //Initialize serial USB to communicate with PC
  SerialUSB.begin();
  // Query limits for motor ID 1 - read right motor
  CWPosLim = Dxl.readWord(1, 6);
  CCWPosLim = Dxl.readWord(1, 8);
  if( CWPosLim != 0 ) {
    sprintf(buffer, "Motor 1: Inappropriate CW Angle Limit=%d\n", CWPosLim);
    SerialUSB.println(buffer);
    flag_go = 0;
  }
  if( CCWPosLim != 1024 ) {
    sprintf(buffer, "Motor 1: Inappropriate CCW Angle Limit=%d\n", CCWPosLim);
    SerialUSB.println(buffer);
    flag_go = 0;
  }
  // Query limits for motor ID 2 - front motor
  CWPosLim = Dxl.readWord(2, 6);
  CCWPosLim = Dxl.readWord(2, 8);
  if( CWPosLim != 0 ) {
    sprintf(buffer, "Motor 2: Inappropriate CW Angle Limit=%d\n", CWPosLim);
    SerialUSB.println(buffer);
    flag_go = 0;
  }
  if( CCWPosLim != 800 ) {
    sprintf(buffer, "Motor 2: Inappropriate CCW Angle Limit=%d\n", CCWPosLim);
    SerialUSB.println(buffer);
    flag_go = 0;
  }
  // Query limits for motor ID 3 - read left motor
  CWPosLim = Dxl.readWord(3, 6);
  CCWPosLim = Dxl.readWord(3, 8);
  if( CWPosLim != 0 ) {
    sprintf(buffer, "Motor 3: Inappropriate CW Angle Limit=%d\n", CWPosLim);
    SerialUSB.println(buffer);
    flag_go = 0;
  }
  if( CCWPosLim != 1024 ) {
    sprintf(buffer, "Motor 3: Inappropriate CCW Angle Limit=%d\n", CCWPosLim);
    SerialUSB.println(buffer);
    flag_go = 0;
  }
  flag_go = 1;
  
  // Set velocity limits - approx 1 rpm (6 degrees/second)
  Dxl.goalSpeed(1,100);
  Dxl.goalSpeed(2,100);
  Dxl.goalSpeed(3,100);
  
  // Set nominal positions 
  Dxl.jointMode(1);
  Dxl.jointMode(2);
  Dxl.jointMode(3);
  Dxl.goalPosition(1, setpoint1);
  Dxl.goalPosition(2, setpoint2);
  Dxl.goalPosition(3, setpoint3);
  
  SerialUSB.attachInterrupt(usbInterrupt); //to receive commands from SerialMonitor
  
  delay(10000); // give time to setup
 
  //prepare loop start 
  time=millis();
}

void usbInterrupt(byte* buffer, byte nCount){
  //only accept sigle byte commands (char)
  if( nCount != 1 ) return;
  switch(buffer[0])
  { 
<<<<<<< HEAD
    case '1': setpoint1+=10; break;
    case 'q': setpoint1-=10; break;
    case '2': setpoint2+=10; break;
    case 'w': setpoint2-=10; break;
    case '3': setpoint3+=10; break;
    case 'e': setpoint3-=10; break;
=======
    case '1': setpoint1++; break;
    case 'q': setpoint1--; break;
    case '2': setpoint2++; break;
    case 'w': setpoint2--; break;
    case '3': setpoint3++; break;
    case 'e': setpoint3--; break;
>>>>>>> 7b96e2bc900f57a118e78d7f09cb20a7cd9fd116
  }
}

void loop() {
  if( !flag_go ) {
     SerialUSB.println("Initialization failed! Halted execution.");
     delay(5000);
  }
  else
  {
    int curpos1, curpos2, curpos3;    //current motor positions
    int delta;                        //computational delay
    int remaining;                    //time remaining to next cycle
    curpos1 = Dxl.readWord(1,36);
    curpos2 = Dxl.readWord(2,36);
    curpos3 = Dxl.readWord(3,36);
    Dxl.goalPosition(1, setpoint1);
    Dxl.goalPosition(2, setpoint2);
    Dxl.goalPosition(3, setpoint3);
    delta = millis()-time;           // how long did it take to read the motor positions
    remaining = sampling_time - delta;
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
    time = millis();          //Serial communication enters in the next computation time
    sprintf(buffer,"Pos1: %d (%d) Pos2: %d (%d) Pos3: %d (%d) Remaining: %d Delta: %d Time: %d \n",
      curpos1, setpoint1, curpos2, setpoint2, curpos3, setpoint3, remaining, delta, time);
    SerialUSB.print(buffer);
  }
}
<<<<<<< HEAD

=======

>>>>>>> 7b96e2bc900f57a118e78d7f09cb20a7cd9fd116
