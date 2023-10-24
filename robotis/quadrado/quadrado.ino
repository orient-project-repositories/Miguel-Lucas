#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sstream>

byte suc_1=0;
byte suc_2=0;
byte suc_3=0;
int pos1=512;
int pos2=512;
int pos3=512;

void setup() {
    // Initialize the dynamixel bus:
    //Dxl.begin(1); //16=115200 bps  34 = 57600 bps
    //Dxl.writeByte(1, 4, 3); // 34 = 57600 bps
    //Dxl.writeByte(2, 4, 3); // 34 = 57600 bps
    //Dxl.writeByte(3, 4, 3); // 34 = 57600 bps
    //Dxl.writeByte(4, 4, 3); // 34 = 57600 bps
    Dxl.begin(3); // Initializes to the changed Baud rate 200000 bps
    //Initialize serialUSB to print out log
    SerialUSB.begin();
    
    Dxl.writeWord(1, 8, 1024); //limite do ponteiro (0~90 graus)
    Dxl.writeWord(2, 8, 800);
    Dxl.writeWord(3, 8, 1024);
    Dxl.writeWord(4, 8, 2048);

    Dxl.setPosition(1,512,500); //por o ponteiro a 45 graus (vertical)
    Dxl.setPosition(2,512,500); 
    Dxl.setPosition(3,512,500);
    Dxl.setPosition(4,700,500);
    delay(1000); //espera 1 seguntos   
}

void loop() {  
  
   //delay(500);
    //Dxl.setPosition(1,512,500); 
    //Dxl.setPosition(2,512,500);
    //Dxl.setPosition(3,512,500);
  
    suc_1=Dxl.readByte(1, 46);
    suc_2=Dxl.readByte(2, 46);
    suc_3=Dxl.readByte(3, 46);
    
    if((suc_1==0)&&(suc_2==0)&&(suc_3==0)){
      delay(1500); //meio segundo à frente do Visual Studio
      
      if (pos1 == 512 && pos2 == 512)
	{
    	  pos1 = 462; pos2 = 512; pos3 = 462;
	}
      else if (pos1 == 462 && pos2 == 512)
	{
       	  pos1 = 612; pos2 = 612; pos3 = 412;
	}
      else if (pos1 == 612 && pos2 == 612)
	{
       	  pos1 = 612; pos2 = 412; pos3 = 412;
	}
      else if (pos1 = 612 && pos2 == 412)
	{
  	  pos1 = 512; pos2 = 512; pos3 = 512;
	}
      
      Dxl.setPosition(1,pos1,1000); 
      Dxl.setPosition(2,pos2,1000);
      Dxl.setPosition(3,pos3,1000); 
      
      delay(1500);
    }
}
 

