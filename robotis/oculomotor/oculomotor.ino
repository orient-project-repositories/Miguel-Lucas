#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sstream>

byte suc_1=0;
byte suc_2=0;
byte suc_3=0;
int pos1;
int pos2;
int pos3;
int newpos1;
int newpos2;
int newpos3;

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
  
   delay(500);
    Dxl.setPosition(1,512,500); 
    Dxl.setPosition(2,512,500);
    Dxl.setPosition(3,512,500);
    
    delay(1500);
    
    char buffer[64]={0};
    char buf1[64]={0};
    char buf2[64]={0};
    char buf3[64]={0};
  
    suc_1=Dxl.readByte(1, 46);
    suc_2=Dxl.readByte(2, 46);
    suc_3=Dxl.readByte(3, 46);
    
    if((suc_1==0)&&(suc_2==0)&&(suc_3==0)){
      delay(1500); //meio segundo à frente do Visual Studio
      
      le_buf_4num(buffer, buf1, buf2, buf3);      
      pos1=passa_num(buf1);
      pos2=passa_num(buf2);
      pos3=passa_num(buf3);
      
      Dxl.setPosition(1,pos1,500); 
      Dxl.setPosition(2,pos2,500);
      Dxl.setPosition(3,pos3,500);
      
      while(1){
        
        suc_1=Dxl.readByte(1, 46);
        suc_2=Dxl.readByte(2, 46);
        suc_3=Dxl.readByte(3, 46);
        
        if((suc_1==0)&&(suc_2==0)&&(suc_3==0)){
          delay(1000);
          newpos1=Dxl.readWord(1, 36);
          newpos2=Dxl.readWord(2, 36);
          newpos3=Dxl.readWord(3, 36);
         
          SerialUSB.print(newpos1);
          SerialUSB.print(" ");
          SerialUSB.print(newpos2);
          SerialUSB.print(" ");
          SerialUSB.print(newpos3);
          SerialUSB.print("\n");
          
          break;
        }
      }    
    }
}

void le_buf_4num(char *buffer, char *buffer1, char *buffer2, char *buffer3)
{
  int i=0;  
  //SerialUSB.print("--->AQUI ");
  //SerialUSB.print("\n");
  do {
        //delay(100);
        buffer[i]=SerialUSB.read();
        i++;
        //SerialUSB.print(" buffer: ");
        //SerialUSB.println(buffer);
      }while(i<12);
      //SerialUSB.print("buffer: ");
      //SerialUSB.println(buffer);
      
  i=0;    
  do {
        buffer1[i]=buffer[i];
        i++;
      }while(i<4);
  do {        
        buffer2[i-4]=buffer[i];
        i++;
      }while(i<8);
  do {
        buffer3[i-8]=buffer[i];
        i++;
      }while(i<12);
}

int passa_num(char *buffer)
{
  int aux=0, a=0;
  for(a; a<(4); a++)
  {
    aux=int((buffer[a]-int('0')))*expoente(a, 4)+aux;
  }
  return aux;
}

int expoente(int a, int i)
{
  int aux=1;
  for(a; a<(i-1); a++)
  {
    aux=aux*10;
  }
  
 return aux;
}

