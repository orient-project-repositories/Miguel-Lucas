#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sstream>

byte suc_1=0;
byte suc_2=0;
byte suc_3=0;

int newpos1;
int newpos2;
int newpos3;
int curspeed1;
int curspeed2;
int curspeed3;

int pos1;
int pos2;
int pos3;

int freq=10; //10 microseconds

int time;
int time1;
int time3=0;
int time4=0;

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
    
    time=0;
    time1=0;
  
    suc_1=Dxl.readByte(1, 46);
    suc_2=Dxl.readByte(2, 46);
    suc_3=Dxl.readByte(3, 46);  
    
    if((suc_1==0)&&(suc_2==0)&&(suc_3==0))
    {
      //SerialUSB.print("---------------------------------\n");
      
      char buffer[64]={0};
      char buf1[64]={0};
      char buf2[64]={0};
      char buf3[64]={0};
      
      le_buf_4num(buffer, buf1, buf2, buf3);      
      pos1=passa_num(buf1);
      pos2=passa_num(buf2);
      pos3=passa_num(buf3);
      
      newpos1=Dxl.readWord(1, 36);
      //curspeed1=Dxl.readWord(1, 38);
      newpos2=Dxl.readWord(2, 36);
      //curspeed2=Dxl.readWord(2, 38);
      newpos3=Dxl.readWord(3, 36);
      //curspeed3=Dxl.readWord(3, 38);
      
      time=micros();
      
      SerialUSB.print(newpos1);
      SerialUSB.print(" ");
      SerialUSB.print(newpos2);
      SerialUSB.print(" ");
      SerialUSB.print(newpos3);
      //SerialUSB.print(" ");
      //SerialUSB.print(curspeed1);
      //SerialUSB.print(" ");
      //SerialUSB.print(curspeed2);
      //SerialUSB.print(" ");
      //SerialUSB.print(curspeed3);
      SerialUSB.print("\n");
      
      Dxl.setPosition(1,pos1,1000); 
      Dxl.setPosition(2,pos2,1000);
      Dxl.setPosition(3,pos3,1000);    
    }
    
    time1=micros(); 
    int espera=(freq-(time1-time)/1000);
    //SerialUSB.print("Espera_1: ");
    //SerialUSB.println(espera);
    if (espera<-5)
      SerialUSB.println("Erro em 1");
    
    if (espera>0)
      delay (espera);
   
   int i=0;
    while(i<30)
    {
      time3=micros();
 
      newpos1=Dxl.readWord(1, 36);
      //curspeed1=Dxl.readWord(1, 38);
      newpos2=Dxl.readWord(2, 36);
      //curspeed2=Dxl.readWord(2, 38);
      newpos3=Dxl.readWord(3, 36);
      //curspeed3=Dxl.readWord(3, 38);
    
      SerialUSB.print(newpos1);
      SerialUSB.print(" ");
      SerialUSB.print(newpos2);
      SerialUSB.print(" ");
      SerialUSB.print(newpos3);
      //SerialUSB.print(" ");
      //SerialUSB.print(curspeed1);
      //SerialUSB.print(" ");
      //SerialUSB.print(curspeed2);
      //SerialUSB.print(" ");
      //SerialUSB.print(curspeed3);
      SerialUSB.print("\n");
      
      time4=micros();
      
      int espera=(freq-(time4-time3)/1000);
      //SerialUSB.print("  ");
      //SerialUSB.print( espera );
      //SerialUSB.print("  ");
      //SerialUSB.print("Espera: ");
      //SerialUSB.println(espera);
      //if (espera<-5){
        //SerialUSB.println("Erro em 2");
      //}
      if (espera>=0)
      delay (espera);
      i=i+1;
    }
}
    //int tt4=micros();
    
    //int tt5=micros();
    //int bloc3=(tt5-tt4)/1000;
    //SerialUSB.print("bloc3=");
    //SerialUSB.println(bloc3);

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
