//Automatic Ice pack program for the arduino mega 2560 by Bruno Diaz
//first draft

//all the libraries needed
//for barometer
//for i2C communication
#include <Wire.h>
//othe libraries
#include <stdio.h>
#include <math.h>

//for liquid display
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 50, 51, 52, 53);

//value for debuggin timing purposes
int count=0;



//intergers for timing the time for ice pack to be on and off the skin
int onTime;
int offTime;

//duty cycle for the on motor
int duty;

//values for pressure and tempature
double Pressure[2][3];
//for tempature
double Temp[2][3];
//first array
//0: first reading
//1: second reafing
//second array
//0: pressure on barometer 0
//1: pressure on barometer 0
//2: pressure on barometer 0

//the temparture on the icepack or person.
//0=ice pack
//1=person
float Temperature[2];


//the various sensor statuses
char stat[3];

//valures for bmp180 sensor outputs
 char status;
  double T,P;
//for timing
double time[3];
//0 for long time between stages
//1 for time delays of sensors
//2 for time between threshsolds

//target for the baldders
double target_p;
//for fuzzy logic
double overshoot=.05;
//double undershoot=1;


//defintions
#define BMP180_ADDR 0x77 // 7-bit address

#define    BMP180_REG_CONTROL 0xF4
#define    BMP180_REG_RESULT 0xF6

#define    BMP180_COMMAND_TEMPERATURE 0x2E
#define    BMP180_COMMAND_PRESSURE0 0x34
#define    BMP180_COMMAND_PRESSURE1 0x74
#define    BMP180_COMMAND_PRESSURE2 0xB4
#define    BMP180_COMMAND_PRESSURE3 0xF4

//sensor calibration data
int AC1[3],AC2[3],AC3[3],VB1[3],VB2[3],MB[3],MC[3],MD[3];
unsigned int AC4[3],AC5[3],AC6[3];
double c5[3],c6[3],mc[3],md[3],x0[3],x1[3],x2[3],y0[3],y1[3],y2[3],p0[3],p1[3],p2[3];
char _error;

boolean left,right,col_1,col_2,col_3;
boolean selected;
double on_time;
double off_time;
double min_time=60000;
double max_time=5940000;
int selection;
double pressure_increment=5;
double pressure_min=0;
double pressure_max=15000;
double ice_temp;
double body_temp;
double deflated=2;

// the setup routine runs once when you press reset:
void setup() {
  Serial.println("begin");
  
   lcd.begin(20, 4);
  col_1=false;
  col_2=false;
  col_3=false; 
  left=false;
  right=false;
  selection=1;
  target_p=0;

  

  
   //set up serial communictions
   Serial.begin(9600);
  Serial.println("REBOOT");
  // set up the LCD's number of columns and rows:
   //pins for the multiplexer
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  //pins for the solanoid valves
  //pinMode(24, OUTPUT);
  //pinMode(25, OUTPUT);
 
  //buttons
  pinMode(44, OUTPUT);
   //pinMode(45, OUTPUT);
   pinMode(46, OUTPUT);
    //pinMode(47, OUTPUT);
 
 //48 49pins
 //keep the valves off
 digitalWrite(45,LOW);
 //47 black
 digitalWrite(44,LOW);
 //keep the pumps off
 analogWrite(7,0);
 analogWrite(6,0);
 
 
 Serial.println("before init");
  //initializing the sensors
  SensorInit();
  Serial.println("after init");
  //inflateToTwo(255,0);
  //deflateToOne();
 
    //lcd.setCursor(0,0);
    //lcd.print("begin");
    setup_lcd();
    target_p=0;
    on_time=180000;
    off_time=180000;
}

// the loop routine runs over and over again forever:
//valves are 44 and 46
void loop() {
  
    //lcd.clear();
    //setup_lcd();
   // while(1)
    //{
      //tempsensor();
      //button_check();
      //command_check();
      //update_lcd();
      //ReadPoll(0);
      //delay(100);
      //if(B_1()>(target_p+overshoot)){pump(6,0);}
      //if(B_1()<(target_p-overshoot+overshoot)){pump(6,40);}
    //}
    

    
    
    
    while(target_p==0)
    {
      ReadPoll(0);
      digitalWrite(44,HIGH);
      digitalWrite(46,HIGH);
    }
     digitalWrite(44,LOW);
      digitalWrite(46,LOW);
       // pump(6,50);
    //delay(5000);
    //pump(6,0);
   // while(1)
   // {digitalWrite(44,HIGH);
    //}
    
    //start up starts on the off stage
        while(time[0]<off_time)
    {
      ReadPoll(0);
      //Tdelay(100);
      if(B_1()>(target_p+overshoot)){pump(6,0);/*digitalWrite(44,HIGH);*/}
      if(B_1()<(target_p-overshoot)){pump(6,40);/*digitalWrite(44,LOW);*/}
    }
    pump(6,0);
    digitalWrite(44,LOW);
    time[0]=0;
    int lvl;
    
    
    while(1)
    {
      lvl=0;
      //transfers to on stage
      digitalWrite(44,HIGH);
      ReadPoll(0);
      while(B_1()>deflated)
      {
        LL("a");
      ReadPoll(0);
      lvl=lvl+fuzzy_pump_adjust(.025,diff());
      if(lvl>40){lvl=40;}
      if(lvl<0){lvl=0;}
      pump(7,lvl);
      }
      LL("b");
      while(B_2()<(target_p+overshoot))
      {
        LL("c");
        pump(7,20);
      ReadPoll(0);
      }
       digitalWrite(44,LOW);
       pump(7,0);
      
      //holds the ice on
              while(time[0]<on_time)
    {
  LL("d");
      ReadPoll(0);
      //Tdelay(100);
      if(B_2()>(target_p+overshoot)){pump(7,0);/*digitalWrite(46,HIGH);*/}
      if(B_2()<(target_p-overshoot+overshoot)){pump(7,40);/*digitalWrite(46,LOW);*/}
    }
    digitalWrite(46,LOW);
    time[0]=0;
    //
    //
    //
      //transfers to off stage
      lvl=0;
            digitalWrite(46,HIGH);
      while(B_2()>deflated)
      {
        LL("e");
      ReadPoll(0);
      lvl=lvl+fuzzy_pump_adjust(.025,diff());
      if(lvl>40){lvl=40;}
      if(lvl<1){lvl=0;}
      pump(6,lvl);
      }
      while(B_1()<(target_p+overshoot))
      {
        LL("f");
        pump(6,20);
      ReadPoll(0);
      }
       digitalWrite(46,LOW);
       pump(6,0);
      
      ///
      ///
      ///holds the ice off
              while(time[0]<off_time)
    {
      LL("g");
      ReadPoll(0);
      //Tdelay(100);
      if(B_1()>(target_p+overshoot)){pump(6,0);/*digitalWrite(44,HIGH);*/}
      if(B_1()<(target_p-overshoot+overshoot)){/*pump(6,40);digitalWrite(44,LOW);*/}
    }
    time[0]=0;
      digitalWrite(44,LOW);
      
    }
    

 
  
}
 ///end 
  


//start the tempature readings
//returns max time for the sensors to complete calculation in ms
int StartT()
{
   multiplexer(0);
  status = startTemperature();
  stat[0]=status;
 
  multiplexer(1);
  status = startTemperature();
  stat[1]=status;
 
  multiplexer(2);
  status = startTemperature();
  stat[2]=status;
 
  if ((stat[0] == 0)||(stat[1] == 0)||(stat[2] == 0))
  {
  Serial.println("error starting temperature measurement\n");
  Serial.println(WhichStatusError(stat[0],stat[1],stat[2]));
  }
 
  int MaxStatus=0+max(stat[0], max(stat[1],stat[2]));
  return MaxStatus;
}

//starts the pressure readings
//returns the time for the sensors to complete calculations in ms
int StartP()
{
      multiplexer(0);
      status = startPressure(3);
      stat[0]=status;
     
      multiplexer(1);
      status = startPressure(3);
      stat[1]=status;
     
      multiplexer(2);
      status = startPressure(3);
      stat[2]=status;
     
     
      if ((stat[0] == 0)||(stat[1] == 0)||(stat[2] == 0))
      {
        Serial.println("error starting pressure measurement\n");
      Serial.println(WhichStatusError(stat[0],stat[1],stat[2]));
      }
     
     
    int MaxStatus=0+max(stat[0], max(stat[1],stat[2]));
    return MaxStatus;
}

//get the readings for tempature and stores them in either the A or B values depending
//on the whatstage parameter
int GetT(int Whatstage)
{
  //holding values
double T0H=0;
double T1H=0;
double T2H=0;



  //move through the multiplexer to get the values from all the sensors
      multiplexer(0);
    status = getTemperature(T,0);
    T0H=T;
    stat[0]=status;
   
    multiplexer(1);
    status = getTemperature(T,1);
    T1H=T;
    stat[1]=status;
   
    multiplexer(2);
    status = getTemperature(T,2);
    T2H=T;
    stat[2]=status;
 
  //if the statuses all return something
   if ((stat[0] != 0)||(stat[1] != 0)||(stat[2] != 0))
  //transfer holding values to aproproate stage values
  {

    Temp[Whatstage][0]=T0H;
    Temp[Whatstage][1]=T1H;
    Temp[Whatstage][2]=T2H;

}
 else{ Serial.println("error retrieving temperature measurement\n");
    Serial.println(WhichStatusError(stat[0],stat[1],stat[2]));}
    int MaxStatus=0+max(stat[0], max(stat[1],stat[2]));
    return MaxStatus;
   
   
}
int GetP(int Whatstage)
{
        //holding values
        double PressureH[3];
       
        if(Whatstage<2)
        {
        multiplexer(0);
        stat[0] = getPressure(P,Temp[Whatstage][0],0);
        PressureH[0]=P;
       
        multiplexer(1);
        stat[1] = getPressure(P,Temp[Whatstage][1],1);
        PressureH[1]=P;
       
        multiplexer(2);
        stat[2] = getPressure(P,Temp[Whatstage][2],2);
        PressureH[2]=P;
        }
       
        if ((stat[0] != 0)||(stat[1] != 0)||(stat[2] != 0))
        {
            Pressure[Whatstage][0]=PressureH[0];
            Pressure[Whatstage][1]=PressureH[1];
            Pressure[Whatstage][2]=PressureH[2];
        }
        else{ Serial.println("error retrieving pressure measurement\n");
        Serial.println(WhichStatusError(stat[0],stat[1],stat[2]));}
        int MaxStatus=0+max(stat[0], max(stat[1],stat[2]));
    return MaxStatus;
         
}
//assing which ouput is open on the multiplexer
///2 is now the same as 3 due to debugging
void multiplexer(int output)
{
  switch(output)
    {
      case 0:
      digitalWrite(30,LOW);
      digitalWrite(31,LOW);
      digitalWrite(32,LOW);
      digitalWrite(33,LOW);
      break;
     
      case 1:
      digitalWrite(30,HIGH);
      digitalWrite(31,LOW);
      digitalWrite(32,LOW);
      digitalWrite(33,LOW);
      break;
     
     //case 2:
      //digitalWrite(30,HIGH);
      //digitalWrite(31,LOW);
      //digitalWrite(32,LOW);
      //digitalWrite(33,LOW);
      break;
      
            case 2:
      digitalWrite(30,HIGH);
      digitalWrite(31,HIGH);
      digitalWrite(32,LOW);
      digitalWrite(33,LOW);
      break;
     
      case 3:
      digitalWrite(30,HIGH);
      digitalWrite(31,HIGH);
      digitalWrite(32,LOW);
      digitalWrite(33,LOW);
      break;
     
      case 4:
      digitalWrite(30,LOW);
      digitalWrite(31,LOW);
      digitalWrite(32,HIGH);
      digitalWrite(33,LOW);
      break;
     
      case 5:
      digitalWrite(30,HIGH);
      digitalWrite(31,LOW);
      digitalWrite(32,HIGH);
      digitalWrite(33,LOW);
      break;
     
      case 6:
      digitalWrite(30,LOW);
      digitalWrite(31,HIGH);
      digitalWrite(32,HIGH);
      digitalWrite(33,LOW);
      break;
     
      case 7:
      digitalWrite(30,HIGH);
      digitalWrite(31,HIGH);
      digitalWrite(32,HIGH);
      digitalWrite(33,LOW);
      break;
     
      case 8:
      digitalWrite(30,LOW);
      digitalWrite(31,LOW);
      digitalWrite(32,LOW);
      digitalWrite(33,HIGH);
      break;
     
      case 9:
      digitalWrite(30,HIGH);
      digitalWrite(31,LOW);
      digitalWrite(32,LOW);
      digitalWrite(33,HIGH);
      break;
     
      case 10:
      digitalWrite(30,LOW);
      digitalWrite(31,HIGH);
      digitalWrite(32,LOW);
      digitalWrite(33,HIGH);
      break;
     
      case 11:
      digitalWrite(30,HIGH);
      digitalWrite(31,HIGH);
      digitalWrite(32,LOW);
      digitalWrite(33,HIGH);
      break;
     
      case 12:
      digitalWrite(30,LOW);
      digitalWrite(31,LOW);
      digitalWrite(32,HIGH);
      digitalWrite(33,HIGH);
      break;
     
      case 13:
      digitalWrite(30,HIGH);
      digitalWrite(31,LOW);
      digitalWrite(32,HIGH);
      digitalWrite(33,HIGH);
      break;
     
      case 14:
      digitalWrite(30,LOW);
      digitalWrite(31,HIGH);
      digitalWrite(32,HIGH);
      digitalWrite(33,HIGH);
      break;
     
      case 15:
      digitalWrite(30,HIGH);
      digitalWrite(31,HIGH);
      digitalWrite(32,HIGH);
      digitalWrite(33,HIGH);
      break;
     
    }
}

//initiates the sensors
void SensorInit()
{
      for (int i=0; i < 3; i++)
      {
  multiplexer(i);
     if (begin(i)){
    Serial.println("BMP180 init success for sensor ");
    Serial.println(i);
      }
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail\n\n for sensor ");
     Serial.println(i);
    while(1); // Pause forever.
  }
      }
}

void CommandSend(int one,int two,int three)
{
}
 void CommandCheck()
{
}
String WhichStatusError(char sta0,char sta1,char sta2)
{
   String sta="error  ";
 
  if((sta0==0)&&(sta1==0)&&(sta2==0))
  {
    sta=" from all sensors ";
  }
  if((stat[0]==0)&&(stat[1] != 0)&&(stat[2] != 0))
  {
    sta=" from sensor 0 ";
  }
  if((stat[1]==0)&&(stat[0] != 0)&&(stat[2] != 0))
  {
    sta=" from sensor 1";
  }
  if((stat[2]==0)&&(stat[0] != 0)&&(stat[1] != 0))
  {
    sta=" from sensor 2 ";
  }
  if((stat[0]==0)&&(stat[1]==0)&&(stat[2] != 0))
  {
    sta=" from sensors 0 and 1";
  }
  if((stat[0] != 0)&&(stat[1]==0)&&(stat[2]==0))
  {
    sta=" from sensors 1 and 2" ;
  }
  if((stat[0]==0)&&(stat[1] != 0)&&(stat[2]==0))
  {
    sta="from sensors 0 and 2";
  }
 
  return sta;
}
//what to do when a button is pressed
void ButtonPress()
{
}
//what to to display the user interface
void DisplayUI()
{
}

//read out the tempature and pressure while polling
// for user or program commands
//readpoll is here
void ReadPoll(int w)
{
 int del=0;
  // for tempature
  time[1]=0;
  int staa=0;
  while(staa==0)
  {
    staa=StartT();
  }
  //time[0]=staa;
  del=staa;
  while(time[1]<del)
  {
    button_check();
    command_check();
    phone_check();
    command_check();
    Tdelay(1);
  }
  staa=0;
  while(staa==0)
  {
    staa=GetT(w);
  }
  del=staa;
  time[1]=0;
  while(time[1]<del)
  {
    button_check();
    command_check();
    phone_check();
    command_check();
    Tdelay(1);
  }
 
 
  //for pressure
    
   staa=0;
  while(staa==0)
  {
    staa=StartP();
  }
  time[1]=0;
  del=staa;
  while(time[1]<del)
  {
    button_check();
    command_check();
    phone_check();
    command_check();
    Tdelay(1);
  }
  staa=0;
  while(staa==0)
  {
    staa=GetP(w);
  }
  time[1]=0;
  del=staa;
  while(time[1]<del)
  {
    button_check();
    command_check();
    phone_check();
    command_check();
    Tdelay(1);
  }
  
  
      tempsensor();
      update_lcd();
      time_adjust(43);
      time[1]=0;
}
//functions for digital barometer
//
//
char begin(int N)
// Initialize library for subsequent pressure measurements
{
    double c3[2],c4[2],b1[2];
   
    // Start up the Arduino's "wire" (I2C) library:
   
    Wire.begin();

    // The BMP180 includes factory calibration data stored on the device.
    // Each device has different numbers, these must be retrieved and
    // used in the calculations when taking pressure measurements.

    // Retrieve calibration data from device:
   
    if (readInt(0xAA,AC1[N]) &&
        readInt(0xAC,AC2[N]) &&
        readInt(0xAE,AC3[N]) &&
        readUInt(0xB0,AC4[N]) &&
        readUInt(0xB2,AC5[N]) &&
        readUInt(0xB4,AC6[N]) &&
        readInt(0xB6,VB1[N]) &&
        readInt(0xB8,VB2[N]) &&
        readInt(0xBA,MB[N]) &&
        readInt(0xBC,MC[N]) &&
        readInt(0xBE,MD[N]))
    {

        // All reads completed successfully
       
        // Compute floating-point polynominals:

        c3[N] = 160.0 * pow(2,-15) * AC3[N];
        c4[N] = pow(10,-3) * pow(2,-15) * AC4[N];
        b1[N] = pow(160,2) * pow(2,-30) * VB1[N];
        c5[N] = (pow(2,-15) / 160) * AC5[N];
        c6[N] = AC6[N];
        mc[N] = (pow(2,11) / pow(160,2)) * MC[N];
        md[N] = MD[N] / 160.0;
        x0[N] = AC1[N];
        x1[N] = 160.0 * pow(2,-13) * AC2[N];
        x2[N] = pow(160,2) * pow(2,-25) * VB2[N];
        y0[N] = c4[N] * pow(2,15);
        y1[N] = c4[N] * c3[N];
        y2[N] = c4[N] * b1[N];
        p0[N] = (3791.0 - 8.0) / 1600.0;
        p1[N] = 1.0 - 7357.0 * pow(2,-20);
        p2[N] = 3038.0 * 100.0 * pow(2,-36);

   
        // Success!
        return(1);
    }
    else
    {
        // Error reading calibration data; bad component or connection?
        return(0);
    }
}


char readInt(char address, int &value)
// Read a signed integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
    unsigned char data[2];

    data[0] = address;
    if (readBytes(data,2))
    {
        value = (((int)data[0]<<8)|(int)data[1]);
        //if (*value & 0x8000) *value |= 0xFFFF0000; // sign extend if negative
        return(1);
    }
    value = 0;
    return(0);
}


char readUInt(char address, unsigned int &value)
// Read an unsigned integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
    unsigned char data[2];

    data[0] = address;
    if (readBytes(data,2))
    {
        value = (((unsigned int)data[0]<<8)|(unsigned int)data[1]);
        return(1);
    }
    value = 0;
    return(0);
}


char readBytes(unsigned char *values, char length)
// Read an array of bytes from device
// values: external array to hold data. Put starting register in values[0].
// length: number of bytes to read
{
    char x;

    Wire.beginTransmission(BMP180_ADDR);
    Wire.write(values[0]);
    _error = Wire.endTransmission();
    if (_error == 0)
    {
        Wire.requestFrom(BMP180_ADDR,length);
        while(Wire.available() != length) ; // wait until bytes are ready
        for(x=0;x<length;x++)
        {
            values[x] = Wire.read();
        }
        return(1);
    }
    return(0);
}


char writeBytes(unsigned char *values, char length)
// Write an array of bytes to device
// values: external array of data to write. Put starting register in values[0].
// length: number of bytes to write
{
    char x;
   
    Wire.beginTransmission(BMP180_ADDR);
    Wire.write(values,length);
    _error = Wire.endTransmission();
    if (_error == 0)
        return(1);
    else
        return(0);
}

char startTemperature(void)
// Begin a temperature reading.
// Will return delay in ms to wait, or 0 if I2C error
{
    unsigned char data[2], result;
   
    data[0] = BMP180_REG_CONTROL;
    data[1] = BMP180_COMMAND_TEMPERATURE;
    result = writeBytes(data, 2);
    if (result) // good write?
        return(5); // return the delay in ms (rounded up) to wait before retrieving data
    else
        return(0); // or return 0 if there was a problem communicating with the BMP
}


char getTemperature(double &T, int N)
// Retrieve a previously-started temperature reading.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startTemperature() to have been called prior and sufficient time elapsed.
// T: external variable to hold result.
// Returns 1 if successful, 0 if I2C error.
{
    unsigned char data[2];
    char result;
    double tu, a;
   
    data[0] = BMP180_REG_RESULT;

    result = readBytes(data, 2);
    if (result) // good read, calculate temperature
    {
        tu = (data[0] * 256.0) + data[1];

        //example from Bosch datasheet
        //tu = 27898;

        //example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
        //tu = 0x69EC;
       
        a = c5[N] * (tu - c6[N]);
        T = a + (mc[N] / (a + md[N]));

        /*       
        Serial.println();
        Serial.print("tu: "); Serial.println(tu);
        Serial.print("a: "); Serial.println(a);
        Serial.print("T: "); Serial.println(*T);
        */
    }
    return(result);
}


char startPressure(char oversampling)
// Begin a pressure reading.
// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
// Will return delay in ms to wait, or 0 if I2C error.
{
    unsigned char data[2], result, delay;
   
    data[0] = BMP180_REG_CONTROL;

    switch (oversampling)
    {
        case 0:
            data[1] = BMP180_COMMAND_PRESSURE0;
            delay = 5;
        break;
        case 1:
            data[1] = BMP180_COMMAND_PRESSURE1;
            delay = 8;
        break;
        case 2:
            data[1] = BMP180_COMMAND_PRESSURE2;
            delay = 14;
        break;
        case 3:
            data[1] = BMP180_COMMAND_PRESSURE3;
            delay = 26;
        break;
        default:
            data[1] = BMP180_COMMAND_PRESSURE0;
            delay = 5;
        break;
    }
    result = writeBytes(data, 2);
    if (result) // good write?
        return(delay); // return the delay in ms (rounded up) to wait before retrieving data
    else
        return(0); // or return 0 if there was a problem communicating with the BMP
}


char getPressure(double &P, double &T,int N)
// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startPressure() to have been called prior and sufficient time elapsed.
// Requires recent temperature reading to accurately calculate pressure.

// P: external variable to hold pressure.
// T: previously-calculated temperature.
// Returns 1 for success, 0 for I2C error.

// Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().
{
    unsigned char data[3];
    char result;
    double pu,s,x,y,z;
   
    data[0] = BMP180_REG_RESULT;

    result = readBytes(data, 3);
    if (result) // good read, calculate pressure
    {
        pu = (data[0] * 256.0) + data[1] + (data[2]/256.0);

        //example from Bosch datasheet
        //pu = 23843;

        //example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf, pu = 0x982FC0;   
        //pu = (0x98 * 256.0) + 0x2F + (0xC0/256.0);
       
        s = T - 25.0;
        x = (x2[N] * pow(s,2)) + (x1[N] * s) + x0[N];
        y = (y2[N] * pow(s,2)) + (y1[N] * s) + y0[N];
        z = (pu - x) / y;
        P = (p2[N] * pow(z,2)) + (p1[N] * z) + p0[N];

        /*
        Serial.println();
        */
    }
    return(result);
}
char getError(void)
    // If any library command fails, you can retrieve an extended
    // error code using this command. Errors are from the wire library:
    // 0 = Success
    // 1 = Data too long to fit in transmit buffer
    // 2 = Received NACK on transmit of address
    // 3 = Received NACK on transmit of data
    // 4 = Other error
{
    return(_error);
}
//debugging function to check all the calibration data and send it through the serial port
void checkCalibration(int N)
{
Serial.print(AC1[N]);
Serial.print(" ");
Serial.print(AC2[N]);
Serial.print(" ");
Serial.print(AC3[N]);
Serial.print(" ");
Serial.print(VB1[N]);
Serial.print(" ");
Serial.print(VB2[N]);
Serial.print(" ");
Serial.print(MB[N]);
Serial.print(" ");
Serial.print(MC[N]);
Serial.print(" ");
Serial.print(MD[N]);
Serial.print(" ");
Serial.print(AC4[N]);
Serial.print(" ");
Serial.print(AC5[N]);
Serial.print(" ");
Serial.print(AC6[N]);
Serial.print(" ");
Serial.print(c5[N]);
Serial.print(" ");
Serial.print(c6[N]);
Serial.print(" ");
Serial.print(mc[N]);
Serial.print(" ");
Serial.print(md[N]);
Serial.print(" ");
Serial.print(x0[N]);
Serial.print(" ");
Serial.print(x1[N]);
Serial.print(" ");
Serial.print(x2[N]);
Serial.print(" ");
Serial.print(y0[N]);
Serial.print(" ");
Serial.print(y1[N]);
Serial.print(" ");
Serial.print(y2[N]);
Serial.print(" ");
Serial.print(p0[N]);
Serial.print(" ");
Serial.print(p1[N]);
Serial.print(" ");
Serial.print(p2[N]);
}
//timing delay
void Tdelay(int de)
{
  delay(de);
  for(int i=0;i<3;i++)
  {
    time[i]=time[i]+de;
  }
  return;
}
//for testing pump rates
void deflateToOne()
{
  ReadPoll(0);
    analogWrite(7,0);
    digitalWrite(24,HIGH);
  while((Pressure[0][1]-Pressure[0][0])>1)
  {
    ReadPoll(0);
    lcd.setCursor(0,0);
    lcd.println((Pressure[0][1]-Pressure[0][0]));
    lcd.setCursor(8,8);
    lcd.println("deflating");
  }
  digitalWrite(24,LOW);
  return;
}
double inflateToTwo(int du, int run)
{
  time[2]=0;
  double took;
  ReadPoll(0);
    analogWrite(7,du);
    digitalWrite(24,LOW);
  while((Pressure[0][1]-Pressure[0][0])<6)
  {
    ReadPoll(0);
    lcd.setCursor(0,0);
    lcd.println((Pressure[0][1]-Pressure[0][0]));
    lcd.setCursor(5,1);
    lcd.println("testing");
    lcd.setCursor(0,1);
    lcd.println(du);
    lcd.setCursor(13,1);
    lcd.println(run);
    lcd.setCursor(8,0);
    lcd.println(time[2]);
  }
  analogWrite(7,0);
  took=time[2];
  time[2]=0;
  return took;
}
int pump(int pump_n ,int level)
{
 int arduino_duty;
switch(level) 
{
  case 0:arduino_duty=0;
  break;
  case 1:arduino_duty=36;
  break;
  case 2:arduino_duty=36;
  break;
  case 3:arduino_duty=37;
  break;
  case 4:arduino_duty=38;
  break;
  case 5:arduino_duty=39;
  break;
  case 6:arduino_duty=41;
  break;
  case 7:arduino_duty=42;
  break;
  case 8:arduino_duty=42;
  break;
  case 9:arduino_duty=42;
  break;
  case 10:arduino_duty=43;
  break;
  case 11:arduino_duty=43;
  break;
  case 12:arduino_duty=43;
  break;
  case 13:arduino_duty=44;
  break;
  case 14:arduino_duty=44;
  break;
  case 15:arduino_duty=45;
  break;
  case 16:arduino_duty=45;
  break;
  case 17:arduino_duty=47;
  break;
  case 18:arduino_duty=49;
  break;
  case 19:arduino_duty=51;
  break;
  case 20:arduino_duty=53;
  break;
  case 21:arduino_duty=55;
  break;
  case 22:arduino_duty=57;
  break;
  case 23:arduino_duty=61;
  break;
  case 24:arduino_duty=63;
  break;
  case 25:arduino_duty=65;
  break;
  case 26:arduino_duty=69;
  break;
  case 27:arduino_duty=71;
  break;
  case 28:arduino_duty=75;
  break;
  case 29:arduino_duty=79;
  break;
  case 30:arduino_duty=83;
  break;
  case 31:arduino_duty=87;
  break;
  case 32:arduino_duty=93;
  break;
  case 33:arduino_duty=97;
  break;
  case 34:arduino_duty=101;
  break;
  case 35:arduino_duty=109;
  break;
  case 36:arduino_duty=113;
  break;
  case 37:arduino_duty=121;
  break;
  case 38:arduino_duty=127;
  break;
  case 39:arduino_duty=131;
  break;
  case 40:arduino_duty=137;
  break;
  case 41:arduino_duty=145;
  break;
  case 42:arduino_duty=153;
  break;
  case 43:arduino_duty=165;
  break;
  case 44:arduino_duty=173;
  break;
  case 45:arduino_duty=195;
  break;
  case 46:arduino_duty=209;
  break;
  case 47:arduino_duty=227;
  break;
  case 48:arduino_duty=231;
  break;
  case 49:arduino_duty=241;
  break;
  case 50:arduino_duty=255;
}
if(level>50)
{
  arduino_duty=255;
}
analogWrite(pump_n,arduino_duty);
}
int pump_adjustA(double diff_signed, int time_elapsed)
{
  int back=0;
  double adj=1;
  double diff=abs(diff_signed);
   double rate=diff/time_elapsed;
   int sign;
   if(diff_signed>0)
   {
     sign=1;
   }
   if(diff_signed<0)
   {
     sign=-1;
   }
   
   // reduce region 0
   //originally 0.000178269559916532
   //region 0
   if(rate<0.00001)
   {
     back=0;
     return back;
   }
   //region 1
   if((rate>=(0.00001*adj))&&(diff<(0.00106313379129866*adj)))
   {
     back=1*sign;
     return back;
   }
   //region 2
   if((rate>=(0.00106313379129866*adj)&&(diff<(0.00227113906359189*adj))))
   {
     back=2*sign;
     return back;
   }
   //region 3
   if((rate>=(0.00227113906359189*adj))&&(diff<(0.00301764159702879*adj)))
   {
     back=3*sign;
     return back;
   }
   //region 4
   if((rate>=(0.00301764159702879*adj))&&(diff<=(0.00357535753575358*adj)))
   {
     back=4*sign;
     return back;
   }
   if(rate>(0.00357535753575358*adj))
   {
     back=5*sign;
     return back;
   }
   back=0;
   return back;
}
//takes the two readings from readpool(0) and (1) 
//to calculate diffrence in single bladder
//need to add int later
double single_diff()
{
  double diffr=((Pressure[1][0]-Pressure[1][1])-(Pressure[0][0]-Pressure[0][1]));
  return diffr; 
}
//same but diffrence between two bladders.
///change order maybey refrence single diff
double double_diff()
{
  
  double db_diff, first_reading,second_reading;
  
   first_reading=(Pressure[0][1]-Pressure[0][0])+(Pressure[0][2]-Pressure[0][0]);
   second_reading=(Pressure[1][1]-Pressure[1][0])+(Pressure[1][2]-Pressure[1][0]);
  
  
    //first_reading=(Pressure[0][0]-Pressure[0][1])+(Pressure[0][0]-Pressure[0][2]);
    //second_reading=(Pressure[1][0]-Pressure[1][1])+(Pressure[1][0]-Pressure[1][2]);
    //db_diff=(first_reading-second_reading);
    db_diff=second_reading-first_reading;
  return db_diff;
}

int fuzzy_pump_adjust(double threshold,double value)
{
  double sign;
  //sign=1;
  //return 1;
  if(value>0){sign=1;}
  if(value<0){sign=-1;}
  double val=abs(value);
  
int number=0;
int num;


if(val<(threshold*.125)){num=0;return num;}
if((val>=(threshold*.125))&&(val<(threshold*.25))){number=6;}
if((val>=(threshold*.25))&&(val<(threshold*.5))){number=12;}
if((val>=(threshold*.5))&&(val<(threshold))){number=25;}
if((val>=threshold)&&(val<(threshold*2))){number=50;}
if((val>=(threshold*2))&&(val<(threshold*4))){number=75;}
if((val>=(threshold*4))&&(val<(threshold*8))){num=1*sign;return num;}//2
if((val>=(threshold*8))&&(val<(threshold*16))){num=2*sign;return num;}//4
if((val>=(threshold*16))&&(val<(threshold*32))){num=2*sign;return num;}//8
if(val>=(threshold*32)){num=2*sign;return num;}//16

  
  int rand=random(0,101);
  if(rand<number)
  {
    num=1*sign;
    return num;
  }
  else
  {
    num=0 ;
    return num;
  }
}
double diff()
{
  double de=target_p-((B_1()+B_2()));
  return de;
  
}
void button_check()
{
  int toggle=50;
  if(analogRead(A0)>1000)
  {
    //col_1=true;
    toggle=0;
    Tdelay(1);
  }
   if(analogRead(A1)>1000)
  {
    //col_2=true;
    toggle=1;
    Tdelay(1);
  }
   if(analogRead(A2)>1000)
  {
    //col_3=true;
    toggle=2;
    Tdelay(1);
  }
   if(analogRead(A3)>1000)
  {
    //right=true;
    toggle=3;
    Tdelay(1);
  }
   if(analogRead(A4)>1000)
  {
    //left=true;
    toggle=4;
    Tdelay(1);
  }
  
  if(toggle!=50)
  {
    switch (toggle)
    {
      case 0:if(analogRead(A0)>1000){col_1=true;Tdelay(50);return;}
      break;
      case 1:if(analogRead(A1)>1000){col_2=true;Tdelay(50);return;}
      break;
      case 2:if(analogRead(A2)>1000){col_3=true;Tdelay(50);return;}
      break;
      case 3:if(analogRead(A3)>1000){left=true;Tdelay(50);return;}
      break;
      case 4:if(analogRead(A4)>1000){right=true;Tdelay(50);return;}
      break;
    }
  }
  
}
void phone_check()
{
}
void command_check()
{
  if(right==true)
  {
    switch (selection){
    case 3:on_time=on_time+(60000); if(on_time<min_time){on_time=min_time;}
    if(on_time>max_time){on_time=max_time;}
    break;
    case 2:off_time=off_time+(60000); if(off_time<min_time){off_time=min_time;}
    if(off_time>max_time){off_time=max_time;}
    break;
    case 1:target_p=target_p+pressure_increment;
    if(target_p<pressure_min){target_p=pressure_min;}
    if(target_p>pressure_max){target_p=pressure_max;}
    break;
    }
  }
  if(left==true)
  {
    switch (selection){
    case 3:on_time=on_time-(60000); if(on_time<min_time){on_time=min_time;}
    if(on_time>max_time){on_time=max_time;}
    break;
    case 2:off_time=off_time-(60000); if(off_time<min_time){off_time=min_time;}
    if(off_time>max_time){off_time=max_time;}
    break;
    case 1:target_p=target_p-pressure_increment;
    if(target_p<pressure_min){target_p=pressure_min;}
    if(target_p>pressure_max){target_p=pressure_max;}
    break;
    }
  }
  if(col_1==true){selection=1;}
  if(col_2==true){selection=2;}
  if(col_3==true){selection=3;}
  
  //booleans are reset
  col_1=false;
  col_2=false;
  col_3=false; 
  left=false;
  right=false;
}
//updates the lcd
void setup_lcd()
{
  lcd.clear();
  
  lcd.setCursor(0,0);
  lcd.print("ice:");
  
  lcd.setCursor(7,0);
  lcd.print("C");
  
  lcd.setCursor(9,0);
  lcd.print("body:");
  
  lcd.setCursor(17,0);
  lcd.print("C");
  
   lcd.setCursor(0,1);
  lcd.print("pressure:");
  
  lcd.setCursor(0,2);
  lcd.print("off time:");
  
  lcd.setCursor(13,2);
  lcd.print("minutes");
  
  lcd.setCursor(0,3);
  lcd.print("on time:");
  
  lcd.setCursor(13,3);
  lcd.print("minutes");
}
void update_lcd()
{
  //displays which is the selector
  switch(selection)
  {
    case 1:
    lcd.setCursor(9,1);lcd.print("<");lcd.setCursor(11,1);lcd.print(">");
    lcd.setCursor(9,2);lcd.print(" ");lcd.setCursor(11,2);lcd.print(" ");
    lcd.setCursor(9,3);lcd.print(" ");lcd.setCursor(11,3);lcd.print(" ");
    break;
    case 2:
    lcd.setCursor(9,1);lcd.print(" ");lcd.setCursor(11,1);lcd.print(" ");
    lcd.setCursor(9,2);lcd.print("<");lcd.setCursor(11,2);lcd.print(">");
    lcd.setCursor(9,3);lcd.print(" ");lcd.setCursor(11,3);lcd.print(" ");
    break;
    case 3:
    lcd.setCursor(9,1);lcd.print(" ");lcd.setCursor(11,1);lcd.print(" ");
    lcd.setCursor(9,2);lcd.print(" ");lcd.setCursor(11,2);lcd.print(" ");
    lcd.setCursor(9,3);lcd.print("<");lcd.setCursor(11,3);lcd.print(">");
    break;
    default: 
    lcd.setCursor(9,1);lcd.print("!");lcd.setCursor(11,1);lcd.print("!");
    lcd.setCursor(9,2);lcd.print("!");lcd.setCursor(11,2);lcd.print("!");
    lcd.setCursor(9,3);lcd.print("!");lcd.setCursor(11,3);lcd.print("!");
  }
  //lcd.setCursor(4,0);
  //lcd.print(Temperature[0],1);
  
   lcd.setCursor(0,0);
  lcd.print(time[0]/1000);
  
  //lcd.setCursor(14,0);
  //lcd.print(Temperature[0],1);
  
  lcd.setCursor(10,1);
  lcd.print(target_p,2);
  
  lcd.setCursor(10,2);
  lcd.print((off_time/60000),0);
  
  lcd.setCursor(10,3);
  lcd.print((on_time/60000),0);
  ///////////////////////////////////////
  lcd.setCursor(15,0);
  lcd.print(B_1(),2);
  
  lcd.setCursor(15,1);
  lcd.print(B_2(),2);
}
void tempsensor()
{

float i=analogRead(A5);
float p=analogRead(A6);

  
  float tempVolt_1 = i/ 204.6;
  float tempVolt_2 = p/ 204.6;
  
    Temperature[0]=(100 * tempVolt_1)-120.93;
  
    Temperature[1]= (100 * tempVolt_2) -120.93;
    
    return;
}

double B_1()
{
  double re=Pressure[0][1]-Pressure[0][0];
  return re;
}
double B_2()
{
  double re=Pressure[0][2]-Pressure[0][0];
  return re;
}
void time_adjust(double a)
{
    for(int i=0;i<3;i++)
  {
    time[i]=time[i]+a;
  }
}
void LL(String stuff)
{lcd.setCursor(13,3);
 lcd.print(" ");
 lcd.setCursor(14,3);
 lcd.print(stuff);
 lcd.setCursor(15,3);
 lcd.print(" ");
}
