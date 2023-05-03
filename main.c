#include "D:\jobESP\DC Voltage Monitoring Relay\firmware\DIN RAIL No Crytal\PIC24 CCS Project -24 UN\main.h"
#include <stdio.h>
#include <string.h>

#define UnderSet_addr 0x00
#define OverSet_addr 0x01
#define UnderResSet_addr 0x02
#define OverResSet_addr 0x03
#define UnderTimSet_addr 0x04
#define OverTimSet_addr 0x05
#define UnderResTimSet_addr 0x06
#define OverResTimSet_addr 0x07

int1 FlashLEDUnder = false;
int1 FlashLEDOver = false;

volatile unsigned int8 flash_LED_Under =0,flash_LED_Over =0;

volatile int8 toggle_pin_under =0;
volatile int8 toggle_pin_over =0;

// Update 18/5/63 Right
#define LED_Healty PIN_B5
#define LED_Under PIN_B4
#define LED_Over  PIN_B0

#define Over_Rly PIN_B7
#define Under_Rly PIN_B6

#define HIGH  1
#define LOW   0

//   for din rail
#define BT_DW PIN_B13
#define BT_UP PIN_B14
#define BT_SET PIN_B15

//   for surface mount
//#define BT_DW PIN_B14
//#define BT_UP PIN_B15
//#define BT_SET PIN_B13

#define EXP_OUT_DO      PIN_B12
#define EXP_OUT_CLOCK   PIN_B11
#define EXP_OUT_ENABLE  PIN_B10

#define EEPROM_SDA  PIN_B9
#define EEPROM_SCL  PIN_B8

//#use i2c(Master,slow,sda=PIN_B9,scl=PIN_B8, stream=MCP342X_STREAM, FORCE_HW)//
//#use i2c(Master, fast=450000,sda=PIN_B9,scl=PIN_B8, stream=MCP342X_STREAM, FORCE_SW)
//#use i2c(Master,slow,sda=PIN_B9,scl=PIN_B8, stream=MCP342X_STREAM)
//#use rs232(baud=9600,parity=N,xmit=PIN_B4,rcv=PIN_B5,bits=8)
//#use i2c(Master,sda=PIN_B9,scl=PIN_B8, stream=MCP342X_STREAM,restart_wdt)

#define NORMAL_VOLT 24
#define MAX_VOLT 35
#define MIN_VOLT 15
#define MAX_TIME 60
#define MIN_TIME 0

volatile float DC_V;
volatile float DC_V7seg;
float DC_Average[10];
boolean set_mode = false;
volatile unsigned int32 menuCount;
volatile signed char flash_dot,flash_LED;

volatile unsigned int8 DebugLoopCount =0;

volatile unsigned int8 HangCount =0,oldloop =0,Updateloop =1;

volatile unsigned int8 ReadI2CCount =0,I2C_wait;

volatile int1 Ack1 =1,Ack2 =1,Ack3 =1;

int16 real_data ;

float vout = 0.0;
float vin = 0.0;

//float VbuffArray[22];

//float R1 = 100000.0; // resistance of R1 (100K) 220
//float R1 = 56000.0; // resistance of R1 (100K) -125
float R1 = 10000.0;
float R2 =    680.0; // resistance of R2 (100) - see text!

//#use i2c(Master,sda=I2C_SDA,scl=I2C_SCL)

unsigned char roi, sib, hnoi;

/* Segment byte maps for numbers 0 to 9 */
const unsigned char SEGMENT_MAP[] = {0xC0,0xF9,0xA4,0xB0,0x99,0x92,0x82,0xF8,0x80,0x90};
/* Byte maps to select digit 1 to 4 */
const unsigned char SEGMENT_SELECT[] = {0xFE,0xFD,0xFB,0xF7};

//const byte CHR[] ={0xAA, 0xA3,};
enum{State_nor,State_PreUnder,State_Under,State_PreUnderRes,State_UnderRes,State_PreOver,State_Over,State_PreOverRes,State_OverRes};

// UNDER OVER
enum{nor, UnderSet, OverSet,UnderResSet,OverResSet,UnderTimSet,OverTimSet,UnderResTimSet,OverResTimSet};
// UNDER Only
enum{Nor_UnOly, UnderSet_UnOly, UnderResSet_UnOly, UnderTimSet_UnOly, UnderResTimSet_UnOly};

volatile unsigned int8 mode = Nor_UnOly,State = State_nor;
volatile signed int16 UnderResTimeCount=0 ,OverResTimeCount=0, UnderTimeCount=0, OverTimeCount =0;
volatile signed int16  OverTimSetValue,UnderResTimSetValue, UnderTimSetValue, OverResTimSetValue;
volatile float UnderValue, OverValue, UnderResValue, OverResValue;
volatile signed int16 StartMeasureCount = 600;

volatile unsigned int8 TIMER_Flag =0;

#define MCP342X_CONTINUOUS 0x10
#define MCP342X_ONE_SHOT   0x00

#define MCP342X_18BITS     0x0C
#define MCP342X_16BITS     0x08
#define MCP342X_14BITS     0x04
#define MCP342X_12BITS     0x00

#define MCP342X_8X_GAIN    0x03
#define MCP342X_4X_GAIN    0x02
#define MCP342X_2X_GAIN    0x01
#define MCP342X_1X_GAIN    0x00

#define MCP342X_DEVICE_CODE       0xD0
#define MCP342X_START_CONVERSTION 0x80

#define MCP342X_GENERAL_CALL_ADDRESS   0x00
#define MCP342X_GENERAL_CALL_LATCH     0x04

#define MCP342X_MODE MCP342X_CONTINUOUS
#ifndef MCP342X_MODE
 #define MCP342X_MODE MCP342X_ONE_SHOT
#endif

#define MCP342X_BITS MCP342X_16BITS
#ifndef MCP342X_BITS
 #define MCP342X_BITS MCP342X_16BITS
#endif

#define MCP342X_GAIN MCP342X_1X_GAIN
#ifndef MCP342X_GAIN
 #define MCP342X_GAIN MCP342X_1X_GAIN
#endif

#define MCP342X_ADDRESS 0
#ifndef MCP342X_ADDRESS
 #define MCP342X_ADDRESS 0
#endif

#define MCP342X_CHANNEL 0
#ifndef MCP342X_CHANNEL
 #define MCP342X_CHANNEL 0
#endif

/**********************************************************

Software I2C Library for PIC24 Devices.

**********************************************************/
#define MCP342X_SCL  PIN_B8
#define MCP342X_SDA  PIN_B9

void SoftI2CInit()
{
   SET_TRIS_B( 0xE000 );// SDA SCL OUTPUT 
   output_high(MCP342X_SCL);
   output_high(MCP342X_SDA);     
}

void i2cNack(void)
{
   output_high(MCP342X_SDA);
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz
   output_high(MCP342X_SCL);
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz
   output_low(MCP342X_SCL);
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz
   output_high(MCP342X_SCL);
}
void i2cAck(void)
{
   output_low(MCP342X_SDA);
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz
   output_high(MCP342X_SCL);
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz
   output_low(MCP342X_SCL);
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz
   output_high(MCP342X_SCL);
}
void SoftI2CStart()
{
   output_high(MCP342X_SCL);
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz
   
   output_low(MCP342X_SDA);
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz
   
   //   Add by  jj
   output_low(MCP342X_SCL);
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz
}
void SoftI2CStop()
{
    output_low(MCP342X_SDA);
    delay_cycles( 5 ); // 1 microsec @ 32 Mhz
    output_high(MCP342X_SCL);
    delay_cycles( 5 ); // 1 microsec @ 32 Mhz
    output_high(MCP342X_SDA);
    delay_cycles( 5 ); // 1 microsec @ 32 Mhz
}
unsigned int8 SoftI2CWriteByte(unsigned int8 data)
{
    unsigned int8 i;
       
    for(i=0;i<8;i++)
    {
      output_low(MCP342X_SCL);
      delay_cycles( 5 ); // 1 microsec @ 32 Mhz    
      if(data & 0x80)
         output_high(MCP342X_SDA);
      else
         output_low(MCP342X_SDA);
      delay_cycles( 5 ); // 1 microsec @ 32 Mhz    
      output_high(MCP342X_SCL);
      delay_cycles( 5 ); // 1 microsec @ 32 Mhz
      
      data=data<<1;
   } 
   //The 9th clock (ACK Phase)
   output_low(MCP342X_SCL);
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz

   //Stay hight wait for slave clear(low) signal to ack.
   output_float (MCP342X_SDA);//SDA Input
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz
      
   output_high(MCP342X_SCL);
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz
   
   //#**** Read Ack bit from Slave ****#//
   //SET_TRIS_B( 0xE200 ); //SDA Input
   unsigned int8 ack = input(MCP342X_SDA);
   SET_TRIS_B( 0xE000 ); //SDA Output
   
   output_low(MCP342X_SCL);
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz
   
   return ack;  
}

unsigned char i2cReadByte(void)
{
   unsigned char inByte, n;
   SET_TRIS_B( 0xE200 ); // SET SDA Input
   
   // Add By jj
   output_low(MCP342X_SCL);
   delay_cycles( 5 ); // 1 microsec @ 32 Mhz
     
   for (n=0; n<8; n++)
   {
     output_high(MCP342X_SCL);
     delay_cycles( 5 ); // 1 microsec @ 32 Mhz
     
     if(input(MCP342X_SDA))
      inByte = (inByte << 1) | 0x01; // msbit first
     else
      inByte = inByte << 1;
     
     output_low(MCP342X_SCL);
     delay_cycles( 5 ); // 1 microsec @ 32 Mhz  
   }
   SET_TRIS_B( 0xE000 ); // SET SDA Output
   return(inByte);
}

// MCP3425 Initial
void Soft_adc_init()
{
  SoftI2CInit();
  
  SoftI2CStart();
  Ack1 = SoftI2CWriteByte(MCP342X_DEVICE_CODE | (MCP342X_ADDRESS << 1));  //send write command
  Ack2 = SoftI2CWriteByte (MCP342X_MODE | MCP342X_BITS | MCP342X_GAIN | (MCP342X_CHANNEL << 5));  //send device configuration
  SoftI2CStop();
}

// MCP3425 Read ADC 16 bit  Total 104 uSec. time Operate
signed int16 Soft_read_adc_mcp(void)
{
  union
  {
    signed int16 sint16;
    unsigned int8 b[2];
  } result;
  unsigned int8 status = 0x80;
  
  SoftI2CStart();
  Ack1 = SoftI2CWriteByte(MCP342X_DEVICE_CODE | (MCP342X_ADDRESS << 1) | 1);  //send read command
  result.b[1] = i2cReadByte();
  i2cAck();
  result.b[0] = i2cReadByte();
  i2cAck();
  status = i2cReadByte();
  i2cNack();
  SoftI2CStop();
  
  return(result.sint16);
}

// END Software I2C Library

/*============EEPROM==========================*/

#use i2c(Master,Slow,sda=PIN_A0,scl=PIN_A1,restart_wdt,force_hw)
#define EEPROM_ADDRESS BYTE
#define EEPROM_SIZE    128

void init_ext_eeprom() {
   output_float(EEPROM_SCL);
   output_float(EEPROM_SDA);
}

BOOLEAN ext_eeprom_ready() {
   int1 ack;
   i2c_start();            // If the write command is acknowledged,
   ack = i2c_write(0xa0);  // then the device is ready.
   i2c_stop();
   return !ack;
}

void write_ext_eeprom(BYTE address, BYTE data) {
   while(!ext_eeprom_ready());
   i2c_start();
   i2c_write(0xa0);
   i2c_write(address);
   i2c_write(data);
   i2c_stop();
}

BYTE read_ext_eeprom(BYTE address) {
   BYTE data;

   while(!ext_eeprom_ready());
   i2c_start();
   i2c_write(0xa0);
   i2c_write(address);
   i2c_start();
   i2c_write(0xa1);
   data=i2c_read(0);
   i2c_stop();
   return(data);
}
////////////////////////////////////////////////////
void delay_nanosec(int16 us) { //about 800 nanosec
   volatile int16 counter = us;
   while(counter--);
}

//////////////////////////////////////
void EEPROM_WriteFloat(float value, int address) {
   unsigned char *p = (unsigned char *)&value;
   int i;
   for(i = 0; i < sizeof(float); i++) {
      write_ext_eeprom(address + i, *p);
      p++;
   }
}

//////////////////////////////////
float EEPROM_ReadFloat(int address) {
   float value;
   unsigned char *p = (unsigned char *)&value;
   int i;
   for(i = 0; i < sizeof(float); i++) {
      *p = read_ext_eeprom(address + i);
      p++;
   }
   return value;
}

/********************************6B595 Driver*********************************/
void SegmentDisplay()
{
  Signed int8 j=0,data_out,data_U2,data_U1;
  static int8 cnt=0 ;
    //+++++++++++++++++++++++++++++++++++++++++++++++++++++++
  static int V_to_Seg; 
  if(mode == nor)  
   V_to_Seg = DC_V7seg;
  else if(mode == UnderSet)
   V_to_Seg = UnderValue;
  else if(mode == OverSet)
   V_to_Seg = OverValue;
  else if(mode == UnderResSet)
   V_to_Seg = UnderResValue;
  else if(mode == OverResSet)
   V_to_Seg = OverResValue;
  else if(mode == UnderTimSet)
   V_to_Seg = UnderTimSetValue;
  else if(mode == OverTimSet)
   V_to_Seg = OverTimSetValue;
  else if(mode == UnderResTimSet)
   V_to_Seg = UnderResTimSetValue;
  else if(mode == OverResTimSet)
   V_to_Seg = OverResTimSetValue;
  
  //  ========== Hold Display Show ================ 
  static int16 Count_Display = 7; //7*62 = 434ms
  if(--Count_Display <= 0)
  {
     Count_Display = 7;    
     roi = V_to_Seg/100;
     sib = V_to_Seg%100;
     sib = sib/10;   
     hnoi = V_to_Seg%10;
  }
   
   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 
   //restart_wdt();
   if(++cnt > 2)cnt = 0;
   switch(cnt)
   {      
     case 0: // --1 
      data_U2 = hnoi;
      data_U1 = cnt;
     break;

     case 1: //--2
       data_U2 = sib;
       data_U1 = cnt;
     break;
      
     case 2: // --3
       data_U2 = roi;
       data_U1 = cnt;
     break;  
   }
     
   data_out = SEGMENT_MAP[data_U2];
   if((set_mode)&&(cnt == 0))
   {
     if(flash_dot>20)
      data_out &=~ 0b10000000;
     else
      data_out = data_out;
   }  
   if((cnt ==1)&&(mode == nor))//dot for 24 48 V
   {
     data_out  &=(~0x80);
   }  
   for(j=7;j>=0;j--)
   {
      output_bit(EXP_OUT_DO,bit_test(data_out,j));
      output_high(EXP_OUT_CLOCK);
      delay_us(1);
      output_low(EXP_OUT_CLOCK);
   } 
   /*=========================================*/
   data_out = SEGMENT_SELECT[data_U1];
   
   //$$$$$$$$$$$$$$$jj
   // if VDC < 100 not show lak roi
   if((cnt ==2)&&(roi == 0))
     data_out |=0b00000100;
   else if(cnt ==1)
   {
     if((sib == 0)&&(roi==0))
      data_out |=0b00000010;  
   }
   
   if(mode == nor)  
   {
      data_out = data_out |= 0b11110000; 
   }
   else if(mode == UnderSet)
   {
     data_out = data_out &= ~0x10;//80
     data_out = data_out |= 0b11100000;//clear LED Error
   }
   else if(mode == OverSet)
   {
      data_out = data_out &= ~0x20;//40
      data_out = data_out |= 0b11010000;//clear LED Error
   }
   else if(mode == UnderResSet)
   {
      data_out = data_out &= ~0b01010000;
      data_out = data_out |=  0b10100000;//clear LED Error
   }
   else if(mode == OverResSet)
   {
      data_out = data_out &= ~0b01100000;
      data_out = data_out |=  0b10010000;//clear LED Error
   }
   else if(mode == UnderTimSet)
   {
      data_out = data_out &= ~0b10010000;
      data_out = data_out |=  0b01100000;//clear LED Error
   }
   else if(mode == OverTimSet)
   {
      data_out = data_out &= ~0b10100000;
      data_out = data_out |=  0b01010000;//clear LED Error
   }
   else if(mode == UnderResTimSet)
   {
      data_out = data_out &= ~0b11010000;//00000000
      data_out = data_out |=  0b00100000;//clear LED Error
   }
   else if(mode == OverResTimSet)
   {
      data_out = data_out &= ~0b11100000;//00000000
      data_out = data_out |=  0b00010000;//clear LED Error
   }
   for(j=7;j>=0;j--)
   {      
      output_bit(EXP_OUT_DO,bit_test(data_out,j));
      output_high(EXP_OUT_CLOCK);
      delay_us(1);
      output_low(EXP_OUT_CLOCK);
   }  
   output_high(EXP_OUT_ENABLE);
   delay_us(1);
   output_low(EXP_OUT_ENABLE);
}
void StartTimer(void)
{
   TIMER_Flag = 1;
}
void StopTimer(void)
{
   TIMER_Flag = 0;
}

void Read_VDC(void)
{
  real_data = Soft_read_adc_mcp();
 
  vout = (real_data * 2.048) / 32767.0; // see text
  vin = vout / (R2/(R1+R2)); 
  DC_V = vin;
  vin = vin*10.0;  // for 24 48 V
  
  DC_V7seg = vin;
  
  if((DC_V<0.0)||(DC_V>300.0))
  {
    DC_V = 0.0;
    DC_V7seg = 0.0;
  }
  
/*
  int8 i;
  for(i=0;i<10;i++)
  {
    DC_Average[i] = DC_Average[i+1];
  }
  DC_Average[10] = DC_V7seg; 
*/
  int8 i;   
  for(int i = 9; i >= 1; i--) {
   DC_Average[i] = DC_Average[i-1];
  }
  DC_Average[0] = DC_V7seg;

/*  
  DC_V7seg = DC_Average[0];
  DC_V7seg = DC_V7seg + DC_Average[1];
  DC_V7seg = DC_V7seg + DC_Average[2];
  DC_V7seg = DC_V7seg + DC_Average[3];
  DC_V7seg = DC_V7seg + DC_Average[4];
  DC_V7seg = DC_V7seg + DC_Average[5];
  DC_V7seg = DC_V7seg + DC_Average[6];
  DC_V7seg = DC_V7seg + DC_Average[7];
  DC_V7seg = DC_V7seg + DC_Average[8];
  DC_V7seg = DC_V7seg + DC_Average[9];
  DC_V7seg = DC_V7seg/10;
*/

 //////////////////////
  DC_V7seg = 0;
  for (int i = 0; i < 10; i++) {
   DC_V7seg += DC_Average[i];
  }
  DC_V7seg = DC_V7seg/10;
   
//enum{State_nor,State_PreUnder,State_Under,State_PreUnderRes,State_UnderRes,State_PreOver,State_Over,State_PreOverRes,State_OverRes}; 
// ************************UNDER*********************
   if(StartMeasureCount <=0)
   {
     if((DC_V <= UnderValue)&&(State == State_nor))
     {
       UnderTimeCount = UnderTimSetValue*1000;
       switch(UnderTimeCount)
       {
         case 1000:
            UnderTimeCount =UnderTimeCount-80;
         break;
         case 2000:
            UnderTimeCount =UnderTimeCount-100;
         break;
         case 3000:
            UnderTimeCount =UnderTimeCount-100;
         break;
         case 4000:
            UnderTimeCount =UnderTimeCount-70;
         break;
         case 5000:
            UnderTimeCount =UnderTimeCount-100;
         break;
         default:
            UnderTimeCount =UnderTimeCount-0;
         break;
       }
       FlashLEDUnder = true;     
       State = State_PreUnder;
       if(UnderTimeCount ==0)
       {
         output_low(Under_Rly);//on rly
         output_low(Over_Rly);//on rly
         State = State_Under;
         FlashLEDUnder = false;  
         output_high(LED_Under);//on led
         output_low(LED_Over);
       }
       else
       {
         if(Timer_flag ==0)
         {
            StartTimer();
         }
         output_high(PIN_B3);
       }   
     }
     if((DC_V >= UnderValue) && (State == State_PreUnder)) 
     {
       output_low(LED_Under);
       output_high(LED_Over); 
       UnderTimeCount = 0;
       State = State_nor;
       FlashLEDUnder = false; //on led 
       StopTimer();
     }
     
     if((DC_V >= UnderResValue) && (State == State_Under))
     {
       UnderResTimeCount = UnderResTimSetValue*1000; //40*25ms = 1 Sec
       switch(UnderResTimeCount)
       {
         case 1000:
            UnderResTimeCount =UnderResTimeCount-80;
         break;
         case 2000:
            UnderResTimeCount =UnderResTimeCount-50;
         break;
         case 3000:
            UnderResTimeCount =UnderResTimeCount-70;
         break;
         case 4000:
            UnderResTimeCount =UnderResTimeCount-70;
         break;
         case 5000:
            UnderResTimeCount =UnderResTimeCount-70;
         break;
         default:
            UnderResTimeCount =UnderResTimeCount-0;
         break;
       }
       FlashLEDOver = true;
       State = State_PreUnderRes;
       if(UnderResTimeCount == 0)
       {
          output_high(Under_Rly); 
          output_high(Over_Rly); 
          if(State == State_PreUnderRes)
          {
             State = State_nor;
          }
          FlashLEDOver = false;
          output_low(LED_Under); 
          output_high(LED_Over); 
       }
       else
       {
         if(Timer_flag ==0)
         {
            StartTimer();
         }
       }     
     }
     if((DC_V <= UnderResValue)&&(State == State_PreUnderRes))
     {
        output_high(LED_Under);
        output_low(LED_Over); 
        State = State_Under;
        UnderResTimeCount = 0;
        FlashLEDUnder = false;
        StopTimer();
     }  
   } 
}

void buttonRead()
{
  enum{st1,st2,st3,st4,st5};
  static unsigned int state,deb; //deb==Debount
  int tempValue;  
  if((input(BT_SET) == 1)&&(input(BT_UP) == 1)&&(input(BT_DW) == 1))
  {        
     state = st1;     
     return; 
  }  
  if(((input(BT_SET) == 0)||(input(BT_UP) == 0)||(input(BT_DW) == 0))&&state == st1)
  {     
     state = st2;
     deb = 1; //debound    
     return;
  }
  if(((input(BT_SET) == 0)||(input(BT_UP) == 0)||(input(BT_DW) == 0))&&state == st2)
  {
    if(deb)
    {     
      deb--;
      return;
    }    
    else   
    { 
      if(input(BT_SET) == 0)
      {
        if(++mode > UnderResTimSet_UnOly)mode = Nor_UnOly;
        /////////////////////////
 
        /////////////////////////
        if(mode == Nor_UnOly)        
        {
           if(set_mode)
           {
             set_mode = false;
             write_ext_eeprom(UnderResTimSet_addr,UnderResTimSetValue);
           }
         }        
         else if((mode == UnderSet_UnOly))
         {  
           menuCount =1500*3000;
           if(set_mode)   
           {
             ;
           }
         }
         else if((mode == UnderResSet_UnOly))
         {
           menuCount =1500*3000;
           if(set_mode)
           { 
             write_ext_eeprom(UnderSet_addr,UnderValue);
           }
         } 
         else if((mode == UnderTimSet_UnOly))
         {
           menuCount =1500*3000;
           if(set_mode)
           { 
             write_ext_eeprom(UnderResSet_addr,UnderResValue); 
           }
         }
         else if((mode == UnderResTimSet_UnOly))
         {
           menuCount =1500*3000;
           if(set_mode)
           {  
             write_ext_eeprom(UnderTimSet_addr,UnderTimSetValue);  
           } 
         }

         menuCount =1500*3000;
         deb = 20; //Press hold 3sec for Enter Set mode.(60*50 =3000)
         state = st3;
         //////////////////////////////////
      }
      if((input(BT_UP) == 0)&&set_mode == true)     
      {
        switch(mode)
        {
          case UnderSet_UnOly:
            tempValue = UnderValue;
            //UnderValue++;
            UnderValue += 0.1;
            if(UnderValue > UnderResValue-1)
            {
               UnderValue = tempValue;
            }
          break;
          
          case UnderResSet_UnOly:
            tempValue = UnderResValue;
            UnderResValue += 0.1;
            if(UnderResValue > NORMAL_VOLT)UnderResValue = tempValue;
          break;
          
          case UnderTimSet_UnOly:
            tempValue = UnderTimSetValue;
            UnderTimSetValue += 0.1;
            if(UnderTimSetValue > MAX_TIME)UnderTimSetValue = tempValue;
          break;

          case UnderResTimSet_UnOly:
            tempValue = UnderResTimSetValue;
            UnderResTimSetValue += 0.1;
            if(UnderResTimSetValue > MAX_TIME) UnderResTimSetValue = tempValue;
          break;
           
        }
        ///////////////////////////////////
        state = st4;
        menuCount =1500*3000;
      }
      if((input(BT_DW) == 0)&&set_mode == true)     
      {
        switch(mode)
        {
          case UnderSet_UnOly:
          tempValue = UnderValue;
          UnderValue -= 0.1;
          if(UnderValue < MIN_VOLT)UnderValue = tempValue;
          break;

          case UnderResSet_UnOly:
          tempValue = UnderResValue;
          UnderResValue -= 0.1;
          if(UnderResValue < UnderValue +1)UnderResValue = tempValue;
          break;

          case UnderTimSet_UnOly:
          tempValue = UnderTimSetValue;
          UnderTimSetValue--;
          if(UnderTimSetValue < MIN_TIME) UnderTimSetValue = tempValue;
          break;

          case UnderResTimSet_UnOly:
          tempValue = UnderResTimSetValue;
          UnderResTimSetValue--;
          if(UnderResTimSetValue < MIN_TIME)UnderResTimSetValue = tempValue;
          break;

        }
        
        state = st4;
        menuCount =1500*3000;
      }
    }
  }
  if((input(BT_SET) == 0)&&state == st3)
  {
    if(deb)
    {
      deb--;
      return;
    }
    else
    {
      //if(set_mode==false)flash_dot =40;
      set_mode = true;
      mode = UnderSet;
    }
    menuCount =1500*3000;  
  }
}
void adjusttime(v0id)
{
   
}

/////////////////////////////////////////////////////////////
#int_TIMER3
void TIMER3_isr(void)//1ms interrupt    
{
   if(TIMER_Flag)
   {
     //flash_LED++; 
     flash_LED_Under++; 
     flash_LED_Over++;  
     /////////////// Under //////////////////////
     if(UnderTimeCount)
     {
       if(flash_LED_Under >=200)
       {        
         if(toggle_pin_under ==1)
         {
            toggle_pin_under =0;
            output_high(LED_Under); 
         }
         else
         {
            toggle_pin_under =1;
            output_low(LED_Under);
         } 
       }
       if(--UnderTimeCount <=0)
       {
         UnderTimeCount = 0;
         output_low(Under_Rly);
         output_low(Over_Rly); 
         State = State_Under;
         FlashLEDUnder = false;
         output_high(LED_Under); 
         output_low(LED_Over);          
         output_low(PIN_B3);
         if(UnderResTimeCount ==0) 
         {
            TIMER_Flag = 0;
         }
       }
     }
     
     if(UnderResTimeCount)
     {
       if(flash_LED_Under >=200)
       {
         if(toggle_pin_under==1)
         {
            toggle_pin_under =0;
            output_high(LED_Over); 
         }
         else
         {
            toggle_pin_under =1;
            output_low(LED_Over);
         }
       }   
       if(--UnderResTimeCount <=0)
       {
         UnderResTimeCount = 0;
         output_high(Under_Rly);
         output_high(Over_Rly); 
         if(State == State_PreUnderRes)
         {
            State = State_nor;
         }
         FlashLEDUnder = false;
         output_low(LED_Under);
         output_high(LED_Over); 
         if((UnderTimeCount ==0) && (OverTimeCount ==0)&& (OverResTimeCount ==0))
         {
            TIMER_Flag = 0;
         }
       }
     } 
     if(flash_LED_Under >=200)
     {
        flash_LED_Under =0;
     }
     if(flash_LED_Over >=200)
     {
        flash_LED_Over =0;
     }  
   }
}
////////////////////////////////////////////////////
#int_TIMER2
void TIMER2_isr(void)//5ms interrupt    
{    
   SegmentDisplay();
   if(--flash_dot <=0)flash_dot =40;
   
   if(ReadI2CCount )
   {
      if(--ReadI2CCount ==0)
      {
         ReadI2CCount =0;
      }
   }
   if(StartMeasureCount)
   {
      if(--StartMeasureCount<=0)
      {
         StartMeasureCount = 0;
         UnderResTimeCount=0;
         OverResTimeCount=0;
         UnderTimeCount=0;
         OverTimeCount =0;
      }
   } 
}
///////////////////////////////////////////////////////
void main()
{
   setup_timer2(TMR_INTERNAL |TMR_DIV_BY_8,0x09C3);//5ms interrupt @ 4(8) Mhz Internals RC (Value in OCR)
   setup_timer3(TMR_INTERNAL |TMR_DIV_BY_1,0x0F9F);//1ms interrupt @ 4(8) Mhz
   enable_interrupts(INT_TIMER3);    
   RESTART_CAUSE(RESTART_BROWNOUT|RESTART_POWER_UP|RESTART_WATCHDOG); //jj 9/7/63  
   enable_interrupts(INT_TIMER2);
   enable_interrupts(INTR_GLOBAL);   
   SET_TRIS_B( 0b1110000000001000);
   output_float(BT_DW);   //Set  as Input Pin
   output_float(BT_UP);   //Set  as Input Pin
   output_float(BT_SET);   //Set  as Input Pin
   
   output_low(EXP_OUT_DO);
   output_low(EXP_OUT_CLOCK);
   output_low(EXP_OUT_ENABLE);
      
   output_low(PIN_B3);

   delay_ms(50);
   
   Soft_adc_init();        
   
   UnderValue = read_ext_eeprom(UnderSet_addr);if(UnderValue < MIN_VOLT || UnderValue > NORMAL_VOLT-1) UnderValue = 15;
   UnderResValue = read_ext_eeprom(UnderResSet_addr);if(UnderResValue < UnderValue + 1 || UnderResValue > NORMAL_VOLT) UnderResValue = UnderValue+5;
   
   UnderTimSetValue = read_ext_eeprom(UnderTimSet_addr);if(UnderTimSetValue < 0 || UnderTimSetValue > 60) UnderTimSetValue = 5;
   UnderResTimSetValue = read_ext_eeprom(UnderResTimSet_addr);if(UnderResTimSetValue < 0 || UnderResTimSetValue > 60) UnderResTimSetValue = 5;
   
   delay_ms(500);
   int8 i;
   for(i=0;i<=10;i++)
   {
      real_data = Soft_read_adc_mcp(); 
      vout = (real_data * 2.048) / 32767.0; // see text
      vin = vout / (R2/(R1+R2));
      DC_V =(int16)vin;  
      
      DC_Average[i] = DC_V;
      
      delay_ms(20);
   
   }
   DC_V = DC_Average[0];
   DC_V = DC_V + DC_Average[1];
   DC_V = DC_V + DC_Average[2];
   DC_V = DC_V + DC_Average[3];
   DC_V = DC_V + DC_Average[4];
   DC_V = DC_V + DC_Average[5];
   DC_V = DC_V + DC_Average[6];
   DC_V = DC_V + DC_Average[7];
   DC_V = DC_V + DC_Average[8];
   DC_V = DC_V + DC_Average[9];
   DC_V = DC_V/10;
   
   if(DC_V <= UnderValue)
   {
      output_low(Under_Rly);//on rly
      output_low(Over_Rly);//on rly
      State = State_Under;
      FlashLEDUnder = false;  
      output_high(LED_Under);//on led
      output_low(LED_Over);
   }
   else
   {
      output_high(Under_Rly);//on rly
      output_high(Over_Rly);//on rly
      output_low(LED_Under);//off rly
      output_high(LED_Over);//off rly
      UnderTimeCount = 0;
      State = State_nor;
      FlashLEDUnder = false; //on led  
   }
     
   setup_wdt(WDT_ON);
   setup_timer1(TMR_INTERNAL|TMR_DIV_BY_8);
     
   unsigned int16 Count_Healty =7;//7 612 us 
   while(1)
   {  
      Count_Healty++;
      output_toggle(PIN_A4); //5.6 uSec perloop
      restart_wdt();
        
      if(Count_Healty %1500 ==0 )// 30 mS.
      {        
         buttonRead();
      }
      
      if(Count_Healty %1000 ==0 ) //20 mS.
      {        
         Read_VDC();//Time for Read VDC is 62 ms. 
      }
      
      if(Count_Healty %32750 ==0 )
      {          
         output_toggle(LED_Healty);
         Count_Healty =1;
         
         // Re-initial if MCP3425 Hang
         if(DC_V < 10 )
         {
            Soft_adc_init();
         }
      }
  
      //*********  Menu Count ***************_/
      if(menuCount)
      { 
         if(--menuCount ==0)
         {
           menuCount =0;
           mode = Nor_UnOly; 
           set_mode = false;
         }
        
      }
   }
}
