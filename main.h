#include <24FJ16GA004.h>

////////////////////////////////////////////////////////
/*
#FUSES WDT
#FUSES WDT64                    //Watch Dog Timer uses 1:64 Postscale
#FUSES HS                       //High speed Osc (> 4mhz)
#FUSES PROTECT                //Code not protected from reading
//#FUSES IESO                     //Internal External Switch Over mode enabled
#FUSES BROWNOUT                 //Reset when brownout detected
//#FUSES BORV20                   //Brownout reset at 2.0V
#FUSES NOPUT                    //No Power Up Timer
#FUSES NOCPD                    //No EE protection
//#FUSES STVREN                   //Stack full/underflow will cause reset
#FUSES NODEBUG                  //No Debug mode for ICD
//#FUSES LVP                      //Low Voltage Programming on B3(PIC16) or B5(PIC18)
//#FUSES NOWRT                    //Program memory not write protected
//#FUSES NOWRTD                   //Data EEPROM not write protected
#FUSES NOEBTR                   //Memory not protected from table reads
#FUSES NOCPB                    //No Boot Block code protection
#FUSES NOEBTRB                  //Boot block not protected from table reads
#FUSES NOWRTC                   //configuration not registers write protected
#FUSES NOWRTB                   //Boot block not write protected
//#FUSES FCMEN                    //Fail-safe clock monitor enabled
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#FUSES PBADEN                   //PORTB pins are configured as analog input channels on RESET
#FUSES LPT1OSC                  //Timer1 configured for low-power operation
#FUSES MCLR                     //Master Clear pin enabled
*/
/////////////////////////////////////////////////////////



//#FUSES NOWDT                    //No Watch Dog Timer


//#FUSES WDT32                    //Watch Dog Timer uses 1:64 Postscale
#FUSES NOJTAG                   //JTAG disabled
#FUSES PROTECT                //Code not protected from reading
#FUSES NOWRT                    //Program memory not write protected
#FUSES NODEBUG                  //No Debug mode for ICD
#FUSES ICSP1                    //ICD uses PGC1/PGD1 pins
#FUSES IOL1WAY                  //Allows only one reconfiguration of peripheral pins
#FUSES WDT        //jj
#FUSES WINDIS                   //Watch Dog Timer in non-Window mode
#FUSES WPRES32                 //Watch Dog Timer PreScalar 1:128
#FUSES WPOSTS16   //jj             //Watch Dog Timer PostScalar 1:32768

#FUSES IESO     //jj                //Internal External Switch Over mode enabled

//#FUSES FRC_PS                   //Fast RC Oscillator with Post Scaler
#FUSES NOCKSFSM                 //Clock Switching is disabled, fail Safe clock monitor is disabled
//#FUSES CKSFSM

#FUSES NOOSCIO                  //OSC2 is clock output
//#FUSES NOPR                     //Pimary oscillaotr disabled
#FUSES I2C1SELD
//#FUSES HS

#use delay(clock=8000000, oscillator=8000000)

//#use i2c(Master,Slow,sda=PIN_A0,scl=PIN_A1,force_hw)
//#use delay(clock=32000000,RESTART_WDT)


//#use delay(clock=20000000)


/*

#FUSES NOWDT                    //No Watch Dog Timer
#FUSES NOJTAG                   //JTAG disabled
#FUSES NOPROTECT                //Code not protected from reading
#FUSES NOWRT                    //Program memory not write protected
#FUSES NODEBUG                  //No Debug mode for ICD
#FUSES ICSP1                    //ICD uses PGC1/PGD1 pins
#FUSES IOL1WAY                  //Allows only one reconfiguration of peripheral pins
#FUSES WINDIS                   //Watch Dog Timer in non-Window mode
#FUSES WPRES128                 //Watch Dog Timer PreScalar 1:128
#FUSES WPOSTS8                  //Watch Dog Timer PostScalar 1:128
#FUSES IESO                     //Internal External Switch Over mode enabled
//#FUSES FRC_PS                   //Fast RC Oscillator with Post Scaler
#FUSES NOCKSFSM                 //Clock Switching is disabled, fail Safe clock monitor is disabled
#FUSES NOOSCIO                  //OSC2 is clock output
#FUSES HS                       //Crystal osc <= 4mhz for PCM/PCH , 3mhz to 10 mhz for PCD
#FUSES I2C1SELD              

#use delay(clock=8000000,RESTART_WDT)
*/
