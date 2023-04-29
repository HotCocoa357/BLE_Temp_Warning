//==============================================================================
//    S E N S I R I O N   AG,  Laubisruetistr. 50, CH-8712 Staefa, Switzerland
//==============================================================================
// Project   :  SHT2x Sample Code (V1.2)
// File      :  SHT2x.c
// Author    :  MST
// Controller:  NEC V850/SG3 (uPD70F3740)
// Compiler  :  IAR compiler for V850 (3.50A)
// Brief     :  Sensor layer. Functions for sensor access
//==============================================================================

// Standard includes
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_ints.h"
#include "hw_i2c.h"
#include "hwspinlock.h"
#include "i2c.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "utils.h"


// Common interface include
#include "i2c_if.h"
#define I2C_BASE                I2CA0_BASE
#include "SHT2x.h"

#define  SHT_Addr  0x40


extern void menu_display();
extern void I2C_init();
extern enum STATES state;
extern enum STATES{init_state=0,
            memu_state,
            temperature_sensor_state,
            air_pressure_state,
            AD_state,
            pwm_sound_state,
            pwm_led_state,
            acceleration_sensor_state,
            tmp75_state
} state;

unsigned char tdata[6]={USER_REG_R,TRIG_RH_MEASUREMENT_HM,TRIG_T_MEASUREMENT_HM,TRIG_RH_MEASUREMENT_POLL,
TRIG_T_MEASUREMENT_POLL,SOFT_RESET};

void
SysCtlDelay(uint32_t ui32Count)
{
    __asm("    subs    r0, #1\n"
          "    bne.n   SysCtlDelay\n"
          "    bx      lr");
}


//==============================================================================
unsigned char SHT2x_CheckCrc(unsigned char data[], unsigned char nbrOfBytes, unsigned char checksum)
//==============================================================================
{
	unsigned char bit;
  unsigned char crc = 0;	
  unsigned char byteCtr;
  //calculates 8-Bit checksum with given polynomial
  for (byteCtr = 0; byteCtr < nbrOfBytes; ++byteCtr)
  { crc ^= (data[byteCtr]);
    for (bit = 8; bit > 0; --bit)
    { if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
      else crc = (crc << 1);
    }
  }
  if (crc != checksum) return CHECKSUM_ERROR;
  else return 0;
}

//===========================================================================
unsigned char SHT2x_ReadUserRegister(unsigned char *pRegisterValue)
//===========================================================================
{
  unsigned char checksum;   //variable for checksum byte
  unsigned char error=0;    //variable for error code

   unsigned char *p;
   p=&tdata[0];
   I2C_IF_Write(SHT_Addr,p,1,0);
   unsigned char RxData[2] = {0,0};
   I2C_IF_Read(SHT_Addr,RxData,2);
   *pRegisterValue=RxData[0];
   checksum=RxData[1];
   error |= SHT2x_CheckCrc (pRegisterValue,1,checksum);
   
   return error;
}

//===========================================================================
unsigned char SHT2x_WriteUserRegister(unsigned char *pRegisterValue)
//===========================================================================
{
  unsigned char error=0;   //variable for error code
  unsigned char TxData[2] = {0,0};
  
  TxData[0]=USER_REG_W;
  TxData[1]=*pRegisterValue;
  
  error=I2C_IF_Write(SHT_Addr,TxData,2,1);

  return error;
}

//===========================================================================
unsigned char SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
//===========================================================================
{
  unsigned char  checksum;   //checksum
  unsigned char  data[2];    //data array for checksum verification
  unsigned char  error=0;    //error variable
//  unsigned short i;          //counting variable
  unsigned char *p1,*p2;
  p1=&tdata[1];
  p2=&tdata[2];
  switch(eSHT2xMeasureType)
  { case HUMIDITY: 
                  error=I2C_IF_Write(SHT_Addr,p1,1,0);
                  break;
                  
    case TEMP    : 
                  error=I2C_IF_Write(SHT_Addr,p2,1,0);
                  break;
    default: break;
  }

//  //-- wait until hold master is released --
//  I2c_StartCondition();
//  error |= I2c_WriteByte (I2C_ADR_R);
//  SCL=HIGH;                     // set SCL I/O port as input
//  for(i=0; i<1000; i++)         // wait until master hold is released or
//  { DelayMicroSeconds(1000);    // a timeout (~1s) is reached
//    if (SCL_CONF==1) break;
//  } 
  MAP_UtilsDelay(40000000);
  //-- read two data bytes and one checksum byte --
  unsigned char RxData[3]={0,0,0};
  I2C_IF_Read(SHT_Addr,RxData,3);
  data[0] = RxData[0];
  data[1] = RxData[1];
  pMeasurand->s16.u8H = data[0];
  pMeasurand->s16.u8L = data[1];
  checksum=RxData[2];
  
  //-- verify checksum --
  error |= SHT2x_CheckCrc (data,2,checksum);
  return error;
}

//===========================================================================
unsigned char SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand)
//===========================================================================
{
  unsigned char  checksum;   //checksum
  unsigned char  data[2];    //data array for checksum verification
  unsigned char  error=0;    //error variable
//  unsigned short i=0;        //counting variable

     unsigned char *p1,*p2;
     p1=&tdata[3];
     p2=&tdata[4];
  //-- write I2C sensor address and command --
  switch(eSHT2xMeasureType)
  { case HUMIDITY: 
                  error=I2C_IF_Write(SHT_Addr,p1,1,1);
                  break;
    case TEMP    : 
                  error=I2C_IF_Write(SHT_Addr,p2,1,1);   
                  break;
    default: break;
  }
  
//  //-- poll every 10ms for measurement ready. Timeout after 20 retries (200ms)--
//  do
//  { I2c_StartCondition();
//    DelayMicroSeconds(10000);  //delay 10ms
//    if(i++ >= 20) break;
//  } while(I2c_WriteByte (I2C_ADR_R) == ACK_ERROR);
//  if (i>=20) error |= TIME_OUT_ERROR;
     MAP_UtilsDelay(8000000);
  //-- read two data bytes and one checksum byte --
  unsigned char RxData[3]={0,0,0};
  I2C_IF_Read(SHT_Addr,RxData,3);
  data[0] = RxData[0];
  data[1] = RxData[1];
  pMeasurand->s16.u8H = data[0];
  pMeasurand->s16.u8L = data[1];
  checksum=RxData[2];
  
  //-- verify checksum --
  error |= SHT2x_CheckCrc (data,2,checksum);
  return error;
}

//===========================================================================
unsigned char SHT2x_SoftReset()
//===========================================================================
{
  unsigned char  error=0;           //error variable
  unsigned char *p;
  p=&tdata[5];
  error=I2C_IF_Write(SHT_Addr,p,1,1);
  
  MAP_UtilsDelay(400000); // wait till sensor has restarted 15ms

  return error;
}

//==============================================================================
float SHT2x_CalcRH(unsigned short u16sRH)
//==============================================================================
{
  float  humidityRH;              // variable for result

  u16sRH &= ~0x0003;          // clear bits [1..0] (status bits)
  //-- calculate relative humidity [%RH] --

  humidityRH = -6.0 + 125.0/65536 * (float)u16sRH; // RH= -6 + 125 * SRH/2^16
  return humidityRH;
}

//==============================================================================
float SHT2x_CalcTemperatureC(unsigned short u16sT)
//==============================================================================
{
  float temperatureC;            // variable for result

  u16sT &= ~0x0003;           // clear bits [1..0] (status bits)

  //-- calculate temperature  --
  temperatureC= -46.85 + 175.72/65536 *(float)u16sT; //T= -46.85 + 175.72 * ST/2^16
  return temperatureC;
}

//==============================================================================
unsigned char SHT2x_GetSerialNumber(unsigned char u8SerialNumber[])
//==============================================================================
{
  unsigned char  error=0;                          //error variable

  //Read from memory location 1
  unsigned char TxData1[2] = {0,0};
  TxData1[0]=0xFA;
  TxData1[1]=0x0F;
  I2C_IF_Write(SHT_Addr,TxData1,2,0);
  unsigned char RxData1[8]={0,0,0,0,0,0,0,0};
  I2C_IF_Read(SHT_Addr,RxData1,8);
  u8SerialNumber[5]=RxData1[0];//Read SNB_3
                              //RxData1[1]  Read CRC SNB_3 (CRC is not analyzed)
  u8SerialNumber[4]=RxData1[2];//Read SNB_2
                              //RxData1[3]  Read CRC SNB_2 (CRC is not analyzed)
  u8SerialNumber[3]=RxData1[4];//Read SNB_1
                              //RxData1[5]  Read CRC SNB_1 (CRC is not analyzed)
  u8SerialNumber[2]=RxData1[6];//Read SNB_0
                              //RxData1[7]  Read CRC SNB_0 (CRC is not analyzed)

  //Read from memory location 2
  unsigned char TxData2[2] = {0,0};
  TxData2[0]=0xFC;
  TxData2[1]=0xC9;
  I2C_IF_Write(SHT_Addr,TxData2,2,0);
  unsigned char RxData2[6]={0,0,0,0,0,0};
  I2C_IF_Read(SHT_Addr,RxData2,6);
  u8SerialNumber[1]=RxData2[0];//Read SNC_1
  u8SerialNumber[0]=RxData2[1];//Read SNC_0
                               //RxData2[2]  Read CRC SNC0/1 (CRC is not analyzed)
  u8SerialNumber[7]=RxData2[3];//Read SNA_1
  u8SerialNumber[6]=RxData2[4];//Read SNA_0
                               //Read CRC SNA0/1 (CRC is not analyzed)

  return error;
}

void sht20_func(void)
{

// variables
  unsigned char  error = 0;              //variable for error code. For codes see system.h
  unsigned char  userRegister;           //variable for user register
//  bool   endOfBattery;           //variable for end of battery

//  nt16 sRH;                    //variable for raw humidity ticks
//  float   humidityRH;             //variable for relative humidity[%RH] as float
//  char humitityOutStr[21];     //output string for humidity value
//  nt16 sT;                     //variable for raw temperature ticks
//  float   temperatureC;           //variable for temperature[C] as float
//  char temperatureOutStr[21];  //output string for temperature value
//  unsigned char  SerialNumber_SHT2x[8];  //64bit serial number
//
//
//  char str_name[30]={0};
//  char str_value[20]={0};

//  I2C_init();
    // I2C Init
    //
  I2C_IF_Open(I2C_MASTER_MODE_FST);
//
//  LCD_Clear();
//  LCD_StringDisplay(1,1,"SHT20-Test");

  error = 0;                                       // reset error status
    // --- Reset sensor by command ---
    error |= SHT2x_SoftReset();
    // --- Read the sensors serial number (64bit) ---
   // error |= SHT2x_GetSerialNumber(SerialNumber_SHT2x);

    // --- Set Resolution e.g. RH 10bit, Temp 13bit ---
    error |= SHT2x_ReadUserRegister(&userRegister);  //get actual user reg

    userRegister = (userRegister & ~SHT2x_RES_MASK) | SHT2x_RES_10_13BIT;
    error |= SHT2x_WriteUserRegister(&userRegister); //write changed user reg

//  while(1)
//  {
//
//
//
////    // --- measure humidity with "Hold Master Mode (HM)"  ---
////    error |= SHT2x_MeasureHM(HUMIDITY, &sRH);
//    // --- measure humidity with "Polling Mode" (no hold master) ---
//    error |= SHT2x_MeasurePoll(HUMIDITY, &sRH);
//
//    // --- measure temperature with "Polling Mode" (no hold master) ---
//    error |= SHT2x_MeasurePoll(TEMP, &sT);
//
//
//    //-- calculate humidity and temperature --
//    temperatureC = SHT2x_CalcTemperatureC(sT.u16);
//    humidityRH   = SHT2x_CalcRH(sRH.u16);
//
//    strcpy(str_name,"Temperature:");
//    sprintf(str_value,"%f",temperatureC);
//    strcat(str_name,str_value);
//    strcat(str_name,"¡æ");
//    LCD_StringDisplay(1,50,str_name);
//
//    strcpy(str_name,"Humidity:");
//    sprintf(str_value,"%f",humidityRH);
//    strcat(str_name,str_value);
//    LCD_StringDisplay(1,100,str_name);
//
//    }
}



