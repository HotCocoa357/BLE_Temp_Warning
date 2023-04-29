#ifndef SHT2x_H
#define SHT2x_H


//---------- Includes ----------------------------------------------------------

//---------- Defines -----------------------------------------------------------
//  CRC
#define POLYNOMIAL 0x131             //P(x)=x^8+x^5+x^4+1 = 100110001
// sensor command
typedef enum{
   TRIG_T_MEASUREMENT_HM    = 0xE3, // command trig. temp meas. hold master
   TRIG_RH_MEASUREMENT_HM   = 0xE5, // command trig. humidity meas. hold master
   TRIG_T_MEASUREMENT_POLL  = 0xF3, // command trig. temp meas. no hold master
   TRIG_RH_MEASUREMENT_POLL = 0xF5, // command trig. humidity meas. no hold master
   USER_REG_W               = 0xE6, // command writing user register
   USER_REG_R               = 0xE7, // command reading user register
   SOFT_RESET               = 0xFE  // command soft reset
}etSHT2xCommand;

typedef enum {
  SHT2x_RES_12_14BIT       = 0x00, // RH=12bit, T=14bit
  SHT2x_RES_8_12BIT        = 0x01, // RH= 8bit, T=12bit
  SHT2x_RES_10_13BIT       = 0x80, // RH=10bit, T=13bit
  SHT2x_RES_11_11BIT       = 0x81, // RH=11bit, T=11bit
  SHT2x_RES_MASK           = 0x81  // Mask for res. bits (7,0) in user reg.
} etSHT2xResolution;

typedef enum {
  SHT2x_EOB_ON             = 0x40, // end of battery
  SHT2x_EOB_MASK           = 0x40, // Mask for EOB bit(6) in user reg.
} etSHT2xEob;

typedef enum {
  SHT2x_HEATER_ON          = 0x04, // heater on
  SHT2x_HEATER_OFF         = 0x00, // heater off
  SHT2x_HEATER_MASK        = 0x04, // Mask for Heater bit(2) in user reg.
} etSHT2xHeater;

// measurement signal selection
typedef enum{
  HUMIDITY,
  TEMP
}etSHT2xMeasureType;

#define LITTLE_ENDIAN  //e.g. PIC, 8051, NEC V850
typedef union {
  unsigned short u16;               // element specifier for accessing whole u16
  signed short i16;               // element specifier for accessing whole i16
  struct {
    #ifdef LITTLE_ENDIAN  // Byte-order is little endian
    unsigned char u8L;              // element specifier for accessing low u8
    unsigned char u8H;              // element specifier for accessing high u8
    #else                 // Byte-order is big endian
    unsigned char u8H;              // element specifier for accessing low u8
    unsigned char u8L;              // element specifier for accessing high u8
    #endif
  } s16;                  // element spec. for acc. struct with low or high u8
} nt16;
// Error codes
typedef enum{
  ACK_ERROR                = 0x01,
  TIME_OUT_ERROR           = 0x02,
  CHECKSUM_ERROR           = 0x04,
  UNIT_ERROR               = 0x08
}etError;

//==============================================================================
unsigned char SHT2x_CheckCrc(unsigned char data[], unsigned char nbrOfBytes, unsigned char checksum);
//==============================================================================
// calculates checksum for n bytes of data and compares it with expected
// checksum
// input:  data[]       checksum is built based on this data
//         nbrOfBytes   checksum is built for n bytes of data
//         checksum     expected checksum
// return: error:       CHECKSUM_ERROR = checksum does not match
//                      0              = checksum matches

//==============================================================================
unsigned char SHT2x_ReadUserRegister(unsigned char *pRegisterValue);
//==============================================================================
// reads the SHT2x user register (8bit)
// input : -
// output: *pRegisterValue
// return: error

//==============================================================================
unsigned char SHT2x_WriteUserRegister(unsigned char *pRegisterValue);
//==============================================================================
// writes the SHT2x user register (8bit)
// input : *pRegisterValue
// output: -
// return: error

//==============================================================================
unsigned char SHT2x_MeasurePoll(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand);
//==============================================================================
// measures humidity or temperature. This function polls every 10ms until
// measurement is ready.
// input:  eSHT2xMeasureType
// output: *pMeasurand:  humidity / temperature as raw value
// return: error
// note:   timing for timeout may be changed

//==============================================================================
unsigned char SHT2x_MeasureHM(etSHT2xMeasureType eSHT2xMeasureType, nt16 *pMeasurand);
//==============================================================================
// measures humidity or temperature. This function waits for a hold master until
// measurement is ready or a timeout occurred.
// input:  eSHT2xMeasureType
// output: *pMeasurand:  humidity / temperature as raw value
// return: error
// note:   timing for timeout may be changed

//==============================================================================
unsigned char SHT2x_SoftReset();
//==============================================================================
// performs a reset
// input:  -
// output: -
// return: error

//==============================================================================
float SHT2x_CalcRH(unsigned short u16sRH);
//==============================================================================
// calculates the relative humidity
// input:  sRH: humidity raw value (16bit scaled)
// return: pHumidity relative humidity [%RH]

//==============================================================================
float SHT2x_CalcTemperatureC(unsigned short u16sT);
//==============================================================================
// calculates temperature
// input:  sT: temperature raw value (16bit scaled)
// return: temperature [°C]

//==============================================================================
unsigned char SHT2x_GetSerialNumber(unsigned char u8SerialNumber[]);
//==============================================================================
// gets serial number of SHT2x according application note "How To
// Read-Out the Serial Number"
// note:   readout of this function is not CRC checked
//
// input:  -
// output: u8SerialNumber: Array of 8 bytes (64Bits)
//         MSB                                         LSB
//         u8SerialNumber[7]             u8SerialNumber[0]
//         SNA_1 SNA_0 SNB_3 SNB_2 SNB_1 SNB_0 SNC_1 SNC_0
// return: error
#endif
