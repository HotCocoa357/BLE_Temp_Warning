// Standard includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "gpio.h"
#include "pin.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "timer.h"

// Common interface includes
#include "uart_if.h"
#include "i2c_if.h"
#include "gpio_if.h"

#include "pinmux.h"
#include "SHT2x.h"
#include "timer_if.h"

#include "LCD_TFT_ILI9341.h"
#include "Font_lib.h"
#include "LCD_Display.h"


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

#define UART_PRINT              Report
//警戒温度
#define WarningTemp 20


void wait_ms(long i)
{
 while(i)
	  {__delay_cycles(20000);;i--;}

}


#define APPLICATION_VERSION     "1.1.1"

#define TIMER_10MS_DELAY        80000000/100    /*10ms/loop for delay*/
unsigned int g_uiDelay_sht20;        //for common

//*****************************************************************************
//
// Globals used by the timer interrupt handler.
//
//*****************************************************************************
static volatile unsigned long g_ulSysTickValue;
static volatile unsigned long g_ulBase;
static volatile unsigned long g_ulIntClearVector;
unsigned long g_ulTimerInts;


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void TmpDetect(float temp);

static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif

    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}



//*****************************************************************************
//
//! The interrupt handler for the first timer interrupt.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
void
TimerBaseIntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    Timer_IF_InterruptClear(g_ulBase);

    g_uiDelay_sht20 ++;
}

//lcd
const char     pcDigits[] = "0123456789"; /* variable used by itoa function */

 char * itoa(int n, char * chBuffer)
{
    int i = 1;
    char * pch = chBuffer;
    if(!pch) return 0;
    while(n / i) i *= 10;

    if(n < 0)
    {
        n = -n;
        *pch++ = '-';
    }
    if (0 == n) i = 10;

    while(i /= 10)
    {
        *pch++ = n / i + '0';
        n %= i;
    }
   return chBuffer;
}

char * ftoa(float dValue, char * chBuffer , int size)
{
    char * pch = chBuffer;
    int i;
    int temp;
    if(!pch)
      return 0;

    if(!(dValue <= 1E-307 && dValue >= -1E-307)){

        if(dValue < 0){
            *pch++ = '-';
            dValue = -dValue;
        }

        temp = (int)dValue;
        itoa(temp , pch);
        unsigned char ucLen = strlen(pch);
        pch += ucLen;
        *pch++ = '.';
        dValue -= (int)dValue;
        ucLen = size - ucLen - 1;

        for(i = 0; i < ucLen; i++){
            dValue = dValue  * 10;
            temp = (int)dValue;
            itoa(temp, pch);
            pch += strlen(pch);
            dValue -= (int)dValue;
        }
    }else
        *pch++ = '0';

    pch--;
    return chBuffer;
}


unsigned int val;

//温度检测报警
void TmpDetect(float temp){
	//温度高于警戒值	亮红灯戒备 并发送警告信息
	if(temp>WarningTemp){
		GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
		GPIO_IF_LedOn(MCU_RED_LED_GPIO);
		GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
		UART_PRINT("\nWarning: Overheated!\n");
		LCD_StringDisplay(40,195,"DANGER");
	}
	//温度低于警戒值	关闭红灯
	else{
		GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
		GPIO_IF_LedOff(MCU_RED_LED_GPIO);
		GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
		UART_PRINT("  SAFE\n");
		LCD_StringDisplay(40,195,"SAFE");
	}
}

void while_SHT20(){
    static nt16 sRH;
    static float   humidityRH;
    static nt16 sT;
    static float   temperatureC;
    unsigned char error = 0;
//    char str_name[30]={0};
//    char str_value[20]={0};
    static unsigned int uiTask=0;
    unsigned char  checksum;   //checksum
    unsigned char tmp = 0xF3,hum = 0xF5;
    int temp;
    char tempch[12];

    unsigned char RxData[3]={0,0,0};

    switch(uiTask){
        case 0:
            I2C_IF_Write(0x40,&tmp,1,1);
            g_uiDelay_sht20 = 0;
            uiTask = 1;
            break;
        case 1:
            if(g_uiDelay_sht20 > 20){   //200ms的数据转化时间
                I2C_IF_Read(0x40,RxData,3);
                sT.s16.u8H = RxData[0];
                sT.s16.u8L = RxData[1];
                checksum=RxData[2];

                //-- verify checksum --
                error |= SHT2x_CheckCrc (RxData,2,checksum);

                temperatureC = SHT2x_CalcTemperatureC(sT.u16);
                uiTask = 2;
            }
            break;
        case 2:
            I2C_IF_Write(0x40,&hum,1,1);
            g_uiDelay_sht20 = 0;
            uiTask = 3;
            break;
        case 3:
            if(g_uiDelay_sht20 > 20){   //200ms的数据转化时间
                I2C_IF_Read(0x40,RxData,3);
                sRH.s16.u8H = RxData[0];
                sRH.s16.u8L = RxData[1];
                checksum=RxData[2];

                //-- verify checksum --
                error |= SHT2x_CheckCrc (RxData,2,checksum);

                humidityRH = SHT2x_CalcTemperatureC(sRH.u16);
                uiTask = 4;
            }
            break;
        case 4:

            temp = temperatureC*100;
            UART_PRINT("Temperature:%d.%d℃     ",temp/100,temp%100);
            temp = humidityRH*100;
          	  //UART_PRINT("Humidity:%d.%d%       ",temp/100,temp%100);
            uiTask = 0;

            //lcd
            memset(tempch,0,sizeof(tempch));
            ftoa(temperatureC,tempch,sizeof(tempch)-1);

            TmpDetect(temperatureC);

            LCD_StringDisplay(40,180,"Temperature:");LCD_StringDisplay(140,180,tempch);
            wait_ms(1000);
            break;
    };
}



/*
 * main.c
 */
void main(void) {
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Power on the corresponding GPIO port B for 9,10,11.
    // Set up the GPIO lines to mode 0 (GPIO)
    //
    PinMuxConfig();
    //led
    GPIO_IF_LedConfigure(LED1|LED2|LED3);


    lcd_init();
       LCD_ILI9341_TFT_background(White);
       LCD_ILI9341_TFT_foreground(Black);
      // LCD_Show_StandbyPage();
       LCD_ILI9341_TFT_cls(White);


    //
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    //
    // Configuring UART
    //
    InitTerm();

    UART_PRINT("智能温度报警：同济大学电信学院 \n");
    UART_PRINT("电子信息工程，赵启满2051518 \n");

    LCD_Show_StandbyPage();

    // Base address for first timer
    //
    g_ulBase = TIMERA0_BASE;
    //
    // Configuring the timers
    //
    Timer_IF_Init(PRCM_TIMERA0, g_ulBase, TIMER_CFG_PERIODIC, TIMER_A, 0);

    //
    // Setup the interrupts for the timer timeouts.
    //
    Timer_IF_IntSetup(g_ulBase, TIMER_A, TimerBaseIntHandler);

    //
    // Turn on the timers feeding values in mSec
    //
    Timer_IF_Start(g_ulBase, TIMER_A, 10);

    GPIO_IF_LedOff(MCU_ALL_LED_IND);
	while(1)
	{
		while_SHT20();
	       MAP_UtilsDelay(8000000);
	}
}


