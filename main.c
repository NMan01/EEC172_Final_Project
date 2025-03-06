//*****************************************************************************
//
// Application Name - Final Project
// Authors - Nadav Weinberger and Khaiber Amin
//
//
//*****************************************************************************
//edit

//*****************************************************************************
//
//! \addtogroup SPI_Demo
//! @{
//
//*****************************************************************************

// Standard includes
#include <pin_mux_config.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "gpio.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"

// Common interface includes
#include "uart_if.h"
#include "gpio_if.h"
#include "i2c_if.h"
//#include "spi_if.h"


#include "oled/Adafruit_SSD1351.h"
#include "oled/oled_test.h"
#include "oled/Adafruit_GFX.h"
#include "oled/glcdfont.h"



#define APPLICATION_VERSION     "1.4.0"
//*****************************************************************************
//
// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode
// MASTER_MODE = 0 : Application in slave mode
//
//*****************************************************************************

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************



//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
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
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Enable the SPI module clock
    //
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    //
    // Initialising the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();

    //
    // Display the Banner
    //
    Message("\n\n\n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\t\t                Lab 2 - Part 3  \n\r");
    Message("\t\t   ********************************************\n\r");
    Message("\n\n\n\r");

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);

    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_0,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVEHIGH |
                     SPI_WL_8));

    // enable SPI
    MAP_SPIEnable(GSPI_BASE);

    // enable I2C
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    //set OLED R to HI
    GPIOPinWrite(GPIOA3_BASE, 0x10, 0x10);

    //set OLED CS to HI
    GPIOPinWrite(GPIOA2_BASE, 0x40, 0x40);


    Adafruit_Init();
    fillScreen(BLACK);

    unsigned char accelerometer_addr = 0x18;
    unsigned char x_reg = 0x03;
//    unsigned char y_reg = 0x05;

    signed char acc_x, acc_y;

    int score = 0;
    char scoreStr[20];
    sprintf(scoreStr, "Score: %d", score);
    Outstr(scoreStr);

//    // put tank in center initially
    int x1 = width()/2 - 5, y1 = height()/2 + 12;
    int x2 = width()/2 + 5, y2 = height()/2 + 12;
    int x3 = width()/2, y3 = height()/2; // front of tank

//    int x1 = 50, y1 = 69;
//    int x2 = 60, y2 = 69;
//    int x3 = 64, y3 = 64; // front of tank
    drawTriangle(x1, y1, x2, y2, x3, y3, GREEN);

    int target_x, target_y;

    // put target in random coordinates
    target_x = (rand() % (width()-12)) + 8;
    target_y = (rand() % (height()-20)) + 16;
    fillCircle(target_x, target_y, 4, RED);

    while (1) {


//        // get x and y acceleration
//        I2C_IF_Write(accelerometer_addr,&x_reg,1,0);
//        I2C_IF_Read(accelerometer_addr, &acc_x, 1);
//
////        I2C_IF_Write(accelerometer_addr,&y_reg,1,0);
////        I2C_IF_Read(accelerometer_addr, &acc_y, 1);
//
//        Report(" X: %d, Y: %d\n", acc_x, acc_y);
//
//        // erase previous ball
////        fillCircle(ball_x, ball_y, 4, BLACK);
//
//        // update ball position
////        ball_x += acc_x;
////        ball_y -= acc_y;
//
//        // check for edges
//        if (ball_x < 4)
//            ball_x = 4;
//        if (ball_x > width()-4)
//            ball_x = width()-4;
//        if (ball_y < 12)
//            ball_y = 12;
//        if (ball_y > height()-4)
//            ball_y = height()-4;
//
//
//        // put ball in new position
//        fillCircle(ball_x, ball_y, 4, GREEN);
//
//        // check if target reached
//        if (abs(ball_x-target_x) < 8 && abs(ball_y-target_y) < 8) {
//            score++;
//
//            // erase target
//            fillCircle(target_x, target_y, 4, BLACK);
//
//            // print new score
//            fillRect(0,0,width(), 8, BLACK);
//            setCursor(0,0);
//            sprintf(scoreStr, "Score: %d", score);
//            Outstr(scoreStr);
//
//
//            // put target in new random coordinates
//            target_x = (rand() % (width()-12)) + 8;
//            target_y = (rand() % (height()-20)) + 16;
//            fillCircle(target_x, target_y, 4, RED);

//        }

    };


}

