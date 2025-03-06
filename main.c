//*****************************************************************************
//
// Application Name - Final Project
// Authors - Nadav Weinberger and Khaiber Amin
//
//
//*****************************************************************************


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
#include <stdint.h>
#include <math.h>
#include <stdbool.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "hw_apps_rcm.h"
#include "hw_nvic.h"
#include "spi.h"
#include "gpio.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "systick.h"

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

#define RIGHT    0
#define UP_RIGHT 45
#define UP       90
#define UP_LEFT  135
#define LEFT     180
#define DOWN_LEFT 225
#define DOWN     270
#define DOWN_RIGHT 315
#define FIRE_COOLDOWN 25
#define PLAYER_BULLET_SPEED 10


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif

//SYSTICK STUFF

// some helpful macros for systick

// the cc3200's fixed clock frequency of 80 MHz
// note the use of ULL to indicate an unsigned long long constant
#define SYSCLKFREQ 80000000ULL

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     100

// macro to convert ticks to microseconds
#define TICKS_TO_US(ticks) \
    ((((ticks) / SYSCLKFREQ) * 1000000ULL) + \
    ((((ticks) % SYSCLKFREQ) * 1000000ULL) / SYSCLKFREQ))\

// macro to convert microseconds to ticks
#define US_TO_TICKS(us) ((SYSCLKFREQ / 1000000ULL) * (us))

// systick reload value set to 40ms period
// (PERIOD_SEC) * (SYSCLKFREQ) = PERIOD_TICKS
#define SYSTICK_RELOAD_VAL 3200000UL

// track systick counter periods elapsed
// if it is not 0, we know the transmission ended
volatile int systick_cnt = 0;

extern void (* const g_pfnVectors[])(void);


volatile bool signalStarted = false;
volatile bool leaderEncountered = false;
volatile bool dataReady = false;
volatile bool sysTickTimeOut = false;
volatile int bitCounter = 0;
volatile unsigned long data = 0;

typedef struct {
    int x, y;
    int direction;  // This will be in the range of [0, 360) to represent direction in degrees
} Projectile;

#define MAX_PROJECTILES 5  // Maximum number of projectiles in flight at once

Projectile projectiles[MAX_PROJECTILES];
int num_projectiles = 0;





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

    // Enable Processor
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

static void SysTickHandler(void) {
    // increment every time the systick handler fires
    systick_cnt++;
    sysTickTimeOut = true;
}

static void SysTickInit(void) {

    // configure the reset value for the systick countdown register
    MAP_SysTickPeriodSet(SYSTICK_RELOAD_VAL);

    // register interrupts on the systick module
    MAP_SysTickIntRegister(SysTickHandler);

    // enable interrupts on systick
    // (trigger SysTickHandler when countdown reaches 0)
    MAP_SysTickIntEnable();

    // enable the systick module itself
    MAP_SysTickEnable();
}


static inline void SysTickReset(void) {
    // any write to the ST_CURRENT register clears it
    // after clearing it automatically gets reset without
    // triggering exception logic
    // see reference manual section 3.2.1
    HWREG(NVIC_ST_CURRENT) = 1;

    // clear the global count variable
//    systick_cnt = 0;
}



void Uart1IntHandler() {
    char buffer[16];
    char c = MAP_UARTCharGet(UARTA1_BASE);
    int idx = 0;

    while (c != '\0') {
        buffer[idx++] = c;
        c = MAP_UARTCharGet(UARTA1_BASE);
    }
    buffer[idx] = '\0';

    //Outstr("type");

    Outstr(buffer);

}

static void GPIOIntHandler(void) {
    if (sysTickTimeOut)
    {
        sysTickTimeOut = false;
        SysTickReset();
        signalStarted = false;
        leaderEncountered = false;
    }

    unsigned long ulStatus;

    ulStatus = MAP_GPIOIntStatus (GPIOA3_BASE, true);
    MAP_GPIOIntClear(GPIOA3_BASE, ulStatus);        // clear interrupts on GPIOA3

    if (signalStarted) // read the current bit
    {
        // read the countdown register and compute elapsed cycles
        uint64_t delta = SYSTICK_RELOAD_VAL - SysTickValueGet();

        // convert elapsed cycles to microseconds
        uint64_t delta_us = TICKS_TO_US(delta);

        if (!leaderEncountered) //at 2nd falling edge (end of leader)
        {
            if (delta_us < 14000 && delta_us > 13000)
            {
                leaderEncountered = true;
                data = 0;
                dataReady = false;
            }
            else
            {
                signalStarted = false;
            }


        }

        else
        {
            // narrow -> 0
            if (delta_us > 1100 && delta_us < 1200)
            {
                // add 0 to data
//                Report("0");
            }

            // wide -> 1
            if (delta_us > 2200 && delta_us < 2300)
            {
                // add 1 to data
                data = data | (1 << bitCounter);
//                Report("1");
            }

            bitCounter ++;

            if (bitCounter >= 32)
            {
                dataReady = true;

                // set bools to false
                signalStarted = false;
                leaderEncountered = false;

                bitCounter = 0;
            }
        }


    }
    else // first falling edge of leader
        signalStarted = true;


    // reset the countdown register
    SysTickReset();
}

//FUNCTIONS FOR ROTATION BEGIN -----------------

//function to rotate a single point around a center by an angle
void rotate_point(int *x, int *y, int angle, int center_x, int center_y) {
    // Convert angle to radians
    double radian = angle * (M_PI / 180.0);

    // Calculate cosine and sine of the angle
    double cos_angle = cos(radian);
    double sin_angle = sin(radian);

    // Translate point to the origin
    int temp_x = *x - center_x;
    int temp_y = *y - center_y;

    // Apply rotation matrix
    int rotated_x = (int)(temp_x * cos_angle - temp_y * sin_angle + center_x);
    int rotated_y = (int)(temp_x * sin_angle + temp_y * cos_angle + center_y);

    // Update original point with rotated coordinates
    *x = rotated_x;
    *y = rotated_y;
}

void rotate_triangle(int *x1, int *y1, int *x2, int *y2, int *x3, int *y3, int angle, int centroid_x, int centroid_y) {
    // Rotate each vertex of the triangle
    rotate_point(x1, y1, angle, centroid_x, centroid_y);
    rotate_point(x2, y2, angle, centroid_x, centroid_y);
    rotate_point(x3, y3, angle, centroid_x, centroid_y);
}

//FUNCTIONS FOR CANNON POSITION -----------------

void drawCannon(int ball_x, int ball_y, int direction){
    switch(direction) {
        case UP:
            drawLine(ball_x, ball_y, ball_x, ball_y - 12, GREEN);
            break;
        case RIGHT:
            drawLine(ball_x, ball_y, ball_x + 12, ball_y, GREEN);
            break;
        case DOWN:
            drawLine(ball_x, ball_y, ball_x, ball_y + 12, GREEN);
            break;
        case LEFT:
            drawLine(ball_x, ball_y, ball_x - 12, ball_y, GREEN);
            break;
        case UP_LEFT:
            drawLine(ball_x, ball_y, ball_x - 12, ball_y - 12, GREEN);
            break;
        case UP_RIGHT:
            drawLine(ball_x, ball_y, ball_x + 12, ball_y - 12, GREEN);
            break;
        case DOWN_LEFT:
            drawLine(ball_x, ball_y, ball_x - 12, ball_y + 12, GREEN);
            break;
        case DOWN_RIGHT:
            drawLine(ball_x, ball_y, ball_x + 12, ball_y + 12, GREEN);
            break;


    }
}

void eraseCannon(int ball_x, int ball_y, int direction) {
    switch(direction) {
        case UP:
            drawLine(ball_x, ball_y, ball_x, ball_y - 12, BLACK);
            break;
        case RIGHT:
            drawLine(ball_x, ball_y, ball_x + 12, ball_y, BLACK);
            break;
        case DOWN:
            drawLine(ball_x, ball_y, ball_x, ball_y + 12, BLACK);
            break;
        case LEFT:
            drawLine(ball_x, ball_y, ball_x -12, ball_y, BLACK);
            break;
        case UP_LEFT:
            drawLine(ball_x, ball_y, ball_x - 12, ball_y - 12, BLACK);
            break;
        case UP_RIGHT:
            drawLine(ball_x, ball_y, ball_x + 12, ball_y - 12, BLACK);
            break;
        case DOWN_LEFT:
            drawLine(ball_x, ball_y, ball_x - 12, ball_y + 12, BLACK);
            break;
        case DOWN_RIGHT:
            drawLine(ball_x, ball_y, ball_x + 12, ball_y + 12, BLACK);
            break;
    }
}
int proj_velocity = 2;

//FUNCTIONS FOR FIRING CANNON

void fireProjectile(int ball_x, int ball_y, int cannonDir) {
    if (num_projectiles < MAX_PROJECTILES) {
        // Initialize the new projectile
        projectiles[num_projectiles].x = ball_x;  // Start at the cannon's position
        projectiles[num_projectiles].y = ball_y;
        projectiles[num_projectiles].direction = cannonDir;

        num_projectiles++;  // Increment the number of active projectiles
    }
}

void moveProjectiles() {
    int i;
    for (i = 0; i < num_projectiles; i++) {
        drawCircle(projectiles[i].x, projectiles[i].y, 3, BLACK);  // You can change the size/color

        // Move the projectile in the direction of cannonDir
        double radian = projectiles[i].direction * (M_PI / 180.0);
        int move_distance = PLAYER_BULLET_SPEED;  // Distance the projectile moves each update
        projectiles[i].x += (int)(move_distance * cos(radian));
        projectiles[i].y -= (int)(move_distance * sin(radian));  // Negative because screen coordinates go downwards

        // Check if the projectile goes off the screen (left, right, top, or bottom)
        if (projectiles[i].x < 0 || projectiles[i].x >= width() || projectiles[i].y < 0 || projectiles[i].y >= height()) {
            // Remove projectile by shifting the remaining projectiles down
            int j;
            for (j = i; j < num_projectiles - 1; j++) {
                projectiles[j] = projectiles[j + 1];
            }
            num_projectiles--;
            i--;  // Decrement the index to stay at the same position after removal
        } else {
            // Draw the projectile at its new position
            drawCircle(projectiles[i].x, projectiles[i].y, 3, WHITE);  // You can change the size/color
        }
    }
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

    // Initialize Board configurations
    BoardInit();

    // Muxing UART and SPI lines.
    PinMuxConfig();

    // Enable SysTick
    SysTickInit();


    // Enable the SPI module clock
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI,PRCM_RUN_MODE_CLK);

    // Initialising the Terminal.
    //InitTerm();

    // Clearing the Terminal.
    ClearTerm();

    // Reset the peripheral
    MAP_PRCMPeripheralReset(PRCM_GSPI);


    // Reset SPI
    MAP_SPIReset(GSPI_BASE);


    // Configure SPI interface
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

    // register GPIO Interrupt Handler
    MAP_GPIOIntRegister(GPIOA3_BASE, GPIOIntHandler);
    // configure falling edge
    MAP_GPIOIntTypeSet(GPIOA3_BASE, 0x40, GPIO_FALLING_EDGE);
    // clear interrupts on GPIOA0
    unsigned long ulStatus;
    ulStatus = MAP_GPIOIntStatus(GPIOA3_BASE, false);
    MAP_GPIOIntClear(GPIOA3_BASE, ulStatus);
    // enable GPIO interrupt
    MAP_GPIOIntEnable(GPIOA3_BASE, 0x40);

    // Initialize UART Terminal
    InitTerm();

    // Clear UART Terminal
    ClearTerm();

    Adafruit_Init();
    fillScreen(BLACK);

    unsigned char accelerometer_addr = 0x18;
    unsigned char x_reg = 0x03;
    unsigned char y_reg = 0x05;

    signed char acc_x, acc_y;

    int score = 0;
    char scoreStr[20];
    sprintf(scoreStr, "Score: %d", score);
    Outstr(scoreStr);



//    // put tank in center initially
    int x1 = width()/2 - 5, y1 = height()/2 + 12;
    int x2 = width()/2 + 5, y2 = height()/2 + 12;
    int x3 = width()/2, y3 = height()/2; // front of tank

    int ball_x = 64;
    int ball_y = 64;

    int centroid_x = (x1 + x2 + x3) / 3;
    int centroid_y = (y1 + y2 + y3) / 3;

    //drawTriangle(x1, y1, x2, y2, x3, y3, GREEN);


    int target_x, target_y;

    // put target in random coordinates
    target_x = (rand() % (width()-12)) + 8;
    target_y = (rand() % (height()-20)) + 16;
    fillCircle(target_x, target_y, 4, RED);

    int cannonDir = 45;

    //IR STUFF
    unsigned long localData;
    //char lastButtonPressed[10];

    //int charX = 0;
    //int charY = height()/2;

    //char transmissionStr[16];
    //int strIdx = 0;

    //int pressCounter; // used for cycling thru chars

    //bool numericModeActive = false;

    SysTickReset();

    char* button;
    static bool buttonPressed = false;

    while (1) {
        Report("%d ", systick_cnt);

        //IR STUFF
        //int modAmount = 3;

                if (dataReady)
                {
                    localData = data;
                    data = 0;
                    dataReady = false;

                    //char* button;

                    switch (localData)
                    {
                    case 4211384160:
                        button = " ";
                        break;
                    case 3125124960:
                        button = "LeftButton";
                        break;
                    case 4010844000:
                        button = "FireButton"; //2
                        break;
                    case 3994132320:
                        button = "RightButton"; //3
                        break;
                    default:
                        button = "?";
                    }

                    //systick_cnt = 0;
                }


        // get x and y acceleration
        I2C_IF_Write(accelerometer_addr,&x_reg,1,0);
        I2C_IF_Read(accelerometer_addr, &acc_x, 1);


        I2C_IF_Write(accelerometer_addr,&y_reg,1,0);
        I2C_IF_Read(accelerometer_addr, &acc_y, 1);

//        Report("Y: %d\n", acc_y);
//
//        int angle = (acc_y * 90) / 128;
//
//        drawTriangle(x1, y1, x2, y2, x3, y3, BLACK);
//
//
//        rotate_triangle(&x1, &y1, &x2, &y2, &x3, &y3, angle, centroid_x, centroid_y); //changes vertice coords too much (millions?)
//
//
//        drawTriangle(x1, y1, x2, y2, x3, y3, GREEN); //stops here




        //Report(" X: %d, Y: %d\n", acc_x, acc_y);

        //erase previous ball
        drawCircle(ball_x, ball_y, 6, BLACK);
        eraseCannon(ball_x, ball_y, cannonDir);


        if ( strcmp(button, "RightButton") == 0 && !buttonPressed) {
            cannonDir -= 45;
            if (cannonDir < 0) cannonDir = 315;
            buttonPressed = true;
        }
        else if (strcmp(button, "LeftButton") == 0 && !buttonPressed) {
            cannonDir += 45;
            if (cannonDir > 315) cannonDir = 0;
            buttonPressed = true;
        }
        else if (strcmp(button, "FireButton") == 0 && !buttonPressed && systick_cnt > FIRE_COOLDOWN) {

            fireProjectile(ball_x, ball_y, cannonDir);
            Report("FIRE");
            buttonPressed = true;
            systick_cnt = 0;
        }

        if (strcmp(button, "LeftButton") != 0 && strcmp(button, "RightButton") != 0 && strcmp(button, "FireButton") != 0) {
            buttonPressed = false;  // Reset the button press state
        }

        moveProjectiles();

        buttonPressed = false;
        Report("%s", button);

        button = " ";


//        cannonDir += 45;
//        if (cannonDir > 315) {
//            cannonDir = 0;
//        }

        //limit tank speed
        if (acc_x > 3) {
            acc_x = 3;
        }
        if (acc_y > 3) {
            acc_y = 3;
        }
        if (acc_x < -3) {
            acc_x = -3;
        }
        if (acc_y < -3) {
            acc_y = -3;
        }

         //update ball position
        ball_x += acc_x;
        ball_y -= acc_y;

        // check for edges
        if (ball_x < 4)
            ball_x = 4;
        if (ball_x > width()-4)
            ball_x = width()-4;
        if (ball_y < 12)
            ball_y = 12;
        if (ball_y > height()-4)
            ball_y = height()-4;


        // put ball in new position
        drawCircle(ball_x, ball_y, 6, GREEN);
        drawCannon(ball_x, ball_y, cannonDir);

        // check if target reached
        if (abs(ball_x-target_x) < 8 && abs(ball_y-target_y) < 8) {
            score++;

            // erase target
            fillCircle(target_x, target_y, 4, BLACK);

            // print new score
            fillRect(0,0,width(), 8, BLACK);
            setCursor(0,0);
            sprintf(scoreStr, "Score: %d", score);
            Outstr(scoreStr);


            // put target in new random coordinates
            target_x = (rand() % (width()-12)) + 8;
            target_y = (rand() % (height()-20)) + 16;
            fillCircle(target_x, target_y, 4, RED);

        }

    };


}

