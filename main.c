//*****************************************************************************
//
// Application Name - Final Project
// Authors - Nadav Weinberger and Khaiber Amin
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
#include <time.h>


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

// oled includes
#include "oled/Adafruit_SSD1351.h"
#include "oled/oled_test.h"
#include "oled/Adafruit_GFX.h"
#include "oled/glcdfont.h"

// title art
#include "tank_art.h"

#include "parson.h"


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

#define MAX_ENEMIES 1

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
    bool isFriendlyProjectile;
} Projectile;

typedef struct {
  int xPos, yPos;
  int cannonDirection;
  int cooldown;
  int lastTimeFired;
  bool canFire;
  bool isAlive;
  //implement movement
} Enemy;

typedef struct {
   int x;
   int y;

} Point ;

#define MAX_PROJECTILES 5  // Maximum number of projectiles in flight at once



Enemy enemies[MAX_ENEMIES];
Projectile projectiles[MAX_PROJECTILES];
int num_projectiles = 0;
int num_enemies = 0;
bool enemyDefeated = true;




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
    HWREG(NVIC_ST_CURRENT) = 1;
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

//FUNCTIONS FOR CANNON POSITION -----------------

void drawCannon(int ball_x, int ball_y, int direction, uint16_t color){
    switch(direction) {
        case UP:
            drawLine(ball_x, ball_y, ball_x, ball_y - 12, color);
            break;
        case RIGHT:
            drawLine(ball_x, ball_y, ball_x + 12, ball_y, color);
            break;
        case DOWN:
            drawLine(ball_x, ball_y, ball_x, ball_y + 12, color);
            break;
        case LEFT:
            drawLine(ball_x, ball_y, ball_x - 12, ball_y, color);
            break;
        case UP_LEFT:
            drawLine(ball_x, ball_y, ball_x - 12, ball_y - 12, color);
            break;
        case UP_RIGHT:
            drawLine(ball_x, ball_y, ball_x + 12, ball_y - 12, color);
            break;
        case DOWN_LEFT:
            drawLine(ball_x, ball_y, ball_x - 12, ball_y + 12, color);
            break;
        case DOWN_RIGHT:
            drawLine(ball_x, ball_y, ball_x + 12, ball_y + 12, color);
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
        projectiles[num_projectiles].isFriendlyProjectile = true;

        num_projectiles++;  // Increment the number of active projectiles
    }
}

void fireEnemyProjectile(Enemy *enemy) {
    //Report("enemy.lastTimeFired: %d", enemy->lastTimeFired);
    //Report("enemy.cooldown: %d", enemy->cooldown);


    if (num_projectiles < MAX_PROJECTILES && enemy->lastTimeFired > enemy->cooldown) {
        //Report("Enemy Firing");
        projectiles[num_projectiles].x = enemy->xPos;  // Start at the cannon's position
        projectiles[num_projectiles].y = enemy->yPos;
        projectiles[num_projectiles].direction = enemy->cannonDirection;
        projectiles[num_projectiles].isFriendlyProjectile = false;
        enemy->lastTimeFired = 0;


        num_projectiles++;
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
            uint16_t color;
            // Draw the projectile at its new position
            if (projectiles[i].isFriendlyProjectile) {
                color = WHITE;
            } else {color = YELLOW;}
            drawCircle(projectiles[i].x, projectiles[i].y, 3, color);  // You can change the size/color
        }
    }
}

//TITLE PAGE FUNCTIONS

void titlePage() {
    // print title
    setTextColor(RED, BLACK);
    setCursor(30,0);
    Outstr("TANK GAME");

    // draw title art
    drawXBitmap(0, -5, tank_art_bits, 128, 128, WHITE);

    // print button prompt
    setTextColor(WHITE, BLACK);
    setCursor(0, 121);
    Outstr("Press any button");

    // wait for button press
    while (1) {
        if (dataReady) {
            break;
        }
    }
    dataReady = false;
    // clear screen
    setCursor(0,0);
    fillScreen(BLACK);
}

//ENEMY TANK FUNCTIONS

void spawnEnemy(Enemy* enemy) {


    enemy->xPos = (rand() % (width()-12)) + 8;
    enemy->yPos = (rand() % (height()-20)) + 16;
    enemy->cannonDirection = DOWN;
    enemy->cooldown = 25;
    enemy ->isAlive = true;
    enemy->lastTimeFired = 0;
    drawCircle(enemy->xPos, enemy->yPos, 6, RED);
    drawCannon(enemy->xPos, enemy->yPos, enemy->cannonDirection, RED);
    num_enemies++;

}


Point getRandCoords() {
    Point pt;
    pt.x = rand() % 128;
    pt.y = rand() % 128;
    return pt;
}

void newTargetPoint() {

}

void updateEnemyPosition(Enemy* enemy, Point targetPoint) {
    //Report("EnemyXPos: %d ", enemy->xPos);
    //Report("EnemyYPos: %d ", enemy->yPos);
    if (enemy->xPos > targetPoint.x) {
        enemy->xPos--;
    } else if (enemy->xPos < targetPoint.x) {
        enemy->xPos++;
    }
    if (enemy->yPos > targetPoint.y) {
        enemy->yPos--;
    } else if (enemy->yPos < targetPoint.y) {
        enemy->yPos++;
    }
//    if (enemy->xPos == targetPoint.x && enemy->yPos == targetPoint.y) {
//
//    }




}

void updateEnemy(Enemy* enemy, int playerXPos, int playerYPos) {
    // Calculate the angle between the enemy and the player

    if (enemy->lastTimeFired <= enemy->cooldown) {
        enemy->lastTimeFired++;
    }
    int angle = atan2(playerYPos - enemy->yPos, playerXPos - enemy->xPos) * (180 / M_PI);

    // Normalize the angle to be in the range of [0, 360)
    if (angle < 0) {
        angle += 360;
    }

    int oldCannonDir = enemy->cannonDirection;
    int newCannonDir;
    //eraseCannon(enemy->xPos, enemy->yPos, enemy->cannonDirection);
    // Determine the nearest direction (in 45-degree increments)
    if (angle >= 337.5 || angle < 22.5) {
        enemy->cannonDirection = RIGHT;
    } else if (angle >= 22.5 && angle < 67.5) {
        enemy->cannonDirection = DOWN_RIGHT;
    } else if (angle >= 67.5 && angle < 112.5) {
        enemy->cannonDirection = DOWN;
    } else if (angle >= 112.5 && angle < 157.5) {
        enemy->cannonDirection = DOWN_LEFT;
    } else if (angle >= 157.5 && angle < 202.5) {
        enemy->cannonDirection = LEFT;
    } else if (angle >= 202.5 && angle < 247.5) {
        enemy->cannonDirection = UP_LEFT;
    } else if (angle >= 247.5 && angle < 292.5) {
        enemy->cannonDirection = UP;
    } else if (angle >= 292.5 && angle < 337.5) {
        enemy->cannonDirection = UP_RIGHT;
    }

    newCannonDir = enemy->cannonDirection;
    if (newCannonDir != oldCannonDir) {
        eraseCannon(enemy->xPos, enemy->yPos, oldCannonDir);
        drawCannon(enemy->xPos, enemy->yPos, enemy->cannonDirection, RED); // Redraw the enemy's cannon facing the player
    }

    //erase previous enemy location
    //drawCircle(enemy->xPos, enemy->yPos, 6, BLACK);
    //eraseCannon(enemy->xPos, enemy->yPos, enemy->cannonDirection);
    //updateEnemyPosition(enemy, getRandCoords());
    //drawCircle(enemy->xPos, enemy->yPos, 6, RED);
    //drawCannon(enemy->xPos, enemy->yPos, enemy->cannonDirection); // Redraw the enemy's cannon facing the player
}


void checkIfPlayerHit(int playerXPos, int playerYPos, Projectile projectiles[]) {
    int i;
    for (i = 0; i < num_projectiles; i++) {
        if (projectiles[i].isFriendlyProjectile == false) {
            if (abs(projectiles[i].x - playerXPos) < 8 && abs(projectiles[i].y - playerYPos) < 8) {
                Report("HIT ");

                int j;
                for (j = i; j < num_projectiles - 1; j++) {
                    projectiles[j] = projectiles[j + 1];
                }
                num_projectiles--;
                i--;  // Decrement the index to stay at the same position after removal
                gameOverScreen();

            }
        }
    }
}

int checkIfEnemyHit(Enemy* enemy, Projectile projectiles[], int score) {
    int i;
    for (i = 0; i < num_projectiles; i++) {
        if (projectiles[i].isFriendlyProjectile == true) {
            if (abs(projectiles[i].x - enemy->xPos) < 8 && abs(projectiles[i].y - enemy->yPos) < 8) {
                Report("KILL ");

                enemy->isAlive = false;

                drawCircle(projectiles[i].x, projectiles[i].y, 3, BLACK);
                drawCircle(enemy->xPos, enemy->yPos, 6, BLACK);
                eraseCannon(enemy->xPos, enemy->yPos, enemy->cannonDirection);

                // Clean up the enemy from the enemies array and adjust the number of enemies
                int j;
                for (j = 0; j < num_enemies; j++) {
                    if (&enemies[j] == enemy) {
                        // Shift all enemies after the dead enemy down by one position
                        int k;
                        for (k = j; k < num_enemies - 1; k++) {
                            enemies[k] = enemies[k + 1];
                        }
                        // Decrement the number of enemies
                        num_enemies--;
                        break;
                    }
                }

                // Remove the projectile that hit the enemy from the projectiles list

                for (j = i; j < num_projectiles - 1; j++) {
                    projectiles[j] = projectiles[j + 1];
                }
                num_projectiles--;  // Decrement the number of projectiles

                i--;

                enemyDefeated = true;
                score++;
                Report("Score: %d ", score);
                //print new score
                //fillRect(0,0,width(), 8, BLACK);
                //char scoreStr[20];
                //setCursor(0,0);
                //sprintf(scoreStr, "Score: %d", score);
                //Outstr(scoreStr);
                return score;

            }
        }
    }
}


//GAME OVER SCREEN FUNCTION

void exitGame() {
    //leaderbaord pg
    main();
}

void gameOverScreen() {
    // print gameover screen
    setTextColor(RED, BLACK);
    setTextSize(3);
    setCursor(20,30);
    Outstr("GAME");
    setCursor(20,60);
    Outstr("OVER");

    // print button prompt
    setTextSize(1);
    setTextColor(WHITE, BLACK);
    setCursor(0, 111);
    Outstr("Press any button ");
    setCursor(0, 121);
    Outstr(" to CONTINUE");

    // wait for button press
    while (1) {
        if (dataReady) {
            dataReady = false;
            exitGame();

            break;
        }
    }
    // clear screen
    fillScreen(BLACK);
}

/**
* create a json-formatted string using the parson library
* given the score and name
*/
void create_post_json(int score, char* name)
{
    // Create root JSON object
    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);

    // create 'state' object
    JSON_Value *state_value = json_value_init_object();
    JSON_Object *state_object = json_value_get_object(state_value);

    // crate 'desired' object
    JSON_Value *desired_value = json_value_init_object();
    JSON_Object *desired_object = json_value_get_object(desired_value);

    // set attributes in 'desired'
    json_object_set_number(desired_object, "high-score", score);
    json_object_set_string(desired_object, "name", name);

    json_object_set_value(state_object, "desired", desired_value);
    json_object_set_value(root_object, "state", state_value);

    // Print the JSON object as a string
    char *json_string = json_serialize_to_string_pretty(root_value);
    Report("%s\n", json_string);

    // Clean up memory
    json_free_serialized_string(json_string);
    json_value_free(root_value);

    while (1) {}
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

    srand(time(NULL));
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

    create_post_json(15, "nadav");

    Adafruit_Init();
    fillScreen(BLACK);

    titlePage();

    unsigned char accelerometer_addr = 0x18;
    unsigned char x_reg = 0x03;
    unsigned char y_reg = 0x05;

    signed char acc_x, acc_y;

    int score = 0;
    char scoreStr[20];
    sprintf(scoreStr, "Score: %d", score);
    Outstr(scoreStr);

    int ball_x = 64;
    int ball_y = 64;


    int cannonDir = 90;

    //IR STUFF
    unsigned long localData;

    SysTickReset();

    char* button;
    static bool buttonPressed = false;

    //initial enemy spawned
    Enemy enemy;
    spawnEnemy(&enemy);
    dataReady = true;

    while (1) {
        //Report("%d ", systick_cnt);
        if (enemyDefeated == true) {
            spawnEnemy(&enemy);
            enemyDefeated = false;
        }

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
        I2C_IF_Read(accelerometer_addr, &acc_y, 1);


        I2C_IF_Write(accelerometer_addr,&y_reg,1,0);
        I2C_IF_Read(accelerometer_addr, &acc_x, 1);

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
            //Report("FIRE");
            buttonPressed = true;
            systick_cnt = 0;
        }

        if (strcmp(button, "LeftButton") != 0 && strcmp(button, "RightButton") != 0 && strcmp(button, "FireButton") != 0) {
            buttonPressed = false;  // Reset the button press state
        }

        moveProjectiles();

        buttonPressed = false;
        //Report("%s", button);

        button = " ";
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
        ball_y += acc_y;

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
        drawCannon(ball_x, ball_y, cannonDir, GREEN);

        //drawCircle(&enemy.xPos, &enemy.yPos, 6, BLACK);
        //eraseCannon(enemy.xPos, enemy.yPos, enemy.cannonDirection);

        if (enemy.isAlive) {
            //Report("Enemy is Alive ");
            updateEnemy(&enemy, ball_x, ball_y);
            fireEnemyProjectile(&enemy);
        }


        checkIfPlayerHit(ball_x, ball_y, projectiles);
        score = checkIfEnemyHit(&enemy, projectiles, score);

//            // print new score
//            fillRect(0,0,width(), 8, BLACK);
            //setCursor(0,0);
            //sprintf(scoreStr, "Score: %d", score);
            //Outstr(scoreStr);
        }

}



