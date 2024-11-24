// Every one of us (Kimi Pahkala, Petrus Ollikainen and Henri Seppälä) were responsible
// for a little bit of everything in the code. We unfortunately cannot exactly pin-point
// which code was written by whom, as we always coded in a group. We can presumably say that
// that everything we did here was with utmost care by us. :')

/* C Standard library */
#include <sensors/buzzer.h>
#include <stdio.h>
#include <time.h>
#include <string.h>

/* XDCtools files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/pin/PINCC26XX.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include "Board.h"
#include "sensors/opt3001.h"
#include "sensors/buzzer.h"

/* Music library, referenced files from */
/* https://projecthub.arduino.cc/tmekinyan/playing-popular-songs-with-arduino-and-a-buzzer-546f4a */
#include "pitches/pitches.h"

/* Music notes to apply to play Nokia theme song */
/* Referenced from https://github.com/hibit-dev/buzzer/blob/master/src/other/nokia/nokia.ino */

int melody[] = {
  NOTE_E5, NOTE_D5, NOTE_FS4, NOTE_GS4,
  NOTE_CS5, NOTE_B4, NOTE_D4, NOTE_E4,
  NOTE_B4, NOTE_A4, NOTE_CS4, NOTE_E4,
  NOTE_A4
};

int durations[] = {
  8, 8, 4, 4,
  8, 8, 4, 4,
  8, 8, 4, 4,
  2
};

/* Task */
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];

/* Buzzer config */
static PIN_Handle hBuzzer;
static PIN_State sBuzzer;
PIN_Config cBuzzer[] = {
  Board_BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
  PIN_TERMINATE
};

// Define global variables for UART task
int UART_ID = 0;
int SpaceCounter = 0;

// State Machine
enum state { WAITING=1, DATA_READY};
enum state programState = WAITING;

// Struct for giving an additional name for readability for
// MPU9250 sensors
typedef struct{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} SensorData;
SensorData sensor;

//Prototypes
void mpu9250_get_data(I2C_Handle *i2c, float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
void mpu9250_setup(I2C_Handle *i2c);
void musicLoop();
void playBuzzer(int freq, int time_buzzer, int time_sleep);

// MPU power pin global variables
static PIN_Handle hMpuPin;
static PIN_State  MpuPinState;

// Pinnien alustukset, molemmille pinneille oma konfiguraatio
// Vakio BOARD_BUTTON_0 vastaa toista painonappia
static PIN_Config MpuPinConfig[] = {
   Board_MPU_POWER  | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};

// Vakio Board_LED0 vastaa toista lediä
static PIN_Config ledConfig[] = {
   Board_LED0 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
   PIN_TERMINATE // Asetustaulukko lopetetaan aina tällä vakiolla
};

// MPU uses its own I2C interface
static const I2CCC26XX_I2CPinCfg i2cMPUCfg = {
    .pinSDA = Board_I2C0_SDA1,
    .pinSCL = Board_I2C0_SCL1
};

void buttonFxn(PIN_Handle handle, PIN_Id pinId) {

    // Vaihdetaan led-pinnin tilaa negaatiolla
       /*uint_t pinValue = PIN_getOutputValue( Board_LED0 );
       pinValue = !pinValue;
       PIN_setOutputValue( ledHandle, Board_LED0, pinValue );*/
}

/* Task Functions */
Void uartTaskFxn(UArg arg0, UArg arg1) {

      UART_Handle uart;
      UART_Params uartParams;

    //Initialize serial communication
      UART_Params_init(&uartParams);
      uartParams.writeDataMode = UART_DATA_TEXT;
      uartParams.readDataMode = UART_DATA_TEXT;
      uartParams.readEcho = UART_ECHO_OFF;
      uartParams.readMode = UART_MODE_BLOCKING;
      uartParams.baudRate = 9600; // nopeus 9600baud
      uartParams.dataLength = UART_LEN_8; // 8
      uartParams.parityType = UART_PAR_NONE; // n
      uartParams.stopBits = UART_STOP_ONE; // 1

      uart = UART_open(Board_UART0, &uartParams);
         if (uart == NULL) {
            System_abort("Error opening the UART");
         }

    while (1) {
        if (programState == DATA_READY) {
                   switch(UART_ID) {
                   case 1: // ID for x-axis
                       UART_write(uart, ".\r\n\0", 4);
                       UART_ID = 0;
                       SpaceCounter = 0;
                       break;
                   case 2: // ID for Y-axis
                       UART_write(uart, "-\r\n\0", 4);
                       UART_ID = 0;
                       SpaceCounter = 0;
                       break;
                   case 3: // ID for Z-axis
                       UART_write(uart, " \r\n\0", 4);
                       UART_ID = 0;
                       if (SpaceCounter == -1) {
                           musicLoop();
                       }
                       SpaceCounter++;
                       break;
                   }
                   programState = WAITING;
               }

        Task_sleep(100000 / Clock_tickPeriod); // 100ms
    }
}

Void buzzerTaskFxn(UArg arg0, UArg arg1) {

    while (1) {
        buzzerOpen(hBuzzer);
        buzzerSetFrequency(2000);
        Task_sleep(50000 / Clock_tickPeriod);
        buzzerClose();

        Task_sleep(950000 / Clock_tickPeriod);
      }
}

// Function that is responsible for a simple buzz
void playBuzzer(int freq, int time_buzzer, int time_sleep) {
    buzzerOpen(hBuzzer);
    buzzerSetFrequency(freq); //Frequency to be played on SensorTag's buzzer
    Task_sleep(time_buzzer / Clock_tickPeriod); // Amount of time that buzzer plays for (e.g. 80000 = 80ms)
    buzzerClose();
    Task_sleep(time_sleep / Clock_tickPeriod); // Amount of time that this task sleeps after playing buzz (e.g. 400000 = 400ms)
}

// Function, that utilizes a "for loop" to play a tune through the buzzer
Void musicLoop() {
  int size = sizeof(durations) / sizeof(durations[0]);
  int note;
  for (note = 0; note < size; note++) {
    //Open Buzzer
    buzzerOpen(hBuzzer);

    //Input the specific frequency to be played
    buzzerSetFrequency(melody[note]);

    //This sleep is responsible for the duration of the tone to be palyed
    int duration = ((1000 / durations[note]) * 1000);
    Task_sleep(duration / Clock_tickPeriod);

    // Stop the specific tone from playing
    buzzerClose();

    //To seperate the tones from playing one after another, set a delay
    //After playing tone. In this case, 130% of tone duration
    int pauseBetweenNotes = duration * 1.30;
    Task_sleep(pauseBetweenNotes / Clock_tickPeriod);
  }
}

Void sensorTaskFxn(UArg arg0, UArg arg1) {

    I2C_Handle      i2cMPU;
    I2C_Params      i2cMPUParams;

    // Open I2C bus
    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // MPU Power On
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
       Task_sleep(100000 / Clock_tickPeriod);
       System_printf("MPU9250: Power ON\n");
       System_flush();

    // Open I2C connection
    i2cMPU = I2C_open(Board_I2C, &i2cMPUParams);
    if (i2cMPU == NULL) {
        System_abort("Error Initializing I2C\n");
    }

    // MPU setup and calibration
        System_printf("MPU9250: Setup and calibration...\n");
        System_flush();

        mpu9250_setup(&i2cMPU);

        System_printf("MPU9250: Setup and calibration OK\n");
        System_flush();

        Task_sleep(10000 / Clock_tickPeriod);

        while (1) {
            mpu9250_get_data(&i2cMPU, &sensor.ax, &sensor.ay, &sensor.az, &sensor.gx, &sensor.gy, &sensor.gz);
            // Keep reading data, until one of the determined sensors exceed the threshold
            if (programState == WAITING) {

                // Below are required thresholds for different movenents on set axises.
                // They are responsible for setting different ID's for UART to print

                if (sensor.ax > 0.5) { // When X-axis acc exceeds this threshold, goes to the below functions
                    UART_ID = 1; // Set identifier for UART Task
                    playBuzzer(2000, 150000, 250000);
                    programState = DATA_READY;

                } else if (sensor.ay > 0.5) { // When Y-axis acc exceeds this threshold, goes to the below functions
                    UART_ID = 2; // Set identifier for UART Task
                    playBuzzer (2000, 400000, 100000);
                    programState = DATA_READY;

                } else if (sensor.az > -0.5) { // When Z-axis acc exceeds this threshold, goes to the below functions
                    UART_ID = 3; // Set identifier for UART Task
                    if (SpaceCounter == 2) {
                        playBuzzer(3000, 50000, 250000); // Play thrice to indicate three spaces have been written
                        playBuzzer(3000, 50000, 250000);
                        playBuzzer(3000, 50000, 250000);
                        SpaceCounter = -1; // Indication to play music in UartTask
                    }
                    else if (SpaceCounter == 1) {
                        playBuzzer(3000, 50000, 250000);
                        playBuzzer(3000, 50000, 250000); // Play twice to indicate two spaces have been written
                    } else {
                        playBuzzer(3000, 50000, 250000); // Play once to indicate one space have been written
                    }
                    programState = DATA_READY;

                // Below are thresholds for handling error data. For example, the SensorTag is in a wrong position according to acceleration
                } else { // Default sleep, when no matching criteria are met
                    Task_sleep(100000 / Clock_tickPeriod); // 100ms
                }

                }
    }
}

Int main(void) {

    // Task variables
    Task_Handle sensorTaskHandle;
    Task_Params sensorTaskParams;
    Task_Handle uartTaskHandle;
    Task_Params uartTaskParams;

    // Initialize board
    Board_initGeneral();
    Board_initI2C();
    
    // Initialize UART
    Board_initUART();

    // Open MPU power pin
    hMpuPin = PIN_open(&MpuPinState, MpuPinConfig);
    if (hMpuPin == NULL) {
         System_abort("Pin open failed!");
    }

    /* Task */
    Task_Params_init(&sensorTaskParams);
    sensorTaskParams.stackSize = STACKSIZE;
    sensorTaskParams.stack = &sensorTaskStack;
    sensorTaskParams.priority=2;
    sensorTaskHandle = Task_create(sensorTaskFxn, &sensorTaskParams, NULL);
    if (sensorTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    Task_Params_init(&uartTaskParams);
    uartTaskParams.stackSize = STACKSIZE;
    uartTaskParams.stack = &uartTaskStack;
    uartTaskParams.priority=2;
    uartTaskHandle = Task_create(uartTaskFxn, &uartTaskParams, NULL);
    if (uartTaskHandle == NULL) {
        System_abort("Task create failed!");
    }

    /* Sanity check */
    System_printf("Hello world!\n");
    System_flush();

    /* Start BIOS */
    BIOS_start();

    return (0);
}




