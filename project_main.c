/* C Standard library */
#include <stdio.h>
#include <time.h>

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

/* Task */
#define STACKSIZE 2048
Char sensorTaskStack[STACKSIZE];
Char uartTaskStack[STACKSIZE];

// JTKJ: Teht�v� 3. Tilakoneen esittely
// JTKJ: Exercise 3. Definition of the state machine
enum state { WAITING=1, DATA_READY };
enum state programState = WAITING;

typedef struct{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
} SensorData;
SensorData gyro; // Määritellään muuttuja gyro yhteiseksi

void mpu9250_get_data(I2C_Handle *i2c, float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
void mpu9250_setup(I2C_Handle *i2c);

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

    // JTKJ: Teht�v� 1. Vilkuta jompaa kumpaa ledi�
    // JTKJ: Exercise 1. Blink either led of the device
    // Vaihdetaan led-pinnin tilaa negaatiolla
       /*uint_t pinValue = PIN_getOutputValue( Board_LED0 );
       pinValue = !pinValue;
       PIN_setOutputValue( ledHandle, Board_LED0, pinValue );*/
}

/* Task Functions */
Void uartTaskFxn(UArg arg0, UArg arg1) {

    // JTKJ: Teht�v� 4. Lis�� UARTin alustus: 9600,8n1
    // JTKJ: Exercise 4. Setup here UART connection as 9600,8n1
      UART_Handle uart;
      UART_Params uartParams;

    //Initialize serial communication
      UART_Params_init(&uartParams);
      uartParams.writeDataMode = UART_DATA_TEXT;
      uartParams.readDataMode = UART_DATA_TEXT;
      uartParams.readEcho = UART_ECHO_OFF;
      uartParams.readMode=UART_MODE_BLOCKING;
      uartParams.baudRate = 9600; // nopeus 9600baud
      uartParams.dataLength = UART_LEN_8; // 8
      uartParams.parityType = UART_PAR_NONE; // n
      uartParams.stopBits = UART_STOP_ONE; // 1

      uart = UART_open(Board_UART0, &uartParams);
         if (uart == NULL) {
            System_abort("Error opening the UART");
         }

    while (1) {

        // JTKJ: Teht�v� 3. Kun tila on oikea, tulosta sensoridata merkkijonossa debug-ikkunaan
        //       Muista tilamuutos
        // JTKJ: Exercise 3. Print out sensor data as string to debug window if the state is correct
        //       Remember to modify state

        // JTKJ: Teht�v� 4. L�het� sama merkkijono UARTilla
        // JTKJ: Exercise 4. Send the same sensor data string with UART

        // Just for sanity check for exercise, you can comment this out
        /*System_printf("uartTask\n");
        System_flush();*/

        // Once per second, you can modify this
        Task_sleep(100000 / Clock_tickPeriod);
    }
}

Void sensorTaskFxn(UArg arg0, UArg arg1) {

    I2C_Handle      i2cMPU;
    I2C_Params      i2cMPUParams;

    // JTKJ: Teht�v� 2. Avaa i2c-v�yl� taskin k�ytt��n
    // JTKJ: Exercise 2. Open the i2c bus
    I2C_Params_init(&i2cMPUParams);
    i2cMPUParams.bitRate = I2C_400kHz;
    i2cMPUParams.custom = (uintptr_t)&i2cMPUCfg;

    // MPU Power On
    PIN_setOutputValue(hMpuPin,Board_MPU_POWER, Board_MPU_POWER_ON);

    // Wait 100ms for the MPU sensor to power up
       Task_sleep(100000 / Clock_tickPeriod);
       System_printf("MPU9250: Power ON\n");
       System_flush();

    // Avataan yhteys
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
    mpu9250_setup(&i2cMPU);
    int elapsedSeconds = 0;
    while (1) {
            int tulostettu = 0;
            int tulokset = 0;
            Char mpu[128];
            // JTKJ: Teht�v� 2. Lue sensorilta dataa ja tulosta se Debug-ikkunaan merkkijonona
            // JTKJ: Exercise 2. Read sensor data and print it to the Debug window as string
            mpu9250_get_data(&i2cMPU, &gyro.ax, &gyro.ay, &gyro.az, &gyro.gx, &gyro.gy, &gyro.gz);
            // Get current time
            if ((gyro.ax > 0.5) && (gyro.ax <= 1.2)) {
                tulokset = 1; // Aseta tulokset 1:ksi
            } else if ((gyro.ay > 0.5) && (gyro.ay <= 1.2)) {
                tulokset = 2; // Aseta tulokset 2:ksi
            } else if ((gyro.az > -0.5) && (gyro.az <= 0.2)) {
                tulokset = 3; // Aseta tulokset 3:ksi
            }

            // Käytä switch-lauseketta tulosten käsittelyyn
            switch (tulokset) {
                case 1:
                    if (!tulostettu) {
                        sprintf(mpu, ". %f\n", gyro.ax); // Tulosta gyro.ax
                        System_printf(mpu);
                        tulostettu = 1; // Merkitse tulostetuksi
                    }
                    break; // Poistu case 1:stä

                case 2:
                    if (!tulostettu) {
                        sprintf(mpu, ", %f\n", gyro.ay); // Tulosta gyro.ay
                        System_printf(mpu);
                        tulostettu = 1; // Merkitse tulostetuksi
                    }
                    break; // Poistu case 2:sta

                case 3:
                    if (!tulostettu) {
                        sprintf(mpu, "- %f\n", gyro.az); // Tulosta gyro.az
                        System_printf(mpu);
                        tulostettu = 1; // Merkitse tulostetuksi
                    }
                    break; // Poistu case 3:sta

                default:
                    // Jos mikään ehto ei täyty, voit käsitellä muita tapauksia
                    break;
            }

            elapsedSeconds++;
            // Once per second, you can modify this
            Task_sleep(50000 / Clock_tickPeriod);

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
    
    // JTKJ: Teht�v� 4. Ota UART k�ytt��n ohjelmassa
    // JTKJ: Exercise 4. Initialize UART
    Board_initUART();
    // JTKJ: Teht�v� 1. Ota painonappi ja ledi ohjelman k�ytt��n
    //       Muista rekister�id� keskeytyksen k�sittelij� painonapille
    // JTKJ: Exercise 1. Open the button and led pins
    //       Remember to register the above interrupt handler for button
    // Ledi käyttöön ohjelmassa
    /*ledHandle = PIN_open( &ledState, ledConfig );
    if(!ledHandle) {
       System_abort("Error initializing LED pin\n");
    }*/

    // Painonappi käyttöön ohjelmassa
    /*buttonHandle = PIN_open(&buttonState, buttonConfig);
    if(!buttonHandle) {
       System_abort("Error initializing button pin\n");
    }*/

    // Painonapille keskeytyksen käsittellijä
    /*if (PIN_registerIntCb(buttonHandle, &buttonFxn) != 0) {
       System_abort("Error registering button callback function");
    }*/

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




