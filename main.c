//* Still has polarity LED bug

//-- FreeRTOS stuff ----------------------
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
void vTask_ADC(void * pvParameters);
void vTask_speed(void * pvParameters);
//----------------------------------------

#include "msp.h"
#include "stdlib.h"

// Configuration prototypes
void init_NVIC(void);
void init_encoder(void);
void init_pot(void);
void init_motor(void);
void init_ADC(void);
void init_display(void);
void init_Timer_A0(void);
void init_Timer_A1(void);

// display prototypes
void sseg_display(void);           // display counter digits on 7-segment display
void wait(uint32_t t);

void PORT3_IRQHandler(void);

// ADC prototypes
uint32_t get_ADC(void);

volatile float x0 = 0, y0 = 0, x1 = 0, y1 = 0, temp = 0;
volatile int16_t counter = 0;   // encoder angle counter
volatile int16_t speed = 0;
uint8_t display[4] = {0,0,0,0}; // 7-seg display array

const uint8_t look_up[10] = { // 7-segment display look up table
0b11000000,  // 0
0b11111001,  // 1
0b10100100,  // 2
0b10110000,  // 3
0b10011001,  // 4
0b10010010,  // 5
0b10000010,  // 6
0b11111000,  // 7
0b10000000,  // 8
0b10010000,  // 9
};

int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    init_NVIC();
    init_encoder();
    init_pot();
    init_motor();
    init_ADC();
    init_display();
    init_Timer_A0();

    xTaskCreate(vTask_ADC, "ADC", 512, NULL, 3, NULL); //create a task for all other jobs.
    xTaskCreate(vTask_speed, "speed", 512, NULL, 5, NULL); //create a task for calculating speed

    vTaskStartScheduler();

    while (1)
    {
    }

}

//  *******************TASK Definition***************************************

void vTask_ADC(void * pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xDelay_all = 10/portTICK_PERIOD_MS;
    // Initialize the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

   while (1)
    {
      vTaskDelayUntil(&xLastWakeTime, xDelay_all);

      temp = get_ADC();   //map 14-bit ADC to 75
      temp *= 75;
      temp /= 16384;

      TIMER_A0->CCR[1] = temp;
      TIMER_A0->CCR[2] = 75-temp;

    }
}

void vTask_speed(void * pvParameters)
{
    TickType_t xLastWakeTime;
    const TickType_t xDelay_speed = 2/(portTICK_PERIOD_MS);   // Interval = 2mS. You can change it to a longer interval.
    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

   while (1)
    {

      vTaskDelayUntil(&xLastWakeTime, xDelay_speed);

      speed = counter*200*60/400;
      counter = 0;

      // low pass algorithm,
      x0 = speed;
      y0 = 0.9691*y1 +0.03093*x0;

      speed = y0;
      y1 = y0;
      x1 = x0;

      sseg_display();

    }
}

//-- Functions
void init_NVIC(void){
    NVIC->ISER[1] = 1 << ((PORT3_IRQn) & 31); //enable P3 interrupt
}

void init_encoder(void){

    P3->DIR &= ~BIT6;   // phaseA input
    P3->IE  |= BIT6;    // enable P3.6 interrupt
    P3->IES &= ~BIT6;   // rising edge

    P5->DIR &= ~BIT3;   // phaseB input

}

void init_pot(void){

    P9->DIR |= BIT4;        // set P9.4 as OUTPUT to drive voltage to potentiometer
    P9->OUT |= BIT4;        // set potentiometer voltage high

}
void init_motor(void){

    P2->OUT &= ~(BIT4 | BIT5);
    P2->DIR |= BIT4 | BIT5;     // PWM output to motor
    P2->SEL0 |= BIT4 | BIT5;    // TA0.1 and TA0.2

}

void init_ADC(void){

    P5->DIR &= ~BIT1;   // ADC input from POT
    P5->SELC |= BIT1;   // analog input A4

    ADC14->CTL0 &= !ADC14_CTL0_ENC;

    ADC14->CTL0 |= ADC14_CTL0_PDIV__4    | // predivider to 4
                   ADC14_CTL0_SHS_0      | // sample-and-hold source select to ADC14SC bit
                   ADC14_CTL0_SHP        | // SAMPCON signal source to sampling timer
                   ADC14_CTL0_SSEL__MCLK | // MCLK as clock source
                   ADC14_CTL0_SHT0__32   | // sampling period to 32 ADC14CLK cycles for MEM0-7 and MEM14-31
                   ADC14_CTL0_ON;          // turn ADC on

    ADC14->CTL1 |= ADC14_CTL1_RES__14BIT;  // resolution to 14 bit

    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_4; // input channel A4

    ADC14->CTL0 |= ADC14_CTL0_ENC;
}
void init_display(void){

    P4->DIR = 0xFF;  // P4 is 7-segment LED output
    P8->DIR = 0xFF;  // P8 is display output
    P5->DIR |= BIT0; // P5.0 is red LED angle polarity indicator

}

void init_Timer_A0(void){

    // Set TA0CCR0 = 375kHz/5KHz = 75, PWM period
    TIMER_A0->CTL |= TIMER_A_CTL_SSEL__SMCLK | // SMCLK as clock source
                     TIMER_A_CTL_MC__UP;       // up mode
    TIMER_A0->CCR[0] = 75;                     // CC register value

    // Set TA0CCR1-2 as PWM output
    TIMER_A0->CCTL[1] |= TIMER_A_CCTLN_OUTMOD_6;    // Toggle-Set
    TIMER_A0->CCTL[2] |= TIMER_A_CCTLN_OUTMOD_6;    // Toggle-Set
}

void sseg_display(void){

    static uint8_t k = 0;
    uint16_t x = abs(speed);
    display[0] =  x/1000;
    display[1] = (x/100)%10;
    display[2] = (x/10)%10;
    display[3] =  x%10;

    // Display digit-k
    P4->OUT = 0xFF;                      // blank 7-seg display
    P8->OUT = 0xFF & ~(BIT5 >> k);       // enable k-th digit in 7-seg display
    P4->OUT = look_up[display[k]];       // display k-th digit in 7-seg display

    // increment k index
    k++;
    if (k >= 4){k = 0;}

    if(speed >= 0){
        P5->OUT |= BIT0;    // red LED on, negative angle (CCW)
    }else{
        P5->OUT &= ~BIT0;   // red LED off, positive angle (CW)
    }

}

void wait(uint32_t t){
    while(t > 0){t--;}
}

uint32_t get_ADC(void){

    // BIT 0-1 ADC14SC and ADC14ENC
    ADC14->CTL0 |= ( BIT0|BIT1 );   // start ADC14 conversion by setting both registers

    while(ADC14->CTL0 & BIT(16));   // ADC14BUSY, wait while ADC14 is busy

    return ADC14->MEM[0];           // get ADC14 result
}

void PORT3_IRQHandler(void){
    if(P3->IV & 0x0E){      // if phaseA is interrupt source (rising edge)
        if(P5->IN & BIT3){  // if phaseB is high
            counter--;      // decrement counter (CCW)
        }else{              // else
            counter++;      // increment counter (CW)
        }
    }
}

//-- FreeRTOS functions
void vApplicationMallocFailedHook(void)
{
    /* vApplicationMallocFailedHook() will only be called if
     configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
     function that will get called if a call to pvPortMalloc() fails.
     pvPortMalloc() is called internally by the kernel whenever a task, queue,
     timer or semaphore is created.  It is also called by various parts of the
     demo application.  If heap_1.c or heap_2.c are used, then the size of the
     heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
     FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
     to query the size of free heap space that remains (although it does not
     provide information on how the remaining heap might be fragmented). */
    taskDISABLE_INTERRUPTS()
    ;
    for (;;)
        ;
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
    /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
     to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
     task.  It is essential that code added to this hook function never attempts
     to block in any way (for example, call xQueueReceive() with a block time
     specified, or call vTaskDelay()).  If the application makes use of the
     vTaskDelete() API function (as this demo application does) then it is also
     important that vApplicationIdleHook() is permitted to return to its calling
     function, because it is the responsibility of the idle task to clean up
     memory allocated by the kernel to any task that has since been deleted. */
}

/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void) pcTaskName;
    (void) pxTask;

    /* Run time stack overflow checking is performed if
     configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
     function is called if a stack overflow is detected. */

    taskDISABLE_INTERRUPTS()
    ;
    for (;;)
        ;
}

/*-----------------------------------------------------------*/

void *malloc(size_t xSize)
{
    /* There should not be a heap defined, so trap any attempts to call
     malloc. */

    Interrupt_disableMaster();
    for (;;)
        ;
}

/*        ***********************************************   */

static void prvConfigureClocks( void )
{
    /* Set Flash wait state for high clock frequency.  Refer to datasheet for
    more details. */

    FlashCtl_setWaitState( FLASH_BANK0, 2 );
    FlashCtl_setWaitState( FLASH_BANK1, 2 );

    /* The full demo configures the clocks for maximum frequency, wheras the
    blinky demo uses a slower clock as it also uses low power features.  Maximum
    freqency also needs more voltage.

    From the datasheet:  For AM_LDO_VCORE1 and AM_DCDC_VCORE1 modes, the maximum
    CPU operating frequency is 48 MHz and maximum input clock frequency for
    peripherals is 24 MHz. */

    PCM_setCoreVoltageLevel( PCM_VCORE1 );
    CS_setDCOCenteredFrequency( CS_DCO_FREQUENCY_48 );
    CS_initClockSignal( CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_128 );
    CS_initClockSignal( CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
    CS_initClockSignal( CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
}
