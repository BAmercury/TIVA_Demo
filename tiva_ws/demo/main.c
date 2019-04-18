#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif


/**
 * main.c
 */
int main(void)
{


    // Enable GPIO port used for on-board LED
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // Check to see if the peripheral access is enabled
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    // Enable the GPIO Pins of Port F, Pin 1 is R, Pin 2 is G, Pin 3 is B
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

    // Main Loop
    while(1)
    {
        // Turn on the Red LED
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1);
        // Delay for a bit
        SysCtlDelay(2000000);
        // Turn on the Green
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_2);
        // Delay
        SysCtlDelay(2000000);
        // Turn on the Blue
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_3);
        // Delay
        SysCtlDelay(2000000);
    }


}
