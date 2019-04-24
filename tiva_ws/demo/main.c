#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/fpu.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
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


void configure_uart_serial(void)
{
    // Enable the GPIO Peripheral used for the UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Enable UART0 Module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    // Wait for the UART0 module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0))
    {
    }

    // Configure GPIO Pins for UART Mode
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART. Set baud rate,
    // The ui32Config is a logical OR of three values:
    // Number of Data Bits | Number of Stop Bits | and Parity
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_PAR_NONE |
            UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8));

    // Finally, enable UART
    UARTEnable(UART0_BASE);





    // Use the internal 16 Mhz oscillator as the UART Clock Source
    // We need a clock source for the baud rate generator
    //UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console IO
    //UARTStdioConfig(0, 115200, 16000000);


    // Print confirmation message
    //UARTprintf("UART is up and running\n");

}


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

    // Configure the UART Module
    configure_uart_serial();
    // Print an array of char's
    char *s="hello world\n";
    while (*s) UARTCharPut(UART0_BASE, *s++);

    // We can print the same thing if we have the uartstdio.c library
    //UARTprintf("Hello World\n");



    // Main Loop
    while(1)
    {
        // Turn on the Red LED
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_1);
        // Delay for a bit
        SysCtlDelay(SysCtlClockGet() / 10 / 3);
        // Turn on the Green
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_2);
        // Delay
        SysCtlDelay(SysCtlClockGet() / 10 / 3);
        // Turn on the Blue
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_3);
        // Delay
        SysCtlDelay(SysCtlClockGet() / 10 / 3);

    }


}
