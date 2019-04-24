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
#include "driverlib/pwm.h"
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
    // Parity| Number of Stop Bits | and Length of Data
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200, (UART_CONFIG_PAR_NONE |
            UART_CONFIG_STOP_ONE | UART_CONFIG_WLEN_8));

    // Finally, enable UART
    UARTEnable(UART0_BASE);

}

void configure_pwm_module(void)
{

    // Enable the Port Register for the PWM Pin that will be used (PB6)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Enable PWM0 Peripheral Module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    // Wait till module is ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
    {
    }
    // Configure the PB6 Pin for PWM
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);


    // Configure the PWM Generator for count down mode
    // With immediate updates to the parameters
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
}


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


    // Output a PWM signal
    configure_pwm_module();
    //
    // Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20
    // microseconds. For a 20 MHz clock, this translates to 400 clock ticks.
    // Use this value to set the period.
    //
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 400);
    //
    // Set the pulse width of PWM0 for a 100% duty cycle.
    // 25% Duty Cycle: 100
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 400);
    //
    // Start the timers in generator 0.
    //
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    //
    // Enable the outputs.
    //
    PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);


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
