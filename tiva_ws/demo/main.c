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
#include "driverlib/adc.h"
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

void configure_adc(void)
{
    // Enable the ADC0 Module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Wait for the module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }

    // We'll use Input 0 for measurement
    // According to the datasheet AIN0 is mapped to pin PE3 of Port E
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    // Setup the GPIO Pin for ADC
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);

    // ADC module uses sequencers to sample data, 4 different types of sequencers all-together
    // Sequence Sampler 3 will be used and configured for processor triggering (Manual triggering)
    // Sampler 3 will take a single sample when trigger occurs
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    // Configure step 0 on sequence sampler 3.
    // Sample Channel 0 (ADC_CTL_CH0) in single-ended mode (default)
    // Configure the interrupt flag (ADC_CTL_IE) to be set when the sample is done
    // Tell the ADC logic that this is the last conversion on sequence sampler 3 (ADC_CTL_END)
    // Sequence 3 only has one programmable step
    // Since we only need to program 1 step, we only need to program step 0
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

    // Configure Hardware Oversampling to Smooth Jitter
    ADCHardwareOversampleConfigure(ADC0_BASE, 64);
    // Enable the Configured ADC
    ADCSequenceEnable(ADC0_BASE, 3);

    // Clear the interrupt status flag. This is done to make sure the interrupt flag is cleared before
    // we start sampling
    ADCIntClear(ADC0_BASE, 3);

}



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

    UARTStdioConfig(0, 115200, 16000000);

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

uint32_t map(uint32_t input, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


int main(void)
{

    // Use this array to store data read from ADC FIFO. Must be as large as FIFO being used
    uint32_t ADC0_buffer[1];

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

    // We can print the same thing if we have the uartstdio.c library
    //UARTprintf("Hello World\n");

    // Enable the ADC Module
    configure_adc();

    // Enable the PWM Module
    configure_pwm_module();
    //
    // Set the period. For a 50 KHz frequency, the period = 1/50,000, or 20
    // microseconds. For a 20 MHz clock, this translates to 400 clock ticks.
    // Use this value to set the period.
    // https://en.wikipedia.org/wiki/Duty_cycles
    //
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 400);

    //
    // Start the timers in generator 0.
    //
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    //
    // Enable the outputs.
    //
    PWMOutputState(PWM0_BASE, (PWM_OUT_0_BIT | PWM_OUT_1_BIT), true);

    // Print an array of char's
    //char *s="hello world\n";
    //while (*s) UARTCharPut(UART0_BASE, *s++);

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





        // Trigger the ADC Conversion
        ADCProcessorTrigger(ADC0_BASE, 3);
        // wait for the conversion process to be completed
        while(!ADCIntStatus(ADC0_BASE, 3, false))
        {
        }
        // Clear the ADC interrupt flag
        ADCIntClear(ADC0_BASE, 3);
        // Read ADC Value
        ADCSequenceDataGet(ADC0_BASE, 3, ADC0_buffer);
        //UARTprintf("Test");
        UARTprintf("AIN0 = %4d\r", ADC0_buffer[0]);
        // Map the measured potentiometer value to duty cycles
        uint32_t pulse_width = map(ADC0_buffer[0], 0, 4096, 0, 400);
        //
        // Set the pulse width of PWM0 for a 100% duty cycle.
        // 25% Duty Cycle: 100
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pulse_width);
        UARTprintf("Pulse Width: %4d\r", pulse_width);


    }


}
