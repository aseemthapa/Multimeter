//MULTIMETER PROJECT
//EMBEDDED II
//Contributors:
//1) Aseem Thapa     (1001543178)
//2) Sudarshan Tiwari(1001553073)

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz
// Stack:           4096 bytes (needed for sprintf)

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

// I2C devices on I2C bus 0 with 2kohm pullups on SDA (PB3) and SCL (PB2)

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "i2c0.h"
#include "i2c0_lcd.h"
#include "clock.h"
#include "wait.h"
#include "adc0.h"

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

//PINS---------------->
//Port B1 <=> Meas_LR
//Port B0 <=> Meas_C
//Port A2 <=> Lowside R
//Port A3 <=> Integrate
//Port A4 <=> Highside R
//Port C7 <=> Voltage input to comparator
//Port E0 <=> ADC
//Port E1 <-> Resistor Pushbotton
//Port E2 <-> Capacitor Pushbotton
//Port E3 <-> Inductor Pushbotton
//Port E4 <-> ESR Pushbotton
//Port E5 <-> Auto Pushbotton

#define Meas_LR 2
#define Meas_C 1
#define LowR 4
#define Int 8
#define HighR 16
#define compIn 128
#define AIN3_MASK 1
#define PBr_mask 2
#define PBc_mask 4
#define PBl_mask 8
#define PBe_mask 16
#define PBa_mask 32


//Global variables:
uint32_t time = 0;      // Used for timing charging of components (also used as resistance output)
uint8_t calcType = 0;   // Used to determine what kind of calculation in CompISR (3 types: 0,1,2)
float ind = 0;          // used in inductance measurement
float cap = 0;          // used in capacitance measurement
bool calc_finished = 1; // for waiting function
char mode = 0;          // select between UART or (PB and LCD) mode

// Initialize Hardware
void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable clocks ports A,B, and C:
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0 | SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R4;
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; //Timer module 1 used
    SYSCTL_RCGCACMP_R |= SYSCTL_RCGCACMP_R0;
    _delay_cycles(3);

    //Set the pins as output
    GPIO_PORTA_DIR_R |= LowR | HighR | Int;
    GPIO_PORTB_DIR_R |= Meas_C | Meas_LR;

    //Set pushbuttons as input:
    GPIO_PORTE_DIR_R &= ~PBr_mask & ~PBc_mask & ~PBl_mask & ~PBe_mask & ~PBa_mask;

    //Set comparator input as input:
    GPIO_PORTC_DIR_R &= ~compIn;

    //Data enable:
    GPIO_PORTA_DEN_R |= LowR | HighR | Int;
    GPIO_PORTB_DEN_R |= Meas_C | Meas_LR;
    GPIO_PORTE_DEN_R |= PBr_mask | PBc_mask | PBl_mask | PBe_mask | PBa_mask;

    //Pullup Resistors:
    GPIO_PORTE_PUR_R |= PBr_mask | PBc_mask | PBl_mask | PBe_mask | PBa_mask;

    //Comparator is analog turn off digital op:
    GPIO_PORTC_DEN_R &= ~compIn;


    // Configure AIN3 as an analog input
    GPIO_PORTE_AFSEL_R |= AIN3_MASK;                 // select alternative functions for AN3 (PE0)
    GPIO_PORTE_DEN_R &= ~AIN3_MASK;                  // turn off digital operation on pin PE0
    GPIO_PORTE_AMSEL_R |= AIN3_MASK;                 // turn on analog operation on pin PE0

    //-----------Comparator initialization-------------------------->:
    GPIO_PORTC_AFSEL_R |= compIn;
    GPIO_PORTC_AMSEL_R |= compIn;   //Turn on Analog operation
    GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC7_M & ~GPIO_PCTL_PC6_M; //Turn off GPIO on PC7 and PC6

    //------------Comparator Configuration-------------------------->:
    COMP_ACREFCTL_R |= 0x0000020D;
    //For voltage comparision 2.245 V. Also EN = 1 and RNG = 0.
    COMP_ACCTL0_R |= COMP_ACCTL0_ASRCP_REF | COMP_ACCTL0_ISEN_RISE;
    //Comparator V+ reference voltage
    NVIC_EN0_R |= 1<<(INT_COMP0 - 16); //Turn on comparator interrupt
    COMP_ACINTEN_R &= ~COMP_ACINTEN_IN0 ; //IN0 Interrupt disabled

    //--------------Timer 1 Configuration--------------------------->:
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;  // configure for periodic mode (count up)
    TIMER1_TAILR_R = 20000000;                       // max wait time
    TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;               // turn off interrupt at start
    TIMER1_TAV_R = 0;                                // start counting up from 0
    TIMER1_CTL_R = 0;
    NVIC_EN0_R |= (1<<(INT_TIMER1A) - 16);           // turn on NVIC Interrupt for timer 1A
}



//-----------------------------------------------------------------------------
// FUNCTION DESCRIPTIONS:
//-----------------------------------------------------------------------------


//Reset routine that makes sure no voltage flows through circuit:
void noVoltReset()
{
    //0 V through whole circuit
    GPIO_PORTA_DATA_R |= LowR;
    GPIO_PORTA_DATA_R &= ~HighR;
    GPIO_PORTB_DATA_R |= Meas_C;
    GPIO_PORTA_DATA_R |= Int;
    GPIO_PORTB_DATA_R &= ~Meas_LR;
}


//Timer interrupt routine:
void Timer1ISR()
{
    TIMER1_ICR_R |= TIMER_ICR_TATOCINT;               // clear interrupt flag
    TIMER1_IMR_R &= ~TIMER_IMR_TATOIM;               // turn off interrupt at start
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // Stop timer 1
    //putsUart0("OVERLOAD\r\n");
    calc_finished = 1;
    time = 0;
    TIMER1_TAV_R = 0;                                //reset timer
    noVoltReset();                                   //reset circuit to 0V flowing
}

//function to display voltage from ADC:
void display_volts()
{
    char volt[20];
    float voltage = 0;
    //Take 10 succesive values and output the average:
    int i = 0;
    while (i<10)
    {
        voltage += readAdc0Ss3();
        i += 1;
    }
    voltage = voltage/10;

    voltage = (voltage*3.3)/4096; //In ADC, 4096 <=> 3.3 V
    voltage = voltage - 0.019;    //Around 0.019 V error

    sprintf(volt,"%4.3f",voltage);
    putsUart0("Voltage: ");
    putsUart0(volt);
    putsUart0("V \r\n");
}



//Routine to calculate resistance:
void calculate_Res()
{
    calc_finished = 0;
    //First drain the capacitor for 5*tau = 165us:
    TIMER1_TAV_R = 0;

    //De integrate circuit:
    GPIO_PORTA_DATA_R |= LowR;
    GPIO_PORTA_DATA_R |= Int;
    GPIO_PORTB_DATA_R &= ~Meas_LR;
    waitMicrosecond(200); //170 needed 200 done for safety measures

    //display_volts();              //debug line to check if capacitor is still at a high enough voltage

    //Integration of circuit:
    GPIO_PORTA_DATA_R &= ~LowR;
    GPIO_PORTB_DATA_R |= Meas_LR; //Turn meas LR to start charging capacitor
    GPIO_PORTB_DATA_R &= ~Meas_C;
    COMP_ACMIS_R |= COMP_ACMIS_IN0;
    COMP_ACINTEN_R |= 1 ; //IN0 Interrupt enabled: Comparator 0 used.
    TIMER1_CTL_R |= TIMER_CTL_TAEN;     //Start timer 1
    TIMER1_IMR_R |= TIMER_IMR_TATOIM;  // turn on interrupt
}

//Routine to calculate capacitance:
void calculate_Cap()
{
    calc_finished = 0;

    TIMER1_TAV_R = 0;

    //For capacitors max wait time made bigger to fit 100 uF
    TIMER1_TAILR_R = 500000000;                       // max wait time for 100 uF

    //De integrate circuit:
    GPIO_PORTA_DATA_R |= LowR;
    GPIO_PORTB_DATA_R |= Meas_C;
    GPIO_PORTA_DATA_R &= ~HighR;
    waitMicrosecond(20000); //high caps have high discharge time so use a big value

    //display_volts();              //debug line to check if capacitor is still at a high enough voltage

    //Integration of circuit:
    GPIO_PORTA_DATA_R &= ~LowR;
    GPIO_PORTA_DATA_R |= HighR;
    COMP_ACMIS_R |= COMP_ACMIS_IN0;
    COMP_ACINTEN_R |= 1 ; //IN0 Interrupt enabled: Comparator 0 used.
    TIMER1_CTL_R |= TIMER_CTL_TAEN; //Start timer 1
    TIMER1_IMR_R |= TIMER_IMR_TATOIM;  // turn on interrupt
}

//Routine to calculate Inductance:
void calculate_Ind()
{
    calc_finished = 0;

    TIMER1_TAV_R = 0;

    //De integrate circuit:
    GPIO_PORTA_DATA_R |= LowR;
    GPIO_PORTB_DATA_R &= ~Meas_LR;
    GPIO_PORTA_DATA_R &= ~Int;
    waitMicrosecond(100000); //high inds have high discharge time so use a big value

    //display_volts();              //debug line to check if inductor is still at a high enough voltage

    //Integration of circuit:
    GPIO_PORTB_DATA_R |= Meas_LR;
    GPIO_PORTB_DATA_R &= ~Meas_C;
    COMP_ACMIS_R |= COMP_ACMIS_IN0;
    COMP_ACINTEN_R |= 1 ; //IN0 Interrupt enabled: Comparator 0 used.
    TIMER1_CTL_R |= TIMER_CTL_TAEN; //Start timer 1
    TIMER1_IMR_R |= TIMER_IMR_TATOIM;  // turn on interrupt
}


//function to calculate ESR in inductance calculation:
float calculate_ESR()
{
    //First turn the Meas_LR and Lowside R to make the circuitt look like:
    //3.3V->-------L+ESR----V--33ohms-------|>(Ground)(V = Voltage taken)
    //To DC Inductors look like open circuits thus this is a series
    //with ESR in 33 ohms in seres (can use voltage divider law)
    //(3.3 - V)/ESR = V/33 <=> ESR = [(3.3 - V)*33]/V
    GPIO_PORTB_DATA_R |= Meas_LR;
    GPIO_PORTB_DATA_R &= ~Meas_C;
    GPIO_PORTA_DATA_R |= LowR;
    GPIO_PORTA_DATA_R &= ~Int;
    GPIO_PORTA_DATA_R &= ~HighR;
    float v = readAdc0Ss3();    //This will be the raw data

    v = (v*3.3)/4096;           //This will give the actual value for resistance
    //display_volts();
    v = v - 0.019;              //around this much error (0 V <=> 0.019 V)
    v = ((3.3 - v)*33)/v;       //This will calculate ESR
    noVoltReset();              //Reset to 0V flowing through circuit
    return v;
}


//Comparator interrupt routine:
void Comp0ISR()
{
    //display_volts();                   //Debug line
    COMP_ACINTEN_R &= ~COMP_ACINTEN_IN0 ; //IN0 Interrupt disabled
    time = TIMER1_TAV_R;            //store the Timer value
    TIMER1_TAV_R = 0;            //reconfigure Timer 1 for next measurement
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN; //Stop timer 1

    //resistance:
    if (calcType == 1)
    {
        time = time/40;             //time in (us)
        GPIO_PORTB_DATA_R &= ~Meas_LR; //Turn off Meas_LR at interrupt
        time = (time*2660) / 3228;   //2600 ohms ~ 3228 us
    }

    //capacitance:
    else if (calcType == 2)
    {
        GPIO_PORTA_DATA_R &= ~HighR; //Turn off HighR at interrupt
        //47 uF ~<=> 5800000 us = 5800ms
        cap = time;
        cap = cap/40;
        cap = (cap*47) / 5800000;
        if (cap < 3) cap = cap - 0.9; //for error management (for small caps)
    }

    //inductance:
    else if (calcType == 3)
    {
        GPIO_PORTA_DATA_R &= ~HighR; //Turn off HighR at interrupt
        ind = time;
        ind = ind/40;                //time in (us)
        //tL/(ESR+33) is constant (350 calculated)
        //Thus L = 350 * (ESR + 33)/t
        ind = 350/ind;
    }

    calc_finished = 1;
    noVoltReset();
}


//AUTO Function implementation: will show what kind of a component is being measured
void autoChecker()
{
    //De integrate circuit:
    GPIO_PORTA_DATA_R |= LowR;
    GPIO_PORTB_DATA_R &= ~Meas_LR;
    waitMicrosecond(50000); //high inds and caps have high discharge time so use a big value

    //Debug Lines:
    //putsUart0("After discharge ");
    //display_volts();

    //First turn the Meas_LR and Lowside R to make the circuitt look like:
    //3.3V->-------L+ESR----V--33ohms-------|>(Ground)(V = Voltage taken)
    //To DC Inductors look like open circuits thus this is a series
    //with ESR in 33 ohms in seres (can use voltage divider law)
    //(3.3 - V)/ESR = V/33 <=> ESR = [(3.3 - V)*33]/V
    GPIO_PORTB_DATA_R |= Meas_LR;
    GPIO_PORTB_DATA_R &= ~Meas_C;
    GPIO_PORTA_DATA_R |= LowR;
    GPIO_PORTA_DATA_R &= ~Int;
    GPIO_PORTA_DATA_R &= ~HighR;

    //Theory for relation of v1 and v2:
    // 1) almost same non-zero voltage => small resistor
    // 2) large spike in v2 => inductor
    // 3) almost same but near zero => either capacitor or large resistor

    float v1 = (readAdc0Ss3() * 3.3)/4096;
    waitMicrosecond(100000);
    float v2 = (readAdc0Ss3() * 3.3)/4096;

    //debug lines:
    /*char str[10];
    putsUart0("At start: ");
    sprintf(str,"%4.3f",v1);
    putsUart0(str);
    putsUart0("\r\n");
    putsUart0("At end: ");
    sprintf(str,"%4.3f",v2);
    putsUart0(str);
    putsUart0("\r\n");*/

    char str[10];
    //spike case: inductor
    if ((v2 - v1) > 0.3)
        {
            if (mode == '0') putsUart0("Inductor Detected. Calculating.....\r\n");
            else
            {
                putsLcd(2,0,"Cap:                ");
                putsLcd(1,0,"Res:                ");
            }
            noVoltReset();
            waitMicrosecond(50000);
            calcType = 3; //3 for inductance
            calculate_Ind();
            float ESR_ind = calculate_ESR();
            while(!calc_finished);

            if (ESR_ind > 500)
            {
                if(mode == '0') putsUart0("High Value of ESR. Cannot calculate Inductance.\r\n");
                else
                {
                    putsLcd(3,4,"   ");
                    putsLcd(3,7,"HIGH ESR     ");
                }
            }
            else
            {
                if (time == 0)
                {
                    if (mode =='0') putsUart0("Inductor calculation OVERFLOW\r\n");
                    else
                    {
                        putsLcd(3,4,"   ");
                        putsLcd(3,7,"OVERFLOW     ");
                    }
                }
                else
                {
                    /*sprintf(out,"%4.3f",ESR_ind);
                    putsUart0("ESR: ");
                    putsUart0(out);
                    putsUart0(" Ohms \r\n");*/
                    ind = ind * (33 + ESR_ind);
                    ind = ind - 70; //Offset for interrupt callibration
                    sprintf(str,"%4.3f",ind);
                    if (mode == '0')
                    {
                        putsUart0("Inductance: ");
                        putsUart0(str);
                        putsUart0(" mH \r\n");
                    }
                    else
                    {
                        putsLcd(3,4,"   ");
                        putsLcd(3,7,str);
                        putsLcd(3,14,"  ");
                        putsLcd(3,16,"mH  ");
                    }

                }
            }
        }

    //Discharge from capacitor
    else if ((v1 > v2) && ((v1-v2)> 1))
    {
        if (mode == '0') putsUart0("Capacitor Detected. Calculating.....\r\n");
        else
        {
            putsLcd(3,0,"Ind:                ");
            putsLcd(1,0,"Res:                ");
        }
        noVoltReset();
        waitMicrosecond(50000);
        calcType = 2; //2 for capacitance
        calculate_Cap();
        while(!calc_finished);
        if (time == 0)
        {
            if (mode == '0') putsUart0("OVERLOAD. Component has high Capacitance/may not be a capacitor.\r\n");
            else
            {
                putsLcd(2,4,"   ");
                putsLcd(2,7,"OVERFLOW     ");
            }
        }
        else
        {
            sprintf(str, "%4.3f", cap);
            if (mode == '0')
            {
                putsUart0("Capacitance: ");
                putsUart0(str);
                putsUart0("(uF)\r\n");
            }
            else
            {
                putsLcd(2,4,"   ");
                putsLcd(2,7,str);
                putsLcd(2,14,"  ");
                putsLcd(2,16,"uF. ");
            }
        }
        TIMER1_TAILR_R = 10000000;
    }

    //Case for no component inserted:
    else if ((v1 < 0.022) && (v2 < 0.022))
    {
        if (mode == '0') putsUart0("No components detected/High value resistance\r\n");
        else putsLcd(1,5,"No Comp/High R ");
    }

    //For resistors the voltage should not change
    else if ((v1 - v2) < 0.1)
    {
        if (mode == '0') putsUart0("Resistor Detected. Calculating.....\r\n");
        else
        {
            putsLcd(2,0,"Cap:                ");
            putsLcd(3,0,"Ind:                ");
        }
        noVoltReset();
        calcType = 1; //1 for resistance
        char str[10];

        float ESR = calculate_ESR();
        //If ESR is low enough don't calculate resistance

        if (ESR > 500 || ESR < 0)
        {
            calculate_Res();
            while(!calc_finished);


            //If there is no timer interrupt->
            if (time != 0 && time > 0)
            {
                sprintf(str, "%7lu", time);
                if (mode == '0')
                {
                    putsUart0("Resistance: ");
                    putsUart0(str);
                    putsUart0("(Ohms)\r\n");
                }
                else
                {
                    putsLcd(1,4,"   ");
                    putsLcd(1,7,str);
                    putsLcd(1,14,"  ");
                    putsLcd(1,16,"Ohms");
                }
            }
            else
            {
                if (mode == '0')putsUart0("High Value of Resistance/Component not resistor.\r\n");
                else putsLcd(1,5,"High Value Res.");
            }
        }

        else
        {
            sprintf(str, "%4.3f", ESR);
            if (mode == '0')
            {
                putsUart0("Resistance: ");
                putsUart0(str);
                putsUart0("(Ohms)\r\n");
            }
            else
            {
                putsLcd(1,4,"   ");
                putsLcd(1,7,str);
                putsLcd(1,14,"  ");
                putsLcd(1,16,"Ohms");
            }
        }

    }

    else
    {
        if (mode == '0') putsUart0("Unknown component.\r\n");
        else putsLcd(1,5,"Unknown Comp   ");
    }

    noVoltReset();   //set to 0V flowing through circuit (RESET STATE)

}


//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    initHw();
    initUart0();
    initAdc0Ss3();

    //ADC configuration:
    setAdc0Ss3Mux(3);
    setAdc0Ss3Log2AverageCount(2);

    putsUart0("***********Multimeter***********\r\n");

    //Initialize to a circuit running 0V through it:
    noVoltReset();

    putsUart0("Select mode: 0) UART Mode 1) PushButton Mode(LCD Output)\r\n");
    mode = getcUart0();

    if (mode == '0') putsUart0("UART Mode selected.\r\n");

    else if (mode == '1') putsUart0("Pushbutton Mode selected.\r\n");

    else
    {
        putsUart0("Invalid Selection. Operating in Uart Mode(Default).\r\n");
        mode = '0';
    }
    bool chk = 0;

    while (true)
    {
        //Push button mode--------------------------------->
        if (mode == '1')
        {
            //Do LCD initialization only once
            if (chk == 0)
            {
                initLcd();
                putsLcd(0,0,"      MULTIMETER    ");
                putsLcd(1,0,"Res:                ");
                putsLcd(2,0,"Cap:                ");
                putsLcd(3,0,"Ind:                ");
            }
            chk = 1;
            char str[10]; //This string stores the outputs of calculations as string

            if (!(GPIO_PORTE_DATA_R & PBr_mask))
            {
                calcType = 1; //1 for resistance
                putsLcd(2,0,"Cap:                ");
                putsLcd(3,0,"Ind:                ");


                float ESR = calculate_ESR();
                //If ESR is low enough don't calculate resistance

                if (ESR > 500 || ESR < 0)
                {
                    calculate_Res();
                    while(!calc_finished);

                    //If there is no timer interrupt->
                    if (time != 0)
                    {
                        /*putsUart0("Resistance: ");*/
                        sprintf(str, "%7lu", time);
                        /*putsUart0(str);
                        putsUart0("(Ohms)\r\n");*/
                        putsLcd(1,4,"   ");
                        putsLcd(1,7,str);
                        putsLcd(1,14,"  ");
                        putsLcd(1,16,"Ohms");
                    }
                    else
                    {
                       // putsUart0("High Value of Resistance/Component not resistor.\r\n");
                       putsLcd(1,5,"High Value Res.");
                    }
                }

                else
                {
                    /*putsUart0("Resistance: ");*/
                    sprintf(str, "%4.3f", ESR);
                    /*putsUart0(str);
                    putsUart0("(Ohms)\r\n");*/
                    putsLcd(1,4,"   ");
                    putsLcd(1,7,str);
                    putsLcd(1,14,"  ");
                    putsLcd(1,16,"Ohms");
                }
                waitMicrosecond(200000);
            }

            else if(!(GPIO_PORTE_DATA_R & PBc_mask))
            {

                putsLcd(1,0,"Res:                ");
                putsLcd(3,0,"Ind:                ");
                calcType = 2; //2 for capacitance
                calculate_Cap();
                while(!calc_finished);
                if (time == 0)
                {
                    //putsUart0("OVERLOAD. Component has high Capacitance/may not be a capacitor.\r\n");
                    putsLcd(2,4,"   ");
                    putsLcd(2,7,"OVERLOAD     ");
                }
                else
                {
                    //putsUart0("Capacitance: ");
                    sprintf(str, "%4.3f", cap);
                    //putsUart0(str);
                    //putsUart0("(uF)\r\n");
                    putsLcd(2,4,"   ");
                    putsLcd(2,7,str);
                    putsLcd(2,14,"  ");
                    putsLcd(2,16,"uF. ");

                }
                TIMER1_TAILR_R = 10000000;                       // RESET max wait time
                waitMicrosecond(200000);
            }

            else if(!(GPIO_PORTE_DATA_R & PBl_mask))
            {

                putsLcd(1,0,"Res:                ");
                putsLcd(2,0,"Cap:                ");
                calcType = 3; //3 for inductance
                calculate_Ind();
                float ESR_ind = calculate_ESR();
                while(!calc_finished);
                if (ESR_ind > 500)
                {
                    //putsUart0("High Value of ESR. Cannot calculate Inductance.\r\n");
                    putsLcd(3,4,"   ");
                    putsLcd(3,7,"HIGH ESR     ");
                }
                else
                {
                    if (time == 0)
                    {
                        //putsUart0("Inductor calculation OVERFLOW\r\n");
                        putsLcd(3,4,"   ");
                        putsLcd(3,7,"OVERFLOW     ");
                    }
                    else
                    {
                        //sprintf(out,"%4.3f",ESR_ind);
                        //putsUart0("ESR: ");
                        //putsUart0(out);
                        //putsUart0(" Ohms \r\n");
                        ind = ind * (33 + ESR_ind);
                        ind = ind - 70; //Offset for interrupt callibration
                        sprintf(str,"%4.3f",ind);
                        putsLcd(3,4,"   ");
                        putsLcd(3,7,str);
                        putsLcd(3,14,"  ");
                        putsLcd(3,16,"mH  ");
                        //putsUart0("Inductance: ");
                        //putsUart0(out);
                        //putsUart0(" uH \r\n");
                    }
                }
                waitMicrosecond(200000);
            }

            else if(!(GPIO_PORTE_DATA_R & PBe_mask))
            {

                putsLcd(1,0,"Res:                ");
                putsLcd(2,0,"Cap:                ");
                float ESR = calculate_ESR();
                char out[20];
                if (ESR > 500)
                {
                    putsLcd(3,4,"   ");
                    //putsUart0("High Value of ESR.\r\n");
                    putsLcd(3,7,"HIGH ESR     ");
                }
                else
                {
                    sprintf(out,"%4.3f",ESR);
                    putsLcd(3,4,"   ");
                    putsLcd(3,7,str);
                    putsLcd(3,14,"  ");
                    putsLcd(3,16,"Ohms");
                    //putsUart0("ESR: ");
                    //putsUart0(out);
                    //putsUart0(" Ohms \r\n");
                }
                waitMicrosecond(200000);
            }

            else if(!(GPIO_PORTE_DATA_R & PBa_mask))
            {
                autoChecker();
                waitMicrosecond(200000);
            }
        }

        //UART mode--------------------------------->
        else
        {
            char str[10];
            char ch = getcUart0();
            //resistance command 'r'
            if (ch == 'r')
            {
                calcType = 1; //1 for resistance

                float ESR = calculate_ESR();
                //If ESR is low enough don't calculate resistance

                if (ESR > 500 || ESR < 0)
                {
                    calculate_Res();
                    while(!calc_finished);

                    //If there is no timer interrupt->
                    if (time != 0)
                    {
                        putsUart0("Resistance: ");
                        sprintf(str, "%7lu", time);
                        putsUart0(str);
                        putsUart0("(Ohms)\r\n");
                    }
                    else
                    {
                        putsUart0("High Value of Resistance/Component not resistor.\r\n");
                    }
                }

                else
                {
                    putsUart0("Resistance: ");
                    sprintf(str, "%4.3f", ESR);
                    putsUart0(str);
                    putsUart0("(Ohms)\r\n");
                }
                ch = '0';
            }

            //capacitance command 'c'
            else if (ch == 'c')
            {
                calcType = 2; //2 for capacitance
                calculate_Cap();
                while(!calc_finished);
                if (time == 0) putsUart0("OVERLOAD. Component has high Capacitance/may not be a capacitor.\r\n");
                else
                {
                    putsUart0("Capacitance: ");
                    sprintf(str, "%4.3f", cap);
                    putsUart0(str);
                    putsUart0("(uF)\r\n");
                }
                ch = '0';
                TIMER1_TAILR_R = 10000000;                       // RESET max wait time
            }

            //inductance command 'i'
            else if (ch == 'l')
            {
                calcType = 3; //3 for inductance
                calculate_Ind();
                float ESR_ind = calculate_ESR();
                while(!calc_finished);
                char out[20];
                if (ESR_ind > 500)
                {
                    putsUart0("High Value of ESR. Cannot calculate Inductance.\r\n");
                }
                else
                {
                    if (time == 0) putsUart0("Inductor calculation OVERFLOW\r\n");
                    else
                    {
                        /*sprintf(out,"%4.3f",ESR_ind);
                        putsUart0("ESR: ");
                        putsUart0(out);
                        putsUart0(" Ohms \r\n");*/
                        ind = ind * (33 + ESR_ind);
                        ind = ind - 70; //Offset for interrupt callibration
                        sprintf(out,"%4.3f",ind);
                        putsUart0("Inductance: ");
                        putsUart0(out);
                        putsUart0(" mH \r\n");

                    }
                }
            }

            //display voltage command 'v'
            else if (ch == 'v')
            {
                display_volts();
            }

            //ESR command 'e', also works good with smaller resistances
            else if (ch == 'e')
            {
                float ESR = calculate_ESR();
                char out[20];
                if (ESR > 500)
                {
                    putsUart0("High Value of ESR.\r\n");
                }
                else
                {
                    sprintf(out,"%4.3f",ESR);
                    putsUart0("ESR: ");
                    putsUart0(out);
                    putsUart0(" Ohms \r\n");
                }
            }

            //auto command (a):
            else if (ch == 'a')
            {
                autoChecker();
            }
        }

    }

}
