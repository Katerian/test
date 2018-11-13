
// PIC24FJ256GB110 Configuration Bit Settings

// 'C' source line config statements

// CONFIG3
#pragma config WPFP = WPFP511           // Write Protection Flash Page Segment Boundary (Highest Page (same as page 170))
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable bit (Segmented code protection disabled)
#pragma config WPCFG = WPCFGDIS         // Configuration Word Code Page Protection Select bit (Last page(at the top of program memory) and Flash configuration words are not protected)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select bit (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMOD = HS             // Primary Oscillator Select (HS oscillator mode selected)
#pragma config DISUVREG = OFF           // Internal USB 3.3V Regulator Disable bit (Regulator is disabled)
#pragma config IOL1WAY = ON             // IOLOCK One-Way Set Enable bit (Write RP Registers Once)
#pragma config OSCIOFNC = OFF           // Primary Oscillator Output Function (OSCO functions as CLKO (FOSC/2))
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-safe Clock Monitor are disabled)
#pragma config FNOSC = PRIPLL              // Oscillator Select (Primary oscillator (XT, HS, EC))
#pragma config PLL_96MHZ = ON           // 96MHz PLL Disable (Enabled)
#pragma config PLLDIV = DIV5            // USB 96 MHz PLL Prescaler Select bits (Oscillator input divided by 5 (20MHz input))
#pragma config IESO = ON                // Internal External Switch Over Mode (IESO mode (Two-speed start-up) enabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler (Prescaler ratio of 1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Standard Watchdog Timer enabled,(Windowed-mode is disabled))
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config ICS = PGx1               // Comm Channel Select (Emulator functions are shared with PGEC1/PGED1)
#pragma config GWRP = OFF               // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF                // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG port is disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


#include <xc.h>
#define FOSC 32000000
#define FCY FOSC/2
#define ADC_CHANNEL 1
#include <libpic30.h>


void ADC_init()
{
    AD1PCFGLbits.PCFG1 = 0; //pin B1 analog
    AD1PCFGLbits.PCFG3 = 0; //pin B3 analog
    AD1CON2bits.VCFG = 0b000; // Vr+ = AVdd (power); Vr- = AVss; (ground)
    AD1CON3bits.ADCS = 0b00000000; // 2*Tcy
    AD1CON3bits.SAMC = 0b11111; // 32*Tad
    AD1CON1bits.SSRC = 0b111; // Clearing SAMP bit ends sampling and starts conversion
    AD1CON1bits.FORM = 0b00; // Data Output Formats - Integer
//    AD1CON2bits.SMPI = 0b0000; // Interrupts at the completion of conversion for each sample/convert sequence
    AD1CON1bits.ADON = 1; // A/D Converter module is operating
}

uint16_t ADC_read(int channel)
{
    AD1CHS = channel;
    AD1CON1bits.SAMP = 1;
    while (AD1CON1bits.DONE == 0); // end of convert
    return ADC1BUF0;
}

void UART_init()
{
    RPINR19bits.U2RXR = 10;
    RPOR8bits.RP17R = 5;
    
    U2MODEbits.BRGH = 1; // High-Speed mode (baud clock generated from FCY/4)
    U2BRG = 34; // 
    U2MODEbits.UARTEN = 1; // uart on
    U2STAbits.UTXEN = 1; //Transmit enabled
   // IFS1bits.U2RXIF = 0; // flag
    IEC1bits.U2RXIE = 1;
    IEC1bits.U2TXIE = 0;
    IPC7bits.U2RXIP = 0b010;
    IPC7bits.U2TXIP = 0b001;
    //U2STAbits.URXISEL = 0b00; //interrupt with any simb (one and more)
}


void PutChar (char val)
{
    while (U2STAbits.UTXBF == 1);
    U2TXREG = val;
}

char GetChar ()
{
    char symbol;
    while (IFS1bits.U2RXIF == 0);  //???? ???? ???? ?????? 1
    symbol = U2RXREG;                //?????????? ???????? ?? ????????
  //  IFS1bits.U2RXIF = 0;
    return symbol;
}

uint16_t GetInt ()
{
    uint16_t symbol;
    while (IFS1bits.U2RXIF == 0);  //???? ???? ???? ?????? 1
    symbol = U2RXREG;                //?????????? ???????? ?? ????????
   // IFS1bits.U2RXIF = 0;
    return symbol;
}

char input = ' ';

uint16_t width = 50;

void __attribute__((__interrupt__, __auto_psv__)) _U2RXInterrupt(void)
{
    input = GetChar();
//    if (input == 'b')
//    {
//        width = GetChar();
//    }
    IFS1bits.U2RXIF = 0;
}

void timer_init()
{
    T1CONbits.TCS = 0; // Clock Source - Internal clock (FOSC/2)
    T1CONbits.TGATE = 0; // Gated time accumulation disabled
    T1CONbits.TSYNC = 0; // bit is ignored 
//    T1CONbits.TCKPS = 0b11; //????????????  1:256
//    PR1 = 391; //  ???????? ??????? ???????
    T1CONbits.TCKPS = 0b00;
    PR1 = 2000;
    IEC0bits.T1IE = 1; // ?????????? ??????????
    IPC0bits.T1IP = 0b010; //  ?????????  1
    T1CONbits.TON = 1; // ????????? ???????
}

void Put16bit (uint16_t val)
{
    char val_H, val_L;
    val_H = val>>8;
    val_L = val;
    PutChar(val_H);
    PutChar(val_L);
}


void __attribute__ ((__interrupt__, __auto_psv__)) _T1Interrupt(void) //??????????
{
    if (input == 'r')
    {
            uint16_t value = 0;
            value = ADC_read(ADC_CHANNEL);
            Put16bit(value);
    }
    if (input == 'd')
    {
	uint16_t value = 0;
    	value = ADC_read(ADC_CHANNEL);
    }
    IFS0bits.T1IF = 0; 
}


void init_PWM ()
{
    RPOR6bits.RP13R = 18; // RB2 Output compare 1
    TMR2 = 0;                     // initial value
    T2CONbits.TCKPS = 0b01;       // prescale 1:8  
  //  T2CONbits.TON = 1;            // timer on
    PR2 =  40000;                // final value
    
    OC1R = 20000;            // duty cycle
    //OC1RS = 40000;                  // period
    
    OC1CON2bits.SYNCSEL = 0b01100; //sync source
    OC1CON2bits.OCTRIG = 0;       
    OC1CON1bits.OCTSEL = 0;       // timer2
   // IEC0bits.T2IE = 1;            // for overflow
   // IEC0bits.OC1IE = 1;   //for compare        
    OC1CON1bits.OCM = 0b110;      //????? ?????? ??? Edge-aligned PWM Mode on OCx 
}

//void __attribute__ ((__interrupt__, __auto_psv__)) _OC1Interrupt(void)
//{
//    IFS0bits.OC1IF = 0;
//}
//
//void __attribute__ ((__interrupt__, __auto_psv__)) _T2Interrupt(void)
//{
//    IFS0bits.T2IF = 0;
//}

int main(void)
{
    AD1PCFGL = 0xFFFF; // all pins digital
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB3 = 1;
//    TRISEbits.TRISE7 = 0;
//    //IFS1bits.U2TXIF = 1;
    init_PWM();
    ADC_init();
    UART_init();
    timer_init();
    uint8_t flag_duty_cycle = 0;
    while(1)
    {
        if (flag_duty_cycle == 1)
        {
             OC1R = PR2/100*input;
        }
        if (flag_duty_cycle == 2)
        {
            PR2 = 40000;
            OC1R = 20000;
            T2CONbits.TON = 1;            // timer on  
        }
        if (input == 'i')
        {
//            PR2 = 40000;
//            OC1R = 20000;
//            T2CONbits.TON = 1;            // timer on    
            flag_duty_cycle = 2;
        }
        if (input == 'c')
        {
            T2CONbits.TON = 0;            // timer off
        }
        if (input == 'b')
        { 
            flag_duty_cycle = 1;
        }
    }
}
