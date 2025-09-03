#include  "../header/bsp.h"    // private library - BSP layer
#include  "../header/halGPIO.h"     // private library - HAL layer

//*********************************************************************
//                        Global Variable
//*********************************************************************
volatile int adc_buf[12] = {0,0,0,0,0,0,0,0,0,0,0,0};


//*********************************************************************
//                        GPIO Configuration
//*********************************************************************
void GPIOconfig(void){
  WDTCTL = WDTHOLD | WDTPW;		// Stop WDT
   
  /* ---------------- LCD CONFIG ---------------- */
  LCD_DATA_WRITE &= ~0xFF;
  LCD_DATA_DIR |= 0xF0;    // P1.4-P1.7 To Output('1')
  LCD_DATA_SEL &= ~0xF0;   // Bit clear P1.4-P1.7
  LCD_CTL_SEL  &= ~0xA8;   // Bit clear P2.3,p2.5,p2.7

/* ---------------- PB1 CONFIG ---------------- */
  P2DIR &= ~PB1;   // input
  P2REN |=  PB1;   // pull enable
  P2OUT |=  PB1;   // pull-up
  P2IES |=  PB1;   // falling edge
  P2IFG &= ~PB1;   // clear
  P2IE  |=  PB1;   // enable

  /* ---------------- PB0 CONFIG ---------------- */
  P2DIR &= ~PB0;   // input
  P2REN |=  PB0;   // pull enable
  P2OUT |=  PB0;   // pull-up
  P2IES |=  PB0;   // falling edge
  P2IFG &= ~PB0;   // clear
  P2IE  |=  PB0;   // enable

  _BIS_SR(GIE);                     // enable interrupts globally
}

//*********************************************************************
//                        Stop all timers
//*********************************************************************
void StopAllTimers(void){
    // Stop Timer0_A (TA0)
    TA0CTL &= ~MC_3;    // MC_3 masks both MC bits; result = MC_0 (stopped)

    // Stop Timer1_A (TA1)
    TA1CTL &= ~MC_3;
}
//*********************************************************************
//                       UAERT Init
//*********************************************************************
void UART_init(void){
    if (CALBC1_1MHZ==0xFF)                  // If calibration constant erased
      {
        while(1);                               // do not load, trap CPU!!
      }
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
    DCOCTL = CALDCO_1MHZ;


    P1SEL  |= (BIT1 | BIT2);                    // P1.1 = RXD, P1.2=TXD
    P1SEL2 |= (BIT1 | BIT2);                     // P1.1 = RXD, P1.2=TXD
    P1DIR |= RXLED + TXLED;

    UCA0CTL1 |= UCSSEL_2;                     // CLK = SMCLK
    UCA0BR0 = 104;                           //
    UCA0BR1 = 0x00;                           //
    UCA0MCTL = UCBRS0;               //
    UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
}

//*********************************************************************
//                      UltraSonic Init -->  Trig, Echo
//*********************************************************************
void TimerA1_TrigInit(void)
{
    P2DIR |= BIT1;               // P2.1 out
    P2SEL |= BIT1;               // route TA1.1

    TA1CCR0  = 60000 - 1;        // period ~ 60 ms @ 1 MHz  (CCR0 is TOP)
    TA1CCTL1 = OUTMOD_7;         // reset/set PWM (HIGH at start, LOW at CCR1)
    TA1CCR1  = 10;               // 10 us HIGH
}

// P2.4 = TA1.2 CCI2A (capture). Rising then falling edge timing in Up mode.
void Echo_Init_P24(void)
{
    P2DIR &= ~BIT4;                      // p2.4 = TA1.2 input
    P2SEL |=  BIT4;                      // TA1.2 function (CCI2A)
    // TA1 already in Up mode from Trig init; just arm CCR2 capture
    TA1CCTL2 = CM_1 | CCIS_0 | SCS | CAP | CCIE;  // rising edge, sync, capture, IRQ
}

//*********************************************************************
//                      Servo Init -->  PWM
//*********************************************************************
void Servo_Init(void){      // P2.6

   BCSCTL3 |= LFXT1S_2;            // LFXT1 = VLO, releases P2.6/P2.7
   // Map TA0.1 to P2.6
   P2DIR  |= BIT6;             // output p2.6
   P2SEL  |= BIT6;
   P2SEL2 &= ~(BIT6) ;            // ensure primary function (TA0.1), not alt

   TA0CCR0  = 25000 - 1;       // 25 ms period @ 1 MHz
   TA0CCTL1 = OUTMOD_7;        // PWM reset/set on CCR1
   TA0CCR1  = SERVO_MIN_TICKS; // phase 0

   TA0CTL   = TASSEL_2 | MC_1 | TACLR;  // SMCLK, Up mode
   TA0_delay_ms(450); // delay 0.45s
   TA0CTL &= ~MC_3;     // stop Timer0
}

//*********************************************************************
//                         ADC10 Init
//*********************************************************************
void adc10_calib_init(void){
    P1SEL  &= ~(BIT0 | BIT3);
    P1SEL2 &= ~(BIT0 | BIT3);
    P1DIR  &= ~(BIT0 | BIT3);
    P1REN  &= ~(BIT0 | BIT3);
    P1OUT  &= ~(BIT0 | BIT3);

    ADC10CTL0 = 0;
    ADC10CTL1 = 0;

    // Enable analog function on P1.3 (A3) and P1.0 (A0)
    ADC10AE0 |= LDR1 | LDR2;

    // ADC10CTL1: highest channel = A3, sequence-of-channels, SMCLK
    ADC10CTL1 = INCH_3         // highest channel is A3
               | CONSEQ_1      // single sequence per trigger
               | ADC10SSEL_3  // SMCLK
               | ADC10DIV_3;      // slower ADC10CLK (margin for LDRs)

    // ADC10CTL0: AVCC/AVSS ref, 64-cycle sample time, enable interrupts
    ADC10CTL0 = SREF_0         // VR+ = AVCC, VR- = AVSS
               | ADC10SHT_3    // 64 x ADC10CLK sample time
               | MSC           // auto advance to next channel
               | ADC10ON       // power ADC core
               | ADC10IE;      // enable ADC10 interrupt

    ADC10DTC1 = 4;                  // 4 transfers (A3 to A0)
    ADC10SA  = (int)adc_buf;   // DTC destination

    ADC10CTL0 &= ~(ENC | ADC10SC | ADC10IFG);  // not enabled, not converting, flag clear
}


//*********************** Delay ************************************
// Core delay: wait for "ticks" timer cycles using CCR2
void TA0_delay_ticks(int ticks){
    int start = TA0R;
    int period = TA0CCR0 + 1;
    int target = (start + ticks) % period;

    TA0CCTL2 &= ~(CCIFG | CAP);   // clear flag, force compare mode
    TA0CCR2 = target;             // set compare point

    // Poll until compare event occurs
    while (!(TA0CCTL2 & CCIFG)) { ; }
    TA0CCTL2 &= ~CCIFG;           // clear for next time
}




