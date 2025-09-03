#include  "../header/api.h"    		// private library - API layer
#include  "../header/halGPIO.h"     // private library - HAL layer
#include "stdio.h"
#include <stdlib.h>
#include <stdio.h>

//*********************************************************************
//                        API Variables
//*********************************************************************
int colors[] = {0b000, 0b001, 0b010, 0b011, 0b100, 0b101, 0b110, 0b111};
int tone_series[] = {1000, 1250, 1500, 1750, 2000, 2250, 2500};
unsigned int count_up = 0;
char count_up_str[5];
unsigned int* count_up_address = &count_up;
unsigned int count_down = 65535;
char count_down_str[5];
unsigned int* count_down_address = &count_down;
const unsigned long v_refrence = 3400000UL;                  // for 4
char afterDigit_str[4];                                      // for 4
char beforeDigit_str[1];                                     // for 4
int ticks = 0;                                               // for servo angle





//---------------------------------------------------------------------
//                       UltraSonic
//---------------------------------------------------------------------
void Telemeter(){
    TA1CTL   = TASSEL_2 | MC_1 | TACLR;  // Start Trig & Eco timer    ;  SMCLK, Up mode

    while(state == state2)
        __bis_SR_register(LPM0_bits + GIE); // wait for capture ISR

    TA1CTL &= ~MC_3;    // Stop Timer1_A (TA1)
}

//---------------------------------------------------------------------
//                      Servo Angle
//---------------------------------------------------------------------
void Servo_SetAngleDeg(int deg){
    if (deg > 179) deg = 180;
    if (deg < 0) deg = 0;

    // linear map with rounding
    ticks = SERVO_MIN_TICKS + ((SERVO_MAX_TICKS - SERVO_MIN_TICKS) * deg + 90) / 180;

    TA0CCR1 = ticks;
    TA0CTL   = TASSEL_2 | MC_1 | TACLR;  // SMCLK, Up mode
    TA0_delay_ms(60);  // delay 0.1s
    TA0CTL &= ~MC_3;    // stop Timer0
}

void Servo_coast_to_coast(int deg){
    if (deg > 179) deg = 180;
    if (deg < 0) deg = 0;

    // linear map with rounding
    ticks = SERVO_MIN_TICKS + ((SERVO_MAX_TICKS - SERVO_MIN_TICKS) * deg + 90) / 180;

    TA0CCR1 = ticks;
    TA0CTL   = TASSEL_2 | MC_1 | TACLR;  // SMCLK, Up mode
    TA0_delay_ms(450);  // delay 0.45s
    TA0CTL &= ~MC_3;    // stop Timer0
}

//---------------------------------------------------------------------
//                      Objects Detector System
//---------------------------------------------------------------------
void Scan_detector(void) {
    unsigned i;
    // ------ first iteration outside the loop, coast to coast for edge cases ---------
    Servo_coast_to_coast(0);

    // ------ 0-180 degrees iterations outside the loop, SetAngleDeg for minimal delay movement ---------
    for (i = 0; i <= 180; i++) {
        Servo_SetAngleDeg(i);

        m1_tx_arm = 1;                             // allow exactly one send for this degree
        TA1CTL = TASSEL_2 | MC_1 | TACLR;          // start telemeter timer
        __bis_SR_register(LPM0_bits + GIE);        // wait for capture ISR

        TA0CTL = TASSEL_2 | MC_1 | TACLR;          // start delay timer
        TA0_delay_ms(40);
        TA0CTL &= ~MC_3;
        TA1CTL &= ~MC_3;
    }
    Servo_coast_to_coast(0);
}

//---------------------------------------------------------------------
//                    ADC10 start conversion for calibration
//---------------------------------------------------------------------
void adc10_calib_trigger(void){
    if (calib_index >= 10) return;   // already full

    while (ADC10CTL1 & ADC10BUSY);   // wait if ADC busy
    ADC10CTL0 &= ~ENC;               // re-arm safely
    ADC10SA = (int)adc_buf;     // reload destination
    ADC10CTL0 |= ENC | ADC10SC;      // start sequence
}

//---------------------------------------------------------------------
//                    Light sources detector system
//---------------------------------------------------------------------
//   Median help func
unsigned int median3(unsigned int a, unsigned int b, unsigned int c) {
    if ((a >= b && a <= c) || (a <= b && a >= c)) return a;
    else if ((b >= a && b <= c) || (b <= a && b >= c)) return b;
    else return c;
}
// main func
void Light_Sources_Detector_Scan(void) {
    unsigned int i;

    if (string2[0] == 'N') {
        readFlashSamples(ldr1_samples, ldr2_samples);
        IE2 |= UCA0TXIE; // trigger TX ISR to send flash table
    }
    if (string2[0] == 'Y') {
        // ---- coast to coast, only for first iteration ----
       Servo_coast_to_coast(0);

       // --- o - 180 ---
       for (i = 0; i <= 180; i++) {
           Servo_SetAngleDeg(i);

           // collect 3 sequences A3..A0 -> 3 samples for each LDR
           for (k = 0; k < 3; k++) {
               while (ADC10CTL1 & ADC10BUSY);     // be safe
               ADC10CTL0 &= ~ENC;                  // allow DTC edits
               ADC10DTC1   = 4;                    // ONE sequence: 4 transfers (A3..A0)
               ADC10SA     = (int)adc_buf;          // 4-word destination
               ADC10CTL0  |= ENC | ADC10SC;        // *** start conversion ***
           }

           // compute medians of the three samples per channel
           unsigned int median1 = median3(adc_buf_set[0], adc_buf_set[2], adc_buf_set[4]); // A3
           unsigned int median2 = median3(adc_buf_set[1], adc_buf_set[3], adc_buf_set[5]); // A0
           avg_median = (median1 + median2) >> 1;      // divide by 2

           // send results
           IE2 |= UCA0TXIE;
    }
       Servo_coast_to_coast(0);
    }
}

void Lights_and_objects (void){
    unsigned int i;
    if (string2[0] == 'N') {
           readFlashSamples(ldr1_samples, ldr2_samples);
           IE2 |= UCA0TXIE; // trigger TX ISR to send flash table
    }

    // ---- coast to coast, only for first iteration ----
          Servo_coast_to_coast(0);

          for (i = 0; i <= 180; i++) {
                  Servo_SetAngleDeg(i);

                  m1_tx_arm = 1;                             // allow exactly one send for this degree
                  TA1CTL = TASSEL_2 | MC_1 | TACLR;          // start telemeter timer
                  __bis_SR_register(LPM0_bits + GIE);        // wait for capture ISR

                  TA0CTL = TASSEL_2 | MC_1 | TACLR;          // start delay timer
                  TA0_delay_ms(40);
                  TA0CTL &= ~MC_3;
                  TA1CTL &= ~MC_3;

                  m2_tx_arm = 1;
                  ADC10CTL0 &= ~ENC;                  // allow DTC edits
                  ADC10DTC1   = 4;                    // ONE sequence: 4 transfers (A3..A0)
                  ADC10SA     = (int)adc_buf;            // 4-word destination
                  ADC10CTL0  |= ENC | ADC10SC;        // *** start conversion ***

                  avg_median = (buf8[0] + buf8[1]) >> 1;      // divide by 2

                  IE2 |= UCA0TXIE;
              }
       Servo_coast_to_coast(0);
}





//-------------------------------------------------------------
//                1. Count up to 2^16 = 65535  (number of leds in a row on LCD)
//------------------------------------------------------------
void count_up_LCD(){
    while(state==state1){
        lcd_clear();
        lcd_home();
        int2str(count_up_str, *count_up_address);
        lcd_puts(count_up_str);
        timer_call_counter();

        *count_up_address = (*count_up_address + 1) % 65536;
    }
}
//-------------------------------------------------------------
//                2. circular tone series -  Buzzer
//-----------------------------------------------------------
void Circular_tone_buzzer(){
    unsigned int freq, period, duty, i;
    TA1CTL = TASSEL_2 + MC_1 ;      // SMCLK, up mode

    while(state == state2){
        for (i = 0; i < 7; i++) {
            freq = tone_series[i];
            period = 1000000 / freq;
            duty = period / 2;

            TA1CCR0 = period - 1;
            TA1CCR1 = duty;
            timer_call_counter();   // delay of delay_time (X) second
        }
    }
    TA1CTL = MC_0 ; // Stop Timer
}
//-------------------------------------------------------------
//                3. Change Delay Time [ms]
//------------------------------------------------------------
void change_delay_time(){
    delay_time = atoi(string1);  // Get delay time from user
    state = state8;
}

//-------------------------------------------------------------
//                5. Enable PB1 press
//-----------------------------------------------------------
void PB1_press(){
    while(state == state5){
        PBsArrIntPend &= ~0x01;
        PBsArrIntEn |= 0x01;
        PBsArrPortDir &= ~0x01;
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
        PBsArrIntEn &= ~0x01;
    }
}

//-------------------------------------------------------------
//                6. Clear LCD and init counters
//------------------------------------------------------------
void clear_counters(){
    disable_interrupts();
    lcd_clear();
    lcd_home();
    count_up = 0;
    //count_down = 65535;
    enable_interrupts();
    state = state9;
}
//-------------------------------------------------------------
//                8. Enable PB1 or PB2 press
//------------------------------------------------------------
void PB1_PB2_press(){
    while(state == state8){
        PBsArrIntPend &= ~0x01;
        PB2sArrIntPend &= ~0x01;
        PBsArrIntEn |= 0x01;
        PB2sArrIntEn |= 0x01;
        PBsArrPortDir &= ~0x01;
        PB2sArrPortDir &= ~0x01;
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
        PBsArrIntEn &= ~0x01;
        PB2sArrIntEn &= ~0x01;
    }
}

//-------------------------------------------------------------
//                Count Down from 65535 to 0
//------------------------------------------------------------
void count_down_LCD(){
    while(state==state3){
        lcd_clear();
        lcd_home();
        lcd_puts("Count Down: ");
        lcd_new_line;
        int2str(count_down_str, *count_down_address);
        lcd_puts(count_down_str);
        timer_call_counter();
        *count_down_address = (*count_down_address - 1);
        if (*count_down_address == 0) *count_down_address = 65535;
    }
}
