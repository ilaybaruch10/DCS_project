#include  "../header/api.h"    		// private library - API layer
#include  "../header/app.h"    		// private library - APP layer
#include  "../header/halGPIO.h"     // private library - HAL layer
#include "stdio.h"
#include <stdlib.h>
#include <stdio.h>


/*
 *          Hardware connections:
 * p1.0 --> LDR2
 * p1.1 -->      UART
 * p1.2 -->      UART
 * p1.3 --> LDR1
 * p1.4 --> LCD D4
 * p1.5 --> LCD D4
 * p1.6 --> LCD D4
 * p1.7 --> LCD D4
 *
 * p2.0 --> PB0
 * p2.1 --> UltraSonic Trig
 * p2.2 --> PB1
 * p2.3 --> LCD E
 * p2.4 --> UltraSonic Echo
 * p2.5 --> LCD RS
 * p2.6 --> Servo PWM
 * p2.7 --> LCD RW
 * */

enum FSMstate state;
enum SYSmode lpm_mode;


void main(void){
  
  state = state9;  // start in idle state on RESET
  lpm_mode = mode0;     // start in idle state on RESET
  sysConfig();     // Configure GPIO, Stop Timers, Init LCD


  while(1){
  switch(state){
        case state1: // Object Detector System
            Scan_detector();
            state = state9;
            break;

        case state2: // Telemeter
            Servo_coast_to_coast(atoi(string1));
            Telemeter();
            break;

        case state3: // LDR samples calibration
            Servo_coast_to_coast(90);
            __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
            //create, saved to Flash, send to PC --> ALL via ISR
            break;

        case state4: // Light Sources Detector Systems
            Light_Sources_Detector_Scan();
            state = state9;
            break;

        case state5: // Upload text files and save to Flash --> via ISR
            __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
            break;

        case state6: //  Erase all Flash (data and struct)
            flash_erase_Seg1();
            flash_erase_Seg2();
            flash_erase_Seg3();
            flash_erase_Seg4();
            flash_erase_infoC();
            state = state9;
            break;

        case state7: //  zip on LCD, read saved files
            __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
            break;

        case state8: // BONUS_ Lights + Sources
              Lights_and_objects();
              state = state9;
              break;

        case state9: // LPM mode
            calib_index = 0;     // for new possible samples
            IE2 |= UCA0RXIE;                          // Enable USCI_A0 RX interrupt
            __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ interrupt
            break;


	}
 }
}

  
  
  
  
  
  
