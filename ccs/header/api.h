#ifndef _api_H_
#define _api_H_
#include  "../header/halGPIO.h"     // private library - HAL layer


extern void count_up_LCD();
extern void count_down_LCD();
extern void clear_counters();
extern void change_delay_time();
extern void PB1_press();
extern void Circular_tone_buzzer();
extern void PB1_PB2_press();
extern void Telemeter();
extern void Servo_SetAngleDeg(int deg);
extern void Scan_detector();
extern void Servo_coast_to_coast(int deg);
extern void adc10_calib_trigger(void);
extern void Light_Sources_Detector_Scan(void);
extern unsigned int median3(unsigned int a, unsigned int b, unsigned int c) ;
extern void Lights_and_objects(void);

#endif







