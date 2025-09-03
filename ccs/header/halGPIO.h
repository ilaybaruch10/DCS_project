#ifndef _halGPIO_H_
#define _halGPIO_H_
#include  "../header/bsp.h"    		// private library - BSP layer
#include  "../header/app.h"    		// private library - APP layer


//*********************************************************************
//                        Global Variable
//*********************************************************************
extern enum FSMstate state;
extern enum SYSmode lpm_mode;
extern char string1[5];                          // for state 1,2
extern char string2[3];                          // for state 4
extern unsigned int delay_time;
extern int pushedIFG;
extern  int m1_tx_arm;   // 0 = disarmed, 1 = send once (Mode 1)
extern volatile int  calib_index;
extern int ldr1_samples[10];
extern int ldr2_samples[10];
extern volatile int adc_buf_set[6];                             // for state 4
extern volatile  int adc_done;                                  // for state 4
extern volatile  int adc_index;                                 // for state 4
extern unsigned int k;                                          // for state 4
extern unsigned int avg_median;                                 // for state 4
extern int buf8[2];                                             // for state 8
extern int m1_tx_arm;
extern int m2_tx_arm;

#define half_sec 500;
#define clk_tmp 131; // ( (2^20) / 8 )*(10^-3) to convert ms to counter value for TACCR0
//----------------------------------------------------------------------

//------------state 5--------
extern void flash_write_byte_isr(int addr, int b);
extern int fs_find_next_start(void);
extern void flash_erase_range(int start, int size);
extern void fs_bump_next(int start, int size);
//---------------------------

extern void sysConfig(void);
extern void delay(unsigned int);
extern void enterLPM(unsigned char);
extern void enable_interrupts();
extern void disable_interrupts();
extern void timer_call_counter();
extern void int2str(char *str, unsigned int num);
extern void TA0_delay_us(int us);
extern void TA0_delay_ms(int ms);
extern void flash_write_segment_d(struct LDRFile *src);
extern struct LDRFile createLDRFile( int *arr1, int *arr2, char *fname, int *size, char *type);
extern void readFlashSamples(int *ldr1_samples, int *ldr2_samples);
extern void s5_print_32_on_lcd(const char *buf);
extern uint16_t InfoC_GetFreeAddr(void);
extern void flash_erase_infoC(void);
extern void flash_erase_Seg1(void);
extern void flash_erase_Seg2(void);
extern void flash_erase_Seg3(void);
extern void flash_erase_Seg4(void);


extern __interrupt void PBs_handler(void);
extern __interrupt void PBs_handler_P2(void);
extern __interrupt void Timer1_ISR(void);
extern __interrupt void Timer0_ISR (void);
extern __interrupt void USCI0RX_ISR(void);
extern __interrupt void USCI0TX_ISR(void);
extern __interrupt void ADC10_ISR(void);


#endif

// #define CHECKBUSY    1  // using this define, only if we want to read from LCD

#ifdef CHECKBUSY
    #define LCD_WAIT lcd_check_busy()
#else
    #define LCD_WAIT DelayMs(5)
#endif

/*----------------------------------------------------------
  CONFIG: change values according to your port pin selection
------------------------------------------------------------*/
#define LCD_EN(a)   (!a ? (P2OUT&=~0X08) : (P2OUT|=0X08)) // P2.3 is lcd enable pin
#define LCD_EN_DIR(a)   (!a ? (P2DIR&=~0X08) : (P2DIR|=0X08)) // P2.3 pin direction

#define LCD_RS(a)   (!a ? (P2OUT&=~0X20) : (P2OUT|=0X20)) // P2.5 is lcd RS pin
#define LCD_RS_DIR(a)   (!a ? (P2DIR&=~0X20) : (P2DIR|=0X20)) // P2.5 pin direction

#define LCD_RW(a)   (!a ? (P2OUT&=~0X80) : (P2OUT|=0X80)) // P2.7 is lcd RW pin
#define LCD_RW_DIR(a)   (!a ? (P2DIR&=~0X80) : (P2DIR|=0X80)) // P2.7 pin direction

#define LCD_DATA_OFFSET 0x04 //data pin selection offset for 4 bit mode, variable range is 0-4, default 0 - Px.0-3, no offset


/*---------------------------------------------------------
  END CONFIG
-----------------------------------------------------------*/
#define FOURBIT_MODE    0x0
#define EIGHTBIT_MODE   0x1
#define LCD_MODE        FOURBIT_MODE

#define OUTPUT_PIN      1
#define INPUT_PIN       0
#define OUTPUT_DATA     (LCD_MODE ? 0xFF : (0x0F << LCD_DATA_OFFSET))
#define INPUT_DATA      0x00

#define LCD_STROBE_READ(value)  LCD_EN(1), \
                asm("nop"), asm("nop"), \
                value=LCD_DATA_READ, \
                LCD_EN(0)

#define lcd_cursor(x)       lcd_cmd(((x)&0x7F)|0x80)
#define lcd_clear()         lcd_cmd(0x01)
#define lcd_putchar(x)      lcd_data(x)
#define lcd_goto(x)         lcd_cmd(0x80+(x))
#define lcd_cursor_right()  lcd_cmd(0x14)
#define lcd_cursor_left()   lcd_cmd(0x10)
#define lcd_display_shift() lcd_cmd(0x1C)
#define lcd_home()          lcd_cmd(0x02)
#define cursor_off          lcd_cmd(0x0C)
#define cursor_on           lcd_cmd(0x0F)
#define lcd_function_set    lcd_cmd(0x3C) // 8bit,two lines,5x10 dots
#define lcd_new_line        lcd_cmd(0xC0)

extern void lcd_cmd(unsigned char);
extern void lcd_data(unsigned char);
extern void lcd_puts(const char * s);
extern void lcd_init();
extern void lcd_strobe();
extern void DelayMs(unsigned int);
extern void DelayUs(unsigned int);
/*
 *  Delay functions for HI-TECH C on the PIC18
 *
 *  Functions available:
 *      DelayUs(x)  Delay specified number of microseconds
 *      DelayMs(x)  Delay specified number of milliseconds
*/






