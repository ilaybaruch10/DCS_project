#ifndef _bsp_H_
#define _bsp_H_
#include  <msp430g2553.h>          // MSP430x2xx
#include <stdint.h>


//*********************************************************************
//                        Global Variable
//*********************************************************************
extern volatile int adc_buf[12];

//*********************************************************************
//                       Flash defines
//*********************************************************************
struct LDRFile{
    int ldr1[10];  // LDR1 samples
    int ldr2[10];  //  LDR2 samples
    char name[10];          // "file" name
    int size;      // total size in bytes
    char type[10];          // file type
};

struct FileEntry {
    uint8_t  name_id;     // 0..9  (this is the “name number”)
    uint8_t  type;        // 0=text, 1=script
    uint16_t size_bytes;  // 0xFFFF while writing; final size when done
    uint16_t start_addr;  // code flash start address (e.g., 0xFC00)
} ;

#define INFO_D_BASE        0x1000u
#define INFO_C_BASE        0x1040u   //Segment C - data struct of file mode
#define INFO_B_BASE        0x1080u   // Segment B - pointer lives here
#define INFO_A_BASE        0x10C0u   // Segment A (cal constants) DO NOT ERASE
#define INFO_C_END         0x107Fu
#define SEGMENT_0_BASE        0xFE00u   // Main flash Segment A: DO NOT ERASE/WRITE
#define MAIN_FLASH_TOP_PLUS1  0x10000u  // one past 0xFFFF
#define SEGMENT_1_BASE        0xFC00u
#define SEGMENT_2_BASE        0xFA00u
#define SEGMENT_3_BASE        0xF800u
#define SEGMENT_4_BASE        0xF600u

#define FILE_AREA_BASE        0xF600u
#define FILE_AREA_END         SEGMENT_0_BASE  // stop before Segment A

#define FLASH_SEG_SIZE   0x0200u   // 512 B erase granularity in main flash
#define INFO_FLASH_SEG_SIZE   0x0040u   // 64 B erase granularity in info flash

#define DATA_CHUNK_SIZE       0x0040u   // 64 B app-level chunk size (optional)

// ---- State7 file browser/viewer context ----
#define FILE_ENTRY_SIZE   6u               // [name, type, sizeL, sizeH, addrL, addrH]
#define MAX_FILE_ENTRIES  10u   // 10 entries in 64B segment
#define ENTRY_ADDR(i)    ( (uint16_t)(INFO_C_BASE + (uint16_t)(i) * FILE_ENTRY_SIZE) )


//*********************************************************************
//                       Defines
//*********************************************************************
#define   debounceVal      12000

// LCDs abstraction
#define LCD_DATA_WRITE     P1OUT
#define LCD_DATA_DIR       P1DIR
#define LCD_DATA_READ      P1IN
#define LCD_DATA_SEL       P1SEL
#define LCD_CTL_SEL        P2SEL

//// PushButton Port2 abstraction for Real Time
#define PB2sArrPort         P2IN
#define PB2sArrIntPend      P2IFG
#define PB2sArrIntEn        P2IE
#define PB2sArrIntEdgeSel   P2IES
#define PB2sArrPortSel      P2SEL
#define PB2sArrPortDir      P2DIR
#define PB2sArrPortOut      P2OUT

// PushButtons Port1 abstraction
#define PBsArrPort	       P1IN
#define PBsArrIntPend	   P1IFG
#define PBsArrIntEn	       P1IE
#define PBsArrIntEdgeSel   P1IES
#define PBsArrPortSel      P1SEL
#define PBsArrPortDir      P1DIR
#define PBsArrPortOut      P1OUT

// PBs
#define PB0                BIT0   // P2.0
#define PB1                BIT2   // P2.2
//#define PB2                0x01
//#define PB3                0x08

// LDR
#define LDR1   BIT3   // P1.3 / A3
#define LDR2   BIT0   // P1.0 / A0
#define INFO_D_START   0x1000

// Servo
#define SERVO_MIN_TICKS      450     // 0
#define SERVO_MAX_TICKS      2175    // 180
#define TA0_TICK_HZ 1000000         // 1 MHz -> 1 tick = 1 µs

// UART
#define TXLED BIT0
#define RXLED BIT6
#define TXD BIT2
#define RXD BIT1

extern void GPIOconfig(void);
extern void ADC_config(void);
extern void TIMER_A0_config(unsigned int counter);
extern void TIMERB_config(void);
extern void TIMERB_config_Task3(void);
extern void StopAllTimers(void);
extern void UART_init(void);
extern void TIMERA_Buzzer_config(void);
extern void TimerA1_TrigInit(void);
extern void Echo_Init_P24(void);
extern void Servo_Init(void);
extern void TA0_delay_ticks(int ticks);
extern void adc10_calib_init(void);
extern void adc10_calib_init2(void);

#endif



