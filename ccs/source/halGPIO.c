#include  "../header/halGPIO.h"     // private library - HAL layer
#include  "../header/api.h"         // private library - API layer

#include "stdio.h"
#include <string.h>
#include <stdint.h>


//*********************************************************************
//                        Global Variable
//*********************************************************************
char string1[5];
unsigned int i;
int j=0;
int jj=0;
int delay_ifg1 = 0;                // delay flag for state 1
int delay_ifg2 = 0;                // delay flag for state 2
int delay_ifg4 = 0;                // delay flag for state 4
int delay_ifg5 = 0;                // delay flag for state 5
int delay_ifg8 = 0;                // delay flag for state 8
int buf8[2] = {0,0};

int pushedIFG = 0;
unsigned int delay_time = 500;
const unsigned int timer_half_sec = 65535;
char string2[3] ;                   // for state 4                     // for 8
int m1_tx_arm = 0;                   // 0 = disarmed, 1 = send once (Mode 1)
int m2_tx_arm = 0;

unsigned int REdge1 = 0;          // for Echo capture
unsigned int REdge2 = 0;          // for Echo capture
unsigned char Count = 0;          // for Echo capture
unsigned int width = 0;           // for Echo capture
unsigned int last_width = 0;      // for Echo capture
int ldr1_samples[10];                          // for calibration LDR1
int ldr2_samples[10];                          // for calibration LDR2
volatile int  calib_index = 0;                 // for calibration index
int volatile  adc_buf_set[6];                            // for state 4
int volatile  adc_done;                                  // for state 4
int volatile  adc_index;                                 // for state 4
unsigned int k;                                          // for state 4
unsigned int avg_median;                                 // for state 4

//-----------state 5-----------
uint8_t  p;             // header index
volatile int  array5[6];     // [0]=name_id, [1]=type, [2] - [5] = size,
uint8_t  rx5_phase;     // 0=collect header, 1=collect data
volatile int rx5_remaining; // remaining data bytes
volatile int rx5_addr;      // flash write address
volatile uint8_t flash_write_error = 0;
int fs_next = FILE_AREA_BASE;
uint16_t   segC_next ;
uint16_t MainMemorystart;
volatile uint8_t tx5_pending = 0;   // pending single byte for TX ('A','E','F','K')
static   uint16_t rx5_size = 0;     // total size from header (keep for fs_bump_next)
int addr;
int addr2;
//--------------------------------------------------------------------

//-----------state 7-----------
volatile uint8_t  s7_mode = 0;            // 0 = browse names, 1 = view content
volatile uint8_t  s7_idx  = 0;            // current entry index 0..9
volatile uint16_t s7_addr = 0;            // selected file start (main flash)
volatile uint16_t s7_size = 0;            // selected file size
volatile uint16_t s7_off  = 0;            // view offset for scrolling
//--------------------------------------------------------------------

//*********************************************************************
//             System Configuration  
//*********************************************************************
void sysConfig(void){ 
	GPIOconfig();
	StopAllTimers();
	lcd_init();
	lcd_clear();
	UART_init();
	Servo_Init();
	Echo_Init_P24();
	TimerA1_TrigInit();
	adc10_calib_init();
	flash_erase_infoC();
}
//*********************************************************************
//          Data structure define createLDRFile
//*********************************************************************

struct LDRFile createLDRFile( int *arr1, int *arr2, char *fname, int *size, char *type)
{
    struct LDRFile f;

    // copy arrays
    int i = 0;
    for (i = 0; i < 10; i++) {
        f.ldr1[i] = arr1[i];
        f.ldr2[i] = arr2[i];
    }

    // copy file name (max 9 chars + null)
    strncpy(f.name, fname, sizeof(f.name) - 1);
    f.name[sizeof(f.name) - 1] = '\0';

    // copy size
    f.size = *size;

    // copy type string (max 9 chars + null)
    strncpy(f.type, type, sizeof(f.type) - 1);
    f.type[sizeof(f.type) - 1] = '\0';

    return f;
}

//*********************************************************************
//                  Text file struct build
//*********************************************************************
struct FileEntry TextStruct_create(uint8_t name,  uint16_t size, uint8_t type, uint16_t strt_address)
{
    struct FileEntry f;

    f.name_id    = name;
    f.type       = type;
    f.size_bytes = size;
    f.start_addr = strt_address;

    return f;
}
//*********************************************************************
//                  Flash helpers
//*********************************************************************
//---------------------------------------------------------------------
//                  Erase Segments
//---------------------------------------------------------------------
void flash_erase_infoC(void)
{
    // Disable interrupts during flash operation (optional but recommended)
    __bic_SR_register(GIE);

    FCTL3 = FWKEY;                // Unlock flash
    FCTL1 = FWKEY | ERASE;        // Enable erase
    *(volatile uint8_t*)INFO_C_BASE = 0;              // Dummy write to trigger erase

    while (FCTL3 & BUSY);         // Wait until done

    FCTL1 = FWKEY;                // Clear ERASE
    FCTL3 = FWKEY | LOCK;         // Relock flash

    __bis_SR_register(GIE);       // Re-enable interrupts if needed
}
void flash_erase_Seg1(void)
{
    // Disable interrupts during flash operation (optional but recommended)
    __bic_SR_register(GIE);

    FCTL3 = FWKEY;                // Unlock flash
    FCTL1 = FWKEY | ERASE;        // Enable erase
    *(volatile uint8_t*)SEGMENT_1_BASE = 0;              // Dummy write to trigger erase

    while (FCTL3 & BUSY);         // Wait until done

    FCTL1 = FWKEY;                // Clear ERASE
    FCTL3 = FWKEY | LOCK;         // Relock flash

    __bis_SR_register(GIE);       // Re-enable interrupts if needed
}
void flash_erase_Seg2(void)
{
    // Disable interrupts during flash operation (optional but recommended)
    __bic_SR_register(GIE);

    FCTL3 = FWKEY;                // Unlock flash
    FCTL1 = FWKEY | ERASE;        // Enable erase
    *(volatile uint8_t*)SEGMENT_2_BASE = 0;              // Dummy write to trigger erase

    while (FCTL3 & BUSY);         // Wait until done

    FCTL1 = FWKEY;                // Clear ERASE
    FCTL3 = FWKEY | LOCK;         // Relock flash

    __bis_SR_register(GIE);       // Re-enable interrupts if needed
}
void flash_erase_Seg3(void)
{
    // Disable interrupts during flash operation (optional but recommended)
    __bic_SR_register(GIE);

    FCTL3 = FWKEY;                // Unlock flash
    FCTL1 = FWKEY | ERASE;        // Enable erase
    *(volatile uint8_t*)SEGMENT_3_BASE = 0;              // Dummy write to trigger erase

    while (FCTL3 & BUSY);         // Wait until done

    FCTL1 = FWKEY;                // Clear ERASE
    FCTL3 = FWKEY | LOCK;         // Relock flash

    __bis_SR_register(GIE);       // Re-enable interrupts if needed
}
void flash_erase_Seg4(void)
{
    // Disable interrupts during flash operation (optional but recommended)
    __bic_SR_register(GIE);

    FCTL3 = FWKEY;                // Unlock flash
    FCTL1 = FWKEY | ERASE;        // Enable erase
    *(volatile uint8_t*)SEGMENT_4_BASE = 0;              // Dummy write to trigger erase

    while (FCTL3 & BUSY);         // Wait until done

    FCTL1 = FWKEY;                // Clear ERASE
    FCTL3 = FWKEY | LOCK;         // Relock flash

    __bis_SR_register(GIE);       // Re-enable interrupts if needed
}
int InfoC_SaveStruct(struct FileEntry *entry)
{
    unsigned i;

    uint16_t addr = InfoC_GetFreeAddr();   // find first erased word
    if (addr == 0) {
        return -1; // segment full
    }

    const uint8_t *src = (const uint8_t *)entry;
    for ( i = 0; i < sizeof(struct FileEntry); i++) {
        flash_write_byte_isr(addr + i, src[i]);
        if (flash_write_error) {
            return -2; // low-level write failed
        }
    }
    return 0;
}

uint16_t InfoC_GetFreeAddr(void)
{
    const uint16_t *p   = (const uint16_t *)INFO_C_BASE;
    const uint16_t *end = (const uint16_t *)INFO_C_END;
    /* Walk forward until we find an erased word (0xFFFF). */
    while (p < end && *p != 0xFFFFu) {
        ++p;
    }
    if (p >= end) {
        /* No free space left */
        return 0u;
    }
    /* Cast back to a 16-bit address (word-aligned by construction). */
    return (uint16_t)(uintptr_t)p;
}
static void flash_lock(void) {
    FCTL1 = FWKEY;           // clear mode bits (WRT/ERASE off)
    FCTL3 = FWKEY + LOCK;    // re-lock flash
}

static void flash_unlock_write(void) {
    FCTL3 = FWKEY;           // unlock (clear LOCK)
    FCTL1 = FWKEY + WRT;     // enable write/program mode
}

static void flash_erase_segment(int base)
{
    if (base >= SEGMENT_0_BASE) {
        // Segment 0 (0xFE00–0xFFFF) – skip!
        return;
    }
    FCTL3 = FWKEY;                 // unlock
    FCTL1 = FWKEY + ERASE;         // erase mode
    *(volatile uint8_t*)base = 0;  // dummy write to trigger erase
    while (FCTL3 & BUSY);
    flash_lock();                  // re-lock
}

void flash_erase_range(int start, int size)
{
    uint16_t a = start & ~(FLASH_SEG_SIZE - 1u);
    uint16_t e = start + size;

    while (a < e) {
        if (a >= SEGMENT_0_BASE) {
            break;                      // never cross into Segment A
        }
        flash_erase_segment(a);
        a = (uint16_t)(a + FLASH_SEG_SIZE);
    }
}

static inline uint8_t within_file_area(uint16_t addr) {
    return ((addr >= FILE_AREA_BASE) && (addr < FILE_AREA_END) || (addr >= INFO_C_BASE) && (addr < INFO_A_BASE) );
}

void flash_write_byte_isr(int addr, int b)
{
    // Range guard: never write into Segment A or outside file area
    if (!within_file_area(addr)) {
        flash_write_error = 1;   // out-of-range (or into Segment A)
        return;
    }
    // Program one byte
    FCTL3 = FWKEY;               // unlock
    FCTL1 = FWKEY + WRT;         // write mode
    *(volatile uint8_t*)addr = b;
    while (FCTL3 & BUSY);        // wait
    FCTL1 = FWKEY;               // clear WRT
    FCTL3 = FWKEY + LOCK;        // lock
}


/* tiny align helper */
static inline int align_up(int addr, int gran)
{
    return (int)((addr + (gran - 1u)) & ~(gran - 1u));
}

static uint16_t InfoMainMemory_startEntry(void)
{
    const uint16_t *p   = (const uint16_t *)FILE_AREA_BASE;
    const uint16_t *end = (const uint16_t *)FILE_AREA_END;

    while (p < end && *p != 0xFFFFu) {
        ++p;
    }
    return (uint16_t)(uintptr_t)p;
}

static uint16_t Find_space(void){
    addr = InfoC_GetFreeAddr();
    if (addr == INFO_C_BASE){
        return FILE_AREA_BASE;
    }
    else{
        // Step back into the previous entry
                uint16_t addr2 = (uint16_t)(*(volatile uint8_t *)(addr - 2))
                               | ((uint16_t)(*(volatile uint8_t *)(addr - 1)) << 8);

                uint16_t size  = (uint16_t)(*(volatile uint8_t *)(addr - 4))
                               | ((uint16_t)(*(volatile uint8_t *)(addr - 3)) << 8);

                // The next free flash address is end of previous file
                return addr2 + size + 1;
            }

    }


/* Advance fs_next after a file of `size` bytes was written at `start`. */
void fs_bump_next(int start, int size)
{
    uint32_t end = (uint32_t)start + (uint32_t)size;     // exclusive end
    if (end > FILE_AREA_END) end = FILE_AREA_END;        // never cross Segment A
    // Optional: keep 2-byte alignment (MSP430 word)
    fs_next = align_up((uint16_t)end, 2u);
}

//*********************************************************************
//                  save into Flash - for LDR only!!
//*********************************************************************

void flash_write_segment_d(struct LDRFile *src)
{
    unsigned int i;
    unsigned int gie = __get_SR_register() & GIE;   /* remember GIE state */
    const unsigned int *srcw = (const unsigned int*)src;
    int *dst = ((  int *)INFO_D_START);

    __disable_interrupt();
    FCTL2 = FWKEY | FSSEL_2 | FN2;
    /* 1) Unlock flash, erase segment D */
    FCTL3 = FWKEY;              /* clear LOCK */
    FCTL1 = FWKEY | ERASE;
    *dst = 0;                   /* dummy write to trigger erase */
    while (FCTL3 & BUSY);       /* wait erase complete */

    /* 2) Program words */
    FCTL1 = FWKEY | WRT;
    for (i = 0; i < (sizeof(struct LDRFile)/2); ++i) {
        dst[i] = srcw[i];
        while (FCTL3 & BUSY);   /* wait each word commit */
    }

    /* 3) Done: clear WRT and lock */
    FCTL1 = FWKEY;              /* clear WRT */
    FCTL3 = FWKEY | LOCK;       /* relock */

    if (gie) __bis_SR_register(GIE);
}

//*********************************************************************
//              read from Flash into ldr1 and ldr2 arrays
//*********************************************************************

void readFlashSamples(int *arr1, int *arr2) {
    int i;
    int *flashPtr =  ((int*)INFO_D_START);

    for (i = 0; i < 10; i++) {
        arr1[i] = *flashPtr++;   // read word from flash  ; ldr1 samples
    }
    for (i = 0; i < 10; i++) {
        arr2[i] = *flashPtr++;   // next block for ldr2 samples
    }
}
//*********************************************************************
//                int to string
//*********************************************************************
void int2str(char *str, unsigned int num){
    int strSize = 0;
    long tmp = num, len = 0;
    int j;
    // Find the size of the intPart by repeatedly dividing by 10
    while(tmp){
        len++;
        tmp /= 10;
    }

    // Print out the numbers in reverse
    for(j = len - 1; j >= 0; j--){
        str[j] = (num % 10) + '0';
        num /= 10;
    }
    strSize += len;
    str[strSize] = '\0';
}
//---------------------------------------------------------------------
//                     Polling delays
//---------------------------------------------------------------------
//******************************************************************
// Delay usec functions
//******************************************************************
void DelayUs(unsigned int cnt){

    unsigned char i;
    for(i=cnt ; i>0 ; i--) asm("nop"); // tha command asm("nop") takes raphly 1usec

}
//******************************************************************
// Delay msec functions
//******************************************************************
void DelayMs(unsigned int cnt){

    unsigned char i;
    for(i=cnt ; i>0 ; i--) DelayUs(1000); // tha command asm("nop") takes raphly 1usec

}
//******************************************************************
//            Polling based Delay function
//******************************************************************
void delay(unsigned int t){  //
    volatile unsigned int i;

    for(i=t; i>0; i--);
}


//---------------------------------------------------------------------
//                     Timer delays
//---------------------------------------------------------------------
//*********************************************************************
//                      Timer call counter
//*********************************************************************
void timer_call_counter(){

    unsigned int num_of_halfSec;

    num_of_halfSec = (int) delay_time / half_sec;
    unsigned int res;
    res = delay_time % half_sec;
    res = res * clk_tmp;

    for (i=0; i < num_of_halfSec; i++){
        TIMER_A0_config(timer_half_sec);
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ int until Byte RXed
    }

    if (res > 1000){
        TIMER_A0_config(res);
        __bis_SR_register(LPM0_bits + GIE);       // Enter LPM0 w/ int until Byte RXed
    }
}
//*********************************************************************
//                delay ms / us
//*********************************************************************
// Delay in microseconds
void TA0_delay_us(int us)
{
    long ticks = ((long)us * TA0_TICK_HZ) / 1000000;
    if (ticks == 0) ticks = 1;
    TA0_delay_ticks((int)ticks);
}

// Delay in milliseconds
void TA0_delay_ms(int ms)
{
    while (ms--) {
        TA0_delay_us(1000);
    }
}


//*********************************************************************
//              Enter from LPM0 mode
//*********************************************************************
void enterLPM(unsigned char LPM_level){
	if (LPM_level == 0x00) 
	  _BIS_SR(LPM0_bits);     /* Enter Low Power Mode 0 */
        else if(LPM_level == 0x01) 
	  _BIS_SR(LPM1_bits);     /* Enter Low Power Mode 1 */
        else if(LPM_level == 0x02) 
	  _BIS_SR(LPM2_bits);     /* Enter Low Power Mode 2 */
	else if(LPM_level == 0x03) 
	  _BIS_SR(LPM3_bits);     /* Enter Low Power Mode 3 */
        else if(LPM_level == 0x04) 
	  _BIS_SR(LPM4_bits);     /* Enter Low Power Mode 4 */
}
//*********************************************************************
//            Enable & Disable interrupts
//*********************************************************************
void enable_interrupts(){
  _BIS_SR(GIE);
}
//   Disable interrupts
void disable_interrupts(){
  _BIC_SR(GIE);
}


//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//                    LCD section
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
//           send a command to the LCD
//******************************************************************
void lcd_cmd(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    if (LCD_MODE == FOURBIT_MODE)
    {
        LCD_DATA_WRITE &= ~OUTPUT_DATA;// clear bits before new write
        LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
        lcd_strobe();
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= (c & (0x0F)) << LCD_DATA_OFFSET;
        lcd_strobe();
    }
    else
    {
        LCD_DATA_WRITE = c;
        lcd_strobe();
    }
}
//******************************************************************
//              send data to the LCD
//******************************************************************
void lcd_data(unsigned char c){

    LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h

    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_RS(1);
    if (LCD_MODE == FOURBIT_MODE)
    {
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= ((c >> 4) & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
            LCD_DATA_WRITE &= (0xF0 << LCD_DATA_OFFSET) | (0xF0 >> 8 - LCD_DATA_OFFSET);
            LCD_DATA_WRITE &= ~OUTPUT_DATA;
            LCD_DATA_WRITE |= (c & 0x0F) << LCD_DATA_OFFSET;
            lcd_strobe();
    }
    else
    {
            LCD_DATA_WRITE = c;
            lcd_strobe();
    }

    LCD_RS(0);
}
//******************************************************************
//          write a string of chars to the LCD
//******************************************************************
void lcd_puts(const char * s){

    while(*s)
        lcd_data(*s++);
}
//******************************************************************
//                  initialize the LCD
//******************************************************************
void lcd_init(){

    char init_value;

    if (LCD_MODE == FOURBIT_MODE) init_value = 0x3 << LCD_DATA_OFFSET;
    else init_value = 0x3F;

    LCD_RS_DIR(OUTPUT_PIN);
    LCD_EN_DIR(OUTPUT_PIN);
    LCD_RW_DIR(OUTPUT_PIN);
    LCD_DATA_DIR |= OUTPUT_DATA;
    LCD_RS(0);
    LCD_EN(0);
    LCD_RW(0);

    DelayMs(15);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayMs(5);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();
    DelayUs(200);
    LCD_DATA_WRITE &= ~OUTPUT_DATA;
    LCD_DATA_WRITE |= init_value;
    lcd_strobe();

    if (LCD_MODE == FOURBIT_MODE){
        LCD_WAIT; // may check LCD busy flag, or just delay a little, depending on lcd.h
        LCD_DATA_WRITE &= ~OUTPUT_DATA;
        LCD_DATA_WRITE |= 0x2 << LCD_DATA_OFFSET; // Set 4-bit mode
        lcd_strobe();
        lcd_cmd(0x28); // Function Set
    }
    else lcd_cmd(0x3C); // 8bit,two lines,5x10 dots

    lcd_cmd(0xF); //Display On, Cursor On, Cursor Blink
    lcd_cmd(0x1); //Display Clear
    lcd_cmd(0x6); //Entry Mode
    lcd_cmd(0x80); //Initialize DDRAM address to zero
}
//******************************************************************
//              lcd strobe functions
//******************************************************************
void lcd_strobe(){
  LCD_EN(1);
  asm("NOP");
 // asm("NOP");
  LCD_EN(0);
}
//******************************************************************
//                  print to lcd
//******************************************************************
void s5_print_32_on_lcd(const char *buf)
{
    char line[17];
    unsigned i;
    unsigned char c;

    /* first line (0..15) */
    for (i = 0; i < 16; i++) {
        c = (unsigned char)buf[i];
        if (c < 0x20 || c > 0x7E) c = ' ';
        line[i] = (char)c;
    }
    line[16] = '\0';
    lcd_cmd(0x80);       /* DDRAM = 0 (line 1) */
    lcd_puts(line);

    /* second line (16..31) */
    for (i = 0; i < 16; i++) {
        c = (unsigned char)buf[16 + i];
        if (c < 0x20 || c > 0x7E) c = ' ';
        line[i] = (char)c;
    }
    line[16] = '\0';
    lcd_cmd(0xC0);       /* DDRAM = 0x40 (line 2) */
    lcd_puts(line);
}



//---------------**************************----------------------------
//               Interrupt Services Routines
//---------------**************************----------------------------
//*********************************************************************
//            Port1 Interrupt Service Routine
//*********************************************************************
#pragma vector=PORT1_VECTOR
  __interrupt void PBs_handler(void){
    delay(debounceVal);
//---------------------------------------------------------------------
//            selector of transition between states
//---------------------------------------------------------------------


//---------------------------------------------------------------------
//            Exit from a given LPM
//---------------------------------------------------------------------
        switch(lpm_mode){
        case mode0:
         LPM0_EXIT;
         break;

        case mode1:
         LPM1_EXIT; // must be called from ISR only
         break;

        case mode2:
         LPM2_EXIT; // must be called from ISR only
         break;

        case mode3:
         LPM3_EXIT; // must be called from ISR only
         break;

        case mode4:
         LPM4_EXIT; // must be called from ISR only
         break;
    }

}
  //*********************************************************************
  //            Port2 Interrupt Service Routine
  //*********************************************************************
  #pragma vector=PORT2_VECTOR  // For Push Buttons
    __interrupt void PB2s_handler(void){
      delay(debounceVal);
  //---------------------------------------------------------------------
  //            selector of transition between states
  //---------------------------------------------------------------------


      if ((P2IFG & PB0) && (state != state3 && state != state7)){
              P2IFG &= ~PB0;           // clear flag
              state = state9;
          }

      if ((P2IFG & PB1) && (state != state3 && state != state7)){
             P2IFG &= ~PB1;           // clear flag
             state = state9;
      }

      if ((P2IFG & PB0) && state == state3) { // if PB0 and flag is on and in state3
           if (calib_index < 10) {
               adc10_calib_trigger();   // start ADC sequence
           }
           if (calib_index == 10){
               // create NEW struct and saved it into Flash
               int size = sizeof(ldr1_samples) + sizeof(ldr2_samples);
               struct LDRFile New_Sample = createLDRFile(ldr1_samples, ldr2_samples, "LDR_Calib", &size, "LDR");
               flash_write_segment_d(&New_Sample);
               // enable TX interrupt, send ldr1,ldr2 samples
               IE2 |= UCA0TXIE;
           }
           P2IFG &= ~PB0;           // clear flag
      }


      if ((P2IFG & PB0) && state == state7) {   // PB0 pressed while in state7
          P2IFG &= ~PB0;  // clear flag

          if (s7_mode == 0) {
              // -------- BROWSE: show next valid file name (0..9) --------
              uint8_t tries = 0;
              while (tries++ < MAX_FILE_ENTRIES) {
                  s7_idx = (uint8_t)((s7_idx +1u) % MAX_FILE_ENTRIES);
                  uint16_t e = ENTRY_ADDR(s7_idx);
                  uint8_t name = *(volatile const uint8_t *)e;   // first byte of struct
                  if (name <= 9u) {
                      lcd_clear();
                      lcd_cmd(0x80);              // line 1
                      lcd_puts("Select file: ");
                      lcd_data((unsigned char)('0' + name));
                      break;
                  }
              }
              // (if none found, display remains unchanged)
          }
          else {
              // -------- VIEW: next 32-char page --------
              if (s7_off + 32u < s7_size) s7_off += 32u;
              s5_print_32_on_lcd((const char *)(s7_addr + s7_off)); // direct from flash
          }
      }


      if ((P2IFG & PB1) && state == state7) {  // PB1 pressed while in state7
          P2IFG &= ~PB1;  // clear flag

          if (s7_mode == 0) {
              // -------- SELECT current entry & open it --------
              uint16_t e   = ENTRY_ADDR(s7_idx);
              uint8_t name = *(volatile const uint8_t *)e;
              if (name <= 9u) {
                  uint16_t size =  (uint16_t)(*(volatile const uint8_t *)(e + 2))
                                 | (uint16_t)(*(volatile const uint8_t *)(e + 3) << 8);
                  uint16_t addr =  (uint16_t)(*(volatile const uint8_t *)(e + 4))
                                 | (uint16_t)(*(volatile const uint8_t *)(e + 5) << 8);

                  s7_size = size;
                  s7_addr = addr;
                  s7_off  = 0;
                  s7_mode = 1;

                  s5_print_32_on_lcd((const char *)s7_addr);     // first page
              } else {
                  lcd_cmd(0x80);
                  lcd_puts("INVALID ENTRY ");
              }
          } else {
              // -------- BACK to list view --------
              s7_mode = 0;
              uint16_t e   = ENTRY_ADDR(s7_idx);
              uint8_t name = *(volatile const uint8_t *)e;
              lcd_cmd(0x80);
              if (name <= 9u) {
                  lcd_clear();
                  lcd_puts("Select file: ");
                  lcd_data((unsigned char)( '0' + name));
              } else {
                  lcd_puts("Select file: -");
              }
          }
      }


  //---------------------------------------------------------------------
  //            Exit from a given LPM
  //---------------------------------------------------------------------
          switch(lpm_mode){
          case mode0:
           LPM0_EXIT;
           break;

          case mode1:
           LPM1_EXIT; // must be called from ISR only
           break;

          case mode2:
           LPM2_EXIT; // must be called from ISR only
           break;

          case mode3:
           LPM3_EXIT; // must be called from ISR only
           break;

          case mode4:
           LPM4_EXIT; // must be called from ISR only
           break;
      }

  }
//*********************************************************************
//                        TIMER0 ISR
//*********************************************************************
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_ISR (void)
{
    LPM0_EXIT;
}

//*********************************************************************
//                        TIMER1 ISR
//*********************************************************************
#pragma vector = TIMER1_A1_VECTOR
__interrupt void Timer1_ISR(void)
{
    switch (__even_in_range(TA1IV, 10)) {
    case 4: // CCR2 (Echo)
        if (Count == 0) {
            REdge1 = TA1CCR2;
            Count  = 1;
            TA1CCTL2 = CM_2 | CCIS_0 | SCS | CAP | CCIE;   // switch to falling
        }
        else {
            REdge2 = TA1CCR2;
            Count  = 0;
            TA1CCTL2 = CM_1 | CCIS_0 | SCS | CAP | CCIE;   // back to rising
        }
        // Up mode wraps at CCR0, not 0x10000
        if (REdge2 >= REdge1) width = REdge2 - REdge1;
        else width = (TA1CCR0 + 1 - REdge1) + REdge2;

        if (state == state1 ) {                 // Mode 1 (scan)
            if (m1_tx_arm) {                   // send only once per loop/degree
                IE2 |= UCA0TXIE;               // trigger TX of 2 bytes
                m1_tx_arm = 0;                 // disarm until next degree
            }
        }
        else if(state == state8){
            if (m1_tx_arm) {                   // send only once per loop/degree
            m1_tx_arm = 0;                 // disarm until next degree
            }
        }else{
            if (last_width + 100 < width || last_width > width + 100) {
                IE2 |= UCA0TXIE;
                last_width = width;
            }
        }

        LPM0_EXIT;
        break;
    default: break;   // CCR1 (Trig) and overflow not used here
    }
}
//*********************************************************************
//                         ADC10 ISR
//*********************************************************************
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
    if (state == state3 && calib_index < 10) {
        ldr1_samples[calib_index] = adc_buf[0]; // A3
        ldr2_samples[calib_index] = adc_buf[3]; // A0
        calib_index++;
        if (calib_index >= 10) {
            // calibration complete
            __bic_SR_register_on_exit(LPM0_bits);
        }
    }
    else if (state == state4) {
        // store this sequence's A3 and A0
          adc_buf_set[2*k + 0] = adc_buf[0];  // A3_k
          adc_buf_set[2*k + 1] = adc_buf[3];  // A0_k
        }
    else if (state == state8 && m2_tx_arm){
           buf8[0] = adc_buf[0];
           buf8[1] = adc_buf[3];
           m2_tx_arm = 0;
    }

   ADC10CTL0 &= ~ADC10IFG;   // clear ADC10 interrupt flag
}

//*********************************************************************
//                           TX ISR
//*********************************************************************
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    if(state == state1){
        UCA0TXBUF = (width & 0xFF);        // SEND low byte
        // wait until TX buffer ready
        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = (width >> 8) & 0xFF;   // SEND high byte
        IE2 &= ~UCA0TXIE;
    }

    else if(state == state2){
        UCA0TXBUF = (width & 0xFF);        // SEND low byte
        // wait until TX buffer ready
        while (!(IFG2 & UCA0TXIFG));
        UCA0TXBUF = (width >> 8) & 0xFF;   // SEND high byte
        IE2 &= ~UCA0TXIE;
    }

    else if (state == state3 ){
        if (calib_index == 10){
            unsigned int i;

            for (i = 0; i < 10; i++) {
              while (!(IFG2 & UCA0TXIFG)); // wait until TX buffer ready
              UCA0TXBUF = (ldr1_samples[i] & 0xFF);
              while (!(IFG2 & UCA0TXIFG)); // wait until TX buffer ready
              UCA0TXBUF = (ldr1_samples[i] >> 8) & 0xFF;
            }
            for (i = 0; i < 10; i++) {
              while (!(IFG2 & UCA0TXIFG)); // wait until TX buffer ready
              UCA0TXBUF = (ldr2_samples[i] & 0xFF);
              while (!(IFG2 & UCA0TXIFG)); // wait until TX buffer ready
              UCA0TXBUF = (ldr2_samples[i] >> 8) & 0xFF;
            }
            IE2 &= ~UCA0TXIE;
            state = state9;
        }
        else{
            IE2 &= ~UCA0TXIE;
        }
    }

    else if(state == state4){
        if (string2[0] == 'N'){
            unsigned int i;
            // send to pc the samples vector to build in pc the look up table
           for (i = 0; i < 10; i++) {
             while (!(IFG2 & UCA0TXIFG)); // wait until TX buffer ready
             UCA0TXBUF = (ldr1_samples[i] & 0xFF);
             while (!(IFG2 & UCA0TXIFG)); // wait until TX buffer ready
             UCA0TXBUF = (ldr1_samples[i] >> 8) & 0xFF;
           }
           for (i = 0; i < 10; i++) {
             while (!(IFG2 & UCA0TXIFG)); // wait until TX buffer ready
             UCA0TXBUF = (ldr2_samples[i] & 0xFF);
             while (!(IFG2 & UCA0TXIFG)); // wait until TX buffer ready
             UCA0TXBUF = (ldr2_samples[i] >> 8) & 0xFF;
           }
        }
        if (string2[0] == 'Y'){
            while (!(IFG2 & UCA0TXIFG));
            UCA0TXBUF = avg_median & 0xFF;
            while (!(IFG2 & UCA0TXIFG));
            UCA0TXBUF = ( avg_median >> 8) & 0xFF;
        }
        string2[0]='Y';
        IE2 &= ~UCA0TXIE;
    }


    else if (state == state5) {
        if (tx5_pending) {
            UCA0TXBUF = tx5_pending;    // send exactly one byte
            tx5_pending = 0;
        }
        IE2 &= ~UCA0TXIE;               // done for now
    }


    else if(state == state6){
        UCA0TXBUF = '6';
        IE2 &= ~UCA0TXIE;
   }
    else if(state == state7){
        UCA0TXBUF = '7';
        IE2 &= ~UCA0TXIE;
    }
    else if(state == state8){
        if (string2[0] == 'N'){
            unsigned int i;
            // send to pc the samples vector to build in pc the look up table
           for (i = 0; i < 10; i++) {
             while (!(IFG2 & UCA0TXIFG)); // wait until TX buffer ready
             UCA0TXBUF = (ldr1_samples[i] & 0xFF);
             while (!(IFG2 & UCA0TXIFG)); // wait until TX buffer ready
             UCA0TXBUF = (ldr1_samples[i] >> 8) & 0xFF;
           }
           for (i = 0; i < 10; i++) {
             while (!(IFG2 & UCA0TXIFG)); // wait until TX buffer ready
             UCA0TXBUF = (ldr2_samples[i] & 0xFF);
             while (!(IFG2 & UCA0TXIFG)); // wait until TX buffer ready
             UCA0TXBUF = (ldr2_samples[i] >> 8) & 0xFF;
           }
        }
        if (string2[0] == 'Y'){
                // sending from LDR
                while (!(IFG2 & UCA0TXIFG));
                UCA0TXBUF = avg_median & 0xFF;
                // wait until TX buffer ready
                while (!(IFG2 & UCA0TXIFG));
                UCA0TXBUF = ( avg_median >> 8) & 0xFF;

                // sending from Echo
                while (!(IFG2 & UCA0TXIFG));
                UCA0TXBUF = (width & 0xFF);
                // wait until TX buffer ready
                while (!(IFG2 & UCA0TXIFG));
                UCA0TXBUF = (width >> 8) & 0xFF;
            }
        string2[0]='Y';
        IE2 &= ~UCA0TXIE;
    }


    else if(state == state9){
        UCA0TXBUF = '9';
        IE2 &= ~UCA0TXIE;
    }
}

//*********************************************************************
//                         RX ISR
//*********************************************************************
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    if((UCA0RXBUF == '1' || delay_ifg1) && delay_ifg2 == 0 && delay_ifg4 == 0 && delay_ifg5 == 0 && delay_ifg8 == 0){
           if (delay_ifg1 == 1){
               string1[j] = UCA0RXBUF;
               j++;
               if (string1[j-1] == '\n'){
                   j = 0;
                   delay_ifg1 = 0;
                   state = state1;
               }
           }
           else{
               delay_ifg1 = 1;
           }
    }
    else if((UCA0RXBUF == '2' || delay_ifg2) && delay_ifg1 == 0 && delay_ifg4 == 0 && delay_ifg5 == 0 && delay_ifg8 == 0){

           if (delay_ifg2 == 1){
               string1[j] = UCA0RXBUF;
               j++;
               if (string1[j-1] == '\n'){
                   j = 0;
                   delay_ifg2 = 0;
                   state = state2;
               }
           }
           else{
               delay_ifg2 = 1;
           }
      }
    else if(UCA0RXBUF == '3' && delay_ifg1 == 0 && delay_ifg2 == 0 && delay_ifg4 == 0 && delay_ifg5 == 0 && delay_ifg8 == 0){
            IE2 &= ~UCA0TXIE;
            state = state3;
    }
    else if((UCA0RXBUF == '4' || delay_ifg4) && delay_ifg2 == 0 && delay_ifg1 == 0 && delay_ifg5 == 0 && delay_ifg8 == 0){
               if (delay_ifg4 == 1){
                   string2[j] = UCA0RXBUF;
                   j++;
                   if (string2[j-1] == '\n'){
                       j = 0;
                       delay_ifg4 = 0;
                       state = state4;
                   }
               }
               else{
                   delay_ifg4 = 1;
               }
        }


    else if ((UCA0RXBUF == '5' || delay_ifg5) && delay_ifg1 == 0 && delay_ifg2 == 0 && delay_ifg4 == 0 && delay_ifg8 == 0) {
        if (!delay_ifg5) {
            // Command '5' received: begin header collection
            delay_ifg5 = 1;
            rx5_phase  = 0;
            p          = 0;
            segC_next       = InfoC_GetFreeAddr();
            MainMemorystart = Find_space();
            state = state5;
            return;
        }

        if (rx5_phase == 0) {
            // -------- HEADER (6 ASCII bytes: n t s s s s) --------
            array5[p++] = UCA0RXBUF;
            if (p == 6) {
                uint8_t  name_id = (uint8_t)(array5[0] - '0');
                uint8_t  type    = (uint8_t)(array5[1] - '0');
                uint16_t size    = (uint16_t)((array5[2]-'0')*1000u + (array5[3]-'0')*100u  + (array5[4]-'0')*10u   + (array5[5]-'0'));

                // Validate
                if (name_id > 9 || (type != 0 && type != 1) || size == 0 || size > 2048) {
                    tx5_pending = 'E';                 //  signal error from TX
                    IE2 |= UCA0TXIE;                   //  trigger TX ISR to send it
                    delay_ifg5 = 0; rx5_phase = 0; p = 0;
                    return;
                }

                // Check room
                if ((uint32_t)MainMemorystart + size > FILE_AREA_END) {
                    tx5_pending = 'F';                 //  no space
                    IE2 |= UCA0TXIE;                   //
                    delay_ifg5 = 0; rx5_phase = 0; p = 0;
                    return;
                }

                // Create & save struct in Info memory
                struct FileEntry F = TextStruct_create(name_id, size, type, MainMemorystart);
           //     flash_erase_range(segC_next, 6);       // room for struct
                if (InfoC_SaveStruct(&F) != 0) {
                    tx5_pending = 'E';                 // save failed
                    IE2 |= UCA0TXIE;
                    delay_ifg5 = 0; rx5_phase = 0; p = 0;
                    return;
                }

                // Prepare data phase
                //flash_erase_range(MainMemorystart, size);  // erase target area first
                rx5_addr      = MainMemorystart;
                rx5_remaining = size;
                rx5_size      = size;                  //  keep size for fs_bump_next
                rx5_phase     = 1;

                // Tell PC to start sending data — from TX ISR only
                tx5_pending = 'A';                     //  ACK header
                IE2 |= UCA0TXIE;                       //  kick TX ISR
            }
            return;
        }
        else {
            // -------- DATA (rx5_remaining bytes) --------
            if (rx5_remaining) {
                flash_write_byte_isr(rx5_addr++, UCA0RXBUF);
                if (--rx5_remaining == 0) {
                   // fs_bump_next(rx5_addr - rx5_size, rx5_size); //  use saved size
                    tx5_pending = 'K';              //  done
                    IE2 |= UCA0TXIE;                //  have TX ISR send 'K'
                    delay_ifg5 = 0; rx5_phase = 0; p = 0;
                    //state = state9;
                }
            }
            return;
        }
    }


    else if(UCA0RXBUF == '6' && delay_ifg1 == 0 && delay_ifg2 == 0 && delay_ifg4 == 0 && delay_ifg5 == 0 && delay_ifg8 == 0){
        state = state6;
        IE2 |= UCA0TXIE;
    }
    else if(UCA0RXBUF == '7' && delay_ifg1 == 0 && delay_ifg2 == 0 && delay_ifg4 == 0 && delay_ifg5 == 0 && delay_ifg8 == 0){
            state = state7;
            IE2 |= UCA0TXIE;
        }
    else if((UCA0RXBUF == '8' || delay_ifg8) && delay_ifg2 == 0 && delay_ifg4 == 0 && delay_ifg5 == 0 && delay_ifg1 == 0){
              if (delay_ifg8 == 1){
                  string2[j] = UCA0RXBUF;
                  j++;
                  if (string2[j-1] == '\n'){
                      j = 0;
                      delay_ifg8 = 0;
                      state = state8;
                  }
              }
              else{
                  delay_ifg8 = 1;
              }
    }

    else if(UCA0RXBUF == '9' && delay_ifg1 == 0 && delay_ifg2 == 0 && delay_ifg4 == 0 && delay_ifg5 == 0){
        state = state9;
        IE2 |= UCA0TXIE;
    }

    switch(lpm_mode){
    case mode0:
        LPM0_EXIT;
        break;
    case mode1:
        LPM1_EXIT; // must be called from ISR only
        break;
    case mode2:
        LPM2_EXIT; // must be called from ISR only
        break;
    case mode3:
        LPM3_EXIT; // must be called from ISR only
        break;
    case mode4:
        LPM4_EXIT; // must be called from ISR only
        break;
    }
}
