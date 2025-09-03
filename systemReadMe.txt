systemReadMe.txt
================

MCU Firmware (MSP430)
---------------------

app.h
-----
Purpose: Basic definitions of FSM states and system modes.
- enum FSMstate – defines the FSM states (state1..state9).
- enum SYSmode – defines system modes (mode0..mode4).

bsp.h
-----
Purpose: Board Support Package – hardware definitions, structures, and flash addresses.
- struct LDRFile – stores LDR samples.
- struct FileEntry – describes a file in flash memory.
- Hardware abstraction macros for LCD, push buttons, LDR, Servo, UART, and Flash.
- Function declarations: GPIOconfig(), ADC_config(), Servo_Init(), UART_init(), TimerA1_TrigInit(), Echo_Init_P24(), etc.

halGPIO.h
---------
Purpose: Hardware Abstraction Layer – global variables and helper functions.
- System setup: sysConfig().
- Delay helpers: delay(), DelayMs(), DelayUs(), TA0_delay_us(), TA0_delay_ms().
- Flash functions: flash_write_byte_isr(), flash_erase_range(), flash_erase_infoC(), flash_erase_Seg1..Seg4().
- Data helpers: createLDRFile(), readFlashSamples(), s5_print_32_on_lcd().
- LCD control: lcd_cmd(), lcd_data(), lcd_puts(), lcd_init(), lcd_strobe().
- ISRs: PBs_handler, PB2s_handler, Timer0_ISR, Timer1_ISR, ADC10_ISR, USCI0RX_ISR, USCI0TX_ISR.

api.h
-----
Purpose: Application API layer.
Functions:
- count_up_LCD(), count_down_LCD(), clear_counters(), change_delay_time().
- PB1_press(), PB1_PB2_press().
- Circular_tone_buzzer().
- Telemeter(), Scan_detector(), Light_Sources_Detector_Scan().
- Servo_SetAngleDeg(), Servo_coast_to_coast().
- adc10_calib_trigger(), median3().

bsp.c
-----
Purpose: Implementation of BSP functions.
- GPIOconfig() – configure pins, LCD, and push buttons.
- StopAllTimers() – stop timers.
- UART_init() – initialize UART.
- TimerA1_TrigInit(), Echo_Init_P24() – ultrasonic sensor setup.
- Servo_Init() – PWM initialization for servo.
- adc10_calib_init() – ADC initialization for LDR channels.
- TA0_delay_ticks() – timer-based delay.

halGPIO.c
---------
Purpose: Implementation of HAL – flash, LCD, delay, ISR.
- sysConfig() – system initialization.
- createLDRFile(), TextStruct_create() – create data structures.
- Flash functions: flash_erase_infoC(), flash_erase_Seg1..Seg4(), flash_write_byte_isr(), InfoC_SaveStruct(), InfoC_GetFreeAddr(), flash_erase_range().
- flash_write_segment_d(), readFlashSamples().
- Delay helpers: int2str(), delay(), DelayUs(), DelayMs(), timer_call_counter(), TA0_delay_us(), TA0_delay_ms().
- LCD functions: lcd_cmd(), lcd_data(), lcd_puts(), lcd_init(), lcd_strobe(), s5_print_32_on_lcd().
- ISRs: PBs_handler(), PB2s_handler(), Timer0_ISR(), Timer1_ISR, ADC10_ISR, USCI0TX_ISR, USCI0RX_ISR.

api.c
-----
Purpose: Application logic.
- Telemeter() – ultrasonic distance measurement.
- Servo_SetAngleDeg(), Servo_coast_to_coast() – servo control.
- Scan_detector() – 0–180° scanning for object detection.
- adc10_calib_trigger() – start ADC sampling.
- Light_Sources_Detector_Scan(), median3() – light source detection with LDR.
- count_up_LCD(), count_down_LCD() – counting on LCD.
- Circular_tone_buzzer() – buzzer tones.
- change_delay_time(), clear_counters().
- PB1_press(), PB1_PB2_press().

main.c
------
Purpose: Main program – system FSM.
- main() – FSM loop:
  - state1: Scan_detector().
  - state2: Servo_coast_to_coast() + Telemeter().
  - state3: LDR calibration and flash storage.
  - state4: Light_Sources_Detector_Scan().
  - state5: File upload and flash save (via ISR).
  - state6: Flash erase.
  - state7: Browse and view files on LCD.
  - state9: LPM (Idle).

-----------------------------------------------------

PC-Side GUI (Python: main_O.py)
-------------------------------

Purpose: A Tkinter GUI application that communicates with the MCU over serial (COM port).

Main Components:
- Constants: conversion factors for ultrasonic echo to cm, light detection tuning.
- Enum Mode – defines modes: OBJECTS, TELEMETER, LIGHT, LIGHT_AND_OBJECTS, TEXT.
- SerialManager – manages serial connection, RX/TX threads, sending/receiving data.

Functions/Classes:
- echo_us_to_cm() – converts echo duration to distance in cm.
- extract_echo_us_ascii() – parses ASCII telemeter data.
- App (Tk class) – GUI main class:
  - on_connect(), on_disconnect(), on_reset_all() – serial connection handling.
  - send_mode() – sends the correct command based on the active GUI tab.
  - on_ldr_calibration(), on_show_ldr_arrays(), on_light_detector() – LDR handling.
  - on_erase_flash(), on_send_7() – Flash operations and file browsing.
  - _poll_serial() – polls data from MCU, updates GUI live.
  - _update_telemeter(), _draw_chart() – updates Telemeter tab.
  - _update_objects_live(), _finalize_objects_sweep(), _draw_objects() – object detection handling.
  - _draw_ldr_view(), _draw_light_sources(), _finalize_light_sweep() – Light Sources tab visualization.
  - File handling for sending text files to MCU via ASCII header and binary data.

Execution:
- if __name__ == "__main__": runs App().mainloop() for GUI startup.

-----------------------------------------------------

Usage Instructions
------------------

1. **MCU Setup**
   - Flash the MSP430 firmware (main.c + headers and source files) onto the MCU.
   - Connect the following peripherals:
     - LCD display (Port1 + control pins on Port2.3/2.5/2.7).
     - Ultrasonic sensor (Trig on P2.1, Echo on P2.4).
     - Servo motor (PWM on P2.6).
     - LDR sensors (P1.0 and P1.3).
     - Push buttons (P2.0 = PB0, P2.2 = PB1).
   - Power and reset the MCU. The system starts in **Idle (state9)**.

2. **PC GUI Setup**
   - Install Python 3.8+ with required modules: 'tkinter', 'pyserial'.
   - Connect the MCU to the PC via USB (check COM port number).
   - Run 'python main_O.py' to start the GUI.

3. **Using the GUI**
   - Press **Connect** to open the serial link (default COM3 @ 9600).
   - Choose a tab for the desired mode:
     - **Telemeter**: Measure distance at a chosen angle (0–180).
     - **Objects**: Perform 180 sweep and detect objects.
     - **Light Sources**: Calibrate LDRs and scan for light sources.
     - **Light+Objects**: Combined mode.
     - **Text files**: Upload `.txt` files to flash, browse and view saved files, or erase flash.
   - Use the **Send to MCU** button on each tab to start the selected operation.

4. **Special Operations**
   - **Erase Flash** (Text tab): Wipes all stored files.
   - **File Browser & Viewer** (Text tab): Browse files on the MCU using LCD + push buttons.
   - **Reset All** (top bar): Resets GUI buffers and state (MCU is unaffected).

5. **Shutdown**
   - Always click **Disconnect** before unplugging the MCU to ensure clean serial closure.
