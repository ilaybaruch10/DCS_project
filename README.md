**DCS EE Final Project — MSP430 Object & Light Scanner**
A compact “radar-style” scanner built on MSP430G2553 that sweeps 0-180° with a micro-servo, measures distance with an ultrasonic sensor, and detects bright light sources with two LDRs. A desktop Python/Tkinter app provides calibration, live plots, and post-scan detection of both objects and lights. Communication is over UART.

**Hardware** 
MCU: TI MSP430G2553 
Servo: SG90 on TA0.1 PWM.
Ultrasonic: HC-SR04.
Light sensors: 2× LDRs in voltage dividers to ADC channels.
Level safety: HC-SR04 ECHO is 5 V—use a divider to ~3.3 V before the MSP430 pin.
Power: 5 V for servo/sensor, 3.3 V for MCU; common GND.

**Connections**
 p1.0 --> LDR2
 p1.1 -->      UART
 p1.2 -->      UART
 p1.3 --> LDR1
 p1.4 --> LCD D4
 p1.5 --> LCD D4
 p1.6 --> LCD D4
 p1.7 --> LCD D4
 
 p2.0 --> PB0
 p2.1 --> UltraSonic Trig
 p2.2 --> PB1
 p2.3 --> LCD E
 p2.4 --> UltraSonic Echo
 p2.5 --> LCD RS
 p2.6 --> Servo PWM
 p2.7 --> LCD RW

**Activation & Usage**
Run the GUI.
Click Connect. By default, the app connects to COM3 @ 9600 8N1.

Using each tab:
**Telemeter**
Enter Angle (0-180) --> Send to MCU.
The app transmits 2ddd (e.g., 2005 for 5°) and shows live distance.

**Objects** (distance sweep)
Set Threshold (cm) --> Send to MCU.
The app sends 1ttt (e.g., 1060 for 60 cm), sweeps 0→180°, and marks detected objects after the sweep.

**Light Sources**
Click LDR Calibration (sends 3) to request a fresh 40-byte frame (two 10-sample arrays, expanded to 2×50 for plotting). 
Click Light Sources Detector.
If calibration is already available, the PC sends 4Y; otherwise, it sends 4N and waits for the 40-byte calibration before sweeping.
During the sweep, the MCU streams 2 bytes per degree (0-180).

**Light + Objects** (combined)
Click Light+Objects Scan.
If LDR LUTs are present, the PC sends 8Y; otherwise, it sends 8N and waits for the 40-byte calibration.
During the sweep, the MCU streams 4 bytes per degree: [0–1] LDR sample, [2–3] echo/telemeter (little-endian).
When the sweep completes, the semicircle view shows both objects and light sources.

**Text files**
Go to Text files --> choose a .txt --> set Name ID (0–9) --> Send to MCU.
The app sends a header: 5 + n + 0 + ssss, where ssss is the zero-padded size (max 2048). It then waits for A (ready), F (no space), or E (save fail), streams the data, and finally waits for K (OK) to confirm finish.
Extras on this tab: Erase Flash (sends 6) and file browser & viewer via MCU LCD (sends 7)

**Quick workflow**
Light Sources -->  LDR Calibration (get the 40-byte curves)
Objects scan to confirm distances.
Light+Objects Scan to see both detections on one plot.
