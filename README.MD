# DCS EE Final Project — MSP430 Object & Light Scanner

A compact “radar-style” scanner built on MSP430G2553 that sweeps 0–180° with a micro-servo, measures distance with an ultrasonic sensor, and detects bright light sources with two LDRs.  
A desktop Python/Tkinter app provides calibration, live plots, and post-scan detection of both objects and lights. Communication is over UART.

---

## Hardware
- **MCU:** TI MSP430G2553  
- **Servo:** SG90 on TA0.1 PWM  
- **Ultrasonic:** HC-SR04  
- **Light sensors:** 2× LDRs in voltage dividers to ADC channels  
- **Level safety:** HC-SR04 ECHO is 5 V — use a divider to ~3.3 V before the MSP430 pin  
- **Power:** 5 V for servo/sensor, 3.3 V for MCU; common GND  

---

## Connections

| Pin  | Function           |
|------|--------------------|
| P1.0 | LDR2               |
| P1.1 | UART RX            |
| P1.2 | UART TX            |
| P1.3 | LDR1               |
| P1.4 | LCD D4             |
| P1.5 | LCD D4             |
| P1.6 | LCD D4             |
| P1.7 | LCD D4             |
| P2.0 | PB0                |
| P2.1 | UltraSonic Trig    |
| P2.2 | PB1                |
| P2.3 | LCD E              |
| P2.4 | UltraSonic Echo    |
| P2.5 | LCD RS             |
| P2.6 | Servo PWM          |
| P2.7 | LCD RW             |

---

## Activation & Usage
Run the GUI and click **Connect**.  
By default, the app connects to **COM3 @ 9600 8N1**.

---

### Telemeter
- Enter **Angle (0–180)** → **Send to MCU**  
- The app transmits `2ddd` (e.g., `2005` for 5°) and shows live distance  

---

### Objects (distance sweep)
- Set **Threshold (cm)** → **Send to MCU**  
- The app sends `1ttt` (e.g., `1060` for 60 cm), sweeps **0→180°**, and marks detected objects after the sweep  

---

### Light Sources
1. Click **LDR Calibration** (`3`) to request a fresh **40-byte frame**  
   (two 10-sample arrays, expanded to 2×50 for plotting)  
2. Click **Light Sources Detector**  
   - If calibration is already available → PC sends `4Y`  
   - Otherwise → PC sends `4N` and waits for the 40-byte calibration before sweeping  
3. During the sweep, the MCU streams **2 bytes per degree (0–180)**  

---

### Light + Objects (combined)
- Click **Light+Objects Scan**  
  - If LDR LUTs are present → PC sends `8Y`  
  - Otherwise → PC sends `8N` and waits for the 40-byte calibration  
- During the sweep, the MCU streams **4 bytes per degree**:  
  - `[0–1]` LDR sample  
  - `[2–3]` echo/telemeter (little-endian)  
- At the end of the sweep, the semicircle view shows both **objects** and **light sources**  

---

### Text files
- Go to **Text files** → choose a `.txt` → set **Name ID (0–9)** → **Send to MCU**  
- The app sends a header: `5 + n + 0 + ssss`  
  - `ssss` = zero-padded size (max 2048)  
- MCU responses:  
  - `A` = ready  
  - `F` = no space  
  - `E` = save fail  
- Then the app streams the data and waits for `K` (OK) to confirm finish  

Extras:  
- **Erase Flash** → sends `6`  
- **File browser & viewer** (MCU LCD) → sends `7`  

---

## Quick workflow
1. **Light Sources** → LDR Calibration (get the 40-byte curves)  
2. **Objects** scan → confirm distances  
3. **Light+Objects Scan** → see both detections on one plot  
