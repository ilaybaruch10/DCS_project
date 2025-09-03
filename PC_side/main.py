
import threading
import time
import re
import queue
from enum import Enum
from collections import deque
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from typing import Optional
import serial
from serial import Serial
import math
from pathlib import Path

# ------------------ Constants ------------------
SPEED_CM_PER_SEC = 34645.0   # ~25°C
TA1_TICKS_PER_SECOND = 1_000_000.0  # MSP430 TA1 runs at 1 MHz
OBJ_MIN_LEN_CM = 7
# ---- Light detector tuning ----
LIGHT_MAX_CM   = 50         # calibration is 1..50 cm
LIGHT_MATCH_TOL = 80        # ADC counts tolerance for LUT match (tune if needed)
LIGHT_MIN_RUN   = 4         # min consecutive degrees to accept as one source
LIGHT_GAP_TOL   = 3     # degrees: merge runs separated by small gap


def echo_us_to_cm(echo_us: float) -> float:
    return (echo_us * SPEED_CM_PER_SEC) / (1_000_000.0 * 2.0)

# ------------------ FSM ------------------
class Mode(Enum):
    OBJECTS = ('Objects Detector', '1')
    TELEMETER = ('Telemeter', '2')
    LIGHT = ('Light Sources', '3')
    LIGHT_AND_OBJECTS = ('Light + Objects', '4')
    TEXT = ('Text files', '5')

    @property
    def label(self): return self.value[0]
    @property
    def tx_char(self): return self.value[1]

# ------------------ Serial manager ------------------
class SerialManager:
    def __init__(self, port='COM3', baud=9600):
        self.port = port
        self.baud = baud
        self.ser: Optional[Serial] = None
        self.rx_q = queue.Queue()
        self.rx_bytes_q = queue.Queue()
        self.tx_q = queue.Queue()
        self._stop = threading.Event()
        self.reader_t: Optional[threading.Thread] = None
        self.writer_t: Optional[threading.Thread] = None

    def connect(self):
        if self.ser and self.ser.is_open: return
        self.ser = Serial(
            self.port,
            baudrate=self.baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
            write_timeout=0.2,
        )
        self.flush()
        self._stop.clear()
        self.reader_t = threading.Thread(target=self._reader_loop, daemon=True)
        self.writer_t = threading.Thread(target=self._writer_loop, daemon=True)
        self.reader_t.start(); self.writer_t.start()

    def disconnect(self):
        self._stop.set()
        for t in (self.reader_t, self.writer_t):
            if t and t.is_alive(): t.join(timeout=0.5)
        if self.ser:
            try: self.ser.close()
            except Exception: pass
        self.ser = None

    def flush(self):
        if self.ser:
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass

    def send(self, s: str):
        if not s.endswith('\n'): s += '\n'
        self.tx_q.put(s.encode('ascii', errors='ignore'))
    
    def send_bytes(self, b: bytes):
        self.tx_q.put(bytes(b))

    def send_char(self, c: str):
        self.tx_q.put((c[0]).encode('ascii', errors='ignore'))

    def _reader_loop(self):
        buf = bytearray()
        while not self._stop.is_set():
            try:
                if not self.ser:
                    time.sleep(0.05); continue
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if chunk:
                    self.rx_bytes_q.put(bytes(chunk))
                    buf.extend(chunk)
                    while b'\n' in buf:
                        line, _, buf = buf.partition(b'\n')
                        try: s = line.decode('ascii', errors='replace').strip()
                        except Exception: s = ''
                        if s: self.rx_q.put(s)
                else:
                    time.sleep(0.02)
            except Exception as e:
                self.rx_q.put(f"__ERR__ {e!r}")
                time.sleep(0.1)

    def _writer_loop(self):
        while not self._stop.is_set():
            try: b = self.tx_q.get(timeout=0.1)
            except queue.Empty: continue
            try:
                if self.ser:
                    self.ser.write(b); self.ser.flush()
            except Exception as e:
                self.rx_q.put(f"__ERR__ TX {e!r}")

# ------------------ Parser for ASCII TELEMETER ------------------
TELEM_PATTERNS = [
    re.compile(r'(?i)(echo|t|time)[_\s-]*us\s*[:=]\s*(\d+(?:\.\d+)?)'),
    re.compile(r'(?i)t\s*[:=]\s*(\d+(?:\.\d+)?)\s*us'),
    re.compile(r'^\s*(\d+(?:\.\d+)?)\s*$'),
]

def extract_echo_us_ascii(line: str, ticks_hz: Optional[float]) -> Optional[float]:
    for pat in TELEM_PATTERNS:
        m = pat.search(line)
        if m: return float(m.group(m.lastindex))
    m = re.search(r'(?i)(ic|ticks)\s*[:=]\s*(\d+(?:\.\d+)?)', line)
    if m and ticks_hz and ticks_hz > 0:
        ticks = float(m.group(2))
        return (ticks / ticks_hz) * 1_000_000.0
    return None

# ------------------ GUI ------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("DCS Project – PC GUI (FSM, Non-Blocking)")
        self.geometry("900x580")
        self.minsize(860, 560)
        self.configure(bg="#0b1020")

        self.ser = SerialManager(port='COM3', baud=9600)

        self.current_mode = tk.StringVar(value=Mode.TELEMETER.name)
        self.connected = tk.BooleanVar(value=False)
        self.status_text = tk.StringVar(value="Disconnected")
        self.distance_cm_last = tk.DoubleVar(value=0.0)

        # Mode 2 state
        self.bin_buf = bytearray()
        self.dist_hist = deque(maxlen=120)

        # Mode 1 state
        self.obj_distance_cm = tk.DoubleVar(value=0.0)
        self.obj_angle_deg   = tk.IntVar(value=0)
        self.obj_bin_buf     = bytearray()
        self.obj_angle_count = 0
        self.obj_sweep = [None]*181
        self.obj_objects = []

        # Mode 3 (LDR) state
        self.ldr_buf = bytearray()
        self.ldr1_raw = None    # 10 samples at 5..50cm
        self.ldr2_raw = None
        self.ldr1_lut = None    # 50 interpolated
        self.ldr2_lut = None
        self.ldr_status = tk.StringVar(value="Press LDR Calibration to fetch 40 bytes.")
        self.ldr_capture_active = False
        self.ldr_capture_deadline = 0.0

        # Mode 4 (Light Detector sweep) state
        self.light_bin_buf = bytearray()
        self.light_angle_count = 0
        self.light_sweep = [None]*181
        self.light_sources = []

        #live angle and which view to draw on the Light tab
        self.light_deg_var = tk.IntVar(value=0)
        self.light_show_sources = False

        # Mode 5 Text-files state 
        self.txt_path = tk.StringVar(value="")
        self.txt_name_id = tk.IntVar(value=0)   # 0..9
        self.txt_status = tk.StringVar(value="Choose a .txt file and press Send.")
        self.txt_progress = tk.DoubleVar(value=0.0)
        self.send_btn = None  # will point to the modebar "Send" button

        # mode 8 (Light + Objects) state
        self.lo_bin_buf = bytearray()
        self.lo_angle_count = 0
        self.lo_light_sweep = [None]*181
        self.lo_echo_sweep = [None]*181
        self.lo_sources = []
        self.lo_objects = []
        self.lo_ldr1_raw = None
        self.lo_ldr2_raw = None
        self.lo_ldr1_lut = None
        self.lo_ldr2_lut = None
        self.lo_capture_active = False
        self.lo_capture_deadline = 0.0
        self.lo_status = tk.StringVar(value="Press Light+Objects Scan to start.")
        
        self._build_ui()
        self.after(50, self._poll_serial)

    # ---------- helpers ----------
    def _drain_bytes(self, target: bytearray):
        try:
            while True:
                target.extend(self.ser.rx_bytes_q.get_nowait())
        except queue.Empty:
            pass

    def _drain_lines_iter(self):
        try:
            while True:
                yield self.ser.rx_q.get_nowait()
        except queue.Empty:
            return

    def _build_ui(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure("TFrame", background="#0b1020")
        style.configure("TLabel", background="#0b1020", foreground="#e6e8ef", font=("Segoe UI", 10))
        style.configure("Header.TLabel", font=("Segoe UI Semibold", 12))
        style.configure("Big.TLabel", font=("Segoe UI", 28, "bold"))
        style.configure("Small.TLabel", font=("Consolas", 10))
        style.configure("TButton", padding=6)
        style.map("TButton", foreground=[('disabled', '#888')])

        # Top bar (connect / status)
        top = ttk.Frame(self, padding=10)
        top.pack(fill="x")
        ttk.Button(top, text="Connect", command=self.on_connect).pack(side="left", padx=4)
        ttk.Button(top, text="Disconnect", command=self.on_disconnect).pack(side="left", padx=4)
        ttk.Button(top, text="Reset All", command=self.on_reset_all).pack(side="left", padx=12)
        ttk.Label(top, textvariable=self.status_text, style="Header.TLabel").pack(side="right")

        # Tabs (single bar of nodes)
        self.nb = ttk.Notebook(self)
        self.nb.pack(fill="both", expand=False, padx=10, pady=(0,4))
        self.nb.bind("<<NotebookTabChanged>>", self._on_tab_change)

        self.page_objects = ttk.Frame(self.nb, padding=12, style="TFrame")
        self.page_telem   = ttk.Frame(self.nb, padding=12, style="TFrame")
        self.page_light   = ttk.Frame(self.nb, padding=12, style="TFrame")
        self.page_combo   = ttk.Frame(self.nb, padding=12, style="TFrame")
        self.page_text    = ttk.Frame(self.nb, padding=12, style="TFrame")

        self.nb.add(self.page_objects, text="Objects")
        self.nb.add(self.page_telem,   text="Telemeter")
        self.nb.add(self.page_light,   text="Light Sources")
        self.nb.add(self.page_combo,   text="Light+Objects")
        self.nb.add(self.page_text, text="Text files")

        # --- Dynamic Mode Bar (controls + Send) ---
        self.modebar = ttk.Frame(self, padding=(10, 4))
        self.modebar.pack(fill="x")
        # entries used by multiple refreshes
        self.angle_var = tk.StringVar(value="090")
        self.thresh_var = tk.StringVar(value="060")
        self._refresh_modebar()

        # --- Telemeter page content ---
        ttk.Label(self.page_telem, text="Live distance (Telemeter)", style="Header.TLabel").pack(anchor="w")
        big = ttk.Label(self.page_telem, textvariable=self.distance_cm_last, style="Big.TLabel")
        big.pack(anchor="center", pady=(6, 0))
        ttk.Label(self.page_telem, text="cm", style="Big.TLabel").pack(anchor="center")
        self.canvas = tk.Canvas(self.page_telem, width=760, height=180, bg="#0f1733", highlightthickness=0)
        self.canvas.pack(fill="x", pady=8)

        # --- Objects page content ---
        ttk.Label(self.page_objects, text="Live (Objects Detector)", style="Header.TLabel").pack(anchor="w")
        live = ttk.Frame(self.page_objects); live.pack(anchor="w", pady=(6, 0))
        ttk.Label(live, text="Angle:").pack(side="left")
        ttk.Label(live, textvariable=self.obj_angle_deg, style="Header.TLabel").pack(side="left", padx=(4,16))
        ttk.Label(live, text="Distance:").pack(side="left")
        ttk.Label(live, textvariable=self.obj_distance_cm, style="Header.TLabel").pack(side="left")
        ttk.Label(live, text="cm").pack(side="left", padx=(4,16))
        self.obj_canvas = tk.Canvas(self.page_objects, width=760, height=300, bg="#0f1733", highlightthickness=0)
        self.obj_canvas.pack(fill="x", pady=8)
        
        ttk.Label(self.page_light, text="Light Sources", style="Header.TLabel").pack(anchor="w")
        ttk.Label(self.page_light, textvariable=self.ldr_status).pack(anchor="w", pady=(6,0))
        self.ldr_canvas = tk.Canvas(self.page_light, width=760, height=260, bg="#0f1733", highlightthickness=0)
        self.ldr_canvas.pack(fill="x", pady=8)

        
        # --- Light+Objects page content ---
        ttk.Label(self.page_combo, text="Light + Objects", style="Header.TLabel").pack(anchor="w")
        ttk.Label(self.page_combo, textvariable=self.lo_status).pack(anchor="w", pady=(6,0))
        self.lo_canvas = tk.Canvas(self.page_combo, width=800, height=260, bg="#0f1733", highlightthickness=0)
        self.lo_canvas.pack(fill="x", pady=8)

        # --- Text files page ---
        ttk.Label(self.page_text, text="Send a .txt file to MCU").pack(anchor="w")

        row = ttk.Frame(self.page_text); row.pack(fill="x", pady=(8,6))
        ttk.Label(row, text="File:").pack(side="left")
        ttk.Entry(row, textvariable=self.txt_path, width=70, state="readonly").pack(side="left", padx=6)
        ttk.Button(row, text="Browse…", command=self._txt_browse).pack(side="left")

        opts = ttk.Frame(self.page_text); opts.pack(fill="x", pady=(2,6))
        ttk.Label(opts, text="Name ID (0–9):").pack(side="left")
        ttk.Spinbox(opts, from_=0, to=9, textvariable=self.txt_name_id, width=3, justify="center").pack(side="left", padx=(6,18))

        ttk.Label(self.page_text, textvariable=self.txt_status).pack(anchor="w", pady=(4,0))
        self.txt_pbar = ttk.Progressbar(self.page_text, orient="horizontal", mode="determinate", maximum=100, length=760)
        self.txt_pbar.pack(fill="x", pady=(6,0))

    # ---------- Mode bar renderer ----------
    def _refresh_modebar(self):
        # wipe bar and reset send_btn for this tab
        for w in self.modebar.winfo_children(): 
            w.destroy()
        self.send_btn = None

        tab = self.nb.tab(self.nb.select(), "text")

        if tab.startswith("Telemeter"):
            ttk.Label(self.modebar, text="Angle (0–180):").pack(side="left")
            ttk.Entry(self.modebar, textvariable=self.angle_var, width=5, justify="center").pack(side="left", padx=6)
            self.send_btn = ttk.Button(self.modebar, text="Send to MCU", command=self.send_mode)
            self.send_btn.pack(side="left", padx=8)

        elif tab.startswith("Objects"):
            ttk.Label(self.modebar, text="Threshold (cm):").pack(side="left")
            ttk.Entry(self.modebar, textvariable=self.thresh_var, width=5, justify="center").pack(side="left", padx=6)
            self.send_btn = ttk.Button(self.modebar, text="Send to MCU", command=self.send_mode)
            self.send_btn.pack(side="left", padx=8)

        elif tab.startswith("Light Sources"):
            ttk.Button(self.modebar, text="LDR Calibration", command=self.on_ldr_calibration).pack(side="left", padx=4)
            ttk.Button(self.modebar, text="Show 50-sample arrays", command=self.on_show_ldr_arrays).pack(side="left", padx=8)
            ttk.Button(self.modebar, text="Light Sources Detector", command=self.on_light_detector).pack(side="left", padx=8)
            # No single "Send" here → leave self.send_btn = None

        elif tab.startswith("Text files"):
            self.send_btn = ttk.Button(self.modebar, text="Send to MCU", command=self.send_mode)
            self.send_btn.pack(side="left", padx=8)
            ttk.Button(self.modebar, text="Erase Flash", command=self.on_erase_flash).pack(side="left", padx=8)
            ttk.Button(self.modebar, text="file browser & viewer", command=self.on_send_7).pack(side="left", padx=8)
        
        elif tab.startswith("Light+Objects"):
            ttk.Label(self.modebar, text="Threshold (cm):").pack(side="left")
            ttk.Entry(self.modebar, textvariable=self.thresh_var, width=5, justify="center").pack(side="left", padx=6)
            ttk.Button(self.modebar, text="Light+Objects Scan", command=self.on_light_objects_scan).pack(side="left", padx=8)
            

        else:
            ttk.Label(self.modebar, text="No controls for this tab").pack(side="left")

    # ------------- Events -------------
    def on_connect(self):
        self.ser.port = 'COM3'
        try:
            self.ser.connect()
            self.connected.set(True)
            self.status_text.set("Connected to COM3 @ 9600 8N1")
        except Exception as e:
            messagebox.showerror("Serial error", f"Could not open COM3\n\n{e}")
            self.status_text.set("Disconnected")

    def on_disconnect(self):
        self.ser.disconnect()
        self.connected.set(False)
        self.status_text.set("Disconnected")

    def _on_tab_change(self, _evt=None):
        tab = self.nb.tab(self.nb.select(), "text")
        if tab.startswith("Objects"): self.current_mode.set(Mode.OBJECTS.name)
        elif tab.startswith("Telemeter"): self.current_mode.set(Mode.TELEMETER.name)
        elif tab.startswith("Light Sources"): self.current_mode.set(Mode.LIGHT.name)
        elif tab.startswith("Light+Objects"): self.current_mode.set(Mode.LIGHT_AND_OBJECTS.name)
        elif tab.startswith("Text files"): self.current_mode.set(Mode.TEXT.name)
        self._refresh_modebar()
        self.on_mode_change()

    def on_mode_change(self):
        # Reset streaming state for a clean view; MCU mode persists separately
        self.bin_buf.clear()
        self.obj_bin_buf.clear()
        self.obj_angle_count = 0
        self.obj_sweep = [None]*181
        self.obj_objects = []

         # only clear capture buffers; keep self.ldr1_lut/self.ldr2_lut
        self.ldr_buf.clear()
        self.ldr_capture_active = False

        self.light_bin_buf.clear()
        self.light_angle_count = 0
        self.light_sweep = [None]*181
        self.light_sources = []

        if self.ser.ser:
            self.ser.flush()
            self._drain_queues()


    def on_light_objects_scan(self):
        if not self.connected.get():
            messagebox.showwarning("Not connected", "Connect to the MCU first.")
            return

        if self.ser.ser:
            self.ser.flush()
            self._drain_queues()
        self.lo_bin_buf.clear()
        self.lo_angle_count = 0
        self.lo_light_sweep = [None]*181
        self.lo_echo_sweep = [None]*181
        self.lo_sources = []
        self.lo_objects = []
        self.lo_ldr1_raw = None
        self.lo_ldr2_raw = None
        self.lo_ldr1_lut = None
        self.lo_ldr2_lut = None
        self.lo_capture_active = False
        self.lo_capture_deadline = 0.0

        # If we already have LDR lookup tables
        if self.ldr1_lut is not None and self.ldr2_lut is not None:
            self.ser.send("8Y")
        else:
            self.ser.send("8N")
            self.lo_capture_active = True
            self.lo_capture_deadline = time.time() + 20.0
            self.lo_status.set("Waiting for calibration before sweep…")


    def send_mode(self):
        """Single Send button in mode bar; acts based on active tab/mode."""
        if not self.connected.get():
            messagebox.showwarning("Not connected", "Connect to the MCU first.")
            return

        mode = Mode[self.current_mode.get()]
        self.bin_buf.clear()
        self.ser.flush()

        if mode is Mode.TELEMETER:
            try: deg = int(self.angle_var.get().strip())
            except ValueError:
                messagebox.showerror("Angle", "Enter an integer 0–180"); return
            deg = max(0, min(180, deg))
            self.ser.send(f"2{deg:03d}")

        elif mode is Mode.OBJECTS:
            try: thr = int(self.thresh_var.get().strip())
            except ValueError:
                messagebox.showerror("Threshold", "Enter threshold in cm (1–450)"); return
            thr = max(1, min(450, thr))
            self.ser.send(f"1{thr:03d}")
            self.obj_bin_buf.clear()
            self.obj_angle_count = 0
            self.obj_sweep = [None]*181
            self.obj_objects = []

        elif mode is Mode.LIGHT:
            # For Light, Send behaves same as “LDR Calibration”
            self.on_ldr_calibration()
            return
       
        elif mode is Mode.TEXT:
            path = self.txt_path.get().strip()
            if not path:
                messagebox.showwarning("Text files", "Choose a .txt file first."); return
            if not path.lower().endswith(".txt"):
                messagebox.showwarning("Text files", "Only .txt files are allowed."); return

            try:
                data = Path(path).read_bytes()
            except Exception as e:
                messagebox.showerror("Text files", f"Can't read file:\n{e}"); return

            size = len(data)
            if size == 0:
                messagebox.showerror("Text files", "File is empty."); return
            if size > 2048:  # 4-digit decimal field
                messagebox.showerror("Text files", f"Max size is 2048 bytes. Selected file is {size}."); return

            # Build ASCII header: '5' + n + t + ssss (t=0 for .txt; ssss zero-padded decimal size)
            n = int(self.txt_name_id.get()) % 10
            ssss = f"{size:04d}"
            header = f"5{n}0{ssss}".encode("ascii")

            # Clear buffers, send header
            self.ser.flush()
            # Optional, if you have it: self._drain_queues()
            if self.send_btn: self.send_btn.state(["disabled"])
            self.txt_pbar['value'] = 0
            self.txt_status.set(f"Header sent (5{n}0{ssss}). Waiting for 'A'…")
            self.ser.send_bytes(header)

           # Wait for MCU response:E (save failed), F (no space), A (ready for data)
            deadline = time.time() + 5.0            
            resp = None
            while time.time() < deadline and not resp:
                try:
                    b = self.ser.rx_bytes_q.get(timeout=0.05)
                    if b:
                        if b.find(b'E') != -1:
                            resp = 'E'
                        elif b.find(b'F') != -1:
                            resp = 'F'
                        elif b.find(b'A') != -1:
                            resp = 'A'
                except Exception:
                    pass

            if resp == 'E':
                self.txt_status.set("Save failed (E).")
                if self.send_btn: self.send_btn.state(["!disabled"])
                return
            elif resp == 'F':
                self.txt_status.set("No space to save file (F).")
                if self.send_btn: self.send_btn.state(["!disabled"])
                return
            elif resp != 'A':
                self.txt_status.set("Timeout or unknown response from MCU.")
                if self.send_btn: self.send_btn.state(["!disabled"])
                return

            # If 'A' received, continue to send file data
            self.txt_status.set("Sending file…")
            sent = 0
            CHUNK = 128
            while sent < size:
                end = min(sent + CHUNK, size)
                self.ser.send_bytes(data[sent:end])
                sent = end
                self.txt_pbar['value'] = int((sent / size) * 100)
                self.update_idletasks()
                time.sleep(0.005)

            # Wait for 'K' response from MCU to confirm transfer finished
            deadline = time.time() + 5.0           
            transfer_finished = False
            while time.time() < deadline and not transfer_finished:
                try:
                    b = self.ser.rx_bytes_q.get(timeout=0.05)
                    if b and b.find(b'K') != -1:
                        transfer_finished = True
                        self.txt_status.set("Transfer finished.")
                        break
                except Exception:
                    pass
            if not transfer_finished:
                self.txt_status.set("Timeout waiting for transfer finish (K).")
            if self.send_btn: self.send_btn.state(["!disabled"])
            return
    
    def on_send_7(self):
        if not self.connected.get():
            messagebox.showwarning("Not connected", "Connect to the MCU first.")
            return
        self.ser.send_char('7')
        deadline = time.time() + 5.0
        transfer_finished = False
        while time.time() < deadline and not transfer_finished:
            try:
                b = self.ser.rx_bytes_q.get(timeout=0.05)
                if b and b.find(b'7') != -1:
                    transfer_finished = True
                    self.txt_status.set("file browser and viewer from LCD using PB0 & PB1")
                    break
            except Exception:
                pass
    
    # --- Add LDR interpolation for Light+Objects ---
    def _interp_lo_ldr_distance(self, sample, deg):
        lut = self.lo_ldr1_lut if deg < 90 else self.lo_ldr2_lut
        if not lut: 
            return None
        idx, best_val = min(enumerate(lut), key=lambda kv: abs(sample - kv[1]))
        err = abs(sample - best_val)
        if err > LIGHT_MATCH_TOL:
            return None
        return idx + 1  # cm

    # --- Add calibration graph ---
    def _draw_lo_calibration(self):
        c = self.lo_canvas
        c.delete("all")
        w = int(c.winfo_width()); h = int(c.winfo_height())
        margin = 36
        vals = []
        for arr in (self.lo_ldr1_lut, self.lo_ldr2_lut, self.lo_ldr1_raw, self.lo_ldr2_raw):
            if arr: vals += arr
        if not vals:
            c.create_text(w//2, h//2, text="No calibration yet.", fill="#e6e8ef", font=("Segoe UI", 12))
            return
        vmin = min(vals); vmax = max(vals)
        if vmax == vmin: vmax = vmin + 1
        c.create_rectangle(margin, margin, w - margin, h - margin, outline="#223056")
        for cm in range(5, 51, 5):
            x = margin + (cm-1) * (w - 2*margin) / 49.0
            c.create_line(x, margin, x, h - margin, fill="#132044")
            c.create_text(x, h - margin + 14, text=str(cm), fill="#9fb3d9")
        for t in range(5):
            v = vmin + t*(vmax - vmin)/4.0
            y = h - margin - ((v - vmin) / (vmax - vmin)) * (h - 2*margin)
            c.create_line(margin, y, w - margin, y, fill="#1b2544")
            c.create_text(margin - 10, y, text=str(int(round(v))), fill="#9fb3d9", anchor="e")

        def xy(cm_index_zero_based, val):
            x = margin + cm_index_zero_based * (w - 2*margin) / 49.0
            y = h - margin - ((val - vmin) / (vmax - vmin)) * (h - 2*margin)
            return x, y

        if self.lo_ldr1_lut:
            pts = []
            for i, v in enumerate(self.lo_ldr1_lut):
                x, y = xy(i, v); pts.extend((x, y))
            if len(pts) >= 4: c.create_line(*pts, width=2, fill="#99c5ff")
            c.create_text(margin, margin-12, text="LDR1", fill="#99c5ff", anchor="w")
            if self.lo_ldr1_raw:
                for j, v in enumerate(self.lo_ldr1_raw):
                    cm_index = 5*(j+1) - 1
                    x, y = xy(cm_index, v)
                    c.create_oval(x-3, y-3, x+3, y+3, fill="#ffffff", outline="")

        if self.lo_ldr2_lut:
            pts = []
            for i, v in enumerate(self.lo_ldr2_lut):
                x, y = xy(i, v); pts.extend((x, y))
            if len(pts) >= 4: c.create_line(*pts, width=2, fill="#7ee1a9")
            c.create_text(margin+60, margin-12, text="LDR2", fill="#7ee1a9", anchor="w")
            if self.lo_ldr2_raw:
                for j, v in enumerate(self.lo_ldr2_raw):
                    cm_index = 5*(j+1) - 1
                    x, y = xy(cm_index, v)
                    c.create_rectangle(x-3, y-3, x+3, y+3, outline="#cfead9")

        c.create_text(w - margin, margin - 12, text="Lookup: index k → (k+1) cm",  fill="#e6e8ef", anchor="e", font=("Segoe UI", 10))

    # --- Finalize and draw combined results ---
    def _finalize_lo_sweep(self):
        # Light sources (same as _finalize_light_sweep)
        runs, i = [], 0
        while i <= 180:
            if self.lo_light_sweep[i] is None:
                i += 1; continue
            j = i
            buf = []
            while j <= 180 and self.lo_light_sweep[j] is not None:
                buf.append((j, self.lo_light_sweep[j]))
                j += 1
            if len(buf) >= LIGHT_MIN_RUN:
                runs.append(buf)
            i = j

        merged = []
        for r in runs:
            if not merged:
                merged.append(r); continue
            last = merged[-1]
            gap = r[0][0] - last[-1][0]
            if gap <= LIGHT_GAP_TOL:
                d1 = sum(v for _, v in last)/len(last)
                d2 = sum(v for _, v in r)/len(r)
                if abs(d1 - d2) <= 2.0:
                    merged[-1] = last + r
                    continue
            merged.append(r)

        sources = []
        for vals in merged:
            degs  = [d for d, _ in vals]
            dists = [r for _, r in vals]
            phi = int(round(sum(degs)/len(degs)))
            rho = int(sorted(dists)[len(dists)//2])
            rho = max(1, min(LIGHT_MAX_CM, rho))
            sources.append({'rho_cm': rho, 'phi_deg': phi})
        self.lo_sources = sources

        # Objects (same as _finalize_objects_sweep)
        groups = []
        i = 0
        while i <= 180:
            if self.lo_echo_sweep[i] is None: i += 1; continue
            j, vals = i, []
            while j <= 180 and self.lo_echo_sweep[j] is not None:
                vals.append((j, self.lo_echo_sweep[j])); j += 1
            if vals: groups.append(vals)
            i = j

        objects = []
        for vals in groups:
            degs  = [d for d, _ in vals]
            dists = [r for _, r in vals]
            if len(degs) >= 5: degs, dists = degs[1:-1], dists[1:-1]
            phi = int(round(sum(degs) / len(degs)))
            rho = float(sum(dists) / len(dists))
            rho_cm = int(round(rho))
            step_deg = (degs[1] - degs[0]) if len(degs) > 1 else 1.0
            dtheta = math.radians((degs[-1] - degs[0]) + step_deg)
            k = min(2, len(dists))
            r1 = sum(dists[:k]) / k; r2 = sum(dists[-k:]) / k
            l_loc  = math.sqrt(max(0.0, r1*r1 + r2*r2 - 2.0*r1*r2*math.cos(dtheta)))
            l_cm = int(round(l_loc))
            l_cm = round(l_cm-2*0.16*dists[0])    
          #  rho_med = sorted(dists)[len(dists)//2]
            #l_iso  = 2.0 * rho_med * math.sin(dtheta / 2.0)
        
            if l_cm>50:
                l_cm = int(round(l_cm * 0.72))
            if l_cm >= OBJ_MIN_LEN_CM:
                objects.append({'rho_cm': rho_cm, 'phi_deg': phi, 'l_cm': l_cm})
        self.lo_objects = objects
        self.lo_light_sweep = [None]*181
        self.lo_echo_sweep = [None]*181

    def _draw_lo_view(self):
        c = self.lo_canvas
        c.delete("all")
        w, h = int(c.winfo_width()), int(c.winfo_height())
        cx, cy = w // 2, h - 24
        try: thr = max(1, int(self.thresh_var.get()))
        except Exception: thr = 450
        R = min(w // 2 - 32, h - 40)
        if R <= 0: return

        c.create_line(12, cy, w - 12, cy, fill="#bfc7da")
        c.create_arc(cx - R, cy - R, cx + R, cy + R, start=0, extent=180,
                    style="arc", outline="#bfc7da", width=2)
        c.create_text(20, cy + 12, text="0 deg",   fill="#e6e8ef", anchor="w", font=("Segoe UI", 10, "bold"))
        c.create_text(w-20, cy + 12, text="180 deg", fill="#e6e8ef", anchor="e", font=("Segoe UI", 10, "bold"))

        # Draw objects
        for obj in self.lo_objects:
            rho = obj['rho_cm']; phi = obj['phi_deg']; lcm = obj['l_cm']
            r_norm = min(max(rho, 0.0), thr) / thr
            rr = r_norm * R
            theta = math.radians(180 - phi)
            x = cx + rr * math.cos(theta); y = cy - rr * math.sin(theta)
            c.create_oval(x-5, y-5, x+5, y+5, fill="#4da3ff", outline="")
            c.create_text(x, y-16, text=f"({rho}cm, {phi}deg, {lcm}cm)",
                        fill="#e6e8ef", font=("Segoe UI", 9))

        # Draw light sources
        for src in self.lo_sources:
            rho = src['rho_cm']; phi = src['phi_deg']
            rr = (rho / float(LIGHT_MAX_CM)) * R
            theta = math.radians(180 - phi)
            x = cx + rr*math.cos(theta); y = cy - rr*math.sin(theta)
            c.create_oval(x-4, y-4, x+4, y+4, fill="#ffff66", outline="")
            c.create_text(x, y-14, text=f"({rho}cm, {phi}deg)", fill="#fffaa0", font=("Segoe UI", 9))


    def _txt_browse(self):
        path = filedialog.askopenfilename( title="Choose a .txt file", filetypes=[("Text files","*.txt")])
        if path:
            self.txt_path.set(path)
            self.txt_status.set("Ready to send.")
    
    def on_erase_flash(self):
        if not self.connected.get():
            messagebox.showwarning("Not connected", "Connect to the MCU first.")
            return
        self.ser.flush()
        self.txt_status.set("Erasing Flash…")
        self.ser.send_char('6')
        # Wait for ack
        deadline = time.time() + 5.0
        ack = False
        while time.time() < deadline and not ack:
            try:
                b = self.ser.rx_bytes_q.get(timeout=0.05)
                if b and b.find(b'6') != -1:
                    ack = True
                    self.txt_status.set("All Flash data erased!")
                    break
            except Exception:
                pass
        if not ack:
            self.txt_status.set("Timeout waiting for erase confirmation.")

    # ------------- Serial polling & UI update -------------
    def _poll_serial(self):
        # ASCII fallback lines
        for line in self._drain_lines_iter():
            if line.startswith("__ERR__"):
                self.status_text.set(line.replace("__ERR__", "Serial")); continue
            cur = self.current_mode.get()
            echo_us = extract_echo_us_ascii(line, ticks_hz=None)
            if echo_us is None: continue
            if cur == Mode.TELEMETER.name:
                self._update_telemeter(echo_us)
            elif cur == Mode.OBJECTS.name:
                self._update_objects_live(echo_us)

        # Raw bytes by active GUI mode only
        cur = self.current_mode.get()

        if cur == Mode.TELEMETER.name:
            self._drain_bytes(self.bin_buf)
            while len(self.bin_buf) >= 2:
                width = int.from_bytes(self.bin_buf[:2], 'little', signed=False)
                del self.bin_buf[:2]
                self._update_telemeter(width * (1_000_000.0 / TA1_TICKS_PER_SECOND))

        elif cur == Mode.OBJECTS.name:
            self._drain_bytes(self.obj_bin_buf)
            while len(self.obj_bin_buf) >= 2:
                width = int.from_bytes(self.obj_bin_buf[:2], 'little', signed=False)
                del self.obj_bin_buf[:2]
                self._update_objects_live(width * (1_000_000.0 / TA1_TICKS_PER_SECOND))

        elif cur == Mode.LIGHT.name:
            # Always drain to the correct buffer (no junk sink!)
            if self.ldr_capture_active:
                self._drain_bytes(self.ldr_buf)          # collecting the 40-byte cal frame
            else:
                self._drain_bytes(self.light_bin_buf)     # collecting the 2B/deg sweep

            # --- calibration frame handling (40 bytes total) ---
            if self.ldr_capture_active and time.time() > self.ldr_capture_deadline and len(self.ldr_buf) < 40:
                n = len(self.ldr_buf)
                self.ldr_status.set(f"Timeout: received only {n}/40 bytes.")
                self.ldr_capture_active = False

            if self.ldr_capture_active and len(self.ldr_buf) >= 40:
                frame = self.ldr_buf[:40]; self.ldr_buf = self.ldr_buf[40:]
                ldr1_u16 = [int.from_bytes(frame[i:i+2], 'little') for i in range(0, 20, 2)]
                ldr2_u16 = [int.from_bytes(frame[i:i+2], 'little') for i in range(20, 40, 2)]
                self.ldr1_raw, self.ldr2_raw = ldr1_u16, ldr2_u16
                self.ldr1_lut = self._expand_10_at_5cm_to_50(ldr1_u16)
                self.ldr2_lut = self._expand_10_at_5cm_to_50(ldr2_u16)
                self.ldr_status.set("Calibration received ✓ (2×50 samples)")
                self.ldr_capture_active = False

                # print LUTs as requested
                print("LDR1_50:", self.ldr1_lut)
                print("LDR2_50:", self.ldr2_lut)

                self._draw_ldr_view()  # show/refresh calibration graph immediately

            # --- sweep handling: 2 bytes per degree, 0..180 ---
            while (not self.ldr_capture_active) and len(self.light_bin_buf) >= 2:
                val = int.from_bytes(self.light_bin_buf[:2], 'little', signed=False)
                del self.light_bin_buf[:2]

                deg = self.light_angle_count
                if deg <= 180:
                    dist_cm = self._interp_ldr_distance(val, deg)
                    self.light_sweep[deg] = dist_cm
                    self.light_deg_var.set(deg)  # live angle
                           
                    print(f"[Light] deg={deg} sample={val} -> dist={dist_cm}")
                    self.light_angle_count += 1

                    if self.light_angle_count > 180:
                        self._finalize_light_sweep()
                        self.light_angle_count = 0
                        self.light_deg_var.set(0)
                        print("LIGHT_SOURCES:", self.light_sources)
                else:
                    # safety resync if somehow out of bounds
                    self.light_angle_count = 0
                    self.light_sweep = [None]*181

        elif cur == Mode.LIGHT_AND_OBJECTS.name:
            self._drain_bytes(self.lo_bin_buf)
            # --- calibration frame handling (40 bytes total) ---
            if self.lo_capture_active and time.time() > self.lo_capture_deadline and len(self.lo_bin_buf) < 40:
                n = len(self.lo_bin_buf)
                self.lo_status.set(f"Timeout: received only {n}/40 bytes.")
                self.lo_capture_active = False

            if self.lo_capture_active and len(self.lo_bin_buf) >= 40:
                frame = self.lo_bin_buf[:40]; self.lo_bin_buf = self.lo_bin_buf[40:]
                ldr1_u16 = [int.from_bytes(frame[i:i+2], 'little') for i in range(0, 20, 2)]
                ldr2_u16 = [int.from_bytes(frame[i:i+2], 'little') for i in range(20, 40, 2)]
                self.lo_ldr1_raw, self.lo_ldr2_raw = ldr1_u16, ldr2_u16
                self.lo_ldr1_lut = self._expand_10_at_5cm_to_50(ldr1_u16)
                self.lo_ldr2_lut = self._expand_10_at_5cm_to_50(ldr2_u16)
                self.lo_status.set("Calibration received ✓ (2×50 samples)")
                self.lo_capture_active = False
                print("LO_LDR1_50:", self.lo_ldr1_lut)
                print("LO_LDR2_50:", self.lo_ldr2_lut)
                self._draw_lo_calibration()

            # --- sweep handling: 4 bytes per degree, 0..180 ---
            try: thr = int(self.thresh_var.get())
            except Exception: thr = 0
            while (not self.lo_capture_active) and len(self.lo_bin_buf) >= 4:
                ldr_val = int.from_bytes(self.lo_bin_buf[:2], 'little', signed=False)
                echo_val = int.from_bytes(self.lo_bin_buf[2:4], 'little', signed=False)
                del self.lo_bin_buf[:4]

                deg = self.lo_angle_count
                if deg <= 180:
                    light_cm = self._interp_lo_ldr_distance(ldr_val, deg)
                    dist_cm = echo_us_to_cm(echo_val * (1_000_000.0 / TA1_TICKS_PER_SECOND))
                    self.lo_light_sweep[deg] = light_cm
                    self.lo_echo_sweep[deg] = dist_cm if (thr > 0 and 0.0 < dist_cm <= thr) else None
                    print(f"[Light+Objects] deg={deg} LDR={ldr_val} -> light_cm={light_cm}, echo={echo_val} -> dist_cm={dist_cm:.2f} sweep={self.lo_echo_sweep[deg]}")
                    self.lo_angle_count += 1

                    if self.lo_angle_count > 180:
                        self._finalize_lo_sweep()
                        self.lo_angle_count = 0
                else:
                    self.lo_angle_count = 0
                    self.lo_light_sweep = [None]*181
                    self.lo_echo_sweep = [None]*181


        # Draw active view only
        if cur == Mode.TELEMETER.name:
            self._draw_chart()
        elif cur == Mode.OBJECTS.name:
            self._draw_objects()
        elif cur == Mode.LIGHT.name:
            if self.light_show_sources and self.light_sources:
                self._draw_light_sources()
            else:
                self._draw_ldr_view()
        elif cur == Mode.LIGHT_AND_OBJECTS.name:                
            self._draw_lo_view()

        self.after(50, self._poll_serial)



    def _update_telemeter(self, echo_us: float):
        dist_cm = echo_us_to_cm(echo_us)
        self.distance_cm_last.set(round(dist_cm, 2))
        self.dist_hist.append(dist_cm)

    def _draw_chart(self):
        c = self.canvas
        c.delete("all")
        w = int(c.winfo_width()); h = int(c.winfo_height())
        c.create_rectangle(8, 8, w - 8, h - 8, outline="#223056")
        for frac in (0.25, 0.5, 0.75):
            y = int(h - 8 - (h - 16) * frac)
            c.create_line(10, y, w - 10, y, fill="#1b2544")
        for x in range(10, w - 10, 60):
            c.create_line(x, 10, x, h - 10, fill="#132044")
        if not self.dist_hist: return
        vals = list(self.dist_hist); n = len(vals)
        max_v = max(50.0, max(vals)); min_v = 0.0
        def xy(i, v):
            X = int(i * (w - 20) / max(1, 119)) + 10
            Y = int(h - 20 - (v - min_v) / (max_v - min_v) * (h - 40))
            return X, Y
        pts = []
        for i, v in enumerate(vals[-120:]): pts.extend(xy(i, v))
        if len(pts) >= 4: c.create_line(*pts, width=2, fill="#99c5ff")
        x, y = xy(n - 1, vals[-1])
        c.create_oval(x - 3, y - 3, x + 3, y + 3, fill="#ffffff", outline="")
        c.create_text(x, 12, text=f"{vals[-1]:.1f} cm", fill="#e6e8ef", font=("Segoe UI", 10))

    def _update_objects_live(self, echo_us: float):
        dist_cm = echo_us_to_cm(echo_us)
        self.obj_distance_cm.set(round(dist_cm, 2))
        deg = max(0, min(180, self.obj_angle_count))
        self.obj_angle_deg.set(deg)
        try: thr = int(self.thresh_var.get())
        except Exception: thr = 0
        self.obj_sweep[deg] = dist_cm if (thr > 0 and 0.0 < dist_cm <= thr) else None
        print(f"[Objects] deg={deg} dist_cm={dist_cm:.2f} sweep={self.obj_sweep[deg]}")
        self.obj_angle_count += 1
        if self.obj_angle_count > 180:
            self._finalize_objects_sweep(); self.obj_angle_count = 0

    def _finalize_objects_sweep(self):
        groups = []
        i = 0
        while i <= 180:
            if self.obj_sweep[i] is None: i += 1; continue
            j, vals = i, []
            while j <= 180 and self.obj_sweep[j] is not None:
                vals.append((j, self.obj_sweep[j])); j += 1
            if vals: groups.append(vals)
            i = j

        objects = []
        for vals in groups:
            degs  = [d for d, _ in vals]
            dists = [r for _, r in vals]
            if len(degs) >= 5: degs, dists = degs[1:-1], dists[1:-1]
            phi = int(round(sum(degs) / len(degs)))
            rho = float(sum(dists) / len(dists))
            rho_cm = int(round(rho))
            step_deg = (degs[1] - degs[0]) if len(degs) > 1 else 1.0
            dtheta = math.radians((degs[-1] - degs[0]) + step_deg)
            k = min(2, len(dists))
            r1 = sum(dists[:k]) / k; r2 = sum(dists[-k:]) / k
            l_loc  = math.sqrt(max(0.0, r1*r1 + r2*r2 - 2.0*r1*r2*math.cos(dtheta)))

            l_cm = int(round(l_loc))
            l_cm = round(l_cm-2*0.15*dists[0])
            if l_cm>50:
                l_cm = int(round(l_cm * 0.72))

            if l_cm >= OBJ_MIN_LEN_CM:
                objects.append({'rho_cm': rho_cm, 'phi_deg': phi, 'l_cm': l_cm})

        self.obj_objects = objects
        self.obj_sweep = [None]*181

    def _draw_objects(self):
        c = self.obj_canvas
        c.delete("all")
        w = int(c.winfo_width()); h = int(c.winfo_height())
        cx, cy = w // 2, h - 24
        R = min(w // 2 - 32, h - 40)
        if R <= 0: return
        c.create_line(12, cy, w - 12, cy, fill="#bfc7da")
        c.create_arc(cx - R, cy - R, cx + R, cy + R, start=0, extent=180,
                     style="arc", outline="#bfc7da", width=2)
        c.create_text(20, cy + 12, text="0 deg",   fill="#e6e8ef", anchor="w", font=("Segoe UI", 10, "bold"))
        c.create_text(w-20, cy + 12, text="180 deg", fill="#e6e8ef", anchor="e", font=("Segoe UI", 10, "bold"))
        try: thr = max(1, int(self.thresh_var.get()))
        except Exception: thr = 450
        for obj in self.obj_objects:
            rho = obj['rho_cm']; phi = obj['phi_deg']; lcm = obj['l_cm']
            r_norm = min(max(rho, 0.0), thr) / thr
            rr = r_norm * R
            theta = math.radians(180 - phi)
            x = cx + rr * math.cos(theta); y = cy - rr * math.sin(theta)
            c.create_oval(x-5, y-5, x+5, y+5, fill="#4da3ff", outline="")
            c.create_text(x, y-16, text=f"({rho}cm, {phi}deg, {lcm}cm)",
                          fill="#e6e8ef", font=("Segoe UI", 9))

    # ---- LDR plotting ----
    # def _draw_ldr_view(self):
    #     c = self.ldr_canvas
    #     c.delete("all")
    #     w = int(c.winfo_width()); h = int(c.winfo_height())
    #     margin = 36
    #     vals = []
    #     for arr in (self.ldr1_lut, self.ldr2_lut, self.ldr1_raw, self.ldr2_raw):
    #         if arr: vals += arr
    #     if not vals:
    #         c.create_text(w//2, h//2, text="No calibration yet.", fill="#e6e8ef", font=("Segoe UI", 12))
    #         return
    #     vmin = min(vals); vmax = max(vals)
    #     if vmax == vmin: vmax = vmin + 1
    #     c.create_rectangle(margin, margin, w - margin, h - margin, outline="#223056")
    #     for cm in range(5, 51, 5):
    #         x = margin + (cm-1) * (w - 2*margin) / 49.0
    #         c.create_line(x, margin, x, h - margin, fill="#132044")
    #         c.create_text(x, h - margin + 14, text=str(cm), fill="#9fb3d9")
    #     for t in range(5):
    #         v = vmin + t*(vmax - vmin)/4.0
    #         y = h - margin - ((v - vmin) / (vmax - vmin)) * (h - 2*margin)
    #         c.create_line(margin, y, w - margin, y, fill="#1b2544")
    #         c.create_text(margin - 10, y, text=str(int(round(v))), fill="#9fb3d9", anchor="e")

    #     def xy(cm_index_zero_based, val):
    #             x = margin + cm_index_zero_based * (w - 2*margin) / 49.0
    #             y = h - margin - ((val - vmin) / (vmax - vmin)) * (h - 2*margin)
    #             return x, y

    #     if self.ldr1_lut:
    #         pts = []
    #         for i, v in enumerate(self.ldr1_lut):
    #             x, y = xy(i, v); pts.extend((x, y))
    #         if len(pts) >= 4: c.create_line(*pts, width=2, fill="#99c5ff")
    #         c.create_text(margin, margin-12, text="LDR1", fill="#99c5ff", anchor="w")
    #         if self.ldr1_raw:
    #             for j, v in enumerate(self.ldr1_raw):
    #                 cm_index = 5*(j+1) - 1
    #                 x, y = xy(cm_index, v)
    #                 c.create_oval(x-3, y-3, x+3, y+3, fill="#ffffff", outline="")

    #     if self.ldr2_lut:
    #         pts = []
    #         for i, v in enumerate(self.ldr2_lut):
    #             x, y = xy(i, v); pts.extend((x, y))
    #         if len(pts) >= 4: c.create_line(*pts, width=2, fill="#7ee1a9")
    #         c.create_text(margin+60, margin-12, text="LDR2", fill="#7ee1a9", anchor="w")
    #         if self.ldr2_raw:
    #             for j, v in enumerate(self.ldr2_raw):
    #                 cm_index = 5*(j+1) - 1
    #                 x, y = xy(cm_index, v)
    #                 c.create_rectangle(x-3, y-3, x+3, y+3, outline="#cfead9")

    #     c.create_text(w - margin, margin - 12, text="Lookup: index k → (k+1) cm",  fill="#e6e8ef", anchor="e", font=("Segoe UI", 10))

    def _draw_ldr_view(self):
        c = self.ldr_canvas
        c.delete("all")
        w = int(c.winfo_width()); h = int(c.winfo_height())
        margin = 36
        vals = []
        for arr in (self.ldr1_lut, self.ldr2_lut, self.ldr1_raw, self.ldr2_raw):
            if arr: vals += arr
        if not vals:
            c.create_text(w//2, h//2, text="No calibration yet.", fill="#e6e8ef", font=("Segoe UI", 12))
            return
        vmin = min(vals); vmax = max(vals)
        if vmax == vmin: vmax = vmin + 1
        c.create_rectangle(margin, margin, w - margin, h - margin, outline="#223056")
        for cm in range(5, 51, 5):
            x = margin + (cm-1) * (w - 2*margin) / 49.0
            c.create_line(x, margin, x, h - margin, fill="#132044")
            c.create_text(x, h - margin + 14, text=str(cm), fill="#9fb3d9")
        for t in range(5):
            v = vmin + t*(vmax - vmin)/4.0
            y = h - margin - ((v - vmin) / (vmax - vmin)) * (h - 2*margin)
            c.create_line(margin, y, w - margin, y, fill="#1b2544")
            c.create_text(margin - 10, y, text=str(int(round(v))), fill="#9fb3d9", anchor="e")

        def xy(cm_index_zero_based, val):
            x = margin + cm_index_zero_based * (w - 2*margin) / 49.0
            y = h - margin - ((val - vmin) / (vmax - vmin)) * (h - 2*margin)
            return x, y

        if self.ldr1_lut:
            pts = []
            for i, v in enumerate(self.ldr1_lut):
                x, y = xy(i, v); pts.extend((x, y))
            if len(pts) >= 4: c.create_line(*pts, width=2, fill="#99c5ff")
            c.create_text(margin, margin-12, text="LDR1", fill="#99c5ff", anchor="w")
            if self.ldr1_raw:
                for j, v in enumerate(self.ldr1_raw):
                    cm_index = 5*(j+1) - 1
                    x, y = xy(cm_index, v)
                    c.create_oval(x-3, y-3, x+3, y+3, fill="#ffffff", outline="")

        if self.ldr2_lut:
            pts = []
            for i, v in enumerate(self.ldr2_lut):
                x, y = xy(i, v); pts.extend((x, y))
            if len(pts) >= 4: c.create_line(*pts, width=2, fill="#7ee1a9")
            c.create_text(margin+60, margin-12, text="LDR2", fill="#7ee1a9", anchor="w")
            if self.ldr2_raw:
                for j, v in enumerate(self.ldr2_raw):
                    cm_index = 5*(j+1) - 1
                    x, y = xy(cm_index, v)
                    c.create_rectangle(x-3, y-3, x+3, y+3, outline="#cfead9")

        c.create_text(w - margin, margin - 12, text="Lookup: index k → (k+1) cm",  fill="#e6e8ef", anchor="e", font=("Segoe UI", 10))

    def on_reset_all(self):
        self.bin_buf.clear(); self.dist_hist.clear(); self.distance_cm_last.set(0.0)
        self.obj_bin_buf.clear(); self.obj_angle_count = 0; self.obj_sweep = [None]*181; self.obj_objects = []
        self.obj_distance_cm.set(0.0); self.obj_angle_deg.set(0)
        self.ldr_buf.clear(); self.ldr1_raw = self.ldr2_raw = None; self.ldr1_lut = self.ldr2_lut = None
        self.ldr_capture_active = False; self.ldr_capture_deadline = 0.0
        self.ldr_status.set("Press LDR Calibration to fetch 40 bytes.")
        self.light_deg_var.set(0)
        self.light_show_sources = False

        for canv in (self.canvas, self.obj_canvas, self.ldr_canvas, getattr(self, 'lo_canvas', None)):
                
            (canv.delete("all") if canv else None)

    # ---- interpolation: 10 anchors at 5..50 cm -> 50 values ----
    @staticmethod
    def _expand_10_at_5cm_to_50(samples10):
        if not samples10 or len(samples10) < 2: return None
        if len(samples10) != 10:  # fallback if unexpected count
            return App._expand_N_to_50(samples10)
        anchors_idx = [5*(j+1)-1 for j in range(10)]  # 4,9,...,49
        out = [0]*50
        for k in range(0, anchors_idx[0]+1): out[k] = samples10[0]
        for k in range(anchors_idx[-1], 50): out[k] = samples10[-1]
        for j in range(9):
            x0, y0 = anchors_idx[j],   samples10[j]
            x1, y1 = anchors_idx[j+1], samples10[j+1]
            span = x1 - x0
            for k in range(x0, x1+1):
                f = 0 if span == 0 else (k - x0)/span
                out[k] = int(round((1.0 - f)*y0 + f*y1))
        return out

    @staticmethod
    def _expand_N_to_50(sN):
        if not sN or len(sN) < 2: return None
        N = len(sN); out = []
        for k in range(50):
            pos = k * ((N - 1) / 49.0)
            j = int(pos)
            if j >= N - 1: val = sN[-1]
            else:
                f = pos - j
                val = (1.0 - f) * sN[j] + f * sN[j + 1]
            out.append(int(round(val)))
        return out

    def _drain_queues(self):
        try:
            while True: self.ser.rx_bytes_q.get_nowait()
        except queue.Empty: pass
        try:
            while True: self.ser.rx_q.get_nowait()
        except queue.Empty: pass

    # ---- Light: capture 40 bytes fresh every time ----
    def on_ldr_calibration(self):
        if not self.connected.get():
            messagebox.showwarning("Not connected", "Connect to the MCU first.")
            return
        # ensure we're on the Light tab and bar reflects it
        self.nb.select(self.page_light)
        self._on_tab_change()

        # hard reset capture buffers & old arrays
        if self.ser.ser: self.ser.flush()
        self._drain_queues()
        self.ldr_buf.clear()
        self.ldr1_raw = self.ldr2_raw = None
        self.ldr1_lut = self.ldr2_lut = None
        self.ldr_capture_active = True
        self.ldr_capture_deadline = time.time() + 20.0  # 20s window
        self.ldr_status.set("Request sent… waiting (≤20s) for 40 bytes from MCU.")
        self.light_show_sources = False

        # always send a fresh '3'
        self.ser.send_char('3')

    def on_show_ldr_arrays(self):
        win = tk.Toplevel(self)
        win.title("LDR 50-sample arrays")
        win.geometry("760x360")
        txt = tk.Text(win, wrap="word", font=("Consolas", 10))
        txt.pack(fill="both", expand=True)
        def fmt(arr): return "None" if arr is None else "[" + ", ".join(str(v) for v in arr) + "]"
        txt.insert("end", "LDR1_50:\n" + fmt(self.ldr1_lut) + "\n\n")
        txt.insert("end", "LDR2_50:\n" + fmt(self.ldr2_lut) + "\n")
        txt.configure(state="disabled")

    def on_light_detector(self):
        if not self.connected.get():
            messagebox.showwarning("Not connected", "Connect to the MCU first.")
            return

        if self.ser.ser:
            self.ser.flush()
            self._drain_queues()
        self._start_light_sweep()
        self.ldr_buf.clear()

        # If we already have LDR lookup tables
        if self.ldr1_lut is not None and self.ldr2_lut is not None:
            self.ser.send("4Y")
        else:
            self.ser.send("4N")
            # MCU will send calibration first (40 bytes)
            self.ldr_capture_active = True
            self.ldr_capture_deadline = time.time() + 20.0
            self.ldr_status.set("Waiting for calibration before sweep…")
            


    def _start_light_sweep(self):
        self.light_bin_buf.clear()
        self.light_angle_count = 0
        self.light_sweep = [None]*181
        self.light_sources = []
        self.light_deg_var.set(0)
        self.light_show_sources = True   # switch Light tab to “sources” view

        
    def _interp_ldr_distance(self, sample, deg):
        """
        Return distance in cm (1..50) if the ADC sample matches the LUT within a
        tolerance; otherwise return None to mark 'no light here'.
        """
        lut = self.ldr1_lut if deg < 90 else self.ldr2_lut
        if not lut: 
            return None
        # find nearest LUT value
        idx, best_val = min(enumerate(lut), key=lambda kv: abs(sample - kv[1]))
        err = abs(sample - best_val)
        if err > LIGHT_MATCH_TOL:
            return None
        return idx + 1  # cm


    def _finalize_light_sweep(self):
        # group consecutive degrees that produced a valid distance
        runs, i = [], 0
        while i <= 180:
            if self.light_sweep[i] is None:
                i += 1; continue
            j = i
            buf = []
            while j <= 180 and self.light_sweep[j] is not None:
                buf.append((j, self.light_sweep[j]))
                j += 1
            if len(buf) >= LIGHT_MIN_RUN:
                runs.append(buf)
            i = j

        # merge near-by runs with similar distance
        merged = []
        for r in runs:
            if not merged:
                merged.append(r); continue
            last = merged[-1]
            gap = r[0][0] - last[-1][0]
            if gap <= LIGHT_GAP_TOL:
                d1 = sum(v for _, v in last)/len(last)
                d2 = sum(v for _, v in r)/len(r)
                if abs(d1 - d2) <= 2.0:  # cm
                    merged[-1] = last + r
                    continue
            merged.append(r)

        # make one source per run (median distance, mean angle)
        sources = []
        for vals in merged:
            degs  = [d for d, _ in vals]
            dists = [r for _, r in vals]
            phi = int(round(sum(degs)/len(degs)))
            rho = int(sorted(dists)[len(dists)//2])   # median cm
            # clamp to calibration range
            rho = max(1, min(LIGHT_MAX_CM, rho))
            sources.append({'rho_cm': rho, 'phi_deg': phi})

        self.light_sources = sources
        self.light_sweep = [None]*181
        # keep showing sources view
        self.light_show_sources = True


    def _draw_light_sources(self):
        c = self.ldr_canvas
        c.delete("all")
        w, h = int(c.winfo_width()), int(c.winfo_height())
        cx, cy = w // 2, h - 24
        R = min(w // 2 - 32, h - 40)
        if R <= 0: return

        # frame + degree labels
        c.create_line(12, cy, w - 12, cy, fill="#bfc7da")
        c.create_arc(cx - R, cy - R, cx + R, cy + R, start=0, extent=180,
                    style="arc", outline="#bfc7da", width=2)
        c.create_text(20, cy + 12, text="0 deg", fill="#e6e8ef", anchor="w", font=("Segoe UI", 10, "bold"))
        c.create_text(w-20, cy + 12, text="180 deg", fill="#e6e8ef", anchor="e", font=("Segoe UI", 10, "bold"))

        # sources
        for src in self.light_sources:
            rho = src['rho_cm']; phi = src['phi_deg']
            rr = (rho / float(LIGHT_MAX_CM)) * R   # 1..50 cm
            theta = math.radians(180 - phi)
            x = cx + rr*math.cos(theta); y = cy - rr*math.sin(theta)
            c.create_oval(x-4, y-4, x+4, y+4, fill="#ffff66", outline="")
            c.create_text(x, y-14, text=f"({rho}cm, {phi}deg)", fill="#fffaa0", font=("Segoe UI", 9))

    
if __name__ == "__main__":
    App().mainloop()


