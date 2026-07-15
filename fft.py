#!/usr/bin/env python3
# =============================================================================
#  fft.py  —  Intel-host side of the LattePanda 50 Hz acquisition system
#
#  Receives framed binary ADC blocks from the Arduino Leonardo over USB-CDC,
#  validates them, then runs the DSP pipeline and shows a live display:
#
#     serial ->  parse/validate/CRC  ->  reassemble N-sample block
#            ->  1) DC-offset removal
#                2) Blackman-Harris window
#                3) rFFT
#                4) features: 50 Hz fundamental (mag/phase/freq), harmonics 2..5,
#                             THD, true RMS, fundamental RMS
#                5) (post-feature) harmonic-domain clean + IFFT reconstruction,
#                             plus moving-average and Savitzky-Golay traces
#                6) live time-domain + spectrum + numeric readout
#
#  Matches lattapanda_adc.ino packet format exactly (little-endian,
#  CRC-16/CCITT-FALSE).  Python 3.8+.
# =============================================================================

# ---- dependency bootstrap (auto-install if missing) ------------------------
import importlib, subprocess, sys
for _pkg, _imp in [("numpy", "numpy"), ("scipy", "scipy"),
                   ("pyserial", "serial"), ("matplotlib", "matplotlib")]:
    try:
        importlib.import_module(_imp)
    except ImportError:
        subprocess.check_call([sys.executable, "-m", "pip", "install", _pkg])

import struct, threading, time, argparse, collections
import numpy as np
from scipy.signal.windows import blackmanharris
from scipy.signal import savgol_filter
import serial
import serial.tools.list_ports
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============================ CONFIG ========================================
FS_ACTUAL   = 16_000_000 / 1667      # 9598.08 Hz — MUST match the Arduino OCR1A
N_FFT       = 2048                   # block size for analysis (~213 ms window)
LINE_HZ     = 50.0                   # nominal fundamental
N_HARM      = 5                      # track harmonics 1..5 (50..250 Hz)
ADC_VREF    = 3.3                    # AREF volts (EXTERNAL 3.3 V) -> codes/1023
ADC_MAX     = 1023.0                 # 10-bit full scale
FE_GAIN     = 1.0                    # front-end volts/volt (1.0 => display ADC volts)
SPP         = 128                    # samples per packet (matches sketch)
SYNC        = b"\xAA\x55"
HDR_FMT     = "<HHBI"                # seq, count, status, timestamp (after sync)
HDR_LEN     = struct.calcsize(HDR_FMT)          # 9
PKT_LEN     = 2 + HDR_LEN + 2 * SPP + 2         # full packet length (269)

# ============================ CRC-16 ========================================
def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE — identical to the Arduino implementation."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc & 0xFFFF

# ======================= THREAD-SAFE SAMPLE STORE ===========================
class SampleStore:
    """Latest-N sample ring shared between the reader thread and the GUI."""
    def __init__(self, n):
        self.n = n
        self.buf = np.zeros(n, dtype=np.float64)
        self.lock = threading.Lock()
        self.dropped = 0            # packets lost (seq gaps)
        self.bad_crc = 0            # rejected packets
        self.overflow = 0           # Arduino ring overflows reported
        self.last_ts = 0

    def push(self, samples):
        with self.lock:
            k = len(samples)
            if k >= self.n:
                self.buf[:] = samples[-self.n:]
            else:
                self.buf[:-k] = self.buf[k:]     # shift out oldest
                self.buf[-k:] = samples          # append newest

    def snapshot(self):
        with self.lock:
            return self.buf.copy(), self.dropped, self.bad_crc, self.overflow, self.last_ts

# ========================= SERIAL READER THREAD =============================
class SerialReader(threading.Thread):
    """Reads bytes, resynchronises on SYNC, validates CRC, emits sample blocks.
    Dropped packets are detected via the sequence number and handled gracefully
    (the missing span is simply skipped; the display keeps running)."""
    def __init__(self, port, store):
        super().__init__(daemon=True)
        self.store = store
        self.running = True
        self.last_seq = None
        self.ser = serial.Serial(port, baudrate=1_000_000, timeout=0.05)
        time.sleep(0.2)
        self.ser.reset_input_buffer()

    def run(self):
        buf = bytearray()
        while self.running:
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException:
                # transient USB error: back off and retry, keep the app alive
                time.sleep(0.1)
                continue
            if chunk:
                buf.extend(chunk)
            # extract every complete, valid packet currently in buf
            while True:
                i = buf.find(SYNC)
                if i < 0:
                    del buf[:-1]                 # keep last byte (partial sync)
                    break
                if len(buf) - i < PKT_LEN:
                    if i:
                        del buf[:i]              # drop junk before the sync
                    break
                pkt = bytes(buf[i:i + PKT_LEN])
                if self._consume(pkt):
                    del buf[:i + PKT_LEN]        # good packet -> advance past it
                else:
                    del buf[:i + 1]              # bad -> skip one byte, rescan
            time.sleep(0.001)

    def _consume(self, pkt) -> bool:
        seq, count, status, ts = struct.unpack_from(HDR_FMT, pkt, 2)
        if count != SPP:
            return False
        crc_rx, = struct.unpack_from("<H", pkt, 2 + HDR_LEN + 2 * SPP)
        if crc16_ccitt(pkt[2:2 + HDR_LEN + 2 * SPP]) != crc_rx:
            self.store.bad_crc += 1
            return False
        # valid packet -> decode samples
        samples = np.frombuffer(pkt, dtype="<u2", count=SPP,
                                offset=2 + HDR_LEN).astype(np.float64)
        # drop detection via sequence number
        if self.last_seq is not None:
            gap = (seq - self.last_seq - 1) & 0xFFFF
            if gap:
                self.store.dropped += gap
        self.last_seq = seq
        if status & 0x01:
            self.store.overflow += 1
        self.store.last_ts = ts
        self.store.push(samples)
        return True

    def stop(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass

# ============================ DSP PIPELINE ==================================
WIN   = blackmanharris(N_FFT)
CG    = WIN.sum()                    # coherent gain (window sum)
FREQS = np.fft.rfftfreq(N_FFT, d=1.0 / FS_ACTUAL)
BINHZ = FS_ACTUAL / N_FFT

def codes_to_volts(x):
    return x * (ADC_VREF / ADC_MAX) / FE_GAIN

def _quad_interp(mag, k):
    """Parabolic peak interpolation -> sub-bin offset in [-0.5, 0.5]."""
    if k <= 0 or k >= len(mag) - 1:
        return 0.0
    a, b, c = mag[k - 1], mag[k], mag[k + 1]
    d = a - 2 * b + c
    return 0.0 if d == 0 else 0.5 * (a - c) / d

def _amp_phase(X, mag, freq):
    """Amplitude (leakage-tolerant band-power) and phase at a target freq."""
    k = int(round(freq / BINHZ))
    k = max(1, min(k, len(mag) - 2))
    lo, hi = max(1, k - 2), min(len(mag) - 1, k + 3)     # main-lobe band
    amp = (2.0 / CG) * np.sqrt(np.sum(mag[lo:hi] ** 2))  # peak amplitude (V)
    phase = np.angle(X[k])
    return amp, phase, k

def analyze(block_codes):
    v = codes_to_volts(block_codes)
    v -= v.mean()                                        # 1) DC removal
    rms_time = float(np.sqrt(np.mean(v ** 2)))           # true RMS
    xw = v * WIN                                         # 2) window
    X = np.fft.rfft(xw)                                  # 3) FFT
    mag = np.abs(X)

    # fundamental: refine frequency near LINE_HZ
    band = (FREQS > LINE_HZ * 0.7) & (FREQS < LINE_HZ * 1.3)
    k1 = np.argmax(np.where(band, mag, 0))
    f1 = (k1 + _quad_interp(mag, k1)) * BINHZ
    if not (LINE_HZ * 0.7 < f1 < LINE_HZ * 1.3):
        f1 = LINE_HZ

    # 4) features: fundamental + harmonics (tracked at multiples of measured f1)
    harm = []
    for h in range(1, N_HARM + 1):
        amp, ph, k = _amp_phase(X, mag, f1 * h)
        harm.append((h, f1 * h, amp, ph, k))
    a1 = harm[0][2] if harm[0][2] > 1e-12 else 1e-12
    thd = float(np.sqrt(sum(a ** 2 for _, _, a, _, _ in harm[1:])) / a1)

    # 5) post-feature: reconstruct a clean fundamental + full harmonic model
    t = np.arange(N_FFT) / FS_ACTUAL
    fundamental = harm[0][2] * np.cos(2 * np.pi * harm[0][1] * t + harm[0][3])
    recon = np.zeros(N_FFT)
    for _, fh, a, ph, _ in harm:
        recon += a * np.cos(2 * np.pi * fh * t + ph)
    #   moving-average + Savitzky-Golay (applied here, AFTER feature extraction)
    ma_w = 5
    ma = np.convolve(v, np.ones(ma_w) / ma_w, mode="same")
    sg = savgol_filter(v, window_length=11, polyorder=3)   # small -> no distortion

    return dict(v=v, fundamental=fundamental, recon=recon, ma=ma, sg=sg,
                f1=f1, a1=harm[0][2], phase=harm[0][3],
                thd=thd, rms_time=rms_time, rms_fund=harm[0][2] / np.sqrt(2),
                harm=harm, mag=mag)

# ============================ LIVE DISPLAY ==================================
def run_gui(store):
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(12, 7))
    gs = fig.add_gridspec(2, 2, width_ratios=[3, 1])
    ax_t = fig.add_subplot(gs[0, 0])     # time domain
    ax_f = fig.add_subplot(gs[1, 0])     # spectrum
    ax_r = fig.add_subplot(gs[:, 1]); ax_r.axis("off")   # readout
    fig.suptitle("LattePanda 50 Hz Acquisition — live", fontsize=13)

    show = int(3 * FS_ACTUAL / LINE_HZ)  # ~3 cycles for the time plot
    tms = np.arange(show) / FS_ACTUAL * 1000.0
    (l_raw,)  = ax_t.plot(tms, np.zeros(show), lw=0.8, alpha=0.5, label="raw (DC-removed)")
    (l_sg,)   = ax_t.plot(tms, np.zeros(show), lw=1.0, alpha=0.8, label="Savitzky-Golay")
    (l_fund,) = ax_t.plot(tms, np.zeros(show), lw=2.0, label="clean 50 Hz")
    ax_t.set_xlabel("time (ms)"); ax_t.set_ylabel("volts"); ax_t.legend(loc="upper right", fontsize=8)
    ax_t.grid(alpha=0.2)

    fmask = FREQS <= 1000
    (l_spec,) = ax_f.plot(FREQS[fmask], np.zeros(fmask.sum()), lw=1.0)
    hmarks = ax_f.scatter([], [], c="red", s=25, zorder=5)
    ax_f.set_xlabel("frequency (Hz)"); ax_f.set_ylabel("amplitude (V)")
    ax_f.set_xlim(0, 1000); ax_f.grid(alpha=0.2)
    txt = ax_r.text(0.0, 1.0, "", va="top", ha="left", family="monospace", fontsize=10)

    def update(_):
        buf, dropped, bad_crc, overflow, ts = store.snapshot()
        if np.all(buf == 0):
            return
        r = analyze(buf)

        l_raw.set_ydata(r["v"][-show:])
        l_sg.set_ydata(r["sg"][-show:])
        l_fund.set_ydata(r["fundamental"][-show:])
        m = max(1e-3, np.max(np.abs(r["v"][-show:])) * 1.2)
        ax_t.set_ylim(-m, m)

        spec = r["mag"][fmask] * (2.0 / CG)
        l_spec.set_ydata(spec)
        ax_f.set_ylim(0, max(1e-3, spec.max() * 1.2))
        hx = [fh for _, fh, _, _, _ in r["harm"]]
        hy = [a for _, _, a, _, _ in r["harm"]]
        hmarks.set_offsets(np.column_stack([hx, hy]))

        rows = [" f0    : %8.3f Hz" % r["f1"],
                " Apk   : %8.4f V"  % r["a1"],
                " Arms  : %8.4f V"  % r["rms_fund"],
                " phase : %8.2f deg" % np.degrees(r["phase"]),
                "",
                " THD   : %7.3f %%" % (r["thd"] * 100),
                " RMS   : %8.4f V"  % r["rms_time"],
                "",
                " harmonic  freq     amp    %%f0",
                " ------------------------------"]
        for h, fh, a, _, _ in r["harm"]:
            rows.append(" h%d  %8.1f  %7.4f  %5.1f" % (h, fh, a, 100 * a / r["a1"]))
        rows += ["", " dropped : %d" % dropped,
                     " bad crc : %d" % bad_crc,
                     " mcu ovf : %d" % overflow,
                     " ts(ms)  : %d" % ts]
        txt.set_text("\n".join(rows))
        return l_raw, l_sg, l_fund, l_spec, hmarks, txt

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.tight_layout()
    plt.show()
    return ani

# ============================== PORT PICK ===================================
def find_port():
    for p in serial.tools.list_ports.comports():
        if (p.vid == 0x2341) or ("leonardo" in (p.description or "").lower()) \
           or ("arduino" in (p.description or "").lower()):
            return p.device
    return None

# ================================ MAIN ======================================
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", help="serial port (auto-detect if omitted)")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("No Arduino serial port found. Pass one with -p (e.g. -p COM5 "
                 "or -p /dev/ttyACM0).")
    print("Port: %s   Fs = %.2f Hz   N = %d   bin = %.3f Hz"
          % (port, FS_ACTUAL, N_FFT, BINHZ))

    store = SampleStore(N_FFT)
    reader = SerialReader(port, store)
    reader.start()
    try:
        _ani = run_gui(store)        # keep a ref so the animation is not GC'd
    finally:
        reader.stop()

if __name__ == "__main__":
    main()
