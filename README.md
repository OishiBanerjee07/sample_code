# PowerLineDAQ_32U4 — Firmware for the LattePanda 3 Delta co-processor

High-accuracy, jitter-free acquisition of a scaled/isolated power-line voltage
(0–3.3 V at the pin) on the ATmega32U4, streamed to the x86 main processor
over the internal USB-CDC link for harmonic analysis up to the 13th harmonic.

## Files

| File | Purpose |
|---|---|
| `PowerLineDAQ_32U4.ino` | Firmware. Open in Arduino IDE, board = **Arduino Leonardo**, upload to the LattePanda co-processor port. |
| `host_example.py` | Reference host program (Windows/Linux, `pip install pyserial numpy`). Captures data, checks integrity, computes RMS, harmonics 1–13 and THD. |

## How accuracy is obtained

1. **Hardware-timed sampling.** Timer1 (CTC) auto-triggers the ADC via the
   Timer1-Compare-Match-B trigger source (`ADTS = 0101`, datasheet §24.9.4).
   Sample instants are crystal-locked; there is no interrupt-latency jitter.
   Timing jitter is the main enemy of FFT-based harmonic measurements, so
   this matters more than raw bit count.
2. **Oversampling + averaging.** The ADC runs OSR× faster than the output
   rate and OSR conversions are summed per output sample (default OSR = 4:
   ADC at 20 kSa/s → 5 kSa/s output, ≈11 effective bits). OSR up to 16 is
   supported at lower output rates.
3. **ADC clock kept as slow as the rate allows.** ≤200 kHz where possible
   (full 10-bit accuracy per datasheet §24.4); `ADHSM` is enabled only when
   the requested conversion rate demands a 500 kHz ADC clock, and the lost
   accuracy is recovered by averaging.
4. **Digital input buffers disabled** on all analog pins (`DIDR0/DIDR2`)
   to cut switching noise into the sample-and-hold.
5. **Integrity-checked transport.** Framed binary packets with sequence
   numbers and checksums, plus an overflow flag, so the host *knows* the
   record is gap-free before running the FFT.

Default configuration: **5000 samples/s output** → 100 samples per 50 Hz
cycle; the 13th harmonic (650 Hz at 50 Hz, 780 Hz at 60 Hz) sits far below
the 2.5 kHz Nyquist limit.

## Maximum duration

Unlimited. The 32U4 has 2.5 KB of SRAM, so long records cannot be stored
on-chip; instead the firmware streams continuously through a 512-sample ring
buffer. At 5 kSa/s the link carries ~12 KB/s — a tiny fraction of the
12 Mbit/s full-speed USB connection to the x86. Capture length is set by the
host (`START <n>` for a finite record, `START` alone for continuous until
`STOP`).

## Wiring / analog front end (important)

- Signal input: **pin A0** (ADC7) by default; changeable with `CH`.
- The isolated divider must **bias the AC waveform to mid-scale** (the ADC is
  unipolar): e.g. 1.65 V bias for a 3.3 V span.
- Add an **anti-aliasing RC low-pass** before the pin — the firmware cannot
  remove energy above Nyquist after sampling. With the default 20 kSa/s ADC
  rate, fc ≈ 2–5 kHz works well (e.g. 1 kΩ + 68 nF ≈ 2.3 kHz). Keep source
  impedance ≤ 10 kΩ for full sample-and-hold accuracy.
- **Reference options.** Default reference is AVcc (5 V) — always safe, but
  a 3.3 V signal then uses only ~2/3 of the ADC range. For ~1.5× more
  resolution, connect the LattePanda **3.3 V rail to the AREF pin** and issue
  `REF EXT` (then `SAVE`). The firmware deliberately refuses to switch back
  from EXT to an internal reference without a reset, per the datasheet
  warning about driving AREF while an internal reference is selected.

## Command set (ASCII lines over the CDC port)

```
START [n]        begin acquisition (n output samples; omit/0 = continuous)
STOP             end acquisition, drain buffer
RATE <hz>        output rate 500..10000 (default 5000)
OSR <1|2|4|8|16> oversampling ratio (default 4); rate*osr <= 38000
CH <0..7>        ADC channel (default 7 = pin A0)
REF <AVCC|EXT>   ADC reference (see wiring notes)
MODE <RAW|CAL>   RAW = summed counts; CAL = offset-corrected (zero-centred)
CALZ             offset calibration (input quiescent, ~1 s)
CALG <mVrms>     gain calibration against a known applied RMS (mV at the pin)
GETCAL           report offset_x16, uV_per_LSB, validity flags
SETCAL <o> <u>   set calibration manually
SAVE             persist calibration + rate/osr/channel to EEPROM
INFO             status line
*IDN?            identification
```

## Binary data packet (device → host while running)

```
byte 0-1   0xA5 0x5A                      sync
byte 2     flags  b0=CAL mode  b1=overflow-since-last  b2=final packet
byte 3     seq    wraps at 256 (host detects lost packets)
byte 4     nsamp  samples in payload (24, less in the final packet)
byte 5     header checksum  (two's complement of bytes 2..4)
byte 6..   nsamp × int16 little-endian samples
last byte  payload checksum (two's complement of payload byte sum)
```

Sample value = sum of OSR raw conversions (RAW) or that sum minus the
calibrated offset (CAL). Convert to volts on the host:

```
V = value / OSR * uV_per_LSB * 1e-6
```

## Calibration procedure

1. **Offset:** disconnect the mains input so the front end outputs only its
   quiescent bias, then send `CALZ`. The firmware averages 4096 samples and
   stores the DC offset (Q4 fixed point, raw counts × 16).
2. **Gain:** apply a known, stable sine — either from a calibrator, or the
   mains itself measured simultaneously with a trusted true-RMS meter, scaled
   through your divider to the pin. Send `CALG <mVrms-at-the-pin>`. The
   firmware measures the AC RMS in counts and stores **µV per LSB**, which
   absorbs divider ratio, reference-voltage error and ADC gain error in one
   constant.
3. `SAVE` to persist. `GETCAL` any time to read the constants; the host
   applies `uV_per_LSB` even in RAW mode if it wishes.

## Quick start

```bash
# 2-second capture at 5 kSa/s, 50 Hz mains, print harmonics 1..13 + THD
python host_example.py COM3 --seconds 2 --fund 50

# one-time calibration (quiescent input first, then apply 810 mVrms at pin)
python host_example.py COM3 --calz --calg 810 --mode CAL
```

## Performance summary

| Parameter | Default | Range |
|---|---|---|
| Output sample rate | 5000 Sa/s | 500–10000 Sa/s |
| Oversampling | 4× (≈11 eff. bits) | 1–16× |
| ADC conversion rate | 20 kSa/s | ≤ 38 kSa/s |
| Sampling jitter | 0 (hardware-triggered) | — |
| Duration | unlimited (streaming) | host-controlled |
| Link load @ default | ≈12 KB/s | USB FS 12 Mbit/s |
