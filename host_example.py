#!/usr/bin/env python3
"""
host_example.py — runs on the LattePanda x86 (Windows/Linux) side.

Talks to the ATmega32U4 running PowerLineDAQ_32U4.ino over the internal
USB-CDC COM port, captures N seconds of waveform, verifies packet integrity,
converts to volts using the device calibration, and prints the RMS value and
harmonic content up to the 13th harmonic.

Dependencies:  pip install pyserial numpy
Usage:         python host_example.py COM3 --seconds 2 --fund 50
               (on Linux the port is typically /dev/ttyACM0)
"""

import argparse
import struct
import sys
import time

import numpy as np
import serial

SYNC = b"\xA5\x5A"
HDR_LEN = 6


def read_line(ser, timeout=3.0):
    ser.timeout = timeout
    line = ser.readline().decode(errors="replace").strip()
    return line


def read_packet(ser):
    """Return (flags, seq, samples[np.int16]) or None on sync loss/timeout."""
    # hunt for sync
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b == b"\xA5":
            b2 = ser.read(1)
            if b2 == b"\x5A":
                break
    hdr = ser.read(4)
    if len(hdr) < 4:
        return None
    flags, seq, nsamp, hcs = hdr
    if (flags + seq + nsamp + hcs) & 0xFF != 0:
        return None  # header checksum failed
    payload = ser.read(nsamp * 2 + 1)
    if len(payload) < nsamp * 2 + 1:
        return None
    if sum(payload) & 0xFF != 0:
        return None  # payload checksum failed
    samples = np.frombuffer(payload[:-1], dtype="<i2")
    return flags, seq, samples


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--rate", type=int, default=5000)
    ap.add_argument("--osr", type=int, default=4)
    ap.add_argument("--fund", type=float, default=50.0, help="mains frequency")
    ap.add_argument("--mode", choices=["RAW", "CAL"], default="RAW")
    ap.add_argument("--calz", action="store_true", help="run offset calibration first")
    ap.add_argument("--calg", type=float, metavar="MVRMS",
                    help="run gain calibration against a known mVrms at the pin")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 2_000_000, timeout=2)
    time.sleep(1.5)                     # allow CDC enumeration after open
    ser.reset_input_buffer()

    def cmd(c):
        ser.write((c + "\n").encode())
        r = read_line(ser)
        print(f"  > {c:<20} < {r}")
        return r

    cmd("*IDN?")
    cmd(f"RATE {args.rate}")
    cmd(f"OSR {args.osr}")

    if args.calz:
        ser.write(b"CALZ\n")
        for _ in range(3):
            r = read_line(ser, timeout=5)
            print("  <", r)
            if r.startswith(("OK", "ERR")):
                break
    if args.calg:
        ser.write(f"CALG {args.calg}\n".encode())
        for _ in range(3):
            r = read_line(ser, timeout=5)
            print("  <", r)
            if r.startswith(("OK", "ERR")):
                break
    if args.calz or args.calg:
        cmd("SAVE")

    cmd(f"MODE {args.mode}")
    getcal = cmd("GETCAL")              # "CAL <offset_x16> <uV_per_LSB> <valid>"
    uv_per_lsb = float(getcal.split()[2]) if getcal.startswith("CAL") else 4882.8

    n_wanted = int(args.seconds * args.rate)
    resp = cmd(f"START {n_wanted}")
    if not resp.startswith("OK START"):
        sys.exit("device refused START")

    data, lost, overflow = [], 0, False
    last_seq = None
    while len(data) < n_wanted:
        pkt = read_packet(ser)
        if pkt is None:
            print("!! sync/timeout")
            break
        flags, seq, samples = pkt
        if last_seq is not None and seq != (last_seq + 1) & 0xFF:
            lost += 1
        last_seq = seq
        if flags & 0x02:
            overflow = True
        data.append(samples)
        if flags & 0x04:
            break
    print("  <", read_line(ser))        # trailing "OK STOP n"

    x = np.concatenate(data).astype(np.float64)
    print(f"\ncaptured {len(x)} samples, lost-seq events: {lost}, overflow: {overflow}")

    # counts -> volts (samples are OSR-summed raw counts)
    volts = (x / args.osr) * (uv_per_lsb * 1e-6)
    if args.mode == "RAW":
        volts -= volts.mean()           # remove bias in RAW mode

    fs = args.rate
    # Use an integer number of fundamental cycles to avoid FFT leakage
    n_cyc = int(len(volts) / fs * args.fund)
    n_use = int(round(n_cyc * fs / args.fund))
    v = volts[:n_use]
    win = np.hanning(len(v))
    spec = np.fft.rfft(v * win)
    freqs = np.fft.rfftfreq(len(v), 1 / fs)
    mag = np.abs(spec) * 2 / win.sum() / np.sqrt(2)   # ~RMS amplitude per bin

    print(f"\nRMS (pin): {np.sqrt(np.mean(v**2)):.4f} V")
    print(f"{'harmonic':>9} {'freq Hz':>9} {'Vrms':>10} {'% of fund':>10}")
    fund_mag = None
    for h in range(1, 14):
        f = h * args.fund
        idx = np.argmin(np.abs(freqs - f))
        m = mag[max(0, idx - 2): idx + 3].max()       # tolerate small freq error
        if h == 1:
            fund_mag = m
        pct = 100 * m / fund_mag if fund_mag else 0
        print(f"{h:>9} {f:>9.1f} {m:>10.5f} {pct:>9.2f}%")

    thd = np.sqrt(sum(
        mag[max(0, np.argmin(np.abs(freqs - h * args.fund)) - 2):
            np.argmin(np.abs(freqs - h * args.fund)) + 3].max() ** 2
        for h in range(2, 14))) / fund_mag * 100
    print(f"\nTHD (2nd..13th): {thd:.2f} %")

    ser.close()


if __name__ == "__main__":
    main()
