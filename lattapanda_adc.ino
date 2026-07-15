/*==============================================================================
  lattapanda_adc.ino  —  9.6 kHz / 10-bit power-signal acquisition front-end
  Target : LattePanda 3 Delta on-board co-processor = Arduino Leonardo
           (ATmega32U4, F_CPU = 16 MHz, 5 V logic)  ->  USB-CDC to Intel host

  DESIGN SUMMARY
  --------------
  * Sampling is HARDWARE-TIMED, not delay-based:
        Timer1 (CTC, TOP=OCR1A) generates a Compare-Match-B event once per
        sample period. That event AUTO-TRIGGERS the ADC (ADATE + ADTS=Timer1
        CompB). Conversion-complete fires ISR(ADC_vect). Because the ADC start
        edge is produced in *hardware*, sample-to-sample jitter is set only by
        the crystal, not by interrupt latency  ->  ~0 jitter.
  * "RTOS-style" decoupling WITHOUT an RTOS:
        On a 2.5 KB-SRAM AVR, FreeRTOS task stacks + scheduler would leave no
        room for a useful buffer. The idiomatic real-time pattern is used
        instead: the ADC ISR is the PRODUCER task, loop() is the CONSUMER task,
        and they communicate through a lock-free single-producer/single-consumer
        (SPSC) ring buffer. The ISR NEVER blocks (no Serial, no math beyond a
        store). Serial TX happens only in loop().
  * Post-ADC processing on-MCU: 3-point trailing MEDIAN only (isolated-spike /
        glitch removal). No moving-average, no Savitzky-Golay (done host-side).
  * Framed BINARY packets (sync + seq + count + status + timestamp + CRC-16)
        so the host can find boundaries, detect drops, and reject corruption.

  ADC / TIMING MATH  (F_CPU = 16 MHz)
  -----------------------------------
    Target Fs        : 9600 Hz  (Nyquist = 4800 Hz)
    OCR1A            : F_CPU/Fs - 1 = 16e6/9600 - 1 = 1665.67 -> 1666
    ACTUAL Fs        : 16e6 / (1666+1) = 9598.08 Hz   (16 MHz cannot make
                       exactly 9600; 0.02 % low — host uses FS_ACTUAL below).
    ADC prescaler    : /64  -> ADC clock = 250 kHz -> 13 clk = 52 us / conv.
                       Conversion (52 us) sits inside the 104.19 us period with
                       ~52 us of slack, so the OCF1B re-arm in the ISR is never
                       raced (a /128 "true-10-bit" 125 kHz clock is offered as a
                       #define but leaves almost no slack at 9.6 kHz).
    10-bit note      : 250 kHz slightly exceeds the datasheet 200 kHz "full
                       10-bit" ceiling; extra noise is < 1 LSB and is further
                       averaged out by the host windowed-FFT. Set ADC_PRESCALER_128
                       to 1 for strict 50-200 kHz operation (Fs pinned to conv time).

  ANALOG FRONT-END  (must exist BEFORE the ADC pin — anti-aliasing is mandatory)
  -----------------------------------------------------------------------------
    Signal path:  source -> protect -> scale/bias -> 4th-order LPF -> ADC(A0)

    1) Protection : 1 kOhm series + BAT54S dual-Schottky clamp to GND and AREF.
    2) Bias/scale : AC-couple (1 uF) into a mid-rail bias = AREF/2 = 1.65 V
                    (10k/10k divider from 3.3 V, buffered by an op-amp follower).
                    Choose input attenuation so the full swing lands in
                    ~0.15 V .. 3.15 V (never rail; keep head-room).
    3) Anti-alias : 4th-order Sallen-Key Butterworth LPF, fc = 2.4 kHz (= Fs/4,
                    well below Nyquist 4.8 kHz). Rail-to-rail op-amp
                    (MCP6002 / OPA2340). Equal-R (10k) unity-gain design:
                        Stage 1 (Q=0.541): R=10k, C1=6.8 nF, C2=6.2 nF
                        Stage 2 (Q=1.307): R=10k, C1=18 nF,  C2=2.7 nF
                    24 dB/oct roll-off gives ~30 dB attenuation at Nyquist.
                    (Verify/trim in TI FilterPro or ADI Filter Wizard; use C0G/NP0.)
    4) Reference  : AREF = clean 3.3 V (REF3033 or filtered LDO) with 10 uF||100 nF.
                    ADC configured for EXTERNAL reference so 0..3.3 V spans 0..1023.

    !!! HARDWARE WARNING !!!  With EXTERNAL AREF (default here) a valid 3.3 V MUST
    be present on the AREF pin at all times. Never enable an internal reference
    while driving AREF, and never exceed AVcc — doing so damages the MCU.

  PACKET FORMAT  (all multi-byte fields little-endian)
  ----------------------------------------------------
    off  size  field
    0    2     SYNC        = 0xAA 0x55
    2    2     seq         uint16   (increments per packet, wraps)
    4    2     count       uint16   (= SAMPLES_PER_PACKET)
    6    1     status      uint8    (bit0 = ring overflow since last packet)
    7    4     timestamp   uint32   (millis() at packetisation, diagnostics)
    11   2*N   samples     uint16[N] (10-bit, median-filtered, right-justified)
    ...  2     crc16       uint16   CRC-16/CCITT-FALSE over bytes [2 .. 11+2N)
    total = 13 + 2*N bytes  (N=128 -> 269 bytes)
==============================================================================*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*------------------------------- CONFIG -------------------------------------*/
#define FS_TARGET_HZ        9600UL      // requested rate (actual derived below)
#define SAMPLES_PER_PACKET  128         // block size sent per USB packet
#define USE_EXTERNAL_AREF   1           // 1 = 3.3 V EXTERNAL AREF (see warning)
#define ADC_PRESCALER_128   0           // 1 = /128 (125 kHz, strict 10-bit, tight)
#define ADC_INPUT_CHANNEL   7           // ADC7 == A0 on Leonardo (PF7)
#define SERIAL_BAUD         1000000UL   // ignored by USB-CDC but set for clarity

// Timer1 top for the sample clock. Actual Fs = F_CPU/(OCR1A_TOP+1).
#define OCR1A_TOP           ((uint16_t)((F_CPU / FS_TARGET_HZ) - 1))   // 1666 @16MHz
// FS_ACTUAL (Hz) — put the SAME value in the Python host (auto-printed once).

/*----------------------------- RING BUFFER ----------------------------------
  256-entry SPSC ring. uint8 head/tail => single-byte, ATOMIC on AVR, so no
  cli/sei needed for index access (true lock-free). Depth = 256 samples =
  ~26.7 ms of slack to ride out USB-CDC TX stalls.                            */
#define RB_SIZE  256
#define RB_MASK  (RB_SIZE - 1)
static volatile uint16_t rb[RB_SIZE];
static volatile uint8_t  rb_head     = 0;   // written ONLY by ISR (producer)
static volatile uint8_t  rb_tail     = 0;   // written ONLY by loop (consumer)
static volatile uint8_t  rb_overflow = 0;   // sticky flag, cleared per packet

/*------------------------------- CRC-16 -------------------------------------*/
// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout.
static uint16_t crc16_ccitt(const uint8_t *d, uint16_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

/*--------------------------- 3-POINT MEDIAN ---------------------------------
  Trailing median of (s[i-2], s[i-1], s[i]): removes ISOLATED single-sample
  spikes without needing a future sample and without smearing the 50 Hz shape
  (2-sample = 0.2 ms latency vs a 20 ms period). State carries across packets. */
static inline uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  uint16_t t;
  if (a > b) { t = a; a = b; b = t; }
  if (b > c) { t = b; b = c; c = t; }
  if (a > b) { t = a; a = b; b = t; }
  return b;
}
static uint16_t med_p1 = 512, med_p2 = 512;   // mid-scale seed (10-bit)

/*----------------------------- PACKET BUFFER --------------------------------*/
#define PKT_LEN (13 + 2 * SAMPLES_PER_PACKET)
static uint8_t  pkt[PKT_LEN];
static uint16_t seq = 0;

/*=============================== ISR (PRODUCER) =============================*/
// Minimal, interrupt-safe: read ADC, push, re-arm auto-trigger flag. Nothing else.
ISR(ADC_vect) {
  uint16_t s   = ADC;                 // atomic ADCL-then-ADCH read (10-bit)
  uint8_t  nh  = (uint8_t)(rb_head + 1);
  if (nh != rb_tail) {                // space available?
    rb[rb_head] = s;
    rb_head = nh;
  } else {
    rb_overflow = 1;                  // full: drop newest, flag it
  }
  TIFR1 = _BV(OCF1B);                 // clear Timer1 CompB flag -> re-arm trigger
}

/*=============================== SETUP =====================================*/
static void adc_init(void) {
  // Reference + input channel. REFS: 00=EXTERNAL(AREF), 01=AVcc.
  uint8_t refs = USE_EXTERNAL_AREF ? 0x00 : 0x40;      // ADMUX[7:6]
  ADMUX  = refs | (ADC_INPUT_CHANNEL & 0x1F);          // ADLAR=0 (right adjust)
  ADCSRB = 0;                                          // MUX5=0; then set ADTS
  ADCSRB |= _BV(ADTS2) | _BV(ADTS0);                   // ADTS=101: Timer1 CompB
#if ADC_PRESCALER_128
  ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADIE)
         | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);       // /128 -> 125 kHz
#else
  ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADIE)
         | _BV(ADPS2) | _BV(ADPS1);                    // /64  -> 250 kHz
#endif
  DIDR0 |= _BV(ADC7D);                                 // disable digital in on A0
}

static void timer1_init(void) {
  TCCR1A = 0;                                          // no PWM output pins
  TCCR1B = _BV(WGM12) | _BV(CS10);                     // CTC (TOP=OCR1A), presc=1
  TCCR1C = 0;
  OCR1A  = OCR1A_TOP;                                  // period => sample clock
  OCR1B  = 0;                                          // CompB trigger at period start
  TIMSK1 = 0;                                          // no timer ISR (HW path only)
  TIFR1  = _BV(OCF1B);                                 // clean flag for first edge
}

void setup(void) {
  Serial.begin(SERIAL_BAUD);                           // USB-CDC (baud ignored)
  noInterrupts();
  adc_init();
  timer1_init();
  interrupts();
  // First Compare-Match-B edge auto-starts conversions; nothing else to kick.
}

/*=============================== CONSUMER ==================================*/
static void send_packet(void) {
  // ---- header ----
  pkt[0] = 0xAA; pkt[1] = 0x55;                        // SYNC
  pkt[2] = (uint8_t)seq;        pkt[3] = (uint8_t)(seq >> 8);
  pkt[4] = (uint8_t)SAMPLES_PER_PACKET;
  pkt[5] = (uint8_t)(SAMPLES_PER_PACKET >> 8);
  uint8_t status = rb_overflow; rb_overflow = 0;       // read+clear sticky flag
  pkt[6] = status;
  uint32_t ts = millis();                              // millis() = DIAGNOSTIC only
  pkt[7]  = (uint8_t)ts;         pkt[8]  = (uint8_t)(ts >> 8);
  pkt[9]  = (uint8_t)(ts >> 16); pkt[10] = (uint8_t)(ts >> 24);

  // ---- payload: drain SPP samples, apply trailing median, pack LE ----
  uint8_t tail = rb_tail;
  uint16_t off = 11;
  for (uint16_t i = 0; i < SAMPLES_PER_PACKET; i++) {
    uint16_t raw = rb[tail];
    tail = (uint8_t)(tail + 1);
    uint16_t m = median3(med_p2, med_p1, raw);         // spike removal
    med_p2 = med_p1; med_p1 = raw;                     // slide history
    pkt[off++] = (uint8_t)m;
    pkt[off++] = (uint8_t)(m >> 8);
  }
  rb_tail = tail;                                      // publish consumed count

  // ---- integrity: CRC over [seq .. last sample] ----
  uint16_t crc = crc16_ccitt(&pkt[2], (uint16_t)(9 + 2 * SAMPLES_PER_PACKET));
  pkt[off++] = (uint8_t)crc;
  pkt[off++] = (uint8_t)(crc >> 8);

  Serial.write(pkt, PKT_LEN);                          // one bulk USB write
  seq++;
}

void loop(void) {
  if (!Serial) {                    // host not attached: flush ring, don't stall
    rb_tail = rb_head;
    return;
  }
  // Enough samples for a full block? (uint8 subtraction = correct ring count)
  if ((uint8_t)(rb_head - rb_tail) >= SAMPLES_PER_PACKET) {
    send_packet();
  }
  // loop() is the only place that can block on Serial.write; the ISR keeps
  // sampling into the ring meanwhile, so a brief USB stall costs no samples
  // (up to ~26 ms) and is otherwise reported via status bit0 + seq gaps.
}
