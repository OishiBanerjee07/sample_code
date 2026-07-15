/*
 * ============================================================================
 *  PowerLineDAQ_32U4  —  High-accuracy AC waveform acquisition firmware
 *  Target : ATmega32U4 co-processor on LattePanda 3 Delta
 *           (appears to the x86 side as an Arduino Leonardo on a COM port)
 *  Purpose: Jitter-free, oversampled sampling of a 0–3.3 V scaled/isolated
 *           power-line voltage signal, streamed continuously over the
 *           internal USB-CDC link so the x86 main processor can perform
 *           harmonic analysis (up to the 13th harmonic and beyond).
 *
 *  KEY DESIGN POINTS
 *  -----------------
 *  1. HARDWARE-TIMED SAMPLING (zero software jitter)
 *     Timer1 runs in CTC mode; its Compare-Match-B event auto-triggers the
 *     ADC (ADTS2:0 = 0b101). Sample instants are therefore locked to the
 *     16 MHz crystal — no interrupt-latency jitter, which is critical for
 *     accurate harmonic (FFT) analysis.
 *
 *  2. OVERSAMPLING + DECIMATION for extra resolution / noise reduction
 *     The ADC runs OSR (1/2/4/8/16) times faster than the output rate.
 *     OSR raw conversions are summed into one output sample. With OSR=4
 *     the 10-bit ADC yields ~11 effective bits; noise is averaged out.
 *     Default: 20 kSa/s ADC rate, OSR=4  ->  5000 output samples/s.
 *     (5 kSa/s gives 100 samples per 50 Hz cycle; the 13th harmonic at
 *     650/780 Hz is sampled 6.4x/7.7x above Nyquist requirements.)
 *
 *  3. MAXIMUM DURATION = UNLIMITED (continuous streaming)
 *     The 32U4 has only 2.5 KB of SRAM, so long captures cannot be
 *     buffered on-chip. Instead samples flow through a 512-sample ring
 *     buffer into the USB-CDC port (12 Mbit/s full-speed link — far
 *     faster than the ~10 KB/s we produce). Capture runs until STOP or
 *     until the sample count requested in START is reached.
 *
 *  4. CALIBRATION (stored in EEPROM, survives power-cycles)
 *       CALZ            - offset (zero) calibration: apply the quiescent
 *                         input (mains disconnected, bias mid-scale) and
 *                         the firmware measures the DC offset.
 *       CALG <mVrms>    - gain calibration: apply a known sine (e.g. from
 *                         a calibrator or measured with a reference DMM),
 *                         firmware measures RMS in counts and stores the
 *                         uV-per-LSB scale factor.
 *       GETCAL / SETCAL / SAVE - inspect, override, persist.
 *     In MODE CAL the streamed samples are offset-corrected (zero-centred);
 *     the host converts counts to volts with the uV/LSB factor.
 *
 *  5. ADC REFERENCE
 *     Default AVcc (5 V) — always safe. For best resolution of a 0–3.3 V
 *     signal, wire the LattePanda 3.3 V rail to the AREF pin and issue
 *     "REF EXT" (gains ~1.5x more counts per volt).  NEVER select AVCC or
 *     2V56 while an external source drives AREF hard without a resistor —
 *     see datasheet §24.5.2 (this firmware refuses to switch away from
 *     EXT once selected, until reset, for that reason).
 *
 *  ---------------------------------------------------------------------------
 *  SERIAL PROTOCOL  (USB CDC, "Serial", baud setting is ignored by CDC)
 *  ---------------------------------------------------------------------------
 *  Host -> device : ASCII commands terminated by '\n' (CR ignored)
 *
 *    START [n]      Begin acquisition. n = number of output samples to
 *                   deliver (optional; 0 or omitted = continuous).
 *                   Reply "OK START <rate> <osr> <mode> <uV_per_LSB>\n"
 *                   then binary data packets follow.
 *    STOP           End acquisition. Buffer is drained, then
 *                   "OK STOP <total_samples_sent>\n".
 *    RATE <hz>      Output sample rate, 500..10000 Sa/s   (default 5000)
 *    OSR <n>        Oversampling ratio 1,2,4,8,16         (default 4)
 *                   (rate*osr must be <= 38000 conversions/s)
 *    CH <0..7>      ADC channel (default 7 = pin A0 on LattePanda header)
 *    REF <AVCC|EXT> ADC reference (default AVCC; EXT = AREF pin, see above)
 *    MODE <RAW|CAL> RAW = plain summed counts; CAL = offset-corrected
 *    CALZ           Zero/offset calibration (~1 s, input at rest)
 *    CALG <mVrms>   Gain calibration against a known applied RMS voltage
 *                   in millivolts at the ADC pin (~1 s of sine required)
 *    GETCAL         Reply "CAL <offset_x16> <uV_per_LSB> <valid>\n"
 *    SETCAL <o> <u> Manually set offset_x16 (int) and uV/LSB (float)
 *    SAVE           Persist calibration + configuration to EEPROM
 *    INFO           Human-readable status line
 *    *IDN?          Identification string
 *
 *  Device -> host during acquisition : fixed-size binary packets
 *
 *    Offset  Size  Field
 *    0       1     0xA5  sync byte 1
 *    1       1     0x5A  sync byte 2
 *    2       1     flags   bit0 = calibrated mode
 *                          bit1 = ring-buffer overflow occurred since last pkt
 *                          bit2 = last packet of a finite capture
 *    3       1     seq     free-running packet counter (wraps at 256) —
 *                          lets the host detect lost packets
 *    4       1     nsamp   number of int16 samples in payload (== 24)
 *    5       1     hdr checksum: two's complement of bytes 2..4
 *    6..53   48    payload: nsamp little-endian int16 samples
 *    54      1     payload checksum: two's complement sum of payload bytes
 *
 *    Sample value = sum of OSR raw 10-bit conversions (0..1023*OSR) in RAW
 *    mode, or that sum minus the calibrated offset (signed, ~zero-centred)
 *    in CAL mode.  Volts = value / OSR * (uV_per_LSB / 1e6).
 *
 *  ---------------------------------------------------------------------------
 *  ANALOG FRONT-END NOTES (external circuit — please verify)
 *  ---------------------------------------------------------------------------
 *  - The isolated divider must BIAS the AC signal to mid-scale (e.g. 1.65 V
 *    when using REF EXT / 3.3 V), since the ADC is unipolar.
 *  - Add an ANTI-ALIASING RC low-pass in front of the pin. The firmware
 *    cannot remove energy above Nyquist after sampling. With the default
 *    20 kSa/s ADC rate, fc ~= 2..3 kHz (e.g. 1 kOhm + 33 nF -> 4.8 kHz,
 *    or 1 kOhm + 68 nF -> 2.3 kHz) passes the 13th harmonic untouched and
 *    kills HF switching noise. Keep source impedance <= 10 kOhm per the
 *    datasheet for full S/H accuracy.
 *  - A 100 nF capacitor from AREF to GND is already on the board.
 * ============================================================================
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>

/* ------------------------------------------------------------------ config */
#define FW_IDN            "LattePanda-32U4 PowerLineDAQ v1.0"
#define F_CPU_HZ          16000000UL
#define RING_SIZE         512          /* samples, power of two, 1 KB RAM   */
#define RING_MASK         (RING_SIZE - 1)
#define PKT_SAMPLES       24           /* fits one 64-byte CDC bank         */
#define PKT_BYTES         (6 + PKT_SAMPLES * 2 + 1)

#define DEF_RATE          5000UL       /* output samples per second         */
#define DEF_OSR           4
#define DEF_CHANNEL       7            /* ADC7 = pin "A0" on LattePanda     */
#define MAX_ADC_SPS       38000UL      /* conv. time @ 500 kHz ADCclk ~27us */

#define CAL_SAMPLES       4096UL       /* samples used by CALZ / CALG       */
#define EE_MAGIC          0xB3A7

/* -------------------------------------------------------------- EEPROM map */
struct EeCal {
  uint16_t magic;
  int32_t  offset_x16;   /* DC offset in raw-ADC counts * 16 (Q4 fixed pt)  */
  float    uV_per_LSB;   /* microvolts per single raw ADC count             */
  uint8_t  valid;        /* bit0 offset valid, bit1 gain valid              */
  uint16_t rate_div;     /* persisted output rate / 10                      */
  uint8_t  osr;
  uint8_t  channel;
  uint8_t  crc;          /* simple sum-complement over previous bytes       */
};
EeCal EEMEM eeCal;

/* ------------------------------------------------------------ shared state */
static volatile int16_t  ring[RING_SIZE];
static volatile uint16_t ringHead = 0;          /* written by ISR            */
static volatile uint16_t ringTail = 0;          /* read by main loop         */
static volatile uint8_t  ovfFlag  = 0;          /* ring overflow latch       */

static volatile uint16_t osAcc    = 0;          /* oversampling accumulator  */
static volatile uint8_t  osCnt    = 0;
static volatile uint8_t  osrIsr   = DEF_OSR;    /* copy used inside the ISR  */
static volatile uint8_t  calModeIsr = 0;
static volatile int32_t  offSumIsr  = 0;        /* offset*OSR, pre-computed  */

/* configuration (main-loop domain) */
static uint32_t outRate   = DEF_RATE;
static uint8_t  osr       = DEF_OSR;
static uint8_t  channel   = DEF_CHANNEL;
static uint8_t  useExtRef = 0;                  /* 0 = AVcc, 1 = AREF pin    */
static uint8_t  calMode   = 0;                  /* 0 = RAW, 1 = CAL          */

/* calibration (main-loop domain) */
static int32_t  calOffsetX16 = 0;
static float    calUvPerLsb  = 4882.8f;         /* 5.0 V / 1024 by default   */
static uint8_t  calValid     = 0;

/* acquisition bookkeeping */
static bool     running        = false;
static uint32_t samplesWanted  = 0;             /* 0 = continuous            */
static uint32_t samplesSent    = 0;
static uint8_t  pktSeq         = 0;

/* command line buffer */
static char     cmdBuf[48];
static uint8_t  cmdLen = 0;

/* =========================================================================
 *  ADC INTERRUPT — runs once per raw conversion (hardware triggered).
 *  Kept minimal: ~4 us worst case at 16 MHz, well inside the 26 us budget
 *  of the fastest supported conversion rate.
 * ========================================================================= */
ISR(ADC_vect)
{
  TIFR1 = _BV(OCF1B);                 /* re-arm the Timer1 auto-trigger     */

  osAcc += ADC;                       /* ADCL then ADCH, compiler handles   */

  if (++osCnt >= osrIsr) {
    int32_t v = osAcc;
    osAcc = 0;
    osCnt = 0;

    if (calModeIsr)
      v -= offSumIsr;                 /* zero-centre using calibration      */

    /* clip to int16 just in case */
    if (v >  32767) v =  32767;
    if (v < -32768) v = -32768;

    uint16_t h = ringHead;
    uint16_t next = (h + 1) & RING_MASK;
    if (next == ringTail) {
      ovfFlag = 1;                    /* host too slow — drop, flag it      */
    } else {
      ring[h] = (int16_t)v;
      ringHead = next;
    }
  }
}

/* =========================================================================
 *  Low-level ADC / Timer setup
 * ========================================================================= */
static uint8_t adcPrescalerBits(uint32_t adcSps, bool &hsm)
{
  /* Choose the slowest ADC clock that still completes one conversion
     (13.5 ADC clocks in auto-trigger mode) inside the trigger period.
     Slower ADC clock == better accuracy (datasheet: 50-200 kHz for full
     10-bit resolution; ADHSM allows faster clocks at reduced accuracy,
     which oversampling then wins back). */
  hsm = false;
  if (adcSps <=  9000) return 0x07;              /* /128 -> 125 kHz  */
  if (adcSps <= 18000) return 0x06;              /* /64  -> 250 kHz  */
  hsm = true;
  return 0x05;                                   /* /32  -> 500 kHz  */
}

static void adcConfigure(void)
{
  ADCSRA = 0;                                    /* stop ADC                */

  /* reference + channel (right-adjusted result) */
  uint8_t ref = useExtRef ? 0x00 : 0x40;         /* REFS1:0 = 00 ext, 01 AVcc */
  ADMUX  = ref | (channel & 0x1F & 0x07);        /* ADC0..7 -> MUX4:0 low 3  */
  ADCSRB = 0;                                    /* MUX5 = 0 for ADC0..7     */

  bool hsm;
  uint8_t ps = adcPrescalerBits(outRate * osr, hsm);
  ADCSRB = (hsm ? _BV(ADHSM) : 0) | 0x05;        /* ADTS2:0=101: T1 comp B   */

  ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | ps;

  /* one throw-away conversion so internal circuitry settles (25 clk) */
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) { }
  (void)ADC;
  ADCSRA |= _BV(ADIF);                           /* clear pending flag       */
}

static void timerConfigure(void)
{
  uint32_t adcSps = outRate * osr;
  uint32_t top = (F_CPU_HZ / adcSps) - 1;        /* prescaler 1              */

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = (uint16_t)top;                        /* CTC TOP                  */
  OCR1B  = (uint16_t)top;                        /* trigger at TOP           */
  TIFR1  = 0xFF;
}

static void acquisitionStart(void)
{
  osAcc = 0; osCnt = 0;
  ringHead = ringTail = 0;
  ovfFlag = 0;
  samplesSent = 0;
  pktSeq = 0;

  osrIsr     = osr;
  calModeIsr = calMode && (calValid & 0x01);
  offSumIsr  = ((calOffsetX16 * (int32_t)osr) + 8) >> 4;  /* offset*OSR      */

  adcConfigure();
  timerConfigure();
  TCCR1B = _BV(WGM12) | _BV(CS10);               /* CTC, clk/1 -> GO         */
  running = true;
}

static void acquisitionStop(void)
{
  TCCR1B = 0;                                    /* stop trigger timer       */
  ADCSRA &= (uint8_t)~(_BV(ADEN) | _BV(ADIE) | _BV(ADATE));
  running = false;
}

/* =========================================================================
 *  Ring buffer -> USB packets
 * ========================================================================= */
static uint16_t ringCount(void)
{
  uint16_t h, t;
  noInterrupts(); h = ringHead; t = ringTail; interrupts();
  return (h - t) & RING_MASK;
}

static void sendPacket(uint8_t nsamp, bool lastPkt)
{
  uint8_t pkt[PKT_BYTES];
  uint8_t flags = 0;

  noInterrupts();
  if (ovfFlag) { flags |= 0x02; ovfFlag = 0; }
  interrupts();
  if (calModeIsr) flags |= 0x01;
  if (lastPkt)    flags |= 0x04;

  pkt[0] = 0xA5;
  pkt[1] = 0x5A;
  pkt[2] = flags;
  pkt[3] = pktSeq++;
  pkt[4] = nsamp;
  pkt[5] = (uint8_t)(0 - (pkt[2] + pkt[3] + pkt[4]));

  uint8_t sum = 0;
  uint8_t *p = &pkt[6];
  for (uint8_t i = 0; i < nsamp; i++) {
    int16_t s = ring[ringTail];
    ringTail = (ringTail + 1) & RING_MASK;
    *p++ = (uint8_t)(s & 0xFF);        sum += (uint8_t)(s & 0xFF);
    *p++ = (uint8_t)((s >> 8) & 0xFF); sum += (uint8_t)((s >> 8) & 0xFF);
  }
  *p = (uint8_t)(0 - sum);

  Serial.write(pkt, 6 + nsamp * 2 + 1);
  samplesSent += nsamp;
}

static void pumpData(void)
{
  if (!running) return;

  /* finite capture finished? */
  bool finite = (samplesWanted != 0);

  while (true) {
    uint16_t avail = ringCount();
    uint32_t remaining = finite ? (samplesWanted - samplesSent) : 0xFFFFFFFF;

    if (finite && remaining == 0) {
      acquisitionStop();
      Serial.print(F("OK STOP "));
      Serial.println(samplesSent);
      return;
    }

    uint8_t want = PKT_SAMPLES;
    if (finite && remaining < want) want = (uint8_t)remaining;

    if (avail < want) return;                      /* not enough data yet    */
    if (Serial.availableForWrite() < (int)(6 + want * 2 + 1))
      return;                                      /* CDC bank busy — retry  */

    bool last = finite && (remaining == want);
    sendPacket(want, last);
  }
}

/* =========================================================================
 *  Calibration helpers — blocking captures using the normal ISR path
 * ========================================================================= */
static bool captureBlock(uint32_t n, int64_t &sum, uint64_t &sumSq)
{
  /* temporarily run acquisition privately (RAW, current rate/osr) */
  uint8_t savedCal = calMode; calMode = 0;
  acquisitionStart();
  sum = 0; sumSq = 0;
  uint32_t got = 0;
  uint32_t guard = millis();

  while (got < n) {
    if (ringCount() > 0) {
      int32_t v = ring[ringTail];
      ringTail = (ringTail + 1) & RING_MASK;
      sum   += v;
      sumSq += (uint64_t)((int64_t)v * v);
      got++;
      guard = millis();
    } else if (millis() - guard > 1000) {          /* watchdog: no samples   */
      acquisitionStop(); calMode = savedCal; return false;
    }
  }
  acquisitionStop();
  calMode = savedCal;
  return true;
}

static void doCalZero(void)
{
  int64_t sum; uint64_t sq;
  Serial.println(F("# CALZ: measuring offset, keep input quiescent..."));
  if (!captureBlock(CAL_SAMPLES, sum, sq)) { Serial.println(F("ERR CALZ timeout")); return; }
  /* mean of OSR-summed samples -> per-raw-count offset in Q4 */
  float meanSum = (float)sum / (float)CAL_SAMPLES;      /* counts * OSR      */
  calOffsetX16  = (int32_t)((meanSum / (float)osr) * 16.0f + 0.5f);
  calValid |= 0x01;
  Serial.print(F("OK CALZ offset_x16=")); Serial.println(calOffsetX16);
}

static void doCalGain(float mVrms)
{
  if (!(calValid & 0x01)) { Serial.println(F("ERR run CALZ first")); return; }
  if (mVrms <= 0.0f)      { Serial.println(F("ERR bad reference value")); return; }

  int64_t sum; uint64_t sq;
  Serial.println(F("# CALG: measuring RMS, apply reference sine now..."));
  if (!captureBlock(CAL_SAMPLES, sum, sq)) { Serial.println(F("ERR CALG timeout")); return; }

  float n     = (float)CAL_SAMPLES;
  float mean  = (float)sum / n;                          /* includes offset  */
  float msq   = (float)sq  / n;
  float var   = msq - mean * mean;                       /* AC power, sum^2  */
  if (var < 1.0f) { Serial.println(F("ERR signal too small")); return; }
  float rmsSum = sqrtf(var);                             /* counts*OSR RMS   */
  float rmsRaw = rmsSum / (float)osr;                    /* raw-count RMS    */

  calUvPerLsb = (mVrms * 1000.0f) / rmsRaw;              /* uV per raw LSB   */
  calValid |= 0x02;
  Serial.print(F("OK CALG uV_per_LSB=")); Serial.println(calUvPerLsb, 3);
}

/* =========================================================================
 *  EEPROM
 * ========================================================================= */
static uint8_t eeCrc(const EeCal &c)
{
  const uint8_t *p = (const uint8_t *)&c;
  uint8_t s = 0;
  for (uint8_t i = 0; i < sizeof(EeCal) - 1; i++) s += p[i];
  return (uint8_t)(0 - s);
}

static void eeSave(void)
{
  EeCal c;
  c.magic      = EE_MAGIC;
  c.offset_x16 = calOffsetX16;
  c.uV_per_LSB = calUvPerLsb;
  c.valid      = calValid;
  c.rate_div   = (uint16_t)(outRate / 10);
  c.osr        = osr;
  c.channel    = channel;
  c.crc        = eeCrc(c);
  eeprom_update_block(&c, &eeCal, sizeof(c));
  Serial.println(F("OK SAVE"));
}

static void eeLoad(void)
{
  EeCal c;
  eeprom_read_block(&c, &eeCal, sizeof(c));
  if (c.magic != EE_MAGIC || c.crc != eeCrc(c)) return;   /* keep defaults   */
  calOffsetX16 = c.offset_x16;
  calUvPerLsb  = c.uV_per_LSB;
  calValid     = c.valid;
  outRate      = (uint32_t)c.rate_div * 10;
  osr          = c.osr;
  channel      = c.channel;
  if (outRate < 500 || outRate > 10000) outRate = DEF_RATE;
  if (osr != 1 && osr != 2 && osr != 4 && osr != 8 && osr != 16) osr = DEF_OSR;
  if (channel > 7) channel = DEF_CHANNEL;
}

/* =========================================================================
 *  Command parser
 * ========================================================================= */
static void printInfo(void)
{
  Serial.print(F("INFO rate=")); Serial.print(outRate);
  Serial.print(F(" osr="));      Serial.print(osr);
  Serial.print(F(" adc_sps="));  Serial.print(outRate * osr);
  Serial.print(F(" ch=A"));      Serial.print(7 - channel);   /* ADC7 = A0 */
  Serial.print(F("(ADC"));       Serial.print(channel);
  Serial.print(F(") ref="));     Serial.print(useExtRef ? F("EXT") : F("AVCC"));
  Serial.print(F(" mode="));     Serial.print(calMode ? F("CAL") : F("RAW"));
  Serial.print(F(" offset_x16=")); Serial.print(calOffsetX16);
  Serial.print(F(" uV_per_LSB=")); Serial.print(calUvPerLsb, 3);
  Serial.print(F(" calvalid=")); Serial.print(calValid);
  Serial.print(F(" running="));  Serial.println(running ? 1 : 0);
}

static void handleCommand(char *cmd)
{
  /* uppercase in place */
  for (char *q = cmd; *q; q++) if (*q >= 'a' && *q <= 'z') *q -= 32;

  char *arg = strchr(cmd, ' ');
  if (arg) { *arg++ = 0; while (*arg == ' ') arg++; }

  if (!strcmp(cmd, "START")) {
    if (running) { Serial.println(F("ERR already running")); return; }
    samplesWanted = arg ? strtoul(arg, NULL, 10) : 0;
    Serial.print(F("OK START ")); Serial.print(outRate);
    Serial.print(' ');            Serial.print(osr);
    Serial.print(' ');            Serial.print(calMode ? F("CAL") : F("RAW"));
    Serial.print(' ');            Serial.println(calUvPerLsb, 3);
    Serial.flush();
    acquisitionStart();
  }
  else if (!strcmp(cmd, "STOP")) {
    if (!running) { Serial.println(F("ERR not running")); return; }
    acquisitionStop();
    /* drain what is left */
    while (ringCount() >= PKT_SAMPLES) sendPacket(PKT_SAMPLES, false);
    uint16_t rem = ringCount();
    if (rem) sendPacket((uint8_t)rem, true);
    Serial.print(F("OK STOP ")); Serial.println(samplesSent);
  }
  else if (!strcmp(cmd, "RATE")) {
    if (running) { Serial.println(F("ERR stop first")); return; }
    uint32_t r = arg ? strtoul(arg, NULL, 10) : 0;
    if (r < 500 || r > 10000 || r * osr > MAX_ADC_SPS) { Serial.println(F("ERR range")); return; }
    outRate = r; Serial.println(F("OK RATE"));
  }
  else if (!strcmp(cmd, "OSR")) {
    if (running) { Serial.println(F("ERR stop first")); return; }
    uint8_t o = arg ? (uint8_t)strtoul(arg, NULL, 10) : 0;
    if ((o != 1 && o != 2 && o != 4 && o != 8 && o != 16) ||
        outRate * o > MAX_ADC_SPS) { Serial.println(F("ERR range")); return; }
    osr = o; Serial.println(F("OK OSR"));
  }
  else if (!strcmp(cmd, "CH")) {
    if (running) { Serial.println(F("ERR stop first")); return; }
    uint8_t c = arg ? (uint8_t)strtoul(arg, NULL, 10) : 255;
    if (c > 7) { Serial.println(F("ERR range")); return; }
    channel = c; Serial.println(F("OK CH"));
  }
  else if (!strcmp(cmd, "REF")) {
    if (running) { Serial.println(F("ERR stop first")); return; }
    if (arg && !strcmp(arg, "EXT"))  { useExtRef = 1; Serial.println(F("OK REF EXT")); }
    else if (arg && !strcmp(arg, "AVCC")) {
      if (useExtRef) { Serial.println(F("ERR reset required to leave EXT")); return; }
      useExtRef = 0; Serial.println(F("OK REF AVCC"));
    }
    else Serial.println(F("ERR use REF AVCC|EXT"));
  }
  else if (!strcmp(cmd, "MODE")) {
    if (arg && !strcmp(arg, "CAL")) {
      if (!(calValid & 0x01)) { Serial.println(F("ERR no offset cal")); return; }
      calMode = 1; Serial.println(F("OK MODE CAL"));
    } else if (arg && !strcmp(arg, "RAW")) {
      calMode = 0; Serial.println(F("OK MODE RAW"));
    } else Serial.println(F("ERR use MODE RAW|CAL"));
  }
  else if (!strcmp(cmd, "CALZ")) {
    if (running) { Serial.println(F("ERR stop first")); return; }
    doCalZero();
  }
  else if (!strcmp(cmd, "CALG")) {
    if (running) { Serial.println(F("ERR stop first")); return; }
    doCalGain(arg ? atof(arg) : 0.0f);
  }
  else if (!strcmp(cmd, "GETCAL")) {
    Serial.print(F("CAL ")); Serial.print(calOffsetX16);
    Serial.print(' ');       Serial.print(calUvPerLsb, 3);
    Serial.print(' ');       Serial.println(calValid);
  }
  else if (!strcmp(cmd, "SETCAL")) {
    char *a2 = arg ? strchr(arg, ' ') : NULL;
    if (!arg || !a2) { Serial.println(F("ERR SETCAL <offset_x16> <uV_per_LSB>")); return; }
    *a2++ = 0;
    calOffsetX16 = strtol(arg, NULL, 10);
    calUvPerLsb  = atof(a2);
    calValid     = 0x03;
    Serial.println(F("OK SETCAL"));
  }
  else if (!strcmp(cmd, "SAVE"))   { eeSave(); }
  else if (!strcmp(cmd, "INFO"))   { printInfo(); }
  else if (!strcmp(cmd, "*IDN?"))  { Serial.println(F(FW_IDN)); }
  else if (cmd[0])                 { Serial.println(F("ERR unknown cmd")); }
}

/* =========================================================================
 *  Arduino entry points
 * ========================================================================= */
void setup()
{
  /* disable digital input buffer on all ADC pins -> lower noise, datasheet */
  DIDR0 = 0xF3;                       /* ADC7..4, ADC1, ADC0 present on PF  */
  DIDR2 = 0x3F;

  /* unused pins: leave as inputs with pullups off is fine on LattePanda   */

  eeLoad();
  Serial.begin(2000000);              /* value ignored by CDC              */
}

void loop()
{
  /* 1. move samples out */
  pumpData();

  /* 2. handle incoming commands */
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuf[cmdLen] = 0;
      cmdLen = 0;
      handleCommand(cmdBuf);
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdLen = 0;                     /* overflow: discard line             */
    }
  }
}
