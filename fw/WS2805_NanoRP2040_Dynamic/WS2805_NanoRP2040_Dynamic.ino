/*
  WS2805 5-channel LED driver test firmware - dynamic chain length
  Board: Arduino Nano RP2040 Connect
  Data output: Arduino D2 (RP2040 GPIO25)

  Supports 1..32 WS2805 ICs in cascade, 5 bytes per IC:
      R, G, B, W1, W2
  MSB first.

  The pattern length automatically determines how many ICs are addressed:
      W2 aaa        -> IC1..IC3 W2 = 100 %
      W1 101010101  -> pattern over IC1..IC9
      B 00000001    -> IC1 blue = 10 %, IC2..IC8 = 0 %
      R a9876543    -> 8 ICs; IC8=100 %, ... IC1=30 %

  Pattern orientation:
      leftmost character  = highest addressed IC number
      rightmost character = IC1 (first WS2805 after the MCU)

  PWM coding:
      0 =   0 %
      1 =  10 %
      2 =  20 %
      ...
      9 =  90 %
      a = 100 %

  Serial commands, 115200 baud, CR/LF:
      R|G|B|W1|W2 <1..32 PWM characters>
      CHASE R|G|B|W1|W2 [1..32]
      TEST [1..32]
      TEST8 / TEST16 / TEST24 / TEST32
      OFF [1..32]
      SHOW [1..32]
      HELP

  CHASE and TEST default to the length of the most recent pattern command.
  At power-up this default is 8 ICs.

  OFF without a number deliberately sends 32 zero frames, so even a previously
  addressed longer chain is guaranteed to be switched off.

  The RP2040 PIO generates the WS2805 waveform with a 20 MHz PIO clock:
      0: high 300 ns, low 950 ns
      1: high 650 ns, low 600 ns
      bit period = 1.25 us (800 kbit/s)

  Important implementation detail:
  The PIO now auto-pulls after 8 bits instead of 32 bits. This allows frames
  of any length N*40 bits (N = 1..32), including lengths that are not multiples
  of 32 bits, without appending unwanted dummy bits.
*/

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#include "hardware/pio.h"
#include "hardware/clocks.h"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

static const uint8_t DATA_PIN = 2;        // Arduino D2
static const uint8_t MAX_ICS = 32;
static const uint8_t NUM_CHANNELS = 5;
static const uint8_t DEFAULT_ICS = 8;
static const uint16_t CHASE_DELAY_MS = 30;

enum Channel : uint8_t {
  CH_R = 0,
  CH_G,
  CH_B,
  CH_W1,
  CH_W2
};

// led[0] = IC1, first WS2805 after the MCU.
// led[31] = IC32.
static uint8_t led[MAX_ICS][NUM_CHANNELS] = {};

// Length used by CHASE/TEST/SHOW if no explicit count is supplied.
static uint8_t activeIcs = DEFAULT_ICS;

// -----------------------------------------------------------------------------
// WS2805 PIO program
// -----------------------------------------------------------------------------
//
// PIO clock = 20 MHz -> 50 ns per PIO cycle.
//
// Program timing, using side-set for the data pin:
//
//   bitloop:
//     out x, 1        side 0 [11]  -> 12 cycles low  = 600 ns
//     jmp !x, zero    side 1 [ 5]  ->  6 cycles high = 300 ns
//     jmp bitloop     side 1 [ 6]  -> +7 cycles high = 350 ns
//   zero:
//     nop             side 0 [ 6]  -> +7 cycles low  = 350 ns
//
// Thus:
//   bit 0: high 300 ns, low 350 + 600 = 950 ns
//   bit 1: high 300 + 350 = 650 ns, low 600 ns
//   total: 1.25 us/bit = 800 kbit/s
//
// TX words are used as byte containers. The byte is placed in bits 31..24.
// Auto-pull threshold is 8, so only those 8 MSB-first bits are transmitted.

static uint16_t ws2805Instructions[4];
static struct pio_program ws2805Program = {
  ws2805Instructions,
  4,
  -1
};

static PIO wsPio = pio0;
static uint wsSm = 0;
static uint wsOffset = 0;
static uint wsGpio = 25; // overwritten from digitalPinToPinName(DATA_PIN)

static void buildPioProgram() {
  ws2805Instructions[0] = (uint16_t)(
      pio_encode_out(pio_x, 1) |
      pio_encode_sideset(1, 0) |
      pio_encode_delay(11));

  ws2805Instructions[1] = (uint16_t)(
      pio_encode_jmp_not_x(3) |
      pio_encode_sideset(1, 1) |
      pio_encode_delay(5));

  ws2805Instructions[2] = (uint16_t)(
      pio_encode_jmp(0) |
      pio_encode_sideset(1, 1) |
      pio_encode_delay(6));

  ws2805Instructions[3] = (uint16_t)(
      pio_encode_nop() |
      pio_encode_sideset(1, 0) |
      pio_encode_delay(6));
}

static void initWs2805Pio() {
  buildPioProgram();

  wsGpio = (uint)digitalPinToPinName(DATA_PIN);

  pinMode(DATA_PIN, OUTPUT);
  digitalWrite(DATA_PIN, LOW);

  wsPio = pio0;
  wsSm = (uint)pio_claim_unused_sm(wsPio, true);
  wsOffset = (uint)pio_add_program(wsPio, &ws2805Program);

  pio_gpio_init(wsPio, wsGpio);
  pio_sm_set_consecutive_pindirs(wsPio, wsSm, wsGpio, 1, true);

  pio_sm_config c = pio_get_default_sm_config();
  sm_config_set_wrap(&c, wsOffset + 0, wsOffset + 3);
  sm_config_set_sideset(&c, 1, false, false);
  sm_config_set_sideset_pins(&c, wsGpio);

  // Shift MSB first. Auto-pull a fresh FIFO word after every 8 output bits.
  // This makes arbitrary 40*N-bit WS2805 frames possible.
  sm_config_set_out_shift(&c, false, true, 8);

  // 20 MHz PIO instruction clock.
  const float div = (float)clock_get_hz(clk_sys) / 20000000.0f;
  sm_config_set_clkdiv(&c, div);

  pio_sm_init(wsPio, wsSm, wsOffset, &c);
  pio_sm_set_enabled(wsPio, wsSm, true);
}

// -----------------------------------------------------------------------------
// WS2805 frame output
// -----------------------------------------------------------------------------

static void ws2805PutByte(uint8_t value) {
  // With left shifting / MSB-first output, the useful byte belongs in bits 31..24.
  pio_sm_put_blocking(wsPio, wsSm, ((uint32_t)value) << 24);
}

static void ws2805Show(uint8_t count) {
  if (count < 1) count = 1;
  if (count > MAX_ICS) count = MAX_ICS;

  // 40 bits per IC in datasheet order R, G, B, W1, W2.
  // Disable interrupts while feeding the small PIO FIFO so an unusually long
  // interrupt cannot create a >280 us reset gap inside the frame.
  // Worst case: 32 ICs = 160 bytes = 1280 bits = 1.6 ms.
  noInterrupts();

  for (uint8_t ic = 0; ic < count; ++ic) {
    ws2805PutByte(led[ic][CH_R]);
    ws2805PutByte(led[ic][CH_G]);
    ws2805PutByte(led[ic][CH_B]);
    ws2805PutByte(led[ic][CH_W1]);
    ws2805PutByte(led[ic][CH_W2]);
  }

  // FIFO empty does not necessarily mean the last byte has finished shifting.
  while (!pio_sm_is_tx_fifo_empty(wsPio, wsSm)) {
    // wait
  }

  // One byte needs 8 * 1.25 us = 10 us. 20 us gives comfortable margin.
  delayMicroseconds(20);

  interrupts();

  // WS2805 reset/latch requires LOW for >280 us.
  delayMicroseconds(320);
}

// -----------------------------------------------------------------------------
// PWM / command helpers
// -----------------------------------------------------------------------------

static uint8_t pwmCharToByte(char c, bool &ok) {
  c = (char)tolower((unsigned char)c);

  uint8_t level;
  if (c >= '0' && c <= '9') {
    level = (uint8_t)(c - '0');
  } else if (c == 'a') {
    level = 10;
  } else {
    ok = false;
    return 0;
  }

  // 0..10 -> 0..255, rounded to nearest integer.
  return (uint8_t)((level * 255u + 5u) / 10u);
}

static int channelFromText(const char *s) {
  if (!s) return -1;
  if (!strcasecmp(s, "R"))  return CH_R;
  if (!strcasecmp(s, "G"))  return CH_G;
  if (!strcasecmp(s, "B"))  return CH_B;
  if (!strcasecmp(s, "W1")) return CH_W1;
  if (!strcasecmp(s, "W2")) return CH_W2;
  return -1;
}

static const char *channelName(uint8_t ch) {
  switch (ch) {
    case CH_R:  return "R";
    case CH_G:  return "G";
    case CH_B:  return "B";
    case CH_W1: return "W1";
    case CH_W2: return "W2";
    default:    return "?";
  }
}

static bool parseIcCount(const char *s, uint8_t &count) {
  if (!s || !*s) return false;

  char *end = nullptr;
  const long value = strtol(s, &end, 10);
  if (*end != '\0' || value < 1 || value > MAX_ICS) return false;

  count = (uint8_t)value;
  return true;
}

static bool setChannelPattern(uint8_t ch, const char *pattern) {
  if (!pattern) {
    Serial.println("ERR: Muster fehlt.");
    return false;
  }

  const size_t len = strlen(pattern);
  if (len < 1 || len > MAX_ICS) {
    Serial.println("ERR: Muster muss 1..32 Zeichen lang sein.");
    return false;
  }

  const uint8_t count = (uint8_t)len;
  uint8_t newValues[MAX_ICS];

  for (uint8_t textPos = 0; textPos < count; ++textPos) {
    bool ok = true;
    const uint8_t value = pwmCharToByte(pattern[textPos], ok);
    if (!ok) {
      Serial.println("ERR: Erlaubte PWM-Zeichen sind 0..9 und a.");
      return false;
    }

    // User-facing order: left = IC<count>, right = IC1.
    const uint8_t ic = (count - 1u) - textPos;
    newValues[ic] = value;
  }

  // Only the addressed IC range is changed in RAM for this channel.
  for (uint8_t ic = 0; ic < count; ++ic) {
    led[ic][ch] = newValues[ic];
  }

  // Only these first N ICs receive a fresh 40-bit frame. ICs farther down a
  // physically longer chain do not receive new channel data and retain state.
  activeIcs = count;
  ws2805Show(count);
  return true;
}

static void allOff(uint8_t count) {
  // Keep software state coherent for exactly the range we actually send.
  for (uint8_t ic = 0; ic < count; ++ic) {
    memset(led[ic], 0, NUM_CHANNELS);
  }
  ws2805Show(count);
  activeIcs = count;
}

static void showCurrent(uint8_t count) {
  ws2805Show(count);
  activeIcs = count;
}

static void chaseOne(uint8_t ch, uint8_t count) {
  uint8_t saved[MAX_ICS][NUM_CHANNELS];
  memcpy(saved, led, sizeof(led));

  for (uint8_t ic = 0; ic < count; ++ic) {
    // Clear only the first 'count' ICs. Downstream ICs are not transmitted.
    for (uint8_t n = 0; n < count; ++n) {
      memset(led[n], 0, NUM_CHANNELS);
    }

    led[ic][ch] = 255;
    ws2805Show(count);
    delay(CHASE_DELAY_MS);
  }

  for (uint8_t n = 0; n < count; ++n) {
    memset(led[n], 0, NUM_CHANNELS);
  }
  ws2805Show(count);
  delay(80);

  memcpy(led, saved, sizeof(led));
  ws2805Show(count);
  activeIcs = count;
}

static void selfTest(uint8_t count) {
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
    Serial.print("CHASE ");
    Serial.print(channelName(ch));
    Serial.print(" / ");
    Serial.print(count);
    Serial.println(" ICs");
    chaseOne(ch, count);
  }
}

static void printHelp() {
  Serial.println();
  Serial.println("WS2805 Test - dynamisch 1..32 ICs / RGBW1W2 / D2");
  Serial.println("PWM: 0=0%, 1=10%, ... 9=90%, a=100%");
  Serial.println("Musterlaenge = Anzahl adressierter ICs");
  Serial.println("Muster: links hoechste IC-Nummer, rechts IC1");
  Serial.println();
  Serial.println("Beispiele:");
  Serial.println("  W2 aaa        -> IC1..IC3 W2 = 100%");
  Serial.println("  W1 101010101  -> Muster ueber 9 ICs");
  Serial.println("  B 00000001    -> IC1 B = 10%, IC2..IC8 = 0%");
  Serial.println("  R a9876543    -> IC8=100% ... IC1=30%");
  Serial.println();
  Serial.println("Befehle:");
  Serial.println("  R|G|B|W1|W2 <1..32 PWM-Zeichen>");
  Serial.println("  CHASE R|G|B|W1|W2 [1..32]");
  Serial.println("  TEST [1..32]");
  Serial.println("  TEST8 | TEST16 | TEST24 | TEST32");
  Serial.println("  OFF [1..32]       (ohne Zahl: alle 32 aus)");
  Serial.println("  SHOW [1..32]      (ohne Zahl: letzte Musterlaenge)");
  Serial.println("  HELP");
  Serial.println();
  Serial.print("Aktuelle Standardlaenge fuer CHASE/TEST/SHOW: ");
  Serial.println(activeIcs);
  Serial.println();
}

static bool parseNamedTestCount(const char *cmd, uint8_t &count) {
  if (!strcasecmp(cmd, "TEST8"))  { count = 8;  return true; }
  if (!strcasecmp(cmd, "TEST16")) { count = 16; return true; }
  if (!strcasecmp(cmd, "TEST24")) { count = 24; return true; }
  if (!strcasecmp(cmd, "TEST32")) { count = 32; return true; }
  return false;
}

static void processLine(char *line) {
  while (*line && isspace((unsigned char)*line)) ++line;
  if (!*line) return;

  char *saveptr = nullptr;
  char *cmd = strtok_r(line, " \t", &saveptr);
  char *arg = strtok_r(nullptr, " \t", &saveptr);
  char *arg2 = strtok_r(nullptr, " \t", &saveptr);
  char *extra = strtok_r(nullptr, " \t", &saveptr);

  if (!cmd) return;

  if (!strcasecmp(cmd, "HELP") || !strcmp(cmd, "?")) {
    printHelp();
    return;
  }

  uint8_t namedTestCount = 0;
  if (parseNamedTestCount(cmd, namedTestCount)) {
    if (arg || arg2 || extra) {
      Serial.println("ERR: TEST8/16/24/32 braucht keine weiteren Argumente.");
      return;
    }
    Serial.print("OK TEST ");
    Serial.print(namedTestCount);
    Serial.println(" start");
    selfTest(namedTestCount);
    Serial.print("OK TEST ");
    Serial.print(namedTestCount);
    Serial.println(" done");
    return;
  }

  if (!strcasecmp(cmd, "OFF")) {
    if (arg2 || extra) {
      Serial.println("ERR: Syntax OFF [1..32]");
      return;
    }

    uint8_t count = MAX_ICS;
    if (arg && !parseIcCount(arg, count)) {
      Serial.println("ERR: IC-Anzahl muss 1..32 sein.");
      return;
    }

    allOff(count);
    Serial.print("OK OFF ");
    Serial.println(count);
    return;
  }

  if (!strcasecmp(cmd, "SHOW")) {
    if (arg2 || extra) {
      Serial.println("ERR: Syntax SHOW [1..32]");
      return;
    }

    uint8_t count = activeIcs;
    if (arg && !parseIcCount(arg, count)) {
      Serial.println("ERR: IC-Anzahl muss 1..32 sein.");
      return;
    }

    showCurrent(count);
    Serial.print("OK SHOW ");
    Serial.println(count);
    return;
  }

  if (!strcasecmp(cmd, "TEST")) {
    if (arg2 || extra) {
      Serial.println("ERR: Syntax TEST [1..32]");
      return;
    }

    uint8_t count = activeIcs;
    if (arg && !parseIcCount(arg, count)) {
      Serial.println("ERR: IC-Anzahl muss 1..32 sein.");
      return;
    }

    Serial.print("OK TEST ");
    Serial.print(count);
    Serial.println(" start");
    selfTest(count);
    Serial.print("OK TEST ");
    Serial.print(count);
    Serial.println(" done");
    return;
  }

  if (!strcasecmp(cmd, "CHASE")) {
    const int ch = channelFromText(arg);
    if (ch < 0 || extra) {
      Serial.println("ERR: Syntax CHASE R|G|B|W1|W2 [1..32]");
      return;
    }

    uint8_t count = activeIcs;
    if (arg2 && !parseIcCount(arg2, count)) {
      Serial.println("ERR: IC-Anzahl muss 1..32 sein.");
      return;
    }

    Serial.print("OK CHASE ");
    Serial.print(channelName((uint8_t)ch));
    Serial.print(' ');
    Serial.println(count);
    chaseOne((uint8_t)ch, count);
    return;
  }

  const int ch = channelFromText(cmd);
  if (ch >= 0) {
    if (!arg || arg2 || extra) {
      Serial.println("ERR: Syntax <R|G|B|W1|W2> <1..32 PWM-Zeichen>");
      return;
    }

    if (setChannelPattern((uint8_t)ch, arg)) {
      Serial.print("OK ");
      Serial.print(channelName((uint8_t)ch));
      Serial.print(' ');
      Serial.print(arg);
      Serial.print("  [");
      Serial.print(strlen(arg));
      Serial.println(" ICs]");
    }
    return;
  }

  Serial.println("ERR: Unbekannter Befehl. HELP eingeben.");
}

// -----------------------------------------------------------------------------
// Arduino setup / loop
// -----------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

  const uint32_t start = millis();
  while (!Serial && (millis() - start < 2000)) {
    delay(10);
  }

  initWs2805Pio();

  // Start in a known OFF state for the maximum supported chain length.
  memset(led, 0, sizeof(led));
  ws2805Show(MAX_ICS);
  activeIcs = DEFAULT_ICS;

  Serial.println();
  Serial.println("WS2805 bereit - dynamische Kettenlaenge 1..32.");
  printHelp();
}

void loop() {
  // Max command is e.g. "W2 " + 32 chars + terminator; 80 leaves margin.
  static char line[80];
  static uint8_t pos = 0;

  while (Serial.available() > 0) {
    const char c = (char)Serial.read();

    if (c == '\r' || c == '\n') {
      if (pos > 0) {
        line[pos] = '\0';
        processLine(line);
        pos = 0;
      }
    } else if (pos < sizeof(line) - 1) {
      line[pos++] = c;
    } else {
      pos = 0;
      Serial.println("ERR: Eingabezeile zu lang.");
    }
  }
}
