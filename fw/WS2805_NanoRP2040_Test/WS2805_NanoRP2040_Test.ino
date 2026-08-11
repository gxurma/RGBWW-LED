/*
  WS2805 5-channel LED driver test firmware
  Board: Arduino Nano RP2040 Connect
  Data output: Arduino D2 (RP2040 GPIO25)

  8 WS2805 ICs in cascade, 5 bytes per IC:
      R, G, B, W1, W2
  MSB first.

  Serial commands (115200 baud, line ending CR/LF):
      R  a9876543
      G  0000000a
      B  11111111
      W1 00000000
      W2 01000000
      CHASE R
      CHASE W2
      TEST
      OFF
      SHOW
      HELP

  Pattern orientation:
      leftmost character  = LED/IC 8
      rightmost character = LED/IC 1 (first WS2805 after the MCU)

  PWM coding:
      0 =   0 %
      1 =  10 %
      2 =  20 %
      ...
      9 =  90 %
      a = 100 %

  Note: Therefore "B 11111111" turns all blue channels on at 10 %.
        Use "B aaaaaaaa" for 100 % on all eight blue channels.

  The RP2040 PIO generates the WS2805 waveform with a 20 MHz PIO clock:
      0: high 300 ns, low 950 ns
      1: high 650 ns, low 600 ns
      bit period = 1.25 us (800 kbit/s)
*/

#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>

#include "hardware/pio.h"
#include "hardware/clocks.h"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

static const uint8_t DATA_PIN = 2;        // Arduino D2
static const uint8_t NUM_ICS = 8;
static const uint8_t NUM_CHANNELS = 5;
static const uint16_t CHASE_DELAY_MS = 120;

enum Channel : uint8_t {
  CH_R = 0,
  CH_G,
  CH_B,
  CH_W1,
  CH_W2
};

// led[0] = IC/LED 1, i.e. the first WS2805 after the MCU.
// led[7] = IC/LED 8, i.e. the last WS2805 in this 8-chip chain.
static uint8_t led[NUM_ICS][NUM_CHANNELS] = {};

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
// The JMP targets below are relative to the start of the PIO program.
// pio_add_program() relocates JMP instructions automatically.

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

  // On the Nano RP2040 Connect, Arduino D2 maps to RP2040 GPIO25.
  // Using digitalPinToPinName() avoids hard-coding that mapping in the setup.
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

  // Shift MSB first, automatically pull a new 32-bit word after 32 bits.
  sm_config_set_out_shift(&c, false, true, 32);

  // 20 MHz PIO instruction clock.
  const float div = (float)clock_get_hz(clk_sys) / 20000000.0f;
  sm_config_set_clkdiv(&c, div);

  pio_sm_init(wsPio, wsSm, wsOffset, &c);
  pio_sm_set_enabled(wsPio, wsSm, true);
}

// -----------------------------------------------------------------------------
// WS2805 frame output
// -----------------------------------------------------------------------------

static void ws2805Show() {
  // 8 ICs * 5 bytes = 40 bytes = exactly 10 x 32-bit PIO words.
  uint8_t raw[NUM_ICS * NUM_CHANNELS];
  uint8_t k = 0;

  for (uint8_t ic = 0; ic < NUM_ICS; ++ic) {
    raw[k++] = led[ic][CH_R];
    raw[k++] = led[ic][CH_G];
    raw[k++] = led[ic][CH_B];
    raw[k++] = led[ic][CH_W1];
    raw[k++] = led[ic][CH_W2];
  }

  // Keep the PIO FIFO fed continuously. At 800 kbit/s the complete 320-bit
  // frame takes only 400 us, so this short IRQ-off section is acceptable for
  // a test firmware and prevents an RTOS/USB interrupt from creating a >280 us
  // gap in the middle of the frame.
  noInterrupts();

  for (uint8_t i = 0; i < sizeof(raw); i += 4) {
    const uint32_t word =
        ((uint32_t)raw[i + 0] << 24) |
        ((uint32_t)raw[i + 1] << 16) |
        ((uint32_t)raw[i + 2] << 8)  |
        ((uint32_t)raw[i + 3]);

    pio_sm_put_blocking(wsPio, wsSm, word);
  }

  // FIFO empty means the last 32-bit word may still be in the output shifter.
  while (!pio_sm_is_tx_fifo_empty(wsPio, wsSm)) {
    // wait
  }

  // One 32-bit word needs 32 * 1.25 us = 40 us maximum from the point at
  // which it is pulled into the OSR. 50 us gives margin.
  delayMicroseconds(50);

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

static bool setChannelPattern(uint8_t ch, const char *pattern) {
  if (!pattern || strlen(pattern) != NUM_ICS) {
    Serial.println("ERR: Muster muss genau 8 Zeichen lang sein (LED8 ... LED1)." );
    return false;
  }

  uint8_t newValues[NUM_ICS];

  for (uint8_t textPos = 0; textPos < NUM_ICS; ++textPos) {
    bool ok = true;
    const uint8_t value = pwmCharToByte(pattern[textPos], ok);
    if (!ok) {
      Serial.println("ERR: Erlaubte PWM-Zeichen sind 0..9 und a." );
      return false;
    }

    // User-facing order is LED8 ... LED1.
    const uint8_t ic = (NUM_ICS - 1u) - textPos;
    newValues[ic] = value;
  }

  for (uint8_t ic = 0; ic < NUM_ICS; ++ic) {
    led[ic][ch] = newValues[ic];
  }

  ws2805Show();
  return true;
}

static void allOff() {
  memset(led, 0, sizeof(led));
  ws2805Show();
}

static void chaseOne(uint8_t ch) {
  uint8_t saved[NUM_ICS][NUM_CHANNELS];
  memcpy(saved, led, sizeof(led));

  for (uint8_t ic = 0; ic < NUM_ICS; ++ic) {
    memset(led, 0, sizeof(led));
    led[ic][ch] = 255;
    ws2805Show();
    delay(CHASE_DELAY_MS);
  }

  memset(led, 0, sizeof(led));
  ws2805Show();
  delay(80);

  memcpy(led, saved, sizeof(led));
  ws2805Show();
}

static void selfTest() {
  for (uint8_t ch = 0; ch < NUM_CHANNELS; ++ch) {
    Serial.print("CHASE ");
    Serial.println(channelName(ch));
    chaseOne(ch);
  }
}

static void printHelp() {
  Serial.println();
  Serial.println("WS2805 Test - 8 ICs / RGBW1W2 / D2");
  Serial.println("PWM: 0=0%, 1=10%, ... 9=90%, a=100%");
  Serial.println("Muster: links LED8, rechts LED1");
  Serial.println();
  Serial.println("Befehle:");
  Serial.println("  R  a9876543");
  Serial.println("  G  0000000a");
  Serial.println("  B  11111111");
  Serial.println("  W1 00000000");
  Serial.println("  W2 01000000");
  Serial.println("  CHASE R|G|B|W1|W2");
  Serial.println("  TEST");
  Serial.println("  OFF");
  Serial.println("  SHOW");
  Serial.println("  HELP");
  Serial.println();
}

static void processLine(char *line) {
  // Skip leading spaces.
  while (*line && isspace((unsigned char)*line)) ++line;
  if (!*line) return;

  char *saveptr = nullptr;
  char *cmd = strtok_r(line, " \t", &saveptr);
  char *arg = strtok_r(nullptr, " \t", &saveptr);
  char *extra = strtok_r(nullptr, " \t", &saveptr);

  if (!cmd) return;

  if (!strcasecmp(cmd, "HELP") || !strcmp(cmd, "?")) {
    printHelp();
    return;
  }

  if (!strcasecmp(cmd, "OFF")) {
    allOff();
    Serial.println("OK OFF");
    return;
  }

  if (!strcasecmp(cmd, "SHOW")) {
    ws2805Show();
    Serial.println("OK SHOW");
    return;
  }

  if (!strcasecmp(cmd, "TEST")) {
    Serial.println("OK TEST start");
    selfTest();
    Serial.println("OK TEST done");
    return;
  }

  if (!strcasecmp(cmd, "CHASE")) {
    const int ch = channelFromText(arg);
    if (ch < 0 || extra) {
      Serial.println("ERR: Syntax CHASE R|G|B|W1|W2");
      return;
    }
    Serial.print("OK CHASE ");
    Serial.println(channelName((uint8_t)ch));
    chaseOne((uint8_t)ch);
    return;
  }

  const int ch = channelFromText(cmd);
  if (ch >= 0) {
    if (!arg || extra) {
      Serial.println("ERR: Syntax <R|G|B|W1|W2> <8 PWM-Zeichen>");
      return;
    }

    if (setChannelPattern((uint8_t)ch, arg)) {
      Serial.print("OK ");
      Serial.print(channelName((uint8_t)ch));
      Serial.print(' ');
      Serial.println(arg);
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

  // Start in a known OFF state.
  memset(led, 0, sizeof(led));
  ws2805Show();

  Serial.println();
  Serial.println("WS2805 bereit.");
  printHelp();
}

void loop() {
  static char line[64];
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
