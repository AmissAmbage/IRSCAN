/*
  ESP32 + MLX90640 -> GC9A01A (TFT_eSPI)  ***DIAGNOSTIC LINE-DMA***

  Diagnostic-focused firmware that makes it obvious where startup fails.
  Sequence on boot:
    1. Run a color fill test (RED, GREEN, BLUE) so the display backlight/pins
       are confirmed immediately.
    2. Run an I2C scan and report the first device address to the TFT.
    3. Attempt to initialise the MLX90640, reporting the result.
    4. If frame capture later fails the user gets a checkerboard overlay with
       live failure counters rather than a blank screen.

  Serial commands @115200:
      a            -> toggle autoscale
      rate N       -> N in {0.5,1,2,4,8,16,32,64}
      range m n    -> set fixed °C window and disable autoscale
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <TFT_eSPI.h>

// ---------------------------------------------------------------------------
// Hardware configuration
// ---------------------------------------------------------------------------

#define I2C_SDA   21
#define I2C_SCL   22
#define I2C_FREQ  400000UL          // try 1 MHz first; fall back to 400 kHz

#define MLX_RATE  MLX90640_32_HZ    // start at 32 Hz refresh

// Sensor -> display geometry
static const int SRC_W = 32;
static const int SRC_H = 24;
static const int SCALE_X = 7;
static const int SCALE_Y = 10;
static const int DST_W = SRC_W * SCALE_X;      // 224
static const int DST_H = SRC_H * SCALE_Y;      // 240
static const int X_OFFSET = (240 - DST_W) / 2; // centre on GC9A01A (240x240)

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

Adafruit_MLX90640 mlx;
TFT_eSPI tft;                                 // Pinout configured in User_Setup

static float    frameBuf[SRC_W * SRC_H];
static float    prevFrame[SRC_W * SRC_H];
static bool     havePrevFrame = false;
static uint16_t lineBuf[DST_W];
static uint16_t lut565[256];

static bool  autoscale = false;               // fixed window by default
static float fixMinC = 20.0f;
static float fixMaxC = 40.0f;

static uint32_t frameFails = 0;
static uint32_t consecutiveFails = 0;

// ---------------------------------------------------------------------------
// Utility helpers
// ---------------------------------------------------------------------------

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static inline bool badReading(float t) {
  return isnan(t) || t < -40.0f || t > 300.0f;
}

static void buildColourLUT() {
  for (int i = 0; i < 256; ++i) {
    float x = i / 255.0f;
    float r, g, b;

    if (x < 0.25f) {
      float t = x / 0.25f;
      r = 80.0f * (1.0f - t);
      g = 0.0f;
      b = 80.0f + 175.0f * t;
    } else if (x < 0.45f) {
      float t = (x - 0.25f) / 0.20f;
      r = 0.0f;
      g = 255.0f * t;
      b = 255.0f;
    } else if (x < 0.65f) {
      float t = (x - 0.45f) / 0.20f;
      r = 255.0f * t * 0.5f;
      g = 255.0f;
      b = 255.0f * (1.0f - t);
    } else if (x < 0.85f) {
      float t = (x - 0.65f) / 0.20f;
      r = 127.0f + 128.0f * t;
      g = 255.0f * (1.0f - 0.6f * t);
      b = 0.0f;
    } else {
      float t = (x - 0.85f) / 0.15f;
      r = 255.0f;
      g = 153.0f + 102.0f * t;
      b = 120.0f * t;
    }

    uint8_t R = constrain((int)lroundf(r), 0, 255);
    uint8_t G = constrain((int)lroundf(g), 0, 255);
    uint8_t B = constrain((int)lroundf(b), 0, 255);
    lut565[i] = rgb565(R, G, B);
  }
}

static void centerText(const String &text, int y, uint16_t colour = TFT_WHITE) {
  tft.setTextColor(colour, TFT_BLACK);
  tft.setTextSize(1);
  int x = (240 - text.length() * 6) / 2;
  if (x < 0) x = 0;
  tft.setCursor(x, y);
  tft.print(text);
}

static void bootColourTest() {
  tft.fillScreen(TFT_RED);   delay(200);
  tft.fillScreen(TFT_GREEN); delay(200);
  tft.fillScreen(TFT_BLUE);  delay(200);
  tft.fillScreen(TFT_BLACK);
}

static void drawCheckerWithStatus(const char *status) {
  for (int y = 0; y < 240; y += 20) {
    for (int x = 0; x < 240; x += 20) {
      uint16_t c = (((x / 20) + (y / 20)) & 1) ? TFT_DARKGREY : TFT_BLACK;
      tft.fillRect(x, y, 20, 20, c);
    }
  }

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(6, 6);
  tft.print("MLX90640 GC9A01A DIAG");
  tft.setCursor(6, 24);
  tft.print(status);
  tft.setCursor(6, 44);
  tft.printf("fails=%lu  consec=%lu", (unsigned long)frameFails,
             (unsigned long)consecutiveFails);
}

static void reportI2CScan() {
  tft.fillScreen(TFT_BLACK);
  centerText("I2C scanning...", 100);
  delay(200);

  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      found = addr;
      break;
    }
  }

  tft.fillScreen(TFT_BLACK);
  if (found) {
    char msg[32];
    snprintf(msg, sizeof(msg), "I2C device @ 0x%02X", found);
    centerText("Scan complete:", 90);
    centerText(msg, 110, TFT_YELLOW);
  } else {
    centerText("No I2C devices found", 100, TFT_RED);
  }
  delay(700);
}

// ---------------------------------------------------------------------------
// Mapping functions
// ---------------------------------------------------------------------------

static void mapFixed(const float *src, uint8_t *dst) {
  const int16_t vmin = (int16_t)lroundf(fixMinC * 100.0f);
  const int16_t vmax = (int16_t)lroundf(fixMaxC * 100.0f);
  const int32_t span = max<int32_t>(vmax - vmin, 1);

  for (int i = 0; i < SRC_W * SRC_H; ++i) {
    int32_t value;
    if (badReading(src[i])) {
      value = havePrevFrame ? (int32_t)lroundf(prevFrame[i] * 100.0f) : vmin;
    } else {
      value = (int32_t)lroundf(src[i] * 100.0f);
    }

    int32_t scaled = (value - vmin) * 255 / span;
    if (scaled < 0) scaled = 0;
    if (scaled > 255) scaled = 255;
    dst[i] = (uint8_t)scaled;
  }
}

static void mapAuto(const float *src, uint8_t *dst) {
  float mn = 1e9f;
  float mx = -1e9f;

  for (int i = 0; i < SRC_W * SRC_H; ++i) {
    float t = src[i];
    if (badReading(t)) continue;
    if (t < mn) mn = t;
    if (t > mx) mx = t;
  }

  if (mn > mx) {
    mn = 20.0f;
    mx = 40.0f;
  }

  float span = mx - mn;
  if (span < 0.5f) span = 0.5f;
  float vmin = mn - 0.05f * span;
  float vmax = mx + 0.05f * span;
  float denom = vmax - vmin;
  if (denom < 1e-6f) denom = 1e-6f;

  for (int i = 0; i < SRC_W * SRC_H; ++i) {
    float t = src[i];
    if (badReading(t)) {
      t = havePrevFrame ? prevFrame[i] : vmin;
    }

    int v = (int)lroundf((t - vmin) / denom * 255.0f);
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    dst[i] = (uint8_t)v;
  }
}

static inline void drawRowBlock(const uint8_t *rowIdx, int srcY) {
  static bool init = false;
  static uint8_t sx[DST_W];

  if (!init) {
    for (int x = 0; x < DST_W; ++x) {
      sx[x] = x / SCALE_X;
    }
    init = true;
  }

  for (int x = 0; x < DST_W; ++x) {
    lineBuf[x] = lut565[rowIdx[sx[x]]];
  }

#if defined(USE_DMA_TO_TFT)
  for (int r = 0; r < SCALE_Y; ++r) {
    tft.pushImageDMA(X_OFFSET, srcY * SCALE_Y + r, DST_W, 1, lineBuf);
    tft.dmaWait();
  }
#else
  for (int r = 0; r < SCALE_Y; ++r) {
    tft.pushImage(X_OFFSET, srcY * SCALE_Y + r, DST_W, 1, lineBuf);
  }
#endif
}

// ---------------------------------------------------------------------------
// Serial command handler
// ---------------------------------------------------------------------------

static void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("a")) {
    autoscale = !autoscale;
    Serial.print("autoscale=");
    Serial.println(autoscale ? "ON" : "OFF");
    return;
  }

  if (line.startsWith("rate")) {
    float hz = 0.0f;
    if (sscanf(line.c_str(), "rate %f", &hz) == 1) {
      mlx90640_refreshrate_t rate = MLX90640_32_HZ;
      if (hz <= 0.5f)      rate = MLX90640_0_5_HZ;
      else if (hz <= 1.0f) rate = MLX90640_1_HZ;
      else if (hz <= 2.0f) rate = MLX90640_2_HZ;
      else if (hz <= 4.0f) rate = MLX90640_4_HZ;
      else if (hz <= 8.0f) rate = MLX90640_8_HZ;
      else if (hz <= 16.0f) rate = MLX90640_16_HZ;
      else if (hz <= 32.0f) rate = MLX90640_32_HZ;
      else                  rate = MLX90640_64_HZ;

      mlx.setRefreshRate(rate);
      Serial.print("rate=");
      Serial.println(hz, 1);
    }
    return;
  }

  if (line.startsWith("range")) {
    float lo, hi;
    if (sscanf(line.c_str(), "range %f %f", &lo, &hi) == 2 && hi > lo) {
      fixMinC = lo;
      fixMaxC = hi;
      autoscale = false;
      Serial.print("range=");
      Serial.print(lo);
      Serial.print("..");
      Serial.println(hi);
    }
    return;
  }
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);

#if defined(TFT_BL)
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

  tft.init();
  tft.setSwapBytes(true);
  tft.initDMA();
  tft.setRotation(0);
  bootColourTest();

  buildColourLUT();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_FREQ);
  reportI2CScan();

  tft.fillScreen(TFT_BLACK);
  centerText("Initializing MLX90640...", 92);

  bool ok = mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire);

  tft.fillScreen(TFT_BLACK);
  if (!ok) {
    centerText("MLX90640 NOT FOUND", 100, TFT_RED);
    centerText("Check SDA=21 SCL=22", 120, TFT_YELLOW);
    while (true) {
      delay(200);
    }
  }

  mlx.setMode(MLX90640_CHESS);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX_RATE);

  centerText("MLX90640 initialized", 100, TFT_GREEN);
  delay(600);
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setCursor(6, 6);
  tft.print("MLX90640 GC9A01A DIAG");
}

void loop() {
  handleSerial();

  static uint32_t lastFPSMillis = millis();
  static uint16_t frameCounter = 0;

  bool ok = (mlx.getFrame(frameBuf) == 0);
  if (!ok) {
    frameFails++;
    consecutiveFails++;
    char msg[64];
    snprintf(msg, sizeof(msg), "getFrame FAIL i2c=%lu rate=%d",
             (unsigned long)I2C_FREQ, (int)mlx.getRefreshRate());
    drawCheckerWithStatus(msg);
    delay(10);
    return;
  }

  consecutiveFails = 0;

  static uint8_t mapBuf[SRC_W * SRC_H];
  if (autoscale) {
    mapAuto(frameBuf, mapBuf);
  } else {
    mapFixed(frameBuf, mapBuf);
  }

  for (int sy = 0; sy < SRC_H; ++sy) {
    drawRowBlock(&mapBuf[sy * SRC_W], sy);
  }

  memcpy(prevFrame, frameBuf, sizeof(frameBuf));
  havePrevFrame = true;
  frameCounter++;

  uint32_t now = millis();
  uint32_t dt = now - lastFPSMillis;
  if (dt >= 500) {
    float fps = frameCounter * 1000.0f / dt;
    frameCounter = 0;
    lastFPSMillis = now;

    tft.fillRect(130, 0, 110, 14, TFT_BLACK);
    tft.setCursor(130, 2);
    tft.printf("%4.1f FPS  F:%lu", fps, (unsigned long)frameFails);
  }
}

