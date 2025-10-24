#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MLX90640.h>
#include <TFT_eSPI.h>
#include <float.h>

// ----------------------------- Configuration ------------------------------
namespace {
constexpr uint8_t kI2cSda = 21;
constexpr uint8_t kI2cScl = 22;
constexpr uint32_t kI2cClock = 400000UL;  // 400 kHz default
constexpr mlx90640_refreshrate_t kDefaultRefresh = MLX90640_32_HZ;

constexpr int kSrcWidth = 32;
constexpr int kSrcHeight = 24;
constexpr int kScaleX = 7;
constexpr int kScaleY = 10;
constexpr int kDstWidth = kSrcWidth * kScaleX;  // 224 px
constexpr int kScreenWidth = 240;
constexpr int kScreenHeight = 240;
constexpr int kXOffset = (kScreenWidth - kDstWidth) / 2;

constexpr float kDefaultMinC = 20.0f;
constexpr float kDefaultMaxC = 40.0f;
}  // namespace

// ------------------------------ Peripherals --------------------------------
Adafruit_MLX90640 gMlx;
TFT_eSPI gTft;

// --------------------------- Framebook Keeping -----------------------------
static float gFrame[kSrcWidth * kSrcHeight];
static float gPrevFrame[kSrcWidth * kSrcHeight];
static bool gHavePrev = false;
static uint16_t gLineBuffer[kDstWidth];
static uint16_t gColorLut[256];

static bool gAutoscale = false;
static float gFixedMin = kDefaultMinC;
static float gFixedMax = kDefaultMaxC;

static uint32_t gFrameFails = 0;
static uint32_t gConsecutiveFails = 0;

// ------------------------------- Utilities ---------------------------------
static uint16_t Rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static bool BadReading(float t) {
  return isnan(t) || t < -40.0f || t > 300.0f;
}

static void BuildColorLut() {
  for (int i = 0; i < 256; ++i) {
    const float x = static_cast<float>(i) / 255.0f;
    float r = 0.0f, g = 0.0f, b = 0.0f;
    if (x < 0.25f) {
      const float t = x / 0.25f;
      r = 80.0f * (1.0f - t);
      g = 0.0f;
      b = 80.0f + 175.0f * t;
    } else if (x < 0.45f) {
      const float t = (x - 0.25f) / 0.20f;
      r = 0.0f;
      g = 255.0f * t;
      b = 255.0f;
    } else if (x < 0.65f) {
      const float t = (x - 0.45f) / 0.20f;
      r = 255.0f * 0.5f * t;
      g = 255.0f;
      b = 255.0f * (1.0f - t);
    } else if (x < 0.85f) {
      const float t = (x - 0.65f) / 0.20f;
      r = 127.0f + 128.0f * t;
      g = 255.0f * (1.0f - 0.6f * t);
      b = 0.0f;
    } else {
      const float t = (x - 0.85f) / 0.15f;
      r = 255.0f;
      g = 153.0f + 102.0f * t;
      b = 120.0f * t;
    }
    uint8_t R = static_cast<uint8_t>(constrain(r, 0.0f, 255.0f));
    uint8_t G = static_cast<uint8_t>(constrain(g, 0.0f, 255.0f));
    uint8_t B = static_cast<uint8_t>(constrain(b, 0.0f, 255.0f));
    gColorLut[i] = Rgb565(R, G, B);
  }
}

static void CenterText(const String &text, int y, uint16_t color = TFT_WHITE) {
  gTft.setTextColor(color, TFT_BLACK);
  gTft.setTextSize(1);
  const int x = max(0, (kScreenWidth - text.length() * 6) / 2);
  gTft.setCursor(x, y);
  gTft.print(text);
}

static void BootColorTest() {
  gTft.fillScreen(TFT_RED);
  delay(200);
  gTft.fillScreen(TFT_GREEN);
  delay(200);
  gTft.fillScreen(TFT_BLUE);
  delay(200);
  gTft.fillScreen(TFT_BLACK);
}

static void DrawCheckerWithStatus(const char *status) {
  for (int y = 0; y < kScreenHeight; y += 20) {
    for (int x = 0; x < kScreenWidth; x += 20) {
      const uint16_t c = (((x / 20) + (y / 20)) & 1) ? TFT_DARKGREY : TFT_BLACK;
      gTft.fillRect(x, y, 20, 20, c);
    }
  }
  gTft.setTextColor(TFT_WHITE, TFT_BLACK);
  gTft.setCursor(6, 6);
  gTft.print("MLX90640 GC9A01A DIAG");
  gTft.setCursor(6, 24);
  gTft.print(status);
  gTft.setCursor(6, 44);
  gTft.printf("fails=%lu  consec=%lu", static_cast<unsigned long>(gFrameFails),
              static_cast<unsigned long>(gConsecutiveFails));
}

static void I2cScanReport() {
  gTft.fillScreen(TFT_BLACK);
  CenterText("I2C scanning...", 100);
  delay(200);

  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      found = addr;
      break;
    }
  }

  gTft.fillScreen(TFT_BLACK);
  if (found) {
    char msg[48];
    snprintf(msg, sizeof(msg), "I2C device @ 0x%02X", found);
    CenterText("Scan complete:", 90);
    CenterText(msg, 110, TFT_YELLOW);
  } else {
    CenterText("No I2C devices found", 100, TFT_RED);
  }
  delay(700);
}

// ----------------------------- Mapping -------------------------------------
static void MapFixed(const float *src, uint8_t *dst) {
  const int16_t minC = static_cast<int16_t>(lroundf(gFixedMin * 100.0f));
  const int16_t maxC = static_cast<int16_t>(lroundf(gFixedMax * 100.0f));
  const int32_t span = max<int32_t>(1, maxC - minC);
  for (int i = 0; i < kSrcWidth * kSrcHeight; ++i) {
    float t = src[i];
    if (BadReading(t)) {
      t = gHavePrev ? gPrevFrame[i] : gFixedMin;
    }
    int32_t tempHundredths = static_cast<int32_t>(lroundf(t * 100.0f));
    int32_t v = (tempHundredths - minC) * 255 / span;
    v = constrain(v, 0, 255);
    dst[i] = static_cast<uint8_t>(v);
  }
}

static void MapAuto(const float *src, uint8_t *dst) {
  float minVal = FLT_MAX;
  float maxVal = -FLT_MAX;
  for (int i = 0; i < kSrcWidth * kSrcHeight; ++i) {
    float t = src[i];
    if (!BadReading(t)) {
      if (t < minVal) minVal = t;
      if (t > maxVal) maxVal = t;
    }
  }
  if (!(minVal < maxVal)) {
    minVal -= 0.5f;
    maxVal += 0.5f;
  }
  const float span = maxVal - minVal;
  const float vMin = minVal - 0.05f * span;
  const float vMax = maxVal + 0.05f * span;
  const float denom = max(vMax - vMin, 1e-3f);

  for (int i = 0; i < kSrcWidth * kSrcHeight; ++i) {
    float t = src[i];
    if (BadReading(t)) {
      t = gHavePrev ? gPrevFrame[i] : vMin;
    }
    int v = static_cast<int>(lroundf((t - vMin) / denom * 255.0f));
    dst[i] = static_cast<uint8_t>(constrain(v, 0, 255));
  }
}

static void DrawRowBlock(const uint8_t *row, int sy) {
  static bool init = false;
  static uint8_t mapX[kDstWidth];
  if (!init) {
    for (int x = 0; x < kDstWidth; ++x) {
      mapX[x] = x / kScaleX;
    }
    init = true;
  }

  for (int x = 0; x < kDstWidth; ++x) {
    gLineBuffer[x] = gColorLut[row[mapX[x]]];
  }

#if defined(USE_DMA_TO_TFT)
  for (int r = 0; r < kScaleY; ++r) {
    gTft.pushImageDMA(kXOffset, sy * kScaleY + r, kDstWidth, 1, gLineBuffer);
    gTft.dmaWait();
  }
#else
  gTft.setAddrWindow(kXOffset, sy * kScaleY, kDstWidth, kScaleY);
  for (int r = 0; r < kScaleY; ++r) {
    gTft.pushImage(kXOffset, sy * kScaleY + r, kDstWidth, 1, gLineBuffer);
  }
#endif
}

// ---------------------------- Serial Commands ------------------------------
static void HandleSerial() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.equalsIgnoreCase("a")) {
    gAutoscale = !gAutoscale;
    Serial.print("autoscale=");
    Serial.println(gAutoscale ? "ON" : "OFF");
  } else if (input.startsWith("rate")) {
    float hz = 0.0f;
    if (sscanf(input.c_str(), "rate %f", &hz) == 1) {
      mlx90640_refreshrate_t rate = MLX90640_32_HZ;
      if (hz <= 0.5f) rate = MLX90640_0_5_HZ;
      else if (hz <= 1.0f) rate = MLX90640_1_HZ;
      else if (hz <= 2.0f) rate = MLX90640_2_HZ;
      else if (hz <= 4.0f) rate = MLX90640_4_HZ;
      else if (hz <= 8.0f) rate = MLX90640_8_HZ;
      else if (hz <= 16.0f) rate = MLX90640_16_HZ;
      else if (hz <= 32.0f) rate = MLX90640_32_HZ;
      else rate = MLX90640_64_HZ;
      gMlx.setRefreshRate(rate);
      Serial.print("rate=");
      Serial.println(hz, 1);
    }
  } else if (input.startsWith("range")) {
    float a = 0.0f, b = 0.0f;
    if (sscanf(input.c_str(), "range %f %f", &a, &b) == 2 && b > a) {
      gFixedMin = a;
      gFixedMax = b;
      gAutoscale = false;
      Serial.print("range=");
      Serial.print(a);
      Serial.print("..");
      Serial.println(b);
    }
  }
}

// ------------------------------- Setup -------------------------------------
void setup() {
  Serial.begin(115200);

#if defined(TFT_BL)
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif

  gTft.init();
  gTft.initDMA();
  gTft.setSwapBytes(true);
  gTft.setRotation(0);
  BootColorTest();
  BuildColorLut();

  Wire.begin(kI2cSda, kI2cScl);
  Wire.setClock(kI2cClock);

  I2cScanReport();

  gTft.fillScreen(TFT_BLACK);
  CenterText("Initializing MLX90640...", 92);

  bool ok = gMlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire);
  gTft.fillScreen(TFT_BLACK);
  if (!ok) {
    CenterText("MLX90640 NOT FOUND", 100, TFT_RED);
    CenterText("Check SDA=21 SCL=22", 120, TFT_YELLOW);
    while (true) {
      delay(200);
    }
  }

  gMlx.setMode(MLX90640_CHESS);
  gMlx.setResolution(MLX90640_ADC_18BIT);
  gMlx.setRefreshRate(kDefaultRefresh);

  CenterText("MLX90640 initialized", 100, TFT_GREEN);
  delay(600);
  gTft.fillScreen(TFT_BLACK);
  gTft.setTextColor(TFT_WHITE, TFT_BLACK);
  gTft.setCursor(6, 6);
  gTft.print("MLX90640 GC9A01A DIAG");
}

// -------------------------------- Loop -------------------------------------
void loop() {
  HandleSerial();

  static uint32_t t0 = millis();
  static uint16_t frameCount = 0;

  bool ok = gMlx.getFrame(gFrame) == 0;
  if (!ok) {
    ++gFrameFails;
    ++gConsecutiveFails;
    char msg[64];
    snprintf(msg, sizeof(msg), "getFrame FAIL  i2c=%lu  rate=%d",
             static_cast<unsigned long>(kI2cClock),
             static_cast<int>(gMlx.getRefreshRate()));
    DrawCheckerWithStatus(msg);
    delay(10);
    return;
  }

  gConsecutiveFails = 0;

  static uint8_t indexFrame[kSrcWidth * kSrcHeight];
  if (gAutoscale) {
    MapAuto(gFrame, indexFrame);
  } else {
    MapFixed(gFrame, indexFrame);
  }

  for (int sy = 0; sy < kSrcHeight; ++sy) {
    DrawRowBlock(&indexFrame[sy * kSrcWidth], sy);
  }

  memcpy(gPrevFrame, gFrame, sizeof(gFrame));
  gHavePrev = true;
  ++frameCount;

  const uint32_t dt = millis() - t0;
  if (dt >= 500) {
    const float fps = frameCount * 1000.0f / static_cast<float>(dt);
    frameCount = 0;
    t0 = millis();
    gTft.fillRect(130, 0, 110, 14, TFT_BLACK);
    gTft.setCursor(130, 2);
    gTft.printf("%4.1f FPS  F:%lu", fps, static_cast<unsigned long>(gFrameFails));
  }
}
