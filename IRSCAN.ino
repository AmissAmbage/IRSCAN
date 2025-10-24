#include <Arduino.h>
#include <type_traits>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Adafruit_MLX90640.h>

// Display pin definitions
constexpr int TFT_MOSI = 23;
constexpr int TFT_SCLK = 18;
constexpr int TFT_CS   = 16;
constexpr int TFT_DC   = 2;
constexpr int TFT_RST  = 4;
constexpr int TFT_BL   = 17;

// Imager pin definitions
constexpr int I2C_SCL = 22;
constexpr int I2C_SDA = 21;

constexpr uint16_t SCREEN_WIDTH  = 240;
constexpr uint16_t SCREEN_HEIGHT = 240;
constexpr uint16_t IMAGE_WIDTH   = 32;
constexpr uint16_t IMAGE_HEIGHT  = 24;

struct I2CProfile {
  uint32_t frequencyHz;
  mlx90640_refreshrate refreshRate;
  const char *label;
};

constexpr I2CProfile kI2CProfiles[] = {
    {1000000, MLX90640_32_HZ, "32Hz@1.0M"},
    {800000, MLX90640_32_HZ, "32Hz@0.8M"},
    {400000, MLX90640_16_HZ, "16Hz@0.4M"},
    {200000, MLX90640_8_HZ,  "8Hz@0.2M"},
};

constexpr size_t kProfileCount = sizeof(kI2CProfiles) / sizeof(kI2CProfiles[0]);

uint8_t currentProfileIndex = 0;
uint8_t consecutiveI2CErrors = 0;
uint16_t consecutiveGoodFrames = 0;
unsigned long lastProfileChangeMillis = 0;
uint16_t consecutiveDataNotReadyFailures = 0;

Adafruit_GC9A01A display = Adafruit_GC9A01A(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
Adafruit_MLX90640 mlx;

float frameBuffer[IMAGE_WIDTH * IMAGE_HEIGHT];
String debugMessage = "Boot";
bool haveValidFrame = false;

unsigned long lastFrameMillis = 0;
float currentFPS = 0.0f;

bool overlayDirty = true;
unsigned long lastOverlayDraw = 0;
constexpr bool kShowDebugMessage = false;

bool overlayEnabled = true;
bool overlayCleared = false;

uint32_t lastFrameDurationMicros = 0;
uint32_t frameDurationAccumMicros = 0;
uint16_t frameDurationSamples = 0;

template <typename Sensor>
bool setRefreshRateCompatImpl(Sensor &sensor, mlx90640_refreshrate rate, std::false_type) {
  return sensor.setRefreshRate(rate);
}

template <typename Sensor>
bool setRefreshRateCompatImpl(Sensor &sensor, mlx90640_refreshrate rate, std::true_type) {
  sensor.setRefreshRate(rate);
  return true;
}

template <typename Sensor>
bool setRefreshRateCompat(Sensor &sensor, mlx90640_refreshrate rate) {
  using ReturnType = decltype(sensor.setRefreshRate(rate));
  return setRefreshRateCompatImpl(sensor, rate, typename std::is_void<ReturnType>::type{});
}

static void drawOverlay() {
  overlayDirty = false;

  static int16_t lastRectX = -1;
  static int16_t lastRectY = -1;
  static uint16_t lastRectW = 0;
  static uint16_t lastRectH = 0;

  if (!overlayEnabled) {
    if (!overlayCleared) {
      if (lastRectW > 0) {
        display.fillRect(lastRectX, lastRectY, lastRectW, lastRectH, 0x0000);
        lastRectW = 0;
        lastRectH = 0;
      }
      display.fillScreen(0x0000);
      overlayCleared = true;
    }
    return;
  }

  overlayCleared = false;

  char fpsBuffer[24];
  snprintf(fpsBuffer, sizeof(fpsBuffer), "FPS: %.2f", currentFPS);

  display.setTextSize(2);
  display.setTextWrap(false);
  display.setTextColor(0xFFFF, 0x0000);

  if (lastRectW > 0) {
    display.fillRect(lastRectX, lastRectY, lastRectW, lastRectH, 0x0000);
  }

  int16_t x, y;
  uint16_t w, h;
  display.getTextBounds(fpsBuffer, 0, 0, &x, &y, &w, &h);
  int16_t fpsX = (SCREEN_WIDTH - w) / 2;
  int16_t fpsY = (SCREEN_HEIGHT - h) / 2;

  int16_t rectX = fpsX - 6;
  int16_t rectY = fpsY - 6;
  uint16_t rectW = w + 12;
  uint16_t rectH = h + 12;

  display.fillRect(rectX, rectY, rectW, rectH, 0x0000);
  display.setCursor(fpsX, fpsY);
  display.print(fpsBuffer);

  lastRectX = rectX;
  lastRectY = rectY;
  lastRectW = rectW;
  lastRectH = rectH;

  if (kShowDebugMessage) {
    display.setTextSize(1);
    display.getTextBounds(debugMessage.c_str(), 0, 0, &x, &y, &w, &h);
    int16_t dbgX = (SCREEN_WIDTH - w) / 2;
    int16_t dbgY = fpsY + rectH / 2 + 10;
    display.fillRect(dbgX - 2, dbgY - 2, w + 4, h + 4, 0x0000);
    display.setCursor(dbgX, dbgY);
    display.print(debugMessage);
  }
}

static void updateFPS() {
  static uint32_t frameCount = 0;
  static unsigned long windowStart = 0;

  unsigned long now = millis();
  if (windowStart == 0) {
    windowStart = now;
  }

  ++frameCount;
  lastFrameMillis = now;

  unsigned long elapsed = now - windowStart;
  if (elapsed >= 500) {
    if (elapsed > 2000) {
      frameCount = 1;
      windowStart = now;
      return;
    }
    float windowFPS = (frameCount * 1000.0f) / static_cast<float>(elapsed);
    if (frameDurationSamples > 0) {
      float avgMicros = static_cast<float>(frameDurationAccumMicros) /
                        static_cast<float>(frameDurationSamples);
      if (avgMicros > 1.0f) {
        currentFPS = 1000000.0f / avgMicros;
      } else {
        currentFPS = windowFPS;
      }
    } else {
      currentFPS = windowFPS;
    }
    frameDurationAccumMicros = 0;
    frameDurationSamples = 0;
    frameCount = 0;
    windowStart = now;
    overlayDirty = true;
  }
}

static void setStreamingMessage() {
  debugMessage = String("Stream ") + kI2CProfiles[currentProfileIndex].label;
}

static void applyI2CProfile(uint8_t index, bool announceChange = true) {
  if (index >= kProfileCount) {
    index = kProfileCount - 1;
  }
  currentProfileIndex = index;
  const I2CProfile &profile = kI2CProfiles[currentProfileIndex];

  Wire.setClock(profile.frequencyHz);
  if (!setRefreshRateCompat(mlx, profile.refreshRate)) {
    Serial.println("Failed to update MLX refresh rate");
  }
  lastProfileChangeMillis = millis();
  consecutiveI2CErrors = 0;
  consecutiveGoodFrames = 0;

  if (announceChange) {
    Serial.print("MLX profile -> ");
    Serial.println(profile.label);
    if (haveValidFrame) {
      setStreamingMessage();
    }
  }
}

static bool initializeSensor() {
  for (uint8_t idx = 0; idx < kProfileCount; ++idx) {
    const I2CProfile &profile = kI2CProfiles[idx];
    Wire.setClock(profile.frequencyHz);

    Serial.print("MLX init try ");
    Serial.println(profile.label);

    if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
      Serial.println("  -> not found");
      delay(30);
      continue;
    }

    mlx.setMode(MLX90640_INTERLEAVED);
    mlx.setResolution(MLX90640_ADC_18BIT);
    applyI2CProfile(idx, false);

    Serial.print("MLX init -> ");
    Serial.println(kI2CProfiles[currentProfileIndex].label);
    return true;
  }

  return false;
}

void setup() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  Serial.begin(115200);

  display.begin();
  display.fillScreen(0x0000);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!initializeSensor()) {
    debugMessage = "MLX init fail";
    Serial.println("Failed to find MLX90640 sensor at any profile.");
    currentFPS = 0.0f;
    while (true) {
      drawOverlay();
      lastOverlayDraw = millis();
      delay(120);
    }
  }

  debugMessage = "Init OK";
  drawOverlay();
  lastOverlayDraw = millis();
}

#ifdef MLX90640_DATA_NOT_READY
constexpr int16_t MLX_STATUS_DATA_NOT_READY = MLX90640_DATA_NOT_READY;
#else
constexpr int16_t MLX_STATUS_DATA_NOT_READY = -8;
#endif

static String describeMlxError(int status) {
  switch (status) {
#ifdef MLX90640_NO_ERROR
    case MLX90640_NO_ERROR:
      return "OK";
#endif
#ifdef MLX90640_I2C_READ_ERROR
    case MLX90640_I2C_READ_ERROR:
      return "I2C read";
#endif
#ifdef MLX90640_I2C_WRITE_ERROR
    case MLX90640_I2C_WRITE_ERROR:
      return "I2C write";
#endif
#ifdef MLX90640_INVALID_TEMPERATURE
    case MLX90640_INVALID_TEMPERATURE:
      return "Temp range";
#endif
#ifdef MLX90640_INVALID_PARAMETR
    case MLX90640_INVALID_PARAMETR:
      return "Bad param";
#endif
    case MLX_STATUS_DATA_NOT_READY:
      return "Not ready";
    case 0:
      return "OK";
    case -1:
      return "I2C fault";
    default:
      return String("Err ") + status;
  }
}

static bool acquireFrame(float *buffer) {
  constexpr uint8_t kMaxAttempts = 12;
  constexpr uint16_t kDataNotReadyDelayMs = 6;  // back off a little so conversions can complete
  const unsigned long startWait = millis();
  const uint32_t startMicros = micros();
  bool sawDataNotReady = false;

  for (uint8_t attempt = 0; attempt < kMaxAttempts; ++attempt) {
    int status = mlx.getFrame(buffer);
    if (status == 0) {
      lastFrameDurationMicros = micros() - startMicros;
      frameDurationAccumMicros += lastFrameDurationMicros;
      ++frameDurationSamples;
      consecutiveI2CErrors = 0;
      consecutiveDataNotReadyFailures = 0;
      ++consecutiveGoodFrames;

      if (consecutiveGoodFrames > 180 && currentProfileIndex > 0 &&
          millis() - lastProfileChangeMillis > 2000) {
        applyI2CProfile(currentProfileIndex - 1);
        Serial.println("Attempting faster MLX profile");
      }

      haveValidFrame = true;
      if (!debugMessage.startsWith("Stream ")) {
        setStreamingMessage();
      }
      return true;
    }

    if (status == MLX_STATUS_DATA_NOT_READY) {
      sawDataNotReady = true;
      delay(kDataNotReadyDelayMs);
      continue;
    }

    consecutiveGoodFrames = 0;
    consecutiveDataNotReadyFailures = 0;

    if (status == -1) {
      ++consecutiveI2CErrors;
      if (consecutiveI2CErrors >= 3 && currentProfileIndex + 1 < kProfileCount &&
          millis() - lastProfileChangeMillis > 500) {
        applyI2CProfile(currentProfileIndex + 1);
        debugMessage = String("I2C->") + kI2CProfiles[currentProfileIndex].label + " (" + status + ")";
      } else {
        debugMessage = String("MLX I2C(") + status + ")";
      }
    } else {
      consecutiveI2CErrors = 0;
      debugMessage = String("MLX ") + describeMlxError(status) + " (" + status + ")";
    }

    Serial.print("MLX90640 read error: ");
    Serial.println(status);
    delay(5);
    return false;
  }

  consecutiveGoodFrames = 0;
  if (sawDataNotReady) {
    ++consecutiveDataNotReadyFailures;
    if (consecutiveDataNotReadyFailures >= 6 && currentProfileIndex + 1 < kProfileCount &&
        millis() - lastProfileChangeMillis > 500) {
      applyI2CProfile(currentProfileIndex + 1);
      debugMessage = String("Slow->") + kI2CProfiles[currentProfileIndex].label;
    }
  }

  if (!haveValidFrame) {
    if (millis() - startWait > 150 && debugMessage != "Waiting") {
      debugMessage = "Waiting";
    }
  }
  return false;
}

static void processSerialCommand(const String &line) {
  if (line.equalsIgnoreCase("overlay off")) {
    if (overlayEnabled) {
      overlayEnabled = false;
      overlayDirty = true;
      Serial.println("overlay=OFF");
    }
  } else if (line.equalsIgnoreCase("overlay on")) {
    if (!overlayEnabled) {
      overlayEnabled = true;
      overlayDirty = true;
      Serial.println("overlay=ON");
    }
  } else if (line.equalsIgnoreCase("overlay")) {
    overlayEnabled = !overlayEnabled;
    overlayDirty = true;
    Serial.print("overlay=");
    Serial.println(overlayEnabled ? "ON" : "OFF");
  } else if (line.equalsIgnoreCase("status")) {
    Serial.print("fps=");
    Serial.println(currentFPS, 3);
    Serial.print("overlay=");
    Serial.println(overlayEnabled ? "ON" : "OFF");
    Serial.print("profile=");
    Serial.println(kI2CProfiles[currentProfileIndex].label);
    Serial.print("frame_us=");
    Serial.println(lastFrameDurationMicros);
  }
}

static void handleSerialCommands() {
  static String buffer;

  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      buffer.trim();
      if (buffer.length() > 0) {
        processSerialCommand(buffer);
      }
      buffer = "";
    } else {
      if (buffer.length() < 64) {
        buffer += c;
      }
    }
  }
}

void loop() {
  handleSerialCommands();
  bool frameReady = acquireFrame(frameBuffer);
  unsigned long now = millis();

  if (frameReady) {
    updateFPS();
  } else {
    if (haveValidFrame) {
      unsigned long sinceLast = now - lastFrameMillis;
      if (sinceLast > 500 && currentFPS > 0.0f) {
        currentFPS = 0.0f;
        overlayDirty = true;
      }
      if (sinceLast > 250 && debugMessage != "Waiting") {
        debugMessage = "Waiting";
      }
    } else if (currentFPS != 0.0f) {
      currentFPS = 0.0f;
      overlayDirty = true;
    }

    delay(2);
  }

  if (overlayDirty && (now - lastOverlayDraw >= 100)) {
    drawOverlay();
    lastOverlayDraw = now;
  }
}
