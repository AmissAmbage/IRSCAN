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

struct FrequencyProfile {
  uint32_t frequencyHz;
  const char *label;
};

struct RefreshProfile {
  mlx90640_refreshrate refreshRate;
  const char *label;
  float nominalHz;
};

constexpr FrequencyProfile kFrequencies[] = {
    {200000,  "0.2MHz"},
    {400000,  "0.4MHz"},
    {600000,  "0.6MHz"},
    {800000,  "0.8MHz"},
    {1000000, "1.0MHz"},
};

constexpr RefreshProfile kRefreshRates[] = {
    {MLX90640_0_5_HZ,  "0.5Hz", 0.5f},
    {MLX90640_1_HZ,    "1Hz",   1.0f},
    {MLX90640_2_HZ,    "2Hz",   2.0f},
    {MLX90640_4_HZ,    "4Hz",   4.0f},
    {MLX90640_8_HZ,    "8Hz",   8.0f},
    {MLX90640_16_HZ,   "16Hz",  16.0f},
    {MLX90640_32_HZ,   "32Hz",  32.0f},
#ifdef MLX90640_64_HZ
    {MLX90640_64_HZ,   "64Hz",  64.0f},
#endif
};

constexpr size_t kFrequencyCount = sizeof(kFrequencies) / sizeof(kFrequencies[0]);
constexpr size_t kRefreshCount = sizeof(kRefreshRates) / sizeof(kRefreshRates[0]);

size_t currentFrequencyIndex = 0;
size_t currentRefreshIndex = 0;
bool combinationActive = false;
bool benchmarkComplete = false;
bool topResultsShown = false;

Adafruit_GC9A01A display = Adafruit_GC9A01A(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
Adafruit_MLX90640 mlx;

float frameBuffer[IMAGE_WIDTH * IMAGE_HEIGHT];
String debugMessage = "Boot";
bool showDebugMessage = true;

unsigned long combinationStartMillis = 0;
uint32_t combinationStartMicros = 0;
uint32_t combinationFrameCount = 0;

unsigned long lastFrameMillis = 0;
float currentFPS = 0.0f;

bool overlayDirty = true;
unsigned long lastOverlayDraw = 0;

bool overlayEnabled = true;
bool overlayCleared = false;

uint32_t lastFrameDurationMicros = 0;
uint32_t frameDurationAccumMicros = 0;
uint16_t frameDurationSamples = 0;

struct BenchmarkResult {
  float fps = 0.0f;
  uint32_t frames = 0;
  bool completed = false;
};

BenchmarkResult benchmarkResults[kRefreshCount][kFrequencyCount];

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

  if (showDebugMessage && debugMessage.length() > 0) {
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

static bool applyBenchmarkConfig(size_t refreshIndex, size_t frequencyIndex) {
  if (refreshIndex >= kRefreshCount || frequencyIndex >= kFrequencyCount) {
    return false;
  }

  const RefreshProfile &refresh = kRefreshRates[refreshIndex];
  const FrequencyProfile &freq = kFrequencies[frequencyIndex];

  Wire.setClock(freq.frequencyHz);
  if (!setRefreshRateCompat(mlx, refresh.refreshRate)) {
    Serial.println("Failed to set MLX90640 refresh rate");
    return false;
  }

  char label[48];
  snprintf(label, sizeof(label), "%s @ %s", refresh.label, freq.label);
  debugMessage = label;
  overlayDirty = true;
  return true;
}

static bool initializeSensor() {
  for (size_t freqIdx = 0; freqIdx < kFrequencyCount; ++freqIdx) {
    const FrequencyProfile &freq = kFrequencies[freqIdx];
    Wire.setClock(freq.frequencyHz);

    Serial.print("MLX init try ");
    Serial.println(freq.label);

    if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
      Serial.println("  -> not found");
      delay(30);
      continue;
    }

    mlx.setMode(MLX90640_INTERLEAVED);
    mlx.setResolution(MLX90640_ADC_18BIT);
    currentRefreshIndex = 0;
    currentFrequencyIndex = 0;
    if (!applyBenchmarkConfig(currentRefreshIndex, currentFrequencyIndex)) {
      Serial.println("Failed to apply initial benchmark config");
      return false;
    }
    Serial.print("MLX init -> ");
    Serial.print(kRefreshRates[currentRefreshIndex].label);
    Serial.print(" @ ");
    Serial.println(kFrequencies[currentFrequencyIndex].label);
    return true;
  }

  return false;
}

static String makeCombinationLabel(size_t refreshIndex, size_t frequencyIndex) {
  String label;
  if (refreshIndex < kRefreshCount && frequencyIndex < kFrequencyCount) {
    label.reserve(32);
    label += kRefreshRates[refreshIndex].label;
    label += " @ ";
    label += kFrequencies[frequencyIndex].label;
  } else {
    label = benchmarkComplete ? String("complete") : String("pending");
  }
  return label;
}

static bool startCombinationWindow() {
  while (currentRefreshIndex < kRefreshCount) {
    if (applyBenchmarkConfig(currentRefreshIndex, currentFrequencyIndex)) {
      combinationStartMillis = millis();
      combinationStartMicros = micros();
      combinationFrameCount = 0;
      frameDurationAccumMicros = 0;
      frameDurationSamples = 0;
      currentFPS = 0.0f;
      debugMessage = makeCombinationLabel(currentRefreshIndex, currentFrequencyIndex);
      overlayDirty = true;
      combinationActive = true;

      Serial.print("Benchmarking ");
      Serial.println(debugMessage);
      return true;
    }

    Serial.print("Skipping config ");
    Serial.println(makeCombinationLabel(currentRefreshIndex, currentFrequencyIndex));
    storeCombinationResult(0.0f, 0);
    combinationActive = false;

    if (!advanceCombination()) {
      return false;
    }
  }

  return false;
}

static void storeCombinationResult(float fps, uint32_t frames) {
  if (currentRefreshIndex < kRefreshCount && currentFrequencyIndex < kFrequencyCount) {
    BenchmarkResult &result = benchmarkResults[currentRefreshIndex][currentFrequencyIndex];
    result.fps = fps;
    result.frames = frames;
    result.completed = true;
  }
}

static bool advanceCombination() {
  if (++currentFrequencyIndex >= kFrequencyCount) {
    currentFrequencyIndex = 0;
    ++currentRefreshIndex;
  }

  if (currentRefreshIndex >= kRefreshCount) {
    benchmarkComplete = true;
    return false;
  }

  return true;
}

static void showTopCombinations() {
  if (topResultsShown) {
    return;
  }
  topResultsShown = true;

  struct RankedResult {
    float fps;
    size_t refreshIdx;
    size_t freqIdx;
  } top[3] = {};

  for (size_t r = 0; r < kRefreshCount; ++r) {
    for (size_t f = 0; f < kFrequencyCount; ++f) {
      const BenchmarkResult &res = benchmarkResults[r][f];
      if (!res.completed || res.frames == 0) {
        continue;
      }
      for (size_t slot = 0; slot < 3; ++slot) {
        if (res.fps > top[slot].fps) {
          for (size_t shift = 2; shift > slot; --shift) {
            top[shift] = top[shift - 1];
          }
          top[slot].fps = res.fps;
          top[slot].refreshIdx = r;
          top[slot].freqIdx = f;
          break;
        }
      }
    }
  }

  display.fillScreen(0x0000);
  display.setTextColor(0xFFFF, 0x0000);
  display.setTextSize(2);
  display.setCursor(20, 20);
  display.print("Top 3 configs");

  display.setTextSize(2);
  int16_t y = 60;
  bool anyPrinted = false;
  for (size_t i = 0; i < 3; ++i) {
    if (top[i].fps <= 0.0f) {
      continue;
    }
    String label = makeCombinationLabel(top[i].refreshIdx, top[i].freqIdx);
    display.setCursor(10, y);
    display.print("#");
    display.print(i + 1);
    display.print(" ");
    display.print(label);
    display.setCursor(10, y + 24);
    display.print(top[i].fps, 2);
    display.print(" FPS");
    y += 48;
    anyPrinted = true;
  }

  if (!anyPrinted) {
    display.setCursor(20, 80);
    display.print("No valid data");
  }

  Serial.println("Benchmark complete. Top 3 configurations:");
  bool anySerial = false;
  for (size_t i = 0; i < 3; ++i) {
    if (top[i].fps <= 0.0f) {
      continue;
    }
    String label = makeCombinationLabel(top[i].refreshIdx, top[i].freqIdx);
    Serial.print("  #");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(label);
    Serial.print(" -> ");
    Serial.print(top[i].fps, 3);
    Serial.println(" FPS");
    anySerial = true;
  }
  if (!anySerial) {
    Serial.println("  (no valid frame data)");
  }
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

  if (startCombinationWindow()) {
    drawOverlay();
    lastOverlayDraw = millis();
  } else if (benchmarkComplete && !topResultsShown) {
    showTopCombinations();
  }
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
  constexpr uint8_t kMaxAttempts = 10;
  constexpr uint16_t kDataNotReadyDelayMs = 5;
  const uint32_t startMicros = micros();
  bool sawDataNotReady = false;

  for (uint8_t attempt = 0; attempt < kMaxAttempts; ++attempt) {
    int status = mlx.getFrame(buffer);
    if (status == 0) {
      lastFrameDurationMicros = micros() - startMicros;
      frameDurationAccumMicros += lastFrameDurationMicros;
      ++frameDurationSamples;
      lastFrameMillis = millis();
      return true;
    }

    if (status == MLX_STATUS_DATA_NOT_READY) {
      sawDataNotReady = true;
      delay(kDataNotReadyDelayMs);
      continue;
    }

    debugMessage = String("MLX ") + describeMlxError(status) + " (" + status + ")";
    overlayDirty = true;
    Serial.print("MLX90640 read error: ");
    Serial.println(status);
    delay(5);
    return false;
  }

  if (sawDataNotReady) {
    debugMessage = "Waiting";
    overlayDirty = true;
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
    Serial.print("config=");
    Serial.println(makeCombinationLabel(currentRefreshIndex, currentFrequencyIndex));
    Serial.print("frame_us=");
    Serial.println(lastFrameDurationMicros);
    Serial.print("benchmark=");
    Serial.println(benchmarkComplete ? "DONE" : "RUNNING");
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
  if (benchmarkComplete) {
    if (!topResultsShown) {
      showTopCombinations();
    }
    return;
  }

  if (!combinationActive) {
    if (!startCombinationWindow()) {
      if (benchmarkComplete && !topResultsShown) {
        showTopCombinations();
      }
      return;
    }
  }

  bool frameReady = acquireFrame(frameBuffer);
  unsigned long now = millis();

  if (frameReady) {
    debugMessage = makeCombinationLabel(currentRefreshIndex, currentFrequencyIndex);
    overlayDirty = true;
    ++combinationFrameCount;
    updateFPS();
  }

  unsigned long elapsedMillis = now - combinationStartMillis;
  if (combinationActive && elapsedMillis >= 5000UL) {
    uint32_t elapsedMicros = micros() - combinationStartMicros;
    float fps = 0.0f;
    if (combinationFrameCount > 0 && elapsedMicros > 0) {
      fps = combinationFrameCount * 1000000.0f / static_cast<float>(elapsedMicros);
    }

    storeCombinationResult(fps, combinationFrameCount);

    Serial.print("Completed ");
    Serial.print(makeCombinationLabel(currentRefreshIndex, currentFrequencyIndex));
    Serial.print(": ");
    Serial.print(fps, 3);
    Serial.println(" FPS");

    combinationActive = false;

    if (advanceCombination()) {
      if (!startCombinationWindow()) {
        if (benchmarkComplete && !topResultsShown) {
          showTopCombinations();
        }
        return;
      }
    } else {
      showTopCombinations();
    }
  }

  if (!benchmarkComplete && overlayDirty && (now - lastOverlayDraw >= 100)) {
    drawOverlay();
    lastOverlayDraw = now;
  }
}
