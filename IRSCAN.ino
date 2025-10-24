#include <Arduino.h>
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
constexpr uint8_t  MLX_REFRESH_SETTING = MLX90640_16_HZ;

Adafruit_GC9A01A display = Adafruit_GC9A01A(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
Adafruit_MLX90640 mlx;

float frameBuffer[IMAGE_WIDTH * IMAGE_HEIGHT];
String debugMessage = "Boot";
bool sensorHealthy = true;
bool haveValidFrame = false;

unsigned long lastFrameMillis = 0;
float currentFPS = 0.0f;
uint32_t framePeriodMs = 63;
unsigned long nextFrameRequest = 0;
unsigned long waitingSince = 0;

uint8_t rowLUT[SCREEN_HEIGHT];
uint8_t colLUT[SCREEN_WIDTH];
uint16_t lineBuffer[SCREEN_WIDTH];
uint16_t paletteLUT[256];

static uint32_t estimateFramePeriodMs(uint8_t refreshSetting) {
  switch (refreshSetting) {
  #ifdef MLX90640_64_HZ
    case MLX90640_64_HZ:
      return 16;
  #endif
    case MLX90640_32_HZ:
      return 31;
    case MLX90640_16_HZ:
      return 63;
    case MLX90640_8_HZ:
      return 125;
    case MLX90640_4_HZ:
      return 250;
    case MLX90640_2_HZ:
      return 500;
    case MLX90640_1_HZ:
      return 1000;
  #ifdef MLX90640_0_5_HZ
    case MLX90640_0_5_HZ:
      return 2000;
  #endif
    default:
      return 2000;
  }
}

static void initializePalette() {
  static bool initialized = false;
  if (initialized) {
    return;
  }
  for (uint16_t i = 0; i < 256; ++i) {
    float ratio = i / 255.0f;
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;
    if (ratio <= 0.25f) {
      float local = ratio / 0.25f;
      r = 0.0f;
      g = local * 255.0f;
      b = 255.0f;
    } else if (ratio <= 0.5f) {
      float local = (ratio - 0.25f) / 0.25f;
      r = 0.0f;
      g = 255.0f;
      b = (1.0f - local) * 255.0f;
    } else if (ratio <= 0.75f) {
      float local = (ratio - 0.5f) / 0.25f;
      r = local * 255.0f;
      g = 255.0f;
      b = 0.0f;
    } else {
      float local = (ratio - 0.75f) / 0.25f;
      r = 255.0f;
      g = (1.0f - local) * 255.0f;
      b = 0.0f;
    }
    paletteLUT[i] = display.color565(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b));
  }
  initialized = true;
}

static void drawOverlay() {
  static String lastFpsText;
  static String lastDebugText;
  static int16_t lastFpsBoxX = -1;
  static int16_t lastFpsBoxY = 0;
  static uint16_t lastFpsBoxW = 0;
  static uint16_t lastFpsBoxH = 0;
  static int16_t lastDbgBoxX = -1;
  static int16_t lastDbgBoxY = 0;
  static uint16_t lastDbgBoxW = 0;
  static uint16_t lastDbgBoxH = 0;
  static int16_t lastFpsBaseline = (SCREEN_HEIGHT / 2) - 6;

  char fpsBuffer[16];
  snprintf(fpsBuffer, sizeof(fpsBuffer), "FPS: %.1f", currentFPS);
  String fpsText(fpsBuffer);

  display.setTextSize(1);
  display.setTextColor(0xFFFF, 0x0000);
  display.setTextWrap(false);

  int16_t x, y;
  uint16_t w, h;

  if (fpsText != lastFpsText) {
    if (lastFpsBoxW > 0 && lastFpsBoxH > 0) {
      display.fillRect(lastFpsBoxX, lastFpsBoxY, lastFpsBoxW, lastFpsBoxH, 0x0000);
    }
    display.getTextBounds(fpsText.c_str(), 0, 0, &x, &y, &w, &h);
    int16_t fpsX = (SCREEN_WIDTH - w) / 2;
    int16_t fpsY = (SCREEN_HEIGHT / 2) - (h / 2) - 6;
    display.fillRect(fpsX - 2, fpsY - 2, w + 4, h + 4, 0x0000);
    display.setCursor(fpsX, fpsY);
    display.print(fpsText);
    lastFpsText = fpsText;
    lastFpsBoxX = fpsX - 2;
    lastFpsBoxY = fpsY - 2;
    lastFpsBoxW = w + 4;
    lastFpsBoxH = h + 4;
    lastFpsBaseline = fpsY + h;
  }

  if (debugMessage != lastDebugText) {
    if (lastDbgBoxW > 0 && lastDbgBoxH > 0) {
      display.fillRect(lastDbgBoxX, lastDbgBoxY, lastDbgBoxW, lastDbgBoxH, 0x0000);
    }
    display.getTextBounds(debugMessage.c_str(), 0, 0, &x, &y, &w, &h);
    int16_t dbgX = (SCREEN_WIDTH - w) / 2;
    int16_t dbgY = lastFpsBaseline + 6;
    display.fillRect(dbgX - 2, dbgY - 2, w + 4, h + 4, 0x0000);
    display.setCursor(dbgX, dbgY);
    display.print(debugMessage);
    lastDebugText = debugMessage;
    lastDbgBoxX = dbgX - 2;
    lastDbgBoxY = dbgY - 2;
    lastDbgBoxW = w + 4;
    lastDbgBoxH = h + 4;
  }
}

static void initializeLUTs() {
  for (uint16_t y = 0; y < SCREEN_HEIGHT; ++y) {
    rowLUT[y] = (y * IMAGE_HEIGHT) / SCREEN_HEIGHT;
    if (rowLUT[y] >= IMAGE_HEIGHT) {
      rowLUT[y] = IMAGE_HEIGHT - 1;
    }
  }
  for (uint16_t x = 0; x < SCREEN_WIDTH; ++x) {
    colLUT[x] = (x * IMAGE_WIDTH) / SCREEN_WIDTH;
    if (colLUT[x] >= IMAGE_WIDTH) {
      colLUT[x] = IMAGE_WIDTH - 1;
    }
  }
}

static void updateFPS() {
  unsigned long now = millis();
  if (lastFrameMillis != 0) {
    float delta = (now - lastFrameMillis) / 1000.0f;
    if (delta > 0) {
      currentFPS = 0.8f * currentFPS + 0.2f * (1.0f / delta);  // simple smoothing
    }
  }
  lastFrameMillis = now;
}

void setup() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }

  display.begin();
  display.fillScreen(0x0000);
  initializeLUTs();
  initializePalette();

  Wire.begin(I2C_SDA, I2C_SCL);
  // The MLX90640 is specified for up to 1 MHz fast-mode plus, but many
  // breakouts (and long wiring runs) struggle to remain reliable at that
  // speed. 400 kHz has proven to be a safer setting in practice and avoids
  // spurious I2C read errors that manifest as status -1 from getFrame().
  Wire.setClock(400000);

  if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire)) {
    debugMessage = "MLX init fail";
    Serial.println("Failed to find MLX90640 sensor.");
    while (true) {
      drawOverlay();
      delay(100);
    }
  }

  mlx.setMode(MLX90640_INTERLEAVED);
  mlx.setResolution(MLX90640_ADC_18BIT);
  mlx.setRefreshRate(MLX_REFRESH_SETTING);
  framePeriodMs = estimateFramePeriodMs(MLX_REFRESH_SETTING);
  nextFrameRequest = millis();
  waitingSince = 0;

  debugMessage = "Init OK";
  drawOverlay();
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
  const unsigned long now = millis();
  if (now < nextFrameRequest) {
    if (!haveValidFrame) {
      if (waitingSince != 0 && now - waitingSince > 250 && debugMessage != "Waiting") {
        debugMessage = "Waiting";
      }
    } else if (lastFrameMillis != 0 && now - lastFrameMillis > framePeriodMs * 3U && debugMessage != "Waiting") {
      debugMessage = "Waiting";
    }
    return false;
  }

  int status = mlx.getFrame(buffer);
  if (status == 0) {
    sensorHealthy = true;
    haveValidFrame = true;
    waitingSince = 0;
    if (debugMessage != "Streaming") {
      debugMessage = "Streaming";
    }
    uint32_t guardDelay = framePeriodMs / 4U;
    if (guardDelay == 0) {
      guardDelay = 1;
    }
    nextFrameRequest = now + guardDelay;
    return true;
  }

  if (status == MLX_STATUS_DATA_NOT_READY) {
    if (waitingSince == 0) {
      waitingSince = now;
    }
    if (!haveValidFrame) {
      if (now - waitingSince > 250 && debugMessage != "Waiting") {
        debugMessage = "Waiting";
      }
    } else if (lastFrameMillis != 0 && now - lastFrameMillis > framePeriodMs * 3U && debugMessage != "Waiting") {
      debugMessage = "Waiting";
    }
    uint32_t guardDelay = framePeriodMs / 4U;
    if (guardDelay == 0) {
      guardDelay = 1;
    }
    nextFrameRequest = now + guardDelay;
    return false;
  }

  sensorHealthy = false;
  waitingSince = 0;
  debugMessage = String("MLX ") + describeMlxError(status);
  Serial.print("MLX90640 read error: ");
  Serial.println(status);
  delay(5);
  uint32_t guardDelay = framePeriodMs;
  if (guardDelay == 0) {
    guardDelay = 1;
  }
  nextFrameRequest = now + guardDelay;
  return false;
}

void loop() {
  if (!acquireFrame(frameBuffer)) {
    if (!haveValidFrame) {
      currentFPS = 0.0f;
    } else if (lastFrameMillis != 0 && millis() - lastFrameMillis > framePeriodMs * 4U) {
      currentFPS *= 0.9f;
    }
    delay(1);
    drawOverlay();
    return;
  }

  float minTemp = frameBuffer[0];
  float maxTemp = frameBuffer[0];
  for (float value : frameBuffer) {
    if (value < minTemp) {
      minTemp = value;
    }
    if (value > maxTemp) {
      maxTemp = value;
    }
  }

  const float range = maxTemp - minTemp;
  const bool constantFrame = range < 0.01f;
  const float scale = constantFrame ? 0.0f : 255.0f / range;
  const float bias = constantFrame ? 0.0f : -minTemp * scale;
  const uint16_t constantColor = paletteLUT[128];

  display.startWrite();
  display.setAddrWindow(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);
  for (uint16_t y = 0; y < SCREEN_HEIGHT; ++y) {
    uint16_t srcRow = rowLUT[y];
    const float *rowPtr = &frameBuffer[srcRow * IMAGE_WIDTH];
    for (uint16_t x = 0; x < SCREEN_WIDTH; ++x) {
      float value = rowPtr[colLUT[x]];
      if (constantFrame) {
        lineBuffer[x] = constantColor;
      } else {
        int mapped = static_cast<int>((value * scale) + bias + 0.5f);
        if (mapped < 0) {
          mapped = 0;
        } else if (mapped > 255) {
          mapped = 255;
        }
        lineBuffer[x] = paletteLUT[static_cast<uint8_t>(mapped)];
      }
    }
    display.writePixels(lineBuffer, SCREEN_WIDTH, false);
  }
  display.endWrite();

  updateFPS();
  drawOverlay();
}
