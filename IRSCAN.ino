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

Adafruit_GC9A01A display = Adafruit_GC9A01A(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
Adafruit_MLX90640 mlx;

float frameBuffer[IMAGE_WIDTH * IMAGE_HEIGHT];
String debugMessage = "Boot";
bool sensorHealthy = true;
bool haveValidFrame = false;

unsigned long lastFrameMillis = 0;
float currentFPS = 0.0f;

uint8_t rowLUT[SCREEN_HEIGHT];
uint8_t colLUT[SCREEN_WIDTH];
uint16_t lineBuffer[SCREEN_WIDTH];

// Simple blue-to-red color map for temperature visualization
static uint16_t colorMap(float value, float minValue, float maxValue) {
  value = constrain(value, minValue, maxValue);
  float ratio = (maxValue - minValue) > 0 ? (value - minValue) / (maxValue - minValue) : 0.0f;

  // Use a simple gradient: blue -> cyan -> green -> yellow -> red
  float r = 0, g = 0, b = 0;
  if (ratio <= 0.25f) {
    // Blue to Cyan
    float local = ratio / 0.25f;
    r = 0;
    g = local * 255.0f;
    b = 255.0f;
  } else if (ratio <= 0.5f) {
    // Cyan to Green
    float local = (ratio - 0.25f) / 0.25f;
    r = 0;
    g = 255.0f;
    b = (1.0f - local) * 255.0f;
  } else if (ratio <= 0.75f) {
    // Green to Yellow
    float local = (ratio - 0.5f) / 0.25f;
    r = local * 255.0f;
    g = 255.0f;
    b = 0;
  } else {
    // Yellow to Red
    float local = (ratio - 0.75f) / 0.25f;
    r = 255.0f;
    g = (1.0f - local) * 255.0f;
    b = 0;
  }

  return display.color565(static_cast<uint8_t>(r), static_cast<uint8_t>(g), static_cast<uint8_t>(b));
}

static void drawOverlay() {
  char fpsBuffer[16];
  snprintf(fpsBuffer, sizeof(fpsBuffer), "FPS: %.1f", currentFPS);

  display.setTextSize(1);
  display.setTextColor(0xFFFF, 0x0000);

  int16_t x, y;
  uint16_t w, h;

  display.getTextBounds(fpsBuffer, 0, 0, &x, &y, &w, &h);
  int16_t fpsX = (SCREEN_WIDTH - w) / 2;
  int16_t fpsY = (SCREEN_HEIGHT / 2) - (h / 2) - 6;
  display.fillRect(fpsX - 2, fpsY - 2, w + 4, h + 4, 0x0000);
  display.setCursor(fpsX, fpsY);
  display.print(fpsBuffer);

  display.getTextBounds(debugMessage.c_str(), 0, 0, &x, &y, &w, &h);
  int16_t dbgX = (SCREEN_WIDTH - w) / 2;
  int16_t dbgY = fpsY + h + 12;
  display.fillRect(dbgX - 2, dbgY - 2, w + 4, h + 4, 0x0000);
  display.setCursor(dbgX, dbgY);
  display.print(debugMessage);
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
  mlx.setRefreshRate(MLX90640_32_HZ);

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
  const unsigned long startWait = millis();
  while (true) {
    int status = mlx.getFrame(buffer);
    if (status == 0) {
      sensorHealthy = true;
      if (!haveValidFrame || debugMessage == "Waiting" || debugMessage.startsWith("MLX ")) {
        debugMessage = "Streaming";
      }
      haveValidFrame = true;
      return true;
    }

    if (status == MLX_STATUS_DATA_NOT_READY) {
      // The sensor exposes new frames at the configured refresh rate. Keep
      // polling for a short window so we land on the very next one instead of
      // giving up and forcing the caller to loop again (which tanks FPS).
      if (millis() - startWait < 200) {
        delayMicroseconds(500);
        continue;
      }
      break;
    }

    sensorHealthy = false;
    debugMessage = String("MLX ") + describeMlxError(status);
    Serial.print("MLX90640 read error: ");
    Serial.println(status);
    delay(5);
    return false;
  }

  if (!haveValidFrame && debugMessage != "Waiting") {
    debugMessage = "Waiting";
  }
  return false;
}

void loop() {
  if (!acquireFrame(frameBuffer)) {
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

  display.startWrite();
  display.setAddrWindow(0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1);
  for (uint16_t y = 0; y < SCREEN_HEIGHT; ++y) {
    uint16_t srcRow = rowLUT[y];
    const float *rowPtr = &frameBuffer[srcRow * IMAGE_WIDTH];
    for (uint16_t x = 0; x < SCREEN_WIDTH; ++x) {
      float value = rowPtr[colLUT[x]];
      lineBuffer[x] = colorMap(value, minTemp, maxTemp);
    }
    display.writePixels(lineBuffer, SCREEN_WIDTH, false);
  }
  display.endWrite();

  updateFPS();
  drawOverlay();
}
