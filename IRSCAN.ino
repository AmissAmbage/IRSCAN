#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>
#include <Adafruit_MLX90640.h>
#include <algorithm>

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

Adafruit_GC9A01A display = Adafruit_GC9A01A(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
Adafruit_MLX90640 mlx;

float frameBuffer[32 * 24];
String debugMessage = "Boot";

unsigned long lastFrameMillis = 0;
float currentFPS = 0.0f;

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

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(1000000);  // 1 MHz for faster updates if supported

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
  mlx.setRefreshRate(MLX90640_16_HZ);

  debugMessage = "Init OK";
  drawOverlay();
}

void loop() {
  int status = mlx.getFrame(frameBuffer);
  if (status != 0) {
    debugMessage = String("MLX err ") + status;
    Serial.print("MLX90640 read error: ");
    Serial.println(status);
    delay(10);
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

  constexpr uint16_t imageWidth = 32;
  constexpr uint16_t imageHeight = 24;

  for (uint16_t y = 0; y < imageHeight; ++y) {
    uint16_t drawY0 = (y * SCREEN_HEIGHT) / imageHeight;
    uint16_t drawY1 = ((y + 1) * SCREEN_HEIGHT) / imageHeight;
    uint16_t rectHeight = std::max<uint16_t>(1, drawY1 - drawY0);
    for (uint16_t x = 0; x < imageWidth; ++x) {
      float value = frameBuffer[y * imageWidth + x];
      uint16_t color = colorMap(value, minTemp, maxTemp);
      uint16_t drawX0 = (x * SCREEN_WIDTH) / imageWidth;
      uint16_t drawX1 = ((x + 1) * SCREEN_WIDTH) / imageWidth;
      uint16_t rectWidth = std::max<uint16_t>(1, drawX1 - drawX0);
      display.fillRect(drawX0, drawY0, rectWidth, rectHeight, color);
    }
  }

  updateFPS();
  drawOverlay();
}
