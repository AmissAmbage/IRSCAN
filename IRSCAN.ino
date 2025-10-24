#include <Arduino.h>
#include <Wire.h>
#include <Arduino_GFX_Library.h>
#include <Adafruit_MLX90640.h>

// Display pins
static constexpr int TFT_MOSI = 23;
static constexpr int TFT_SCLK = 18;
static constexpr int TFT_CS   = 16;
static constexpr int TFT_DC   = 2;
static constexpr int TFT_RST  = 4;
static constexpr int TFT_BL   = 17;

// Imager pins
static constexpr int I2C_SCL  = 22;
static constexpr int I2C_SDA  = 21;

static constexpr int SCREEN_WIDTH  = 240;
static constexpr int SCREEN_HEIGHT = 240;
static constexpr int MLX_COLS = 32;
static constexpr int MLX_ROWS = 24;
static constexpr size_t FRAME_SIZE = MLX_COLS * MLX_ROWS;

Arduino_ESP32SPI bus(TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, -1 /* MISO */);
Arduino_GC9A01 tft(&bus, TFT_RST, 1 /* rotation */, true /* IPS */);

Adafruit_MLX90640 mlx;

float frameData[FRAME_SIZE];

float fps = 0.0f;

char debugLine[64] = "";

static uint16_t temperatureToColor(float value, float minValue, float maxValue)
{
    if (isnan(value))
    {
        return 0x0000; // black for invalid readings
    }

    float clamped = constrain(value, minValue, maxValue);
    float normalized = (clamped - minValue) / (maxValue - minValue + 0.0001f);

    // Simple blue -> cyan -> yellow -> red gradient
    uint8_t r = 0, g = 0, b = 0;

    if (normalized <= 0.5f)
    {
        float ratio = normalized / 0.5f;
        r = 0;
        g = static_cast<uint8_t>(ratio * 255);
        b = 255;
    }
    else
    {
        float ratio = (normalized - 0.5f) / 0.5f;
        r = 255;
        g = static_cast<uint8_t>((1.0f - ratio) * 255);
        b = static_cast<uint8_t>((1.0f - ratio) * 128);
    }

    return tft.color565(r, g, b);
}

static void drawFrame()
{
    // Determine min/max temperatures for scaling
    float minTemp = 1000.0f;
    float maxTemp = -1000.0f;

    for (size_t i = 0; i < FRAME_SIZE; ++i)
    {
        if (!isnan(frameData[i]))
        {
            minTemp = min(minTemp, frameData[i]);
            maxTemp = max(maxTemp, frameData[i]);
        }
    }

    if (minTemp > maxTemp)
    {
        minTemp = 0;
        maxTemp = 100;
    }

    const float pixelWidth = static_cast<float>(SCREEN_WIDTH) / MLX_COLS;
    const float pixelHeight = static_cast<float>(SCREEN_HEIGHT) / MLX_ROWS;

    for (int row = 0; row < MLX_ROWS; ++row)
    {
        for (int col = 0; col < MLX_COLS; ++col)
        {
            size_t idx = row * MLX_COLS + col;
            uint16_t color = temperatureToColor(frameData[idx], minTemp, maxTemp);

            int x0 = static_cast<int>(col * pixelWidth);
            int y0 = static_cast<int>(row * pixelHeight);
            int x1 = static_cast<int>((col + 1) * pixelWidth);
            int y1 = static_cast<int>((row + 1) * pixelHeight);

            int w = max(1, x1 - x0);
            int h = max(1, y1 - y0);

            tft.fillRect(x0, y0, w, h, color);
        }
    }
}

static void drawOverlay()
{
    const int overlayWidth = 110;
    const int overlayHeight = 28;
    const int overlayX = (SCREEN_WIDTH - overlayWidth) / 2;
    const int overlayY = (SCREEN_HEIGHT - overlayHeight) / 2;

    tft.fillRect(overlayX, overlayY, overlayWidth, overlayHeight, tft.color565(0, 0, 0));
    tft.drawRect(overlayX, overlayY, overlayWidth, overlayHeight, tft.color565(80, 80, 80));

    tft.setTextColor(0xFFFF, tft.color565(0, 0, 0));
    tft.setTextSize(1);

    tft.setCursor(overlayX + 8, overlayY + 10);
    tft.print("FPS: ");
    tft.print(fps, 1);

    // Debug line below FPS counter
    const int debugY = overlayY + 18;
    tft.setCursor(overlayX + 8, debugY);
    tft.print(debugLine);
}

void setDebugMessage(const char *message)
{
    strncpy(debugLine, message, sizeof(debugLine) - 1);
    debugLine[sizeof(debugLine) - 1] = '\0';
}

void setup()
{
    setDebugMessage("Booting...");

    Serial.begin(115200);

    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    if (!mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire))
    {
        setDebugMessage("MLX init failed");
    }
    else
    {
        mlx.setMode(MLX90640_CHESS);
        mlx.setResolution(MLX90640_ADC_18BIT);
        mlx.setRefreshRate(MLX90640_16_HZ);
        setDebugMessage("MLX ready");
    }

    tft.begin();
    tft.fillScreen(0x0000);
    tft.setTextWrap(false);
}

void loop()
{
    unsigned long frameStart = millis();

    if (!mlx.getFrame(frameData))
    {
        drawFrame();
    }
    else
    {
        setDebugMessage("Frame read err");
    }

    unsigned long frameEnd = millis();
    unsigned long elapsed = frameEnd - frameStart;

    if (elapsed > 0)
    {
        fps = 1000.0f / static_cast<float>(elapsed);
    }

    drawOverlay();
}
