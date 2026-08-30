/*
  Intelligent HF Antenna Matrix
  ESP32 CYD + XPT2046 + MCP23017 + ADS1115 + Blynk

  Improvements in this revision:
  - Static GUI is drawn once; only changed fields are redrawn.
  - Wi-Fi and Blynk status icons are shown in the header.
  - Touch uses polling, lower pressure threshold, averaging, larger hit areas,
    and one-action-per-touch latching.
  - ESP32 task watchdog monitors loopTask.
  - Wi-Fi/Blynk reconnection is non-blocking and rate-limited.
  - Unused frequency-counter code and automatic boot relay test are removed.
  - ADS1115 channels are read with a dummy conversion after MUX changes.

  ADS1115 channel mapping:
    AIN0 -> TX1 forward detector  (VFRW1)
    AIN1 -> TX1 reflected detector (VREF1)
    AIN2 -> TX2 forward detector  (VFRW2)
    AIN3 -> TX2 reflected detector (VREF2)
*/

// -----------------------------------------------------------------------------
// Blynk and Wi-Fi credentials
// -----------------------------------------------------------------------------
#define BLYNK_TEMPLATE_ID   "TMPL5a_V3qF9u"
#define BLYNK_TEMPLATE_NAME "Antenna Matrix"
#define BLYNK_AUTH_TOKEN    "YOUR_BLYNK_AUTH_TOKEN"
#define TFT_BACKLIGHT_PIN 21
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_ADS1X15.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>

char ssid[] = "YOUR_WIFI_NAME";
char pass[] = "YOUR_WIFI_PASSWORD";

// -----------------------------------------------------------------------------
// System constants
// -----------------------------------------------------------------------------
constexpr uint8_t NUM_ANTENNAS = 8;
constexpr uint8_t MCP_ADDR = 0x20;
constexpr uint8_t ADS_ADDR = 0x48;

constexpr uint32_t MEASUREMENT_INTERVAL_MS = 5000;
constexpr uint32_t BLYNK_PUBLISH_INTERVAL_MS = 1000;
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 10000;
constexpr uint32_t BLYNK_RETRY_INTERVAL_MS = 5000;
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 12000;

constexpr float ADC_ZERO_THRESHOLD_V = 0.010f;
constexpr float RF_PRESENT_THRESHOLD_V = 0.150f;
constexpr float FILTER_ALPHA = 0.30f;

// Current provisional power calibration for a nominal -30 dB coupler.
// Replace these values after calibration with a known RF power meter.
constexpr float TX1_POWER_SCALE = 10.0f;
constexpr float TX2_POWER_SCALE = 10.0f;

// -----------------------------------------------------------------------------
// Touchscreen configuration
// -----------------------------------------------------------------------------
constexpr int TOUCH_MOSI = 32;
constexpr int TOUCH_MISO = 39;
constexpr int TOUCH_CLK  = 25;
constexpr int TOUCH_CS   = 33;

// IRQ is intentionally not used. GPIO36 is input-only and cannot use an
// internal pull-up on the classic ESP32. Polling is responsive enough here.
constexpr int TOUCH_MIN_Z = 90;
constexpr uint8_t TOUCH_SAMPLES = 3;
constexpr int TOUCH_HIT_MARGIN = 2;

constexpr int TS_MINX = 278;
constexpr int TS_MAXX = 3702;
constexpr int TS_MINY = 484;
constexpr int TS_MAXY = 3787;

// -----------------------------------------------------------------------------
// Relay configuration
// -----------------------------------------------------------------------------
constexpr uint8_t RELAY_ON = LOW;
constexpr uint8_t RELAY_OFF = HIGH;
constexpr uint16_t RELAY_SETTLE_MS = 25;

const uint8_t selectorPins[NUM_ANTENNAS] = {0, 1, 2, 3, 4, 5, 6, 7};
const uint8_t groundClampPins[NUM_ANTENNAS] = {8, 9, 10, 11, 12, 13, 14, 15};

// -----------------------------------------------------------------------------
// ADS1115 mapping
// -----------------------------------------------------------------------------
constexpr uint8_t CH_VFRW1 = 0;
constexpr uint8_t CH_VREF1 = 1;
constexpr uint8_t CH_VFRW2 = 2;
constexpr uint8_t CH_VREF2 = 3;

// -----------------------------------------------------------------------------
// Hardware objects
// -----------------------------------------------------------------------------
TFT_eSPI tft;
SPIClass touchSPI(VSPI);
XPT2046_Touchscreen touch(TOUCH_CS);
Adafruit_MCP23X17 mcp;
Adafruit_ADS1115 ads;
BlynkTimer blynkTimer;

bool mcpReady = false;
bool adsReady = false;
bool watchdogReady = false;

// -----------------------------------------------------------------------------
// Application state
// -----------------------------------------------------------------------------
enum AntennaState : uint8_t
{
  ANT_OFF = 0,
  ANT_BUS1,
  ANT_BUS2,
  ANT_PARK
};

AntennaState antennaState[NUM_ANTENNAS];
uint8_t selectedAntenna = 0;
bool tx1Locked = false;
bool tx2Locked = false;

struct RfMeasurement
{
  float forwardV = 0.0f;
  float reflectedV = 0.0f;
  float powerW = 0.0f;
  float swr = -1.0f;
  bool swrValid = false;
  bool initialized = false;
};

RfMeasurement tx1Measurement;
RfMeasurement tx2Measurement;

uint32_t lastMeasurementMs = 0;
uint32_t lastWifiAttemptMs = 0;
uint32_t lastBlynkAttemptMs = 0;

bool lastWifiConnected = false;
bool lastBlynkConnected = false;

// -----------------------------------------------------------------------------
// GUI layout
// -----------------------------------------------------------------------------
struct Button
{
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  const char *label;
};

const Button antennaButtons[NUM_ANTENNAS] =
{
  {  6,  36, 50, 27, "ANT1"},
  { 60,  36, 50, 27, "ANT2"},
  {114,  36, 50, 27, "ANT3"},
  {168,  36, 50, 27, "ANT4"},
  {  6,  67, 50, 27, "ANT5"},
  { 60,  67, 50, 27, "ANT6"},
  {114,  67, 50, 27, "ANT7"},
  {168,  67, 50, 27, "ANT8"}
};

const Button buttonBus1 = {224, 36, 90, 27, "BUS 1"};
const Button buttonBus2 = {224, 67, 90, 27, "BUS 2"};
const Button buttonPark = {224, 98, 90, 27, "PARK"};

constexpr int16_t STATUS_X = 6;
constexpr int16_t STATUS_Y = 101;
constexpr int16_t STATUS_W = 212;
constexpr int16_t STATUS_H = 55;

constexpr int16_t TX1_CARD_X = 6;
constexpr int16_t TX2_CARD_X = 163;
constexpr int16_t CARD_Y = 162;
constexpr int16_t CARD_W = 151;
constexpr int16_t CARD_H = 72;

struct MeasurementUiCache
{
  char forwardText[16] = "";
  char reflectedText[16] = "";
  char powerText[16] = "";
  char swrText[16] = "";
  int8_t lockState = -1;
};

struct UiCache
{
  int16_t selected = -1;
  AntennaState states[NUM_ANTENNAS];
  int16_t bus1Antenna = -99;
  int16_t bus2Antenna = -99;
  bool initialized = false;
  int8_t wifiState = -1;
  int8_t blynkState = -1;
  MeasurementUiCache tx1;
  MeasurementUiCache tx2;
};

UiCache uiCache;

/*
 * One small off-screen buffer is reused for both measurement cards.
 * Dynamic text is composed completely in RAM and then transferred to the
 * display in one operation, so the user never sees a cleared/blank field.
 */
TFT_eSprite measurementSprite(&tft);
bool measurementSpriteReady = false;

constexpr int16_t MEASUREMENT_SPRITE_X_OFFSET = 6;
constexpr int16_t MEASUREMENT_SPRITE_Y_OFFSET = 25;
constexpr int16_t MEASUREMENT_SPRITE_W = CARD_W - 12;
constexpr int16_t MEASUREMENT_SPRITE_H = CARD_H - 28;

// -----------------------------------------------------------------------------
// Forward declarations
// -----------------------------------------------------------------------------
void drawStaticGui();
void refreshStateGui(bool force = false);
void refreshMeasurementGui(bool force = false);
void refreshNetworkIcons(bool force = false);
void readMeasurements();
void publishBlynkStatus();
void serviceNetwork();

// -----------------------------------------------------------------------------
// Watchdog
// -----------------------------------------------------------------------------
void setupWatchdog()
{
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t config = {};
  config.timeout_ms = WATCHDOG_TIMEOUT_MS;
  config.idle_core_mask = 0;       // Monitor loopTask explicitly below.
  config.trigger_panic = true;     // Reset after the watchdog panic.

  esp_err_t result = esp_task_wdt_reconfigure(&config);
  if (result == ESP_ERR_INVALID_STATE)
  {
    result = esp_task_wdt_init(&config);
  }

  if ((result == ESP_OK) || (result == ESP_ERR_INVALID_STATE))
  {
    if (esp_task_wdt_status(nullptr) != ESP_OK)
    {
      result = esp_task_wdt_add(nullptr);
    }
    else
    {
      result = ESP_OK;
    }
  }

  watchdogReady = (result == ESP_OK);
#else
  esp_err_t result = esp_task_wdt_init(WATCHDOG_TIMEOUT_MS / 1000U, true);
  if ((result == ESP_OK) || (result == ESP_ERR_INVALID_STATE))
  {
    result = esp_task_wdt_add(nullptr);
    if (result == ESP_ERR_INVALID_ARG)
    {
      result = ESP_OK; // Current task was already subscribed.
    }
  }
  watchdogReady = (result == ESP_OK);
#endif

  Serial.println(watchdogReady ? "Watchdog ready." : "Watchdog setup failed.");
}

inline void feedWatchdog()
{
  if (watchdogReady)
  {
    esp_task_wdt_reset();
  }
}

// -----------------------------------------------------------------------------
// General helpers
// -----------------------------------------------------------------------------
bool isValidAntenna(uint8_t antenna)
{
  return antenna < NUM_ANTENNAS;
}

const char *stateToText(AntennaState state)
{
  switch (state)
  {
    case ANT_BUS1: return "BUS 1";
    case ANT_BUS2: return "BUS 2";
    case ANT_PARK: return "PARK";
    default:       return "OFF";
  }
}

uint16_t stateColour(AntennaState state)
{
  switch (state)
  {
    case ANT_BUS1: return TFT_DARKGREEN;
    case ANT_BUS2: return TFT_ORANGE;
    case ANT_PARK: return TFT_DARKGREY;
    default:       return TFT_NAVY;
  }
}

float clampFloat(float value, float minimum, float maximum)
{
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

float applyZeroThreshold(float voltage)
{
  return (voltage < ADC_ZERO_THRESHOLD_V) ? 0.0f : voltage;
}

float smoothValue(float previous, float current, bool initialized)
{
  if (!initialized || current == 0.0f || previous == 0.0f)
  {
    return current;
  }
  return previous + FILTER_ALPHA * (current - previous);
}

// -----------------------------------------------------------------------------
// RF calculations
// -----------------------------------------------------------------------------
float calculatePowerW(float forwardV, float scale)
{
  if (forwardV < ADC_ZERO_THRESHOLD_V)
  {
    return 0.0f;
  }
  return scale * forwardV * forwardV;
}

bool validSWRInput(float forwardV, float reflectedV)
{
  if (forwardV < RF_PRESENT_THRESHOLD_V) return false;
  if (reflectedV < 0.0f) return false;
  if (reflectedV >= forwardV) return false;
  if ((forwardV - reflectedV) < 0.05f) return false;
  return true;
}

float calculateSWR(float forwardV, float reflectedV)
{
  if (!validSWRInput(forwardV, reflectedV))
  {
    return -1.0f;
  }

  // For matched forward/reflected envelope detectors, the detector outputs
  // are treated as proportional to RF voltage. Final calibration is required.
  float gamma = reflectedV / forwardV;
  gamma = clampFloat(gamma, 0.0f, 0.95f);

  return clampFloat((1.0f + gamma) / (1.0f - gamma), 1.0f, 9.99f);
}

// -----------------------------------------------------------------------------
// Relay control and interlock
// -----------------------------------------------------------------------------
int findAntennaOnBus(AntennaState bus)
{
  for (uint8_t antenna = 0; antenna < NUM_ANTENNAS; antenna++)
  {
    if (antennaState[antenna] == bus)
    {
      return antenna;
    }
  }
  return -1;
}

int currentBusOfAntenna(uint8_t antenna)
{
  if (!isValidAntenna(antenna)) return 0;
  if (antennaState[antenna] == ANT_BUS1) return 1;
  if (antennaState[antenna] == ANT_BUS2) return 2;
  return 0;
}

bool actionBlocked(uint8_t antenna, AntennaState requestedState)
{
  const int currentBus = currentBusOfAntenna(antenna);

  if ((requestedState == ANT_BUS1) && tx1Locked) return true;
  if ((requestedState == ANT_BUS2) && tx2Locked) return true;

  if ((currentBus == 1) && tx1Locked && (requestedState != ANT_BUS1)) return true;
  if ((currentBus == 2) && tx2Locked && (requestedState != ANT_BUS2)) return true;

  return false;
}

void parkAntennaHardware(uint8_t antenna)
{
  if (!mcpReady || !isValidAntenna(antenna)) return;

  // The ground-clamp relay isolates the antenna from the selector path and
  // connects it to ground in the PARK state.
  mcp.digitalWrite(groundClampPins[antenna], RELAY_ON);
  antennaState[antenna] = ANT_PARK;
}

void connectAntennaHardware(uint8_t antenna, AntennaState destination)
{
  if (!mcpReady || !isValidAntenna(antenna)) return;

  // Preserve the relay sequence and polarity used by the working prototype:
  // release the ground clamp, wait, then set the selector relay.
  mcp.digitalWrite(groundClampPins[antenna], RELAY_OFF);
  delay(RELAY_SETTLE_MS);

  const uint8_t selectorLevel =
      (destination == ANT_BUS1) ? RELAY_ON : RELAY_OFF;
  mcp.digitalWrite(selectorPins[antenna], selectorLevel);

  antennaState[antenna] = destination;
}

bool requestAntennaState(uint8_t antenna, AntennaState requestedState)
{
  if (!mcpReady)
  {
    Serial.println("Switching rejected: MCP23017 unavailable.");
    return false;
  }

  if (!isValidAntenna(antenna))
  {
    Serial.println("Switching rejected: invalid antenna.");
    return false;
  }

  if (actionBlocked(antenna, requestedState))
  {
    Serial.println("Switching blocked by TX lock.");
    return false;
  }

  if (requestedState == antennaState[antenna])
  {
    return true;
  }

  Serial.printf("ANT%u -> %s\n", antenna + 1, stateToText(requestedState));

  if (requestedState == ANT_PARK)
  {
    parkAntennaHardware(antenna);
    refreshStateGui();
    return true;
  }

  if ((requestedState != ANT_BUS1) && (requestedState != ANT_BUS2))
  {
    return false;
  }

  const int occupiedAntenna = findAntennaOnBus(requestedState);
  if ((occupiedAntenna >= 0) && (occupiedAntenna != antenna))
  {
    parkAntennaHardware((uint8_t)occupiedAntenna);
    delay(RELAY_SETTLE_MS);
  }

  if ((antennaState[antenna] == ANT_BUS1) ||
      (antennaState[antenna] == ANT_BUS2))
  {
    parkAntennaHardware(antenna);
    delay(RELAY_SETTLE_MS);
  }

  connectAntennaHardware(antenna, requestedState);
  refreshStateGui();
  return true;
}

void initializeRelayOutputs()
{
  for (uint8_t antenna = 0; antenna < NUM_ANTENNAS; antenna++)
  {
    antennaState[antenna] = ANT_PARK;

    // Configure selector first, then force the clamp into the safe PARK state.
    mcp.pinMode(selectorPins[antenna], OUTPUT);
    mcp.digitalWrite(selectorPins[antenna], RELAY_OFF);

    mcp.pinMode(groundClampPins[antenna], OUTPUT);
    mcp.digitalWrite(groundClampPins[antenna], RELAY_ON);
  }
}

// -----------------------------------------------------------------------------
// ADS1115 acquisition
// -----------------------------------------------------------------------------
float readAdcVoltage(uint8_t channel)
{
  if (!adsReady || channel > 3)
  {
    return 0.0f;
  }

  // Discard the first result after switching the internal ADS1115 MUX.
  (void)ads.readADC_SingleEnded(channel);

  constexpr uint8_t SAMPLE_COUNT = 4;
  int32_t sum = 0;

  for (uint8_t sample = 0; sample < SAMPLE_COUNT; sample++)
  {
    sum += ads.readADC_SingleEnded(channel);
  }

  const int16_t averageRaw = (int16_t)lroundf(sum / (float)SAMPLE_COUNT);
  return applyZeroThreshold(ads.computeVolts(averageRaw));
}

void updateMeasurement(
    RfMeasurement &measurement,
    float rawForward,
    float rawReflected,
    float powerScale)
{
  measurement.forwardV = smoothValue(
      measurement.forwardV,
      rawForward,
      measurement.initialized);

  measurement.reflectedV = smoothValue(
      measurement.reflectedV,
      rawReflected,
      measurement.initialized);

  measurement.powerW = calculatePowerW(measurement.forwardV, powerScale);
  measurement.swrValid = validSWRInput(
      measurement.forwardV,
      measurement.reflectedV);

  measurement.swr = measurement.swrValid
      ? calculateSWR(measurement.forwardV, measurement.reflectedV)
      : -1.0f;

  measurement.initialized = true;
}

void readMeasurements()
{
  if (!adsReady)
  {
    return;
  }

  const float fwd1 = readAdcVoltage(CH_VFRW1);
  const float ref1 = readAdcVoltage(CH_VREF1);
  const float fwd2 = readAdcVoltage(CH_VFRW2);
  const float ref2 = readAdcVoltage(CH_VREF2);

  updateMeasurement(tx1Measurement, fwd1, ref1, TX1_POWER_SCALE);
  updateMeasurement(tx2Measurement, fwd2, ref2, TX2_POWER_SCALE);
}

// -----------------------------------------------------------------------------
// GUI primitives
// -----------------------------------------------------------------------------
void drawButton(const Button &button, uint16_t fill, uint16_t text, uint16_t border)
{
  tft.fillRoundRect(button.x, button.y, button.w, button.h, 6, fill);
  tft.drawRoundRect(button.x, button.y, button.w, button.h, 6, border);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(text, fill);
  tft.drawString(button.label,
                 button.x + button.w / 2,
                 button.y + button.h / 2,
                 2);
}

void drawAntennaButton(uint8_t antenna)
{
  const bool selected = (antenna == selectedAntenna);
  const uint16_t fill = stateColour(antennaState[antenna]);
  const uint16_t text = (antennaState[antenna] == ANT_BUS2) ? TFT_BLACK : TFT_WHITE;
  const uint16_t border = selected ? TFT_YELLOW : TFT_LIGHTGREY;

  drawButton(antennaButtons[antenna], fill, text, border);

  if (selected)
  {
    tft.drawRoundRect(
        antennaButtons[antenna].x + 2,
        antennaButtons[antenna].y + 2,
        antennaButtons[antenna].w - 4,
        antennaButtons[antenna].h - 4,
        4,
        TFT_YELLOW);
  }
}

void drawWifiIcon(int16_t x, int16_t y, bool connected)
{
  const uint16_t colour = connected ? TFT_GREEN : TFT_DARKGREY;
  tft.fillRect(x - 12, y - 11, 24, 22, TFT_BLACK);

  // Three simple chevrons make a compact Wi-Fi icon without external assets.
  tft.drawLine(x - 9, y - 4, x, y - 10, colour);
  tft.drawLine(x, y - 10, x + 9, y - 4, colour);
  tft.drawLine(x - 6, y + 1, x, y - 3, colour);
  tft.drawLine(x, y - 3, x + 6, y + 1, colour);
  tft.drawLine(x - 3, y + 5, x, y + 3, colour);
  tft.drawLine(x, y + 3, x + 3, y + 5, colour);
  tft.fillCircle(x, y + 8, 2, colour);
}

void drawCloudIcon(int16_t x, int16_t y, bool connected)
{
  const uint16_t colour = connected ? TFT_GREEN : TFT_DARKGREY;
  tft.fillRect(x - 13, y - 11, 27, 22, TFT_BLACK);
  tft.drawCircle(x - 6, y + 1, 5, colour);
  tft.drawCircle(x, y - 3, 6, colour);
  tft.drawCircle(x + 7, y + 1, 5, colour);
  tft.drawLine(x - 10, y + 6, x + 11, y + 6, colour);
  tft.fillCircle(x + 10, y - 7, 2, connected ? TFT_GREEN : TFT_RED);
}

void drawLockIcon(int16_t x, int16_t y, bool locked, uint16_t background)
{
  const uint16_t colour = locked ? TFT_RED : TFT_GREEN;
  tft.fillRect(x - 9, y - 9, 19, 19, background);
  tft.drawRoundRect(x - 5, y - 1, 11, 9, 2, colour);
  tft.drawCircle(x, y - 2, 5, colour);
  tft.fillRect(x - 6, y - 2, 13, 5, background);
  tft.drawRoundRect(x - 5, y - 1, 11, 9, 2, colour);
}

void drawMeasurementCardStatic(int16_t x, const char *title)
{
  tft.fillRoundRect(x, CARD_Y, CARD_W, CARD_H, 8, TFT_NAVY);
  tft.drawRoundRect(x, CARD_Y, CARD_W, CARD_H, 8, TFT_DARKGREY);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_CYAN, TFT_NAVY);
  tft.drawString(title, x + 8, CARD_Y + 5, 2);

  tft.drawFastHLine(x + 6, CARD_Y + 22, CARD_W - 12, TFT_DARKGREY);
}

void drawStaticGui()
{
  tft.fillScreen(TFT_BLACK);

  // Header
  tft.fillRect(0, 0, 320, 31, TFT_BLACK);
  tft.drawFastHLine(0, 30, 320, TFT_DARKGREY);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("HF ANTENNA MATRIX", 8, 7, 2);

  // Action buttons and antenna buttons
  for (uint8_t antenna = 0; antenna < NUM_ANTENNAS; antenna++)
  {
    drawAntennaButton(antenna);
  }

  drawButton(buttonBus1, TFT_DARKGREEN, TFT_WHITE, TFT_GREEN);
  drawButton(buttonBus2, TFT_ORANGE, TFT_BLACK, TFT_YELLOW);
  drawButton(buttonPark, TFT_MAROON, TFT_WHITE, TFT_RED);

  // Selected-antenna status card
  tft.fillRoundRect(STATUS_X, STATUS_Y, STATUS_W, STATUS_H, 8, TFT_NAVY);
  tft.drawRoundRect(STATUS_X, STATUS_Y, STATUS_W, STATUS_H, 8, TFT_DARKGREY);

  // Measurement cards
  drawMeasurementCardStatic(TX1_CARD_X, "TX1");
  drawMeasurementCardStatic(TX2_CARD_X, "TX2");

  uiCache.initialized = false;
  refreshStateGui(true);
  refreshMeasurementGui(true);
  refreshNetworkIcons(true);
}

void refreshNetworkIcons(bool force)
{
  const bool wifiConnected = (WiFi.status() == WL_CONNECTED);
  const bool blynkConnected = wifiConnected && Blynk.connected();

  if (force || (uiCache.wifiState != (int8_t)wifiConnected))
  {
    drawWifiIcon(270, 14, wifiConnected);
    uiCache.wifiState = wifiConnected;
  }

  if (force || (uiCache.blynkState != (int8_t)blynkConnected))
  {
    drawCloudIcon(302, 14, blynkConnected);
    uiCache.blynkState = blynkConnected;
  }
}

void refreshStateGui(bool force)
{
  const int bus1Antenna = findAntennaOnBus(ANT_BUS1);
  const int bus2Antenna = findAntennaOnBus(ANT_BUS2);
  const bool selectedChanged =
      force || !uiCache.initialized || (uiCache.selected != selectedAntenna);
  const bool selectedStateChanged =
      force || !uiCache.initialized ||
      (uiCache.states[selectedAntenna] != antennaState[selectedAntenna]);

  for (uint8_t antenna = 0; antenna < NUM_ANTENNAS; antenna++)
  {
    if (force ||
        !uiCache.initialized ||
        (uiCache.states[antenna] != antennaState[antenna]) ||
        selectedChanged)
    {
      drawAntennaButton(antenna);
      uiCache.states[antenna] = antennaState[antenna];
    }
  }

  if (force ||
      !uiCache.initialized ||
      selectedChanged ||
      selectedStateChanged ||
      (uiCache.bus1Antenna != bus1Antenna) ||
      (uiCache.bus2Antenna != bus2Antenna))
  {
    char line[28];

    tft.fillRect(STATUS_X + 6, STATUS_Y + 6, STATUS_W - 12, 20, TFT_NAVY);
    snprintf(line, sizeof(line), "ANT%u  %s",
             selectedAntenna + 1,
             stateToText(antennaState[selectedAntenna]));
    tft.setTextDatum(TL_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_NAVY);
    tft.drawString(line, STATUS_X + 8, STATUS_Y + 7, 2);

    tft.fillRoundRect(STATUS_X + 8, STATUS_Y + 31, 82, 18, 4, TFT_DARKGREEN);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREEN);
    snprintf(line, sizeof(line), "BUS1: %d", (bus1Antenna >= 0) ? bus1Antenna + 1 : 0);
    tft.drawString(line, STATUS_X + 13, STATUS_Y + 33, 2);

    tft.fillRoundRect(STATUS_X + 115, STATUS_Y + 31, 82, 18, 4, TFT_ORANGE);
    tft.setTextColor(TFT_BLACK, TFT_ORANGE);
    snprintf(line, sizeof(line), "BUS2: %d", (bus2Antenna >= 0) ? bus2Antenna + 1 : 0);
    tft.drawString(line, STATUS_X + 120, STATUS_Y + 33, 2);
  }

  if (force || (uiCache.tx1.lockState != (int8_t)tx1Locked))
  {
    drawLockIcon(TX1_CARD_X + CARD_W - 17, CARD_Y + 12, tx1Locked, TFT_NAVY);
    uiCache.tx1.lockState = tx1Locked;
  }

  if (force || (uiCache.tx2.lockState != (int8_t)tx2Locked))
  {
    drawLockIcon(TX2_CARD_X + CARD_W - 17, CARD_Y + 12, tx2Locked, TFT_NAVY);
    uiCache.tx2.lockState = tx2Locked;
  }

  uiCache.selected = selectedAntenna;
  uiCache.bus1Antenna = bus1Antenna;
  uiCache.bus2Antenna = bus2Antenna;
  uiCache.initialized = true;
}

void formatMeasurementTexts(
    const RfMeasurement &measurement,
    char *forward,
    size_t forwardSize,
    char *reflected,
    size_t reflectedSize,
    char *power,
    size_t powerSize,
    char *swr,
    size_t swrSize)
{
  if (!adsReady)
  {
    snprintf(forward, forwardSize, "F ADC ERR");
    snprintf(reflected, reflectedSize, "R ADC ERR");
    snprintf(power, powerSize, "P ---");
    snprintf(swr, swrSize, "S ---");
    return;
  }

  snprintf(forward, forwardSize, "F %.2fV", measurement.forwardV);
  snprintf(reflected, reflectedSize, "R %.2fV", measurement.reflectedV);

  if (measurement.powerW < 100.0f)
  {
    snprintf(power, powerSize, "P %.1fW", measurement.powerW);
  }
  else
  {
    snprintf(power, powerSize, "P %.0fW", measurement.powerW);
  }

  if (measurement.swrValid)
  {
    snprintf(swr, swrSize, "S %.2f", measurement.swr);
  }
  else
  {
    snprintf(swr, swrSize, "S ---");
  }
}

bool measurementCardChanged(
    const char *forward,
    const char *reflected,
    const char *power,
    const char *swr,
    const MeasurementUiCache &cache,
    bool force)
{
  return force ||
         (strcmp(forward, cache.forwardText) != 0) ||
         (strcmp(reflected, cache.reflectedText) != 0) ||
         (strcmp(power, cache.powerText) != 0) ||
         (strcmp(swr, cache.swrText) != 0);
}

void saveMeasurementTexts(
    MeasurementUiCache &cache,
    const char *forward,
    const char *reflected,
    const char *power,
    const char *swr)
{
  snprintf(cache.forwardText, sizeof(cache.forwardText), "%s", forward);
  snprintf(cache.reflectedText, sizeof(cache.reflectedText), "%s", reflected);
  snprintf(cache.powerText, sizeof(cache.powerText), "%s", power);
  snprintf(cache.swrText, sizeof(cache.swrText), "%s", swr);
}

void drawMeasurementCardFallback(
    int16_t cardX,
    const char *forward,
    const char *reflected,
    const char *power,
    const char *swr,
    uint16_t swrColour)
{
  /*
   * Fallback for the unlikely case that sprite allocation fails.
   * setTextPadding() replaces the previous text without explicitly clearing
   * the rectangle first, which is still less visible than fillRect()+draw.
   */
  tft.setTextDatum(TL_DATUM);

  tft.setTextPadding(68);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.drawString(forward, cardX + 7, CARD_Y + 28, 2);

  tft.setTextColor(TFT_LIGHTGREY, TFT_NAVY);
  tft.drawString(reflected, cardX + 7, CARD_Y + 49, 2);

  tft.setTextPadding(67);
  tft.setTextColor(TFT_CYAN, TFT_NAVY);
  tft.drawString(power, cardX + 77, CARD_Y + 28, 2);

  tft.setTextColor(swrColour, TFT_NAVY);
  tft.drawString(swr, cardX + 77, CARD_Y + 49, 2);

  tft.setTextPadding(0);
}

void drawMeasurementCardSprite(
    int16_t cardX,
    const char *forward,
    const char *reflected,
    const char *power,
    const char *swr,
    uint16_t swrColour)
{
  if (!measurementSpriteReady)
  {
    drawMeasurementCardFallback(
        cardX, forward, reflected, power, swr, swrColour);
    return;
  }

  /*
   * Build the entire dynamic part of one card off screen.
   * The display receives one completed rectangle; no blank intermediate
   * rectangle is ever visible.
   */
  measurementSprite.fillSprite(TFT_NAVY);
  measurementSprite.setTextDatum(TL_DATUM);
  measurementSprite.setTextPadding(0);

  measurementSprite.setTextColor(TFT_WHITE, TFT_NAVY);
  measurementSprite.drawString(forward, 1, 3, 2);

  measurementSprite.setTextColor(TFT_LIGHTGREY, TFT_NAVY);
  measurementSprite.drawString(reflected, 1, 24, 2);

  measurementSprite.setTextColor(TFT_CYAN, TFT_NAVY);
  measurementSprite.drawString(power, 71, 3, 2);

  measurementSprite.setTextColor(swrColour, TFT_NAVY);
  measurementSprite.drawString(swr, 71, 24, 2);

  measurementSprite.pushSprite(
      cardX + MEASUREMENT_SPRITE_X_OFFSET,
      CARD_Y + MEASUREMENT_SPRITE_Y_OFFSET);
}

void refreshOneMeasurementCard(
    int16_t cardX,
    const RfMeasurement &measurement,
    MeasurementUiCache &cache,
    bool force)
{
  char forward[16];
  char reflected[16];
  char power[16];
  char swr[16];

  formatMeasurementTexts(
      measurement,
      forward, sizeof(forward),
      reflected, sizeof(reflected),
      power, sizeof(power),
      swr, sizeof(swr));

  if (!measurementCardChanged(
          forward, reflected, power, swr, cache, force))
  {
    return;
  }

  drawMeasurementCardSprite(
      cardX,
      forward,
      reflected,
      power,
      swr,
      measurement.swrValid ? TFT_GREEN : TFT_DARKGREY);

  saveMeasurementTexts(cache, forward, reflected, power, swr);
}

void refreshMeasurementGui(bool force)
{
  refreshOneMeasurementCard(
      TX1_CARD_X,
      tx1Measurement,
      uiCache.tx1,
      force);

  refreshOneMeasurementCard(
      TX2_CARD_X,
      tx2Measurement,
      uiCache.tx2,
      force);
}

// -----------------------------------------------------------------------------
// Touch handling
// -----------------------------------------------------------------------------
bool pointInButton(int16_t x, int16_t y, const Button &button)
{
  return x >= (button.x - TOUCH_HIT_MARGIN) &&
         x <= (button.x + button.w + TOUCH_HIT_MARGIN) &&
         y >= (button.y - TOUCH_HIT_MARGIN) &&
         y <= (button.y + button.h + TOUCH_HIT_MARGIN);
}

bool readTouchPoint(int16_t &screenX, int16_t &screenY)
{
  if (!touch.touched())
  {
    return false;
  }

  int32_t rawXSum = 0;
  int32_t rawYSum = 0;
  uint8_t validSamples = 0;

  for (uint8_t sample = 0; sample < TOUCH_SAMPLES; sample++)
  {
    const TS_Point point = touch.getPoint();

    if (point.z >= TOUCH_MIN_Z)
    {
      rawXSum += point.x;
      rawYSum += point.y;
      validSamples++;
    }

    delay(2);
  }

  if (validSamples < 2)
  {
    return false;
  }

  const int32_t rawX = rawXSum / validSamples;
  const int32_t rawY = rawYSum / validSamples;

  screenX = constrain(map(rawX, TS_MINX, TS_MAXX, 0, 320), 0, 319);
  screenY = constrain(map(rawY, TS_MINY, TS_MAXY, 0, 240), 0, 239);
  return true;
}

void handleTouch()
{
  static bool touchLatched = false;

  int16_t x = 0;
  int16_t y = 0;
  const bool pressed = readTouchPoint(x, y);

  if (!pressed)
  {
    touchLatched = false;
    return;
  }

  // Process one action only. A new action requires finger release first.
  if (touchLatched)
  {
    return;
  }
  touchLatched = true;

  for (uint8_t antenna = 0; antenna < NUM_ANTENNAS; antenna++)
  {
    if (pointInButton(x, y, antennaButtons[antenna]))
    {
      selectedAntenna = antenna;
      refreshStateGui();
      return;
    }
  }

  if (pointInButton(x, y, buttonBus1))
  {
    requestAntennaState(selectedAntenna, ANT_BUS1);
    return;
  }

  if (pointInButton(x, y, buttonBus2))
  {
    requestAntennaState(selectedAntenna, ANT_BUS2);
    return;
  }

  if (pointInButton(x, y, buttonPark))
  {
    requestAntennaState(selectedAntenna, ANT_PARK);
    return;
  }
}

// -----------------------------------------------------------------------------
// Blynk
// -----------------------------------------------------------------------------
void publishBlynkStatus()
{
  if ((WiFi.status() != WL_CONNECTED) || !Blynk.connected())
  {
    return;
  }

  const int bus1Antenna = findAntennaOnBus(ANT_BUS1);
  const int bus2Antenna = findAntennaOnBus(ANT_BUS2);

  Blynk.virtualWrite(V0, selectedAntenna + 1);
  Blynk.virtualWrite(V4, (bus1Antenna >= 0) ? bus1Antenna + 1 : 0);
  Blynk.virtualWrite(V5, (bus2Antenna >= 0) ? bus2Antenna + 1 : 0);
  Blynk.virtualWrite(V6, tx1Measurement.powerW);
  Blynk.virtualWrite(V7, tx1Measurement.swrValid ? tx1Measurement.swr : 0.0f);
  Blynk.virtualWrite(V8, tx2Measurement.powerW);
  Blynk.virtualWrite(V9, tx2Measurement.swrValid ? tx2Measurement.swr : 0.0f);
  Blynk.virtualWrite(V10, tx1Locked ? 1 : 0);
  Blynk.virtualWrite(V11, tx2Locked ? 1 : 0);
}

BLYNK_CONNECTED()
{
  Blynk.syncVirtual(V0, V10, V11);
  publishBlynkStatus();
}

BLYNK_WRITE(V0)
{
  const int antenna = param.asInt();
  if ((antenna >= 1) && (antenna <= NUM_ANTENNAS))
  {
    selectedAntenna = (uint8_t)(antenna - 1);
    refreshStateGui();
  }
}

BLYNK_WRITE(V1)
{
  if (param.asInt())
  {
    requestAntennaState(selectedAntenna, ANT_BUS1);
    Blynk.virtualWrite(V1, 0);
  }
}

BLYNK_WRITE(V2)
{
  if (param.asInt())
  {
    requestAntennaState(selectedAntenna, ANT_BUS2);
    Blynk.virtualWrite(V2, 0);
  }
}

BLYNK_WRITE(V3)
{
  if (param.asInt())
  {
    requestAntennaState(selectedAntenna, ANT_PARK);
    Blynk.virtualWrite(V3, 0);
  }
}

BLYNK_WRITE(V10)
{
  tx1Locked = (param.asInt() != 0);
  refreshStateGui();
}

BLYNK_WRITE(V11)
{
  tx2Locked = (param.asInt() != 0);
  refreshStateGui();
}

// -----------------------------------------------------------------------------
// Non-blocking network service
// -----------------------------------------------------------------------------
void startWifiAttempt()
{
  lastWifiAttemptMs = millis();
  Serial.println("Starting Wi-Fi connection...");
  WiFi.begin(ssid, pass);
}

void serviceNetwork()
{
  const uint32_t now = millis();
  const bool wifiConnected = (WiFi.status() == WL_CONNECTED);

  if (!wifiConnected)
  {
    if ((now - lastWifiAttemptMs) >= WIFI_RETRY_INTERVAL_MS)
    {
      startWifiAttempt();
    }
  }
  else
  {
    Blynk.run();

    if (!Blynk.connected() &&
        ((now - lastBlynkAttemptMs) >= BLYNK_RETRY_INTERVAL_MS))
    {
      lastBlynkAttemptMs = now;
      feedWatchdog();
      (void)Blynk.connect(250); // Limit UI interruption to about 250 ms.
      feedWatchdog();
    }
  }

  const bool blynkConnected = wifiConnected && Blynk.connected();

  if ((wifiConnected != lastWifiConnected) ||
      (blynkConnected != lastBlynkConnected))
  {
    Serial.printf("Wi-Fi:%s  Blynk:%s\n",
                  wifiConnected ? "ON" : "OFF",
                  blynkConnected ? "ON" : "OFF");

    lastWifiConnected = wifiConnected;
    lastBlynkConnected = blynkConnected;
    refreshNetworkIcons();
  }
}

// -----------------------------------------------------------------------------
// Serial diagnostics
// -----------------------------------------------------------------------------
void printStates()
{
  Serial.println("---- ANTENNA STATES ----");
  for (uint8_t antenna = 0; antenna < NUM_ANTENNAS; antenna++)
  {
    Serial.printf("ANT%u: %s\n", antenna + 1, stateToText(antennaState[antenna]));
  }
  Serial.printf("BUS1:%d BUS2:%d TX1LOCK:%d TX2LOCK:%d\n",
                findAntennaOnBus(ANT_BUS1) + 1,
                findAntennaOnBus(ANT_BUS2) + 1,
                tx1Locked,
                tx2Locked);
}

void printMeasurements()
{
  Serial.printf("TX1 F=%.4fV R=%.4fV P=%.2fW SWR=",
                tx1Measurement.forwardV,
                tx1Measurement.reflectedV,
                tx1Measurement.powerW);
  if (tx1Measurement.swrValid) Serial.printf("%.2f\n", tx1Measurement.swr);
  else Serial.println("---");

  Serial.printf("TX2 F=%.4fV R=%.4fV P=%.2fW SWR=",
                tx2Measurement.forwardV,
                tx2Measurement.reflectedV,
                tx2Measurement.powerW);
  if (tx2Measurement.swrValid) Serial.printf("%.2f\n", tx2Measurement.swr);
  else Serial.println("---");
}

void processSerialCommand(String command)
{
  command.trim();
  command.toLowerCase();

  if (command == "state")
  {
    printStates();
    return;
  }

  if (command == "adc")
  {
    printMeasurements();
    return;
  }

  if (command == "allp")
  {
    for (uint8_t antenna = 0; antenna < NUM_ANTENNAS; antenna++)
    {
      if (!actionBlocked(antenna, ANT_PARK))
      {
        parkAntennaHardware(antenna);
      }
    }
    refreshStateGui();
    return;
  }

  if (command.startsWith("sel"))
  {
    const int antenna = command.substring(3).toInt();
    if ((antenna >= 1) && (antenna <= NUM_ANTENNAS))
    {
      selectedAntenna = (uint8_t)(antenna - 1);
      refreshStateGui();
    }
    return;
  }

  if (command == "tx1on")  { tx1Locked = true;  refreshStateGui(); return; }
  if (command == "tx1off") { tx1Locked = false; refreshStateGui(); return; }
  if (command == "tx2on")  { tx2Locked = true;  refreshStateGui(); return; }
  if (command == "tx2off") { tx2Locked = false; refreshStateGui(); return; }

  if (command.length() >= 2)
  {
    const char first = command.charAt(0);
    if ((first >= '1') && (first <= '8'))
    {
      const uint8_t antenna = (uint8_t)(first - '1');
      if (command.endsWith("b1")) { requestAntennaState(antenna, ANT_BUS1); return; }
      if (command.endsWith("b2")) { requestAntennaState(antenna, ANT_BUS2); return; }
      if (command.endsWith("p"))  { requestAntennaState(antenna, ANT_PARK); return; }
    }
  }

  Serial.println("Commands: state, adc, allp, sel1..sel8, 1b1..8b1, 1b2..8b2, 1p..8p, tx1on/off, tx2on/off");
}

// -----------------------------------------------------------------------------
// Setup and loop
// -----------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(25);
  delay(200);

  setupWatchdog();
  feedWatchdog();

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  measurementSprite.setColorDepth(16);
  measurementSpriteReady =
      (measurementSprite.createSprite(
          MEASUREMENT_SPRITE_W,
          MEASUREMENT_SPRITE_H) != nullptr);

  Serial.println(
      measurementSpriteReady
          ? "Anti-flicker measurement sprite ready."
          : "WARNING: sprite allocation failed; using padded text fallback.");

  touchSPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
  touch.begin(touchSPI);
  touch.setRotation(1);

  Wire.begin();
  Wire.setClock(100000);
// Keep the CYD backlight permanently enabled.
pinMode(TFT_BACKLIGHT_PIN, OUTPUT);
digitalWrite(TFT_BACKLIGHT_PIN, HIGH);

  mcpReady = mcp.begin_I2C(MCP_ADDR, &Wire);
  if (mcpReady)
  {
    initializeRelayOutputs();
    Serial.println("MCP23017 ready; all antennas PARKED.");
  }
  else
  {
    for (uint8_t antenna = 0; antenna < NUM_ANTENNAS; antenna++)
    {
      antennaState[antenna] = ANT_OFF;
    }
    Serial.println("ERROR: MCP23017 not found; switching disabled.");
  }

  adsReady = ads.begin(ADS_ADDR, &Wire);
  if (adsReady)
  {
    ads.setGain(GAIN_ONE);
    ads.setDataRate(RATE_ADS1115_475SPS);
    readMeasurements();
    Serial.println("ADS1115 ready: A0=F1 A1=R1 A2=F2 A3=R2.");
  }
  else
  {
    Serial.println("ERROR: ADS1115 not found.");
  }

  drawStaticGui();

  Blynk.config(BLYNK_AUTH_TOKEN);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  startWifiAttempt();

  blynkTimer.setInterval(BLYNK_PUBLISH_INTERVAL_MS, publishBlynkStatus);

  lastMeasurementMs = millis();
  feedWatchdog();
}

void loop()
{

  feedWatchdog();

  handleTouch();
  serviceNetwork();
  blynkTimer.run();

  if (Serial.available())
  {
    processSerialCommand(Serial.readStringUntil('\n'));
  }

  const uint32_t now = millis();
  if ((now - lastMeasurementMs) >= MEASUREMENT_INTERVAL_MS)
  {
    lastMeasurementMs = now;
    readMeasurements();
    refreshMeasurementGui();
    ;
  }


  // Give the Wi-Fi and idle tasks CPU time without visible GUI delays.
  delay(1);
}
