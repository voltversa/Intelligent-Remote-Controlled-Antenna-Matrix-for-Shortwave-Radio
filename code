#define BLYNK_TEMPLATE_ID           "TMPL5a_V3qF9u"
#define BLYNK_TEMPLATE_NAME         "Quickstart Device"
#define BLYNK_AUTH_TOKEN            "KVQa9iER-hpuwKJPrIG5CwOObkqpFMzp"


#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_ADS1X15.h>
#include "driver/pcnt.h"
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>



//  WiFi credentials.
char ssid[] = "Proximus-Home-65D8";
char pass[] = "0";
BlynkTimer timer;

bool wifiReady = false;
bool blynkReady = false;

unsigned long lastWifiRetry = 0;
const unsigned long wifiRetryIntervalMs = 10000;   // retry every 10 seconds

float tx1FreqHz = 0.0f;
float tx2FreqHz = 0.0f;
float vfrw1 = 0.0f;
float vref1 = 0.0f;
float vfrw2 = 0.0f;
float vref2 = 0.0f;
float swr1 = 0.0f;
float swr2 = 0.0f;
float pwr1W = 0.0f;
float pwr2W = 0.0f;
unsigned long lastFreqUpdate = 0;
const unsigned long freqUpdateIntervalMs = 200;   // update 5 times/s
// =====================================================
// TOUCH PINS
// =====================================================
#define TOUCH_MOSI 32
#define TOUCH_MISO 39
#define TOUCH_CLK  25
#define TOUCH_CS   33
#define TOUCH_IRQ  36
#define TX1_FREQ_PIN 35
#define TX2_FREQ_PIN 27
SPIClass touchSPI(VSPI);

// =====================================================
// DEVICES
// =====================================================
TFT_eSPI tft = TFT_eSPI();
XPT2046_Touchscreen ts(TOUCH_CS, TOUCH_IRQ);
Adafruit_MCP23X17 mcp;
Adafruit_ADS1115 ads;

#define MCP_ADDR 0x20
#define ADS_ADDR 0x48

// =====================================================
// RELAY LOGIC
// =====================================================
#define RELAY_ON   LOW
#define RELAY_OFF  HIGH
bool tx1Locked = false;
bool tx2Locked = false;
const uint8_t NUM_ANTENNAS = 8;

// GPA0..GPA7 = selector relays
// GPB0..GPB7 = ground-clamp relays
const uint8_t selectorPins[NUM_ANTENNAS] = {0, 1, 2, 3, 4, 5, 6, 7};
const uint8_t gndPins[NUM_ANTENNAS]      = {8, 9, 10, 11, 12, 13, 14, 15};

// =====================================================
// ADS1115 CHANNEL MAPPING
// =====================================================
#define CH_VFRW1 0
#define CH_VREF1 1
#define CH_VFRW2 2
#define CH_VREF2 3

bool adsReady = false;

// =====================================================
// ANTENNA STATE
// =====================================================
enum AntennaState
{
  ANT_OFF = 0,
  ANT_BUS1,
  ANT_BUS2,
  ANT_PARK
};

AntennaState antennaState[NUM_ANTENNAS];

// =====================================================
// GUI STATE
// =====================================================
int selectedAntenna = 0;
unsigned long lastRefresh = 0;

// =====================================================
// TOUCH CALIBRATION
// =====================================================
#define TS_MINX 278
#define TS_MAXX 3702
#define TS_MINY 484
#define TS_MAXY 3787

// =====================================================
// BUTTON STRUCT
// =====================================================
struct Button {
  int x, y, w, h;
  const char* label;
};

Button antButtons[8] = {
  {10, 40, 52, 26, "ANT1"},
  {66, 40, 52, 26, "ANT2"},
  {122, 40, 52, 26, "ANT3"},
  {178, 40, 52, 26, "ANT4"},
  {10, 70, 52, 26, "ANT5"},
  {66, 70, 52, 26, "ANT6"},
  {122, 70, 52, 26, "ANT7"},
  {178, 70, 52, 26, "ANT8"}
};

Button btnBus1 = {236, 40, 74, 26, "BUS1"};
Button btnBus2 = {236, 70, 74, 26, "BUS2"};
Button btnPark = {236, 100, 74, 26, "PARK"};

// Forward declarations
void updateStatusUI();
void updateADCUI();

// =====================================================
// HELPERS
// =====================================================
bool isValidAntenna(uint8_t ant)
{
  return ant < NUM_ANTENNAS;
}

const char* stateToString(AntennaState s)
{
  switch (s)
  {
    case ANT_BUS1: return "BUS1";
    case ANT_BUS2: return "BUS2";
    case ANT_PARK: return "PARK";
    default:       return "OFF";
  }
}

float clampFloat(float x, float minVal, float maxVal)
{
  if (x < minVal) return minVal;
  if (x > maxVal) return maxVal;
  return x;
}
float calculateForwardPowerW(float forwardV)
{
  // Psample = U^2 / (2R), R=50 ohm  => U^2 / 100
  // Coupler is -30 dB => multiply power by 1000
  // So Pline = 10 * U^2

  if (forwardV < 0.01f) return 0.0f;

  return 10.0f * forwardV * forwardV;
}

bool rfPresent(float forwardV)
{
  // Minimum forward voltage needed before SWR is meaningful.
  // You can tune this value.
  return forwardV >= 0.15f;
}

bool validSWRInput(float forwardV, float reflectedV)
{
  if (!rfPresent(forwardV)) return false;

  // Reflected detector voltage should normally be lower than forward.
  // If it is higher, the reading is not reliable.
  if (reflectedV >= forwardV) return false;

  // Ignore very small differences caused by floating/noise.
  if ((forwardV - reflectedV) < 0.05f) return false;

  return true;
}

String swrToDisplay(float forwardV, float reflectedV, float swr)
{
  if (!rfPresent(forwardV))
    return "---";

  if (!validSWRInput(forwardV, reflectedV))
    return "---";

  if (swr < 0)
    return "---";

  if (swr > 9.99f)
    return ">9.99";

  return String(swr, 2);
}
float calculateSWR(float forwardV, float reflectedV)
{
  // No meaningful RF present
  if (forwardV < 0.15f)
    return -1.0f;

  // Invalid detector condition: reflected cannot be equal/higher than forward
  if (reflectedV >= forwardV)
    return -1.0f;

  // Avoid unstable calculation when values are too close
  if ((forwardV - reflectedV) < 0.05f)
    return -1.0f;

  float gamma = sqrt(reflectedV / forwardV);
  gamma = clampFloat(gamma, 0.0f, 0.95f);

  float swr = (1.0f + gamma) / (1.0f - gamma);

  if (swr > 9.99f) swr = 9.99f;
  return swr;
}
void initStates()
{
  for (int i = 0; i < NUM_ANTENNAS; i++)
  {
    antennaState[i] = ANT_OFF;
  }
}

// =====================================================
// LOW-LEVEL HARDWARE ACTIONS
// BUS1/BUS2 swapped as requested
// =====================================================
void rawParkAntenna(uint8_t ant)
{
  if (!isValidAntenna(ant)) return;

  mcp.digitalWrite(gndPins[ant], RELAY_ON);
  antennaState[ant] = ANT_PARK;

  Serial.print("ANT");
  Serial.print(ant + 1);
  Serial.println(" -> PARK");
}

void rawSetAntennaBus1(uint8_t ant)
{
  if (!isValidAntenna(ant)) return;

  mcp.digitalWrite(gndPins[ant], RELAY_OFF);
  delay(20);

  // selector ON = BUS1
  mcp.digitalWrite(selectorPins[ant], RELAY_ON);
  antennaState[ant] = ANT_BUS1;

  Serial.print("ANT");
  Serial.print(ant + 1);
  Serial.println(" -> BUS1");
}

void rawSetAntennaBus2(uint8_t ant)
{
  if (!isValidAntenna(ant)) return;

  mcp.digitalWrite(gndPins[ant], RELAY_OFF);
  delay(20);

  // selector OFF = BUS2
  mcp.digitalWrite(selectorPins[ant], RELAY_OFF);
  antennaState[ant] = ANT_BUS2;

  Serial.print("ANT");
  Serial.print(ant + 1);
  Serial.println(" -> BUS2");
}

// =====================================================
// GLOBAL INTERLOCK HELPERS
// =====================================================
int findAntennaOnBus(AntennaState busState)
{
  for (int i = 0; i < NUM_ANTENNAS; i++)
  {
    if (antennaState[i] == busState)
    {
      return i;
    }
  }
  return -1;
}

void parkAllAntennas()
{
  for (uint8_t i = 0; i < NUM_ANTENNAS; i++)
  {
    rawParkAntenna(i);
    delay(10);
  }
}

// =====================================================
// INTERLOCK LOGIC
// =====================================================
int getBusOfAntenna(uint8_t ant)
{
  if (!isValidAntenna(ant)) return 0;

  if (antennaState[ant] == ANT_BUS1) return 1;
  if (antennaState[ant] == ANT_BUS2) return 2;
  return 0;
}

bool isActionBlocked(uint8_t ant, AntennaState requestedState)
{
  int currentBus = getBusOfAntenna(ant);

  // block putting any antenna onto BUS1 while TX1 is active
  if (requestedState == ANT_BUS1 && tx1Locked) return true;

  // block putting any antenna onto BUS2 while TX2 is active
  if (requestedState == ANT_BUS2 && tx2Locked) return true;

  // block removing/parking an antenna that is currently on a locked bus
  if (requestedState == ANT_PARK)
  {
    if (currentBus == 1 && tx1Locked) return true;
    if (currentBus == 2 && tx2Locked) return true;
  }

  // block moving an antenna away from a locked bus
  if (currentBus == 1 && tx1Locked && requestedState != ANT_BUS1) return true;
  if (currentBus == 2 && tx2Locked && requestedState != ANT_BUS2) return true;

  return false;
}


bool requestAntennaState(uint8_t ant, AntennaState requestedState)
{
  if (!isValidAntenna(ant))
  {
    Serial.println("ERROR: invalid antenna index");
    return false;
  }
  if (isActionBlocked(ant, requestedState))
  {
    Serial.println("Switching blocked: TX lock active");
    return false;
  }
  Serial.print("Request ANT");
  Serial.print(ant + 1);
  Serial.print(" -> ");
  Serial.println(stateToString(requestedState));

  if (requestedState == ANT_PARK)
  {
    rawParkAntenna(ant);
    return true;
  }

  if (requestedState == ANT_BUS1)
  {
    int currentBus1 = findAntennaOnBus(ANT_BUS1);
    if (currentBus1 >= 0 && currentBus1 != ant)
    {
      rawParkAntenna(currentBus1);
      delay(20);
    }

    if (antennaState[ant] == ANT_BUS2)
    {
      rawParkAntenna(ant);
      delay(20);
    }

    rawSetAntennaBus1(ant);
    return true;
  }

  if (requestedState == ANT_BUS2)
  {
    int currentBus2 = findAntennaOnBus(ANT_BUS2);
    if (currentBus2 >= 0 && currentBus2 != ant)
    {
      rawParkAntenna(currentBus2);
      delay(20);
    }

    if (antennaState[ant] == ANT_BUS1)
    {
      rawParkAntenna(ant);
      delay(20);
    }

    rawSetAntennaBus2(ant);
    return true;
  }

  return false;
}

// =====================================================
// ADS1115
// =====================================================
float readVoltageAvg(uint8_t channel)
{
  if (!adsReady) return 0.0f;

  long sum = 0;
  const int samples = 8;

  for (int i = 0; i < samples; i++)
  {
    sum += ads.readADC_SingleEnded(channel);
    delay(2);
  }

  float avgRaw = sum / (float)samples;

  // GAIN_ONE => 0.125 mV/bit
  return avgRaw * 0.125f / 1000.0f;
}

// =====================================================
// DIAGNOSTICS
// =====================================================
void printStates()
{
  Serial.println("------ ANTENNA STATES ------");

  for (int i = 0; i < NUM_ANTENNAS; i++)
  {
    Serial.print("ANT");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(stateToString(antennaState[i]));
  }

  int bus1Ant = findAntennaOnBus(ANT_BUS1);
  int bus2Ant = findAntennaOnBus(ANT_BUS2);

  Serial.print("BUS1: ");
  if (bus1Ant >= 0) Serial.println(bus1Ant + 1);
  else Serial.println("none");

  Serial.print("BUS2: ");
  if (bus2Ant >= 0) Serial.println(bus2Ant + 1);
  else Serial.println("none");
}

void printSerialMenu()
{
  Serial.println();
  Serial.println("===== SERIAL COMMANDS =====");
  Serial.println("1b1 ... 8b1   -> ANTn to BUS1");
  Serial.println("1b2 ... 8b2   -> ANTn to BUS2");
  Serial.println("1p  ... 8p    -> ANTn PARK");
  Serial.println("allp          -> Park all antennas");
  Serial.println("state         -> Print all states");
  Serial.println("sel1 ... sel8 -> Select antenna in GUI");
  Serial.println("tx1on         -> Lock BUS1 switching");
  Serial.println("tx1off        -> Unlock BUS1 switching");
  Serial.println("tx2on         -> Lock BUS2 switching");
  Serial.println("tx2off        -> Unlock BUS2 switching");
  Serial.println();
}

void processSerialCommand(String cmd)
{
  cmd.trim();
  cmd.toLowerCase();

  if (cmd == "allp")
  {
    parkAllAntennas();
    updateStatusUI();
    updateADCUI();
    Serial.println("All antennas parked");
    return;
  }

  if (cmd == "state")
  {
    printStates();
    return;
  }

  if (cmd.startsWith("sel"))
  {
    int antNum = cmd.substring(3).toInt();

    if (antNum >= 1 && antNum <= 8)
    {
      selectedAntenna = antNum - 1;
      Serial.print("Selected antenna from serial: ANT");
      Serial.println(antNum);
      updateStatusUI();
      updateADCUI();
    }
    else
    {
      Serial.println("Invalid antenna number");
    }
    return;
  }

  if (cmd == "tx1on")
  {
    tx1Locked = true;
    Serial.println("TX1 lock ON");
    updateStatusUI();
    return;
  }

  if (cmd == "tx1off")
  {
    tx1Locked = false;
    Serial.println("TX1 lock OFF");
    updateStatusUI();
    return;
  }

  if (cmd == "tx2on")
  {
    tx2Locked = true;
    Serial.println("TX2 lock ON");
    updateStatusUI();
    return;
  }

  if (cmd == "tx2off")
  {
    tx2Locked = false;
    Serial.println("TX2 lock OFF");
    updateStatusUI();
    return;
  }
  if (cmd.length() >= 2)
  {
    char antChar = cmd.charAt(0);

    if (antChar >= '1' && antChar <= '8')
    {
      uint8_t ant = antChar - '1';

      if (cmd.endsWith("b1"))
      {
        requestAntennaState(ant, ANT_BUS1);
        updateStatusUI();
        updateADCUI();
        return;
      }
      else if (cmd.endsWith("b2"))
      {
        requestAntennaState(ant, ANT_BUS2);
        updateStatusUI();
        updateADCUI();
        return;
      }
      else if (cmd.endsWith("p"))
      {
        requestAntennaState(ant, ANT_PARK);
        updateStatusUI();
        updateADCUI();
        return;
      }
    }
  }

  Serial.print("Unknown command: ");
  Serial.println(cmd);
  printSerialMenu();
}
// =====================================================
// BLYNK
// =====================================================
void sendBlynkStatus()
{

 if (WiFi.status() != WL_CONNECTED || !Blynk.connected())
    return;

  int bus1Ant = findAntennaOnBus(ANT_BUS1);
  int bus2Ant = findAntennaOnBus(ANT_BUS2);

  Blynk.virtualWrite(V0, selectedAntenna + 1);
  Blynk.virtualWrite(V4, (bus1Ant >= 0) ? bus1Ant + 1 : 0);
  Blynk.virtualWrite(V5, (bus2Ant >= 0) ? bus2Ant + 1 : 0);
  Blynk.virtualWrite(V6, pwr1W);
  Blynk.virtualWrite(V8, pwr2W);
Blynk.virtualWrite(V7, validSWRInput(vfrw1, vref1) ? swr1 : 0);
Blynk.virtualWrite(V9, validSWRInput(vfrw2, vref2) ? swr2 : 0);
  Blynk.virtualWrite(V10, tx1Locked ? 1 : 0);
  Blynk.virtualWrite(V11, tx2Locked ? 1 : 0);
  Blynk.virtualWrite(V12, (tx1FreqHz < 1.0f) ? 0.0f : tx1FreqHz / 1000.0f);
  Blynk.virtualWrite(V13, (tx2FreqHz < 1.0f) ? 0.0f : tx2FreqHz / 1000.0f);

}

BLYNK_WRITE(V0)
{
  int ant = param.asInt();
  if (ant >= 1 && ant <= 8)
  {
    selectedAntenna = ant - 1;
    updateStatusUI();
    updateADCUI();
  }
}

BLYNK_WRITE(V1)
{
  if (param.asInt())
  {
    requestAntennaState(selectedAntenna, ANT_BUS1);
    updateStatusUI();
    updateADCUI();
  }
}

BLYNK_WRITE(V2)
{
  if (param.asInt())
  {
    requestAntennaState(selectedAntenna, ANT_BUS2);
    updateStatusUI();
    updateADCUI();
  }
}

BLYNK_WRITE(V3)
{
  if (param.asInt())
  {
    requestAntennaState(selectedAntenna, ANT_PARK);
    updateStatusUI();
    updateADCUI();
  }
}

BLYNK_WRITE(V10)
{
  tx1Locked = param.asInt();
  updateStatusUI();
}

BLYNK_WRITE(V11)
{
  tx2Locked = param.asInt();
  updateStatusUI();
}
void connectWiFiAndBlynkNonBlocking()
{
  // Already connected to Wi-Fi
  if (WiFi.status() == WL_CONNECTED)
  {
    wifiReady = true;

    // Try Blynk only if not already connected
    if (!Blynk.connected())
    {
      Serial.println("WiFi connected. Trying Blynk...");
      blynkReady = Blynk.connect(3000);  // try for max 3 seconds

      if (blynkReady)
        Serial.println("Blynk connected.");
      else
        Serial.println("Blynk not connected. Local mode continues.");
    }

    return;
  }

  wifiReady = false;
  blynkReady = false;

  unsigned long now = millis();

  // Retry Wi-Fi only every 10 seconds
  if (now - lastWifiRetry < wifiRetryIntervalMs) return;
  lastWifiRetry = now;

  Serial.println("Trying WiFi connection...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  unsigned long startAttempt = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 3000)
  {
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    wifiReady = true;
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());

    blynkReady = Blynk.connect(3000);

    if (blynkReady)
      Serial.println("Blynk connected.");
    else
      Serial.println("Blynk failed. Local mode continues.");
  }
  else
  {
    Serial.println("WiFi failed. Local mode continues.");
  }
}



void updateFrequencyMeasurement()
{
  unsigned long now = millis();
  if (now - lastFreqUpdate < freqUpdateIntervalMs) return;

  float gateSec = (now - lastFreqUpdate) / 1000.0f;
  lastFreqUpdate = now;

  int16_t count1 = readAndClearPCNT(PCNT_UNIT_0);
  int16_t count2 = readAndClearPCNT(PCNT_UNIT_1);

  tx1FreqHz = count1 / gateSec;
  tx2FreqHz = count2 / gateSec;
}


// =====================================================
// GUI DRAWING
// =====================================================
bool pointInButton(int px, int py, const Button& b)
{
  return (px >= b.x && px <= (b.x + b.w) &&
          py >= b.y && py <= (b.y + b.h));
}

void drawButton(const Button& b, uint16_t fillColor, uint16_t textColor)
{
  tft.fillRoundRect(b.x, b.y, b.w, b.h, 6, fillColor);
  tft.drawRoundRect(b.x, b.y, b.w, b.h, 6, TFT_WHITE);
  tft.setTextColor(textColor, fillColor);
  tft.setTextDatum(MC_DATUM);
  tft.drawString(b.label, b.x + b.w / 2, b.y + b.h / 2, 2);
}

void drawStaticUI()
{
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.drawString("Antenna Matrix", 10, 6, 2);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("Select antenna", 10, 22, 2);

  for (int i = 0; i < 8; i++)
  {
    uint16_t color = (i == selectedAntenna) ? TFT_BLUE : TFT_DARKGREY;
    drawButton(antButtons[i], color, TFT_WHITE);
  }

  drawButton(btnBus1, TFT_GREEN, TFT_BLACK);
  drawButton(btnBus2, TFT_ORANGE, TFT_BLACK);
  drawButton(btnPark, TFT_RED, TFT_WHITE);

  tft.drawRoundRect(8, 100, 220, 60, 15, TFT_WHITE);
  // tft.drawRoundRect(8, 165, 135, 60, 15, TFT_WHITE);
  // tft.drawRoundRect(165, 165, 130, 60, 15, TFT_WHITE);
}

void updateStatusUI()
{
  // redraw antenna buttons
  for (int i = 0; i < 8; i++)
  {
    uint16_t color = (i == selectedAntenna) ? TFT_BLUE : TFT_DARKGREY;
    drawButton(antButtons[i], color, TFT_WHITE);
  }

  // selected antenna + state area
  tft.fillRect(17, 105, 200, 20, TFT_BLUE);
  tft.setTextColor(TFT_WHITE, TFT_BLUE);
  tft.setTextDatum(TL_DATUM);

  tft.drawString("ANT" + String(selectedAntenna + 1), 16, 110, 2);
  tft.drawString("State: " + String(stateToString(antennaState[selectedAntenna])), 120, 110, 2);

  // BUS summary
  int bus1Ant = findAntennaOnBus(ANT_BUS1);
  int bus2Ant = findAntennaOnBus(ANT_BUS2);

  tft.setTextColor(TFT_BLACK, TFT_GREEN);
  tft.fillRect(16, 132, 74, 18, TFT_GREEN);
  tft.drawString("BUS1: " + String((bus1Ant >= 0) ? bus1Ant + 1 : 0), 16, 135, 2);

  tft.setTextColor(TFT_BLACK, TFT_YELLOW);
  tft.fillRect(120, 132, 74, 18, TFT_YELLOW);
  tft.drawString("BUS2: " + String((bus2Ant >= 0) ? bus2Ant + 1 : 0), 120, 135, 2);

  // Frequency area on right
  tft.setTextColor(TFT_WHITE, TFT_BLACK);


    tft.drawString("T1:" + String(tx1FreqHz / 1000.0f, 1) + "k", 236, 130, 2);

  if (tx2FreqHz < 1.0f)
    tft.drawString("T2: ---", 236, 150, 2);
  else
    tft.drawString("T2:" + String(tx2FreqHz / 1000.0f, 1) + "k", 236, 150, 2);
}

void updateADCUI()
{
  vfrw1 = readVoltageAvg(CH_VFRW1);
  vref1 = readVoltageAvg(CH_VREF1);
  vfrw2 = readVoltageAvg(CH_VFRW2);
  vref2 = readVoltageAvg(CH_VREF2);

  swr1 = calculateSWR(vfrw1, vref1);
  swr2 = calculateSWR(vfrw2, vref2);

  pwr1W = calculateForwardPowerW(vfrw1);
  pwr2W = calculateForwardPowerW(vfrw2);
  // Channel 1
  tft.drawString("F1: " + String(vfrw1, 2) + "V", 10, 175, 2);
  tft.drawString("R1: " + String(vref1, 2) + "V", 10, 200, 2);
  tft.drawString("P1: " + String(pwr1W, 2) + "W", 89, 175, 2);

 tft.drawString("S1: " + swrToDisplay(vfrw1, vref1, swr1), 89, 200, 2);

  // Channel 2
  tft.drawString("F2: " + String(vfrw2, 2) + "V", 173, 175, 2);
  tft.drawString("R2: " + String(vref2, 2) + "V", 173, 200, 2);
  tft.drawString("P2: " + String(pwr2W, 2) + "W", 249, 175, 2);

tft.drawString("S2: " + swrToDisplay(vfrw2, vref2, swr2), 249, 200, 2);
}
// =====================================================
// TOUCH HANDLING
// =====================================================
bool getTouch(int& x, int& y)
{
  if (!ts.touched()) return false;

  TS_Point p = ts.getPoint();
  if (p.z < 300) return false;

  x = map(p.x, TS_MINX, TS_MAXX, 0, 320);
  y = map(p.y, TS_MINY, TS_MAXY, 0, 240);

  x = constrain(x, 0, 319);
  y = constrain(y, 0, 239);

  return true;
}

void waitForTouchRelease()
{
  unsigned long start = millis();

  while (ts.touched())
  {
    delay(10);
    if (millis() - start > 1000) break;
  }

  delay(80);
}

void handleTouch()
{
  int x, y;
  if (!getTouch(x, y)) return;

  Serial.print("Touch X=");
  Serial.print(x);
  Serial.print(" Y=");
  Serial.println(y);

  for (int i = 0; i < 8; i++)
  {
    if (pointInButton(x, y, antButtons[i]))
    {
      Serial.print("Pressed ANT");
      Serial.println(i + 1);

      selectedAntenna = i;
      updateStatusUI();
      updateADCUI();
      waitForTouchRelease();
      return;
    }
  }

  if (pointInButton(x, y, btnBus1))
  {
    Serial.println("Pressed BUS1");
    bool ok = requestAntennaState(selectedAntenna, ANT_BUS1);
    Serial.print("BUS1 request result = ");
    Serial.println(ok ? "OK" : "FAIL");
    updateStatusUI();
    updateADCUI();
    waitForTouchRelease();
    return;
  }

  if (pointInButton(x, y, btnBus2))
  {
    Serial.println("Pressed BUS2");
    bool ok = requestAntennaState(selectedAntenna, ANT_BUS2);
    Serial.print("BUS2 request result = ");
    Serial.println(ok ? "OK" : "FAIL");
    updateStatusUI();
    updateADCUI();
    waitForTouchRelease();
    return;
  }

  if (pointInButton(x, y, btnPark))
  {
    Serial.println("Pressed PARK");
    bool ok = requestAntennaState(selectedAntenna, ANT_PARK);
    Serial.print("PARK request result = ");
    Serial.println(ok ? "OK" : "FAIL");
    updateStatusUI();
    updateADCUI();
    waitForTouchRelease();
    return;
  }

  Serial.println("Touch detected, but no button matched.");
  waitForTouchRelease();
}

void setupPCNTUnit(pcnt_unit_t unit, int pin)
{
  pcnt_config_t pcnt_config = {};
  pcnt_config.pulse_gpio_num = pin;
  pcnt_config.ctrl_gpio_num  = PCNT_PIN_NOT_USED;
  pcnt_config.unit           = unit;
  pcnt_config.channel        = PCNT_CHANNEL_0;
  pcnt_config.pos_mode       = PCNT_COUNT_INC;
  pcnt_config.neg_mode       = PCNT_COUNT_DIS;
  pcnt_config.lctrl_mode     = PCNT_MODE_KEEP;
  pcnt_config.hctrl_mode     = PCNT_MODE_KEEP;
  pcnt_config.counter_h_lim  = 32767;
  pcnt_config.counter_l_lim  = 0;

  pcnt_unit_config(&pcnt_config);

  pcnt_filter_disable(unit);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

int16_t readAndClearPCNT(pcnt_unit_t unit)
{
  int16_t count = 0;
  pcnt_get_counter_value(unit, &count);
  pcnt_counter_clear(unit);
  return count;
}

void setupFrequencyInputs()
{
  pinMode(TX1_FREQ_PIN, INPUT);
  pinMode(TX2_FREQ_PIN, INPUT);

  setupPCNTUnit(PCNT_UNIT_0, TX1_FREQ_PIN);
  setupPCNTUnit(PCNT_UNIT_1, TX2_FREQ_PIN);
}
// =====================================================
// SETUP
// IMPORTANT: keep TFT + touch init first on this CYD
// =====================================================
void setup()
{
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  touchSPI.begin(TOUCH_CLK, TOUCH_MISO, TOUCH_MOSI, TOUCH_CS);
  ts.begin(touchSPI);
  ts.setRotation(1);

  drawStaticUI();

  Serial.begin(115200);
  delay(1000);

  Wire.begin();
setupFrequencyInputs();
lastFreqUpdate = millis();

  if (!mcp.begin_I2C(MCP_ADDR))
  {
    Serial.println("ERROR: MCP23017 not found!");
    while (1) delay(100);
  }

  if (!ads.begin(ADS_ADDR))
  {
    Serial.println("ERROR: ADS1115 not found!");
    adsReady = false;
  }
  else
  {
    ads.setGain(GAIN_ONE);
    adsReady = true;
    Serial.println("ADS1115 ready.");
  }

  for (int i = 0; i < 16; i++)
  {
    mcp.pinMode(i, OUTPUT);
  }

  initStates();
  parkAllAntennas();

  updateStatusUI();
  updateADCUI();

  Serial.println("BOOT TEST START");
  rawSetAntennaBus1(0);
  delay(1000);
  rawSetAntennaBus2(0);
  delay(1000);
  rawParkAntenna(0);
  Serial.println("BOOT TEST END");

  updateStatusUI();
  updateADCUI();
WiFi.mode(WIFI_STA);
WiFi.setAutoReconnect(true);

Blynk.config(BLYNK_AUTH_TOKEN);

// Try once, but continue if it fails
connectWiFiAndBlynkNonBlocking();

timer.setInterval(1000L, sendBlynkStatus);

  printSerialMenu();
}

// =====================================================
// LOOP
// =====================================================
void loop()
{
  handleTouch();

  if (Serial.available())
  {
    String cmd = Serial.readStringUntil('\n');
    processSerialCommand(cmd);
  }

  updateFrequencyMeasurement();

  if (millis() - lastRefresh > 1000)
  {
    lastRefresh = millis();
    updateStatusUI();
    updateADCUI();
  }

  connectWiFiAndBlynkNonBlocking();

  if (WiFi.status() == WL_CONNECTED)
  {
    Blynk.run();
  }

  timer.run();
}
