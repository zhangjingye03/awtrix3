#include <PeripheryManager.h>
#include "Adafruit_SHT31.h"
#include "Adafruit_BME280.h"
#include "Adafruit_BMP280.h"
#include "Adafruit_HTU21DF.h"
#include "Adafruit_AHTX0.h"
#include <BH1750.h>
#ifndef ESP32_C3
#include "SoftwareSerial.h"
#include <DFMiniMp3.h>
#endif
#include <MelodyPlayer/melody_player.h>
#include <MelodyPlayer/melody_factory.h>
#include "Globals.h"
#include "DisplayManager.h"
#include "MQTTManager.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <LightDependentResistor.h>
#include <MenuManager.h>
#include <ServerManager.h>
#include <MedianFilterLib.h>
#include <MeanFilterLib.h>
#include <Games/GameManager.h>
#include <math.h>
const int buzzerPin = 2;       // Buzzer an GPIO2
const int baudRate = 50;       // Nachrichtenübertragungsrate
const char *message = "HELLO"; // Die Nachricht, die gesendet werden soll
#define LEDC_CHANNEL 0
#define LEDC_RESOLUTION 8 // 8 bit resolution
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define MEDIAN_WND 7 // A median filter window size of seven should be enough to filter out most spikes
#define MEAN_WND 7   // After filtering the spikes we don't need many samples anymore for the average

#ifndef ESP32_C3
#define DFPLAYER_RX 23
#define DFPLAYER_TX 18
#define BUZZER_PIN 15
#define RESET_PIN 13
#endif

#ifdef awtrix2_upgrade
// Pinouts für das WEMOS_D1_MINI32-Environment
#define LDR_PIN A0
#define BUTTON_UP_PIN D0
#define BUTTON_DOWN_PIN D8
#define BUTTON_SELECT_PIN D4

#define I2C_SCL_PIN D1
#define I2C_SDA_PIN D3

#elif ESP32_C3

#define LDR_PIN 0
#define BUTTON_UP_PIN 5
#define BUTTON_DOWN_PIN 7
#define BUTTON_SELECT_PIN 6
#define I2C_SCL_PIN 9
#define I2C_SDA_PIN 8
#define BUZZER_PIN -1
#define RESET_PIN -1
#ifndef LD2402_RX_PIN
#define LD2402_RX_PIN 4
#endif
#ifndef LD2402_TX_PIN
#define LD2402_TX_PIN -1
#endif
#ifndef LD2402_ENABLED
#define LD2402_ENABLED 1
#endif
#ifndef LD2402_ENGINEERING_MODE
#define LD2402_ENGINEERING_MODE 1
#endif
#define LD2402_BAUD 115200
#elif ESP32_S3
#define BATTERY_PIN 4
#define BUZZER_PIN 5
#define LDR_PIN 6
#define BUTTON_UP_PIN 7
#define BUTTON_DOWN_PIN 8
#define BUTTON_SELECT_PIN 10
#define I2C_SCL_PIN 10
#define I2C_SDA_PIN 11
#else
// Pinouts für das ULANZI-Environment
#define BATTERY_PIN 34

#define LDR_PIN 35
#define BUTTON_UP_PIN 26
#define BUTTON_DOWN_PIN 14
#define BUTTON_SELECT_PIN 27
#define I2C_SCL_PIN 22
#define I2C_SDA_PIN 21
#endif

#define HAS_BUZZER (BUZZER_PIN >= 0)
#define HAS_RESET_BUTTON (RESET_PIN >= 0)
#define PLAYER_PIN (HAS_BUZZER ? BUZZER_PIN : BUTTON_SELECT_PIN)

Adafruit_BME280 bme280;
Adafruit_BMP280 bmp280;
Adafruit_HTU21DF htu21df;
Adafruit_SHT31 sht31;
Adafruit_AHTX0 ahtx0;
Adafruit_Sensor *ahtx0_humidity, *ahtx0_temp;
BH1750 bh1750;
bool bh1750Detected = false;

#if defined(ESP32_C3) && LD2402_ENABLED
HardwareSerial ld2402Serial(1);
String ld2402Line;
bool ld2402ReportedOnce = false;
unsigned long lastLD2402Log = 0;
uint32_t ld2402BytesReceived = 0;
uint32_t ld2402ValidFrames = 0;
unsigned long ld2402LastFrameAt = 0;
bool ld2402CalibrationActive = false;
unsigned long ld2402CalibrationStartedAt = 0;
unsigned long ld2402CalibrationLastQueryAt = 0;
uint16_t ld2402CalibrationProgress = 0;
bool ld2402CalibrationSavePending = false;
const char *ld2402CalibrationState = "idle";
static const unsigned long LD2402_CALIBRATION_QUERY_INTERVAL_MS = 1000;
static const unsigned long LD2402_CALIBRATION_TIMEOUT_MS = 45000;
#endif

#ifdef awtrix2_upgrade
#define USED_PHOTOCELL LightDependentResistor::GL5528
#define PHOTOCELL_SERIES_RESISTOR 1000
#else
#define USED_PHOTOCELL LightDependentResistor::GL5516
#define PHOTOCELL_SERIES_RESISTOR 10000
#endif

#ifndef ESP32_C3
class Mp3Notify
{
};
SoftwareSerial mySoftwareSerial(DFPLAYER_RX, DFPLAYER_TX); // RX, TX
DFMiniMp3<SoftwareSerial, Mp3Notify> dfmp3(mySoftwareSerial);
#endif

#ifdef BUZZER_ACTIVE_LOW
MelodyPlayer player(PLAYER_PIN, 1, HIGH);
#else
MelodyPlayer player(PLAYER_PIN, 1, LOW);
#endif

#ifdef BUTTON_ACTIVE_HIGH
EasyButton button_left(BUTTON_UP_PIN, 35, true, false);
EasyButton button_right(BUTTON_DOWN_PIN, 35, true, false);
EasyButton button_select(BUTTON_SELECT_PIN, 35, true, false);
EasyButton button_reset(HAS_RESET_BUTTON ? RESET_PIN : BUTTON_SELECT_PIN);
#else
EasyButton button_left(BUTTON_UP_PIN);
EasyButton button_right(BUTTON_DOWN_PIN);
EasyButton button_select(BUTTON_SELECT_PIN);
EasyButton button_reset(HAS_RESET_BUTTON ? RESET_PIN : BUTTON_SELECT_PIN);
#endif

LightDependentResistor photocell(LDR_PIN,
                                 PHOTOCELL_SERIES_RESISTOR,
                                 USED_PHOTOCELL,
                                 10,
                                 10);

int readIndex = 0;
int sampleIndex = 0;
unsigned long previousMillis_BatTempHum = 0;
unsigned long previousMillis_LDR = 0;
unsigned long previousMillis_SensorStatus = 0;
const unsigned long interval_BatTempHum = 10000;
const unsigned long interval_LDR = 100;
const unsigned long interval_SensorStatus = 2000;
static bool hasValidTempHum = false;
static float lastValidTemp = 0;
static float lastValidHum = 0;
static unsigned long lastInvalidTempHumLog = 0;
int total = 0;
unsigned long startTime;

MedianFilter<uint16_t> medianFilterBatt(MEDIAN_WND);
MedianFilter<uint16_t> medianFilterLDR(MEDIAN_WND);
MeanFilter<uint16_t> meanFilterBatt(MEAN_WND);
MeanFilter<uint16_t> meanFilterLDR(MEAN_WND);

float brightnessPercent = 0.0;

PeripheryManager_::PeripheryManager_()
{
    this->buttonL = &button_left;
    this->buttonR = &button_right;
    this->buttonS = &button_select;
    this->buttonRST = &button_reset;
}

// The getter for the instantiated singleton instance
PeripheryManager_ &PeripheryManager_::getInstance()
{
    static PeripheryManager_ instance;
    return instance;
}

// Initialize the global shared instance
PeripheryManager_ &PeripheryManager = PeripheryManager.getInstance();

#if defined(ESP32_C3) && LD2402_ENABLED
static void setLD2402State(bool presence, uint16_t distanceCm, int8_t movingState = -1)
{
    const bool stateChanged = !LD2402_AVAILABLE || LD2402_PRESENCE != presence || LD2402_MOVING_STATE != movingState;
    LD2402_AVAILABLE = true;
    LD2402_PRESENCE = presence;
    LD2402_MOVING_STATE = movingState;
    LD2402_DISTANCE_CM = distanceCm;
    ld2402ValidFrames++;
    ld2402LastFrameAt = millis();

    if (stateChanged)
        MQTTManager.publishLD2402State();

    if (DEBUG_MODE && (!ld2402ReportedOnce || millis() - lastLD2402Log >= 5000))
    {
        const char *motionText = movingState > 0 ? "moving" : (movingState == 0 ? "still" : "unknown");
        DEBUG_PRINTF("LD2402 presence=%s moving=%s distance=%u cm", presence ? "true" : "false", motionText, distanceCm);
        ld2402ReportedOnce = true;
        lastLD2402Log = millis();
    }
}

static void parseLD2402Line(String line)
{
    line.trim();
    if (line.length() == 0)
        return;

    String upperLine = line;
    upperLine.toUpperCase();
    if (upperLine.indexOf(F("OFF")) >= 0)
    {
        setLD2402State(false, 0, 0);
        return;
    }

    int8_t movingState = -1;
    if (upperLine.indexOf(F("MOV")) >= 0)
        movingState = 1;
    else if (upperLine.indexOf(F("STILL")) >= 0 || upperLine.indexOf(F("STATIC")) >= 0)
        movingState = 0;

    int firstDigit = -1;
    for (int i = 0; i < line.length(); i++)
    {
        if (isDigit(line.charAt(i)))
        {
            firstDigit = i;
            break;
        }
    }

    if (firstDigit < 0)
        return;

    uint16_t distanceCm = line.substring(firstDigit).toInt();
    setLD2402State(distanceCm > 0, distanceCm, movingState);
}

static void handleLD2402CommandFrame(const uint8_t *frame, uint16_t expectedLength);

static void parseLD2402BinaryByte(uint8_t byte)
{
    static uint8_t frame[160];
    static uint16_t pos = 0;
    static uint16_t expectedLength = 0;
    static const uint8_t header[] = {0xF4, 0xF3, 0xF2, 0xF1};

    if (pos < sizeof(header))
    {
        if (byte == header[pos])
        {
            frame[pos++] = byte;
        }
        else
        {
            pos = (byte == header[0]) ? 1 : 0;
            if (pos == 1)
                frame[0] = byte;
        }
        return;
    }

    if (pos >= sizeof(frame))
    {
        pos = 0;
        expectedLength = 0;
        return;
    }

    frame[pos++] = byte;

    if (pos == 6)
    {
        const uint16_t payloadLength = frame[4] | (frame[5] << 8);
        expectedLength = 4 + 2 + payloadLength + 4;
        if (expectedLength > sizeof(frame) || payloadLength < 3)
        {
            pos = 0;
            expectedLength = 0;
        }
        return;
    }

    if (expectedLength == 0 || pos < expectedLength)
        return;

    const bool hasValidTail = frame[expectedLength - 4] == 0xF8 &&
                              frame[expectedLength - 3] == 0xF7 &&
                              frame[expectedLength - 2] == 0xF6 &&
                              frame[expectedLength - 1] == 0xF5;

    if (hasValidTail)
    {
        const uint8_t targetState = frame[6];
        const uint16_t distanceCm = frame[7] | (frame[8] << 8);
        const bool presence = targetState != 0;
        const int8_t movingState = targetState == 1 ? 1 : (targetState == 2 ? 0 : -1);
        setLD2402State(presence, presence ? distanceCm : 0, movingState);
    }

    pos = 0;
    expectedLength = 0;
}

static void parseLD2402CommandByte(uint8_t byte)
{
    static uint8_t frame[96];
    static uint16_t pos = 0;
    static uint16_t expectedLength = 0;
    static const uint8_t header[] = {0xFD, 0xFC, 0xFB, 0xFA};

    if (pos < sizeof(header))
    {
        if (byte == header[pos])
        {
            frame[pos++] = byte;
        }
        else
        {
            pos = (byte == header[0]) ? 1 : 0;
            if (pos == 1)
                frame[0] = byte;
        }
        return;
    }

    if (pos >= sizeof(frame))
    {
        pos = 0;
        expectedLength = 0;
        return;
    }

    frame[pos++] = byte;

    if (pos == 6)
    {
        const uint16_t payloadLength = frame[4] | (frame[5] << 8);
        expectedLength = 4 + 2 + payloadLength + 4;
        if (expectedLength > sizeof(frame) || payloadLength < 2)
        {
            pos = 0;
            expectedLength = 0;
        }
        return;
    }

    if (expectedLength == 0 || pos < expectedLength)
        return;

    const bool hasValidTail = frame[expectedLength - 4] == 0x04 &&
                              frame[expectedLength - 3] == 0x03 &&
                              frame[expectedLength - 2] == 0x02 &&
                              frame[expectedLength - 1] == 0x01;

    if (hasValidTail)
        handleLD2402CommandFrame(frame, expectedLength);

    pos = 0;
    expectedLength = 0;
}

static void sendLD2402Command(const uint8_t *command, size_t length)
{
    ld2402Serial.write(command, length);
    ld2402Serial.flush();
}

static void enableLD2402EngineeringMode();
static void endLD2402ConfigMode();

static void setLD2402CalibrationState(const char *state)
{
    if (strcmp(ld2402CalibrationState, state) == 0)
        return;

    ld2402CalibrationState = state;
    MQTTManager.publish("ld2402/calibration", ld2402CalibrationState);

    if (DEBUG_MODE)
        DEBUG_PRINTF("LD2402 calibration state=%s", ld2402CalibrationState);
}

static void finishLD2402Calibration(const char *state)
{
    ld2402CalibrationActive = false;
    ld2402CalibrationSavePending = false;
    ld2402CalibrationProgress = 0;
    endLD2402ConfigMode();
    delay(50);
    enableLD2402EngineeringMode();
    setLD2402CalibrationState(state);
}

static void handleLD2402CommandFrame(const uint8_t *frame, uint16_t expectedLength)
{
    if (expectedLength < 12)
        return;

    const uint16_t commandWord = frame[6] | (frame[7] << 8);
    const uint16_t ackStatus = frame[8] | (frame[9] << 8);

    switch (commandWord)
    {
    case 0x0109:
        if (!ld2402CalibrationActive)
            return;

        if (ackStatus == 0)
        {
            ld2402CalibrationProgress = 0;
            setLD2402CalibrationState("running");
        }
        else
        {
            finishLD2402Calibration("failed");
        }
        break;

    case 0x010A:
        if (!ld2402CalibrationActive)
            return;

        if (ackStatus != 0 || expectedLength < 14)
        {
            finishLD2402Calibration("failed");
            return;
        }

        ld2402CalibrationProgress = frame[10] | (frame[11] << 8);
        if (DEBUG_MODE)
            DEBUG_PRINTF("LD2402 calibration progress=%u%%", ld2402CalibrationProgress);

        if (ld2402CalibrationProgress >= 100)
        {
            static const uint8_t saveParameters[] = {
                0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFD, 0x00,
                0x04, 0x03, 0x02, 0x01};
            ld2402CalibrationSavePending = true;
            setLD2402CalibrationState("saving");
            sendLD2402Command(saveParameters, sizeof(saveParameters));
        }
        else
        {
            setLD2402CalibrationState("running");
        }
        break;

    case 0x01FD:
        if (!ld2402CalibrationActive || !ld2402CalibrationSavePending)
            return;

        if (ackStatus == 0)
            finishLD2402Calibration("done");
        else
            finishLD2402Calibration("failed");
        break;

    default:
        break;
    }
}

static void enableLD2402EngineeringMode()
{
#if LD2402_ENGINEERING_MODE
    static const uint8_t enableConfig[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00,
        0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    static const uint8_t setEngineeringOutput[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x12, 0x00,
        0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x04, 0x03,
        0x02, 0x01};
    static const uint8_t endConfig[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00,
        0x04, 0x03, 0x02, 0x01};

    sendLD2402Command(enableConfig, sizeof(enableConfig));
    delay(50);
    sendLD2402Command(setEngineeringOutput, sizeof(setEngineeringOutput));
    delay(50);
    sendLD2402Command(endConfig, sizeof(endConfig));
    delay(50);

    if (DEBUG_MODE)
        DEBUG_PRINTLN(F("LD2402 engineering output mode requested"));
#endif
}

static void endLD2402ConfigMode()
{
    static const uint8_t endConfig[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00,
        0x04, 0x03, 0x02, 0x01};
    sendLD2402Command(endConfig, sizeof(endConfig));
}

static void enterLD2402ConfigMode()
{
    static const uint8_t enableConfig[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00,
        0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
    sendLD2402Command(enableConfig, sizeof(enableConfig));
}

static void pollLD2402()
{
    while (ld2402Serial.available() > 0)
    {
        const uint8_t byte = static_cast<uint8_t>(ld2402Serial.read());
        const char c = static_cast<char>(byte);
        ld2402BytesReceived++;
        parseLD2402BinaryByte(byte);
        parseLD2402CommandByte(byte);

#if LD2402_ENGINEERING_MODE
        (void)c;
        continue;
#else
        if (c == '\r' || c == '\n')
        {
            parseLD2402Line(ld2402Line);
            ld2402Line = "";
            continue;
        }

        if (isPrintable(c))
        {
            ld2402Line += c;
            if (ld2402Line.length() > 80)
                ld2402Line = "";
        }
#endif
    }
}

static void updateLD2402Calibration()
{
    if (!ld2402CalibrationActive)
        return;

    const unsigned long now = millis();
    const unsigned long elapsed = now - ld2402CalibrationStartedAt;

    if (elapsed >= LD2402_CALIBRATION_TIMEOUT_MS)
    {
        finishLD2402Calibration("timeout");
        return;
    }

    if (ld2402CalibrationSavePending)
        return;

    if (now - ld2402CalibrationLastQueryAt >= LD2402_CALIBRATION_QUERY_INTERVAL_MS)
    {
        static const uint8_t progressQuery[] = {
            0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x0A, 0x00,
            0x04, 0x03, 0x02, 0x01};
        ld2402CalibrationLastQueryAt = now;
        sendLD2402Command(progressQuery, sizeof(progressQuery));
    }
}
#endif

static void printSensorStatus()
{
    const char *motionText = LD2402_MOVING_STATE > 0 ? "moving" : (LD2402_MOVING_STATE == 0 ? "still" : "unknown");
#if defined(ESP32_C3) && LD2402_ENABLED
    Serial.printf("[SENSOR] LD2402 available=%s presence=%s moving=%s distance=%u cm rx_bytes=%lu valid_frames=%lu rx_pin=%d | BH1750 available=%s lux=%.3f\n",
                  LD2402_AVAILABLE ? "true" : "false",
                  LD2402_PRESENCE ? "true" : "false",
                  motionText,
                  LD2402_DISTANCE_CM,
                  static_cast<unsigned long>(ld2402BytesReceived),
                  static_cast<unsigned long>(ld2402ValidFrames),
                  LD2402_RX_PIN,
                  BH1750_AVAILABLE ? "true" : "false",
                  BH1750_AVAILABLE ? CURRENT_LUX : 0.0f);
#else
    Serial.printf("[SENSOR] LD2402 disabled | BH1750 available=%s lux=%.3f\n",
                  BH1750_AVAILABLE ? "true" : "false",
                  BH1750_AVAILABLE ? CURRENT_LUX : 0.0f);
#endif
}

static uint8_t calculateAutoBrightness()
{
    if (bh1750Detected)
    {
        const float scaledLux = max(0.0f, CURRENT_LUX * LDR_FACTOR);
        const float clampedLux = min(scaledLux, 1000.0f);
        brightnessPercent = (log10f(clampedLux + 1.0f) / log10f(1001.0f)) * 100.0f;
    }
    else
    {
        brightnessPercent = (LDR_RAW * LDR_FACTOR) / 1023.0 * 100.0;
        brightnessPercent = pow(brightnessPercent, LDR_GAMMA) / pow(100.0, LDR_GAMMA - 1);
    }

    brightnessPercent = constrain(brightnessPercent, 0.0f, 100.0f);
    return map(static_cast<int>(brightnessPercent), 0, 100, MIN_BRIGHTNESS, MAX_BRIGHTNESS);
}

void left_button_pressed()
{
    if (!BLOCK_NAVIGATION)
    {
        if (DFPLAYER_ACTIVE)
            PeripheryManager.playFromFile(DFMINI_MP3_CLICK);

        DisplayManager.leftButton();
        MenuManager.leftButton();
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Left button clicked"));
    }
    else
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Left button clicked but blocked"));
    }
}

void right_button_pressed()
{
    if (!BLOCK_NAVIGATION)
    {
        if (DFPLAYER_ACTIVE)
            PeripheryManager.playFromFile(DFMINI_MP3_CLICK);

        DisplayManager.rightButton();
        MenuManager.rightButton();
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Right button clicked"));
    }
    else
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Right button clicked but blocked"));
    }
}

void select_button_pressed()
{
    if (!BLOCK_NAVIGATION)
    {
        if (DFPLAYER_ACTIVE)
            PeripheryManager.playFromFile(DFMINI_MP3_CLICK);

        DisplayManager.selectButton();
        MenuManager.selectButton();
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Select button clicked"));
    }
    else
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Select button clicked but blocked"));
    }
}

void reset_button_pressed_long()
{
    ServerManager.erase();
    ESP.restart();
}

void select_button_pressed_long()
{
    if (DFPLAYER_ACTIVE)
        PeripheryManager.playFromFile(DFMINI_MP3_CLICK);
    if (AP_MODE)
    {
        ++MATRIX_LAYOUT;
        if (MATRIX_LAYOUT < 0)
            MATRIX_LAYOUT = 2;
        saveSettings();
        ESP.restart();
    }
    else if (!BLOCK_NAVIGATION)
    {
        MenuManager.selectButtonLong();
        DisplayManager.selectButtonLong();
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Select button pressed long"));
    }
}

void select_button_double()
{
    if (DEBUG_MODE)
        DEBUG_PRINTLN(F("Select button double pressed"));
    if (!BLOCK_NAVIGATION)
    {
        if (DFPLAYER_ACTIVE)
            PeripheryManager.playFromFile(DFMINI_MP3_CLICK);

        if (MATRIX_OFF)
        {
            DisplayManager.setPower(true);
        }
        else
        {
            DisplayManager.setPower(false);
        }
    }
}

void PeripheryManager_::playBootSound()
{
    if (DEBUG_MODE)
        DEBUG_PRINTLN(F("Playing bootsound"));
    if (!SOUND_ACTIVE || !HAS_BUZZER)
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Sound output disabled"));
        return;
    }

    if (BOOT_SOUND == "")
    {
        if (DFPLAYER_ACTIVE)
        {
            playFromFile(DFMINI_MP3_BOOT);
        }
        else
        {
            const int nNotes = 6;
            String notes[nNotes] = {"E5", "C5", "G4", "E4", "G4", "C5"};
            const int timeUnit = 150;
            Melody melody = MelodyFactory.load("Bootsound", timeUnit, notes, nNotes);
            player.playAsync(melody);
        }
    }
    else
    {
        playFromFile(BOOT_SOUND);
    }
}

void PeripheryManager_::stopSound()
{
    if (DFPLAYER_ACTIVE)
    {
#ifndef ESP32_C3
        dfmp3.stopAdvertisement();
        delay(50);
        dfmp3.stop();
#else
        // DFPlayer not supported on ESP32-C3.
#endif
    }
    else
    {
        if (HAS_BUZZER)
            player.stop();
    }
}

void PeripheryManager_::setVolume(uint8_t vol)
{
    if (DFPLAYER_ACTIVE)
    {
#ifndef ESP32_C3
        uint8_t curVolume = dfmp3.getVolume(); // need to read volume in order to work. Donno why! :(
        dfmp3.setVolume(vol);
        delay(50);
#else
        // DFPlayer not supported on ESP32-C3.
#endif
    }
    else
    {
        if (HAS_BUZZER)
        {
            int scaledVol = (vol * 255) / 30;
            player.setVolume(scaledVol);
        }
    }
}

bool PeripheryManager_::parseSound(const char *json)
{
    StaticJsonDocument<128> doc;
    DeserializationError error = deserializeJson(doc, json);
    if (error)
    {
        return playFromFile(String(json));
    }
    if (doc.containsKey("sound"))
    {
        return playFromFile(doc["sound"].as<String>());
    }
    return false;
}

const char *PeripheryManager_::playRTTTLString(String rtttl)
{
    if (!DFPLAYER_ACTIVE && SOUND_ACTIVE && HAS_BUZZER)
    {
        static char melodyName[64];
        Melody melody = MelodyFactory.loadRtttlString(rtttl.c_str());
        player.playAsync(melody);
        strncpy(melodyName, melody.getTitle().c_str(), sizeof(melodyName));
        melodyName[sizeof(melodyName) - 1] = '\0';
        return melodyName;
    }
    return nullptr; // RTTTL not supported with DFPlayer
}

const char *PeripheryManager_::playFromFile(String file)
{
    if (!SOUND_ACTIVE || !HAS_BUZZER)
        return "";

    if (DFPLAYER_ACTIVE)
    {
#ifndef ESP32_C3
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Playing MP3 file"));
        if (!DFPLAYER_ACTIVE)
            return NULL;
        dfmp3.stop();
        delay(50);
        dfmp3.playMp3FolderTrack(file.toInt());

        return file.c_str();
#else
        return NULL; // DFPlayer not supported on ESP32-C3.
#endif
    }
    else
    {
        if (!HAS_BUZZER)
            return NULL;

        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Playing RTTTL sound file"));
        if (LittleFS.exists("/MELODIES/" + String(file) + ".txt"))
        {
            static char melodyName[64];
            Melody melody = MelodyFactory.loadRtttlFile("/MELODIES/" + String(file) + ".txt");
            player.playAsync(melody);
            strncpy(melodyName, melody.getTitle().c_str(), sizeof(melodyName));
            melodyName[sizeof(melodyName) - 1] = '\0';
            return melodyName;
        }
        else
        {
            return NULL;
        }
    }
}

bool PeripheryManager_::isPlaying()
{
    if (DFPLAYER_ACTIVE)
    {
#ifndef ESP32_C3
        if ((dfmp3.getStatus() & 0xff) == 0x01) // 0x01 = DfMp3_StatusState_Playing
            return true;
        else
            return false;
#else
        return false; // DFPlayer not supported on ESP32-C3.
#endif
    }
    else
    {
        return HAS_BUZZER && player.isPlaying();
    }
}

void PeripheryManager_::setup()
{
    if (DEBUG_MODE)
        DEBUG_PRINTLN(F("Setup periphery"));
    startTime = millis();
    pinMode(LDR_PIN, INPUT);
    if (HAS_RESET_BUTTON)
        pinMode(RESET_PIN, INPUT);
    if (DFPLAYER_ACTIVE)
    {
#ifndef ESP32_C3
        dfmp3.begin();
        delay(100);
        setVolume(SOUND_VOLUME);
#endif
    }
    button_left.begin();
    button_right.begin();
    button_select.begin();
    if (HAS_RESET_BUTTON)
        button_reset.begin();

    if ((ROTATE_SCREEN && !SWAP_BUTTONS) || (!ROTATE_SCREEN && SWAP_BUTTONS))
    {
        Serial.println("Button rotation");
        button_left.onPressed(right_button_pressed);
        button_right.onPressed(left_button_pressed);
    }
    else
    {
        button_left.onPressed(left_button_pressed);
        button_right.onPressed(right_button_pressed);
    }

    button_select.onPressed(select_button_pressed);
    button_select.onPressedFor(1000, select_button_pressed_long);
    button_select.onSequence(2, 300, select_button_double);

#if defined(ULANZI) && HAS_RESET_BUTTON
    button_reset.onPressedFor(5000, reset_button_pressed_long);
#endif

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    if (bme280.begin(BME280_ADDRESS) || bme280.begin(BME280_ADDRESS_ALTERNATE))
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("BME280 sensor detected"));
        TEMP_SENSOR_TYPE = TEMP_SENSOR_TYPE_BME280;
    }
    else if (bmp280.begin(BMP280_ADDRESS) || bmp280.begin(BMP280_ADDRESS_ALT))
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("BMP280 sensor detected"));
        TEMP_SENSOR_TYPE = TEMP_SENSOR_TYPE_BMP280;
    }
    else if (htu21df.begin())
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("HTU21DF sensor detected"));
        TEMP_SENSOR_TYPE = TEMP_SENSOR_TYPE_HTU21DF;
    }
    else if (sht31.begin(0x44))
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("SHT31 sensor detected"));
        TEMP_SENSOR_TYPE = TEMP_SENSOR_TYPE_SHT31;
    }
    else if (ahtx0.begin())
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("AHTX0 sensor detected"));
        TEMP_SENSOR_TYPE = TEMP_SENSOR_TYPE_AHTX0;
    }

    if (bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire) ||
        bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x5C, &Wire))
    {
        bh1750Detected = true;
        BH1750_AVAILABLE = true;
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("BH1750 light sensor detected"));
    }

#if defined(ESP32_C3) && LD2402_ENABLED
    ld2402Serial.begin(LD2402_BAUD, SERIAL_8N1, LD2402_RX_PIN, LD2402_TX_PIN);
    if (DEBUG_MODE)
        DEBUG_PRINTF("LD2402 UART1 enabled at %u baud, RX GPIO %d, TX GPIO %d", LD2402_BAUD, LD2402_RX_PIN, LD2402_TX_PIN);
    enableLD2402EngineeringMode();
#endif

#ifdef awtrix2_upgrade
#ifndef ESP32_C3
    dfmp3.begin();
#endif
#else

#endif
    if (!LDR_ON_GROUND)
        photocell.setPhotocellPositionOnGround(false);
}

void PeripheryManager_::tick()
{
#if defined(ESP32_C3) && LD2402_ENABLED
    pollLD2402();
    updateLD2402Calibration();
#endif

    if (!MenuManager.inMenu)
    {
        if (ROTATE_SCREEN)
        {
            MQTTManager.sendButton(2, button_left.read());
            ServerManager.sendButton(2, button_left.read());
            MQTTManager.sendButton(0, button_right.read());
            ServerManager.sendButton(0, button_right.read());
        }
        else
        {
            MQTTManager.sendButton(0, button_left.read());
            MQTTManager.sendButton(2, button_right.read());
            ServerManager.sendButton(0, button_left.read());
            ServerManager.sendButton(2, button_right.read());
        }

        MQTTManager.sendButton(1, button_select.read());
        ServerManager.sendButton(1, button_select.read());
    }
    else
    {
        button_left.read();
        button_select.read();
        button_right.read();
    }

    if (HAS_RESET_BUTTON)
        button_reset.read();

    unsigned long currentMillis_BatTempHum = millis();
    if (currentMillis_BatTempHum - previousMillis_BatTempHum >= interval_BatTempHum)
    {
        previousMillis_BatTempHum = currentMillis_BatTempHum;
#ifdef ULANZI
        uint16_t ADCVALUE = analogRead(BATTERY_PIN);
        // Discard values that are totally out of range, especially the first value read after a reboot.
        // Meaningful values for an Ulanzi clock are in the range 400..700
        if ((ADCVALUE > 100) && (ADCVALUE < 1000))
        {
            // Send ADC values through median filter to get rid of the remaining spikes and then calculate the average
            BATTERY_RAW = meanFilterBatt.AddValue(medianFilterBatt.AddValue(ADCVALUE));
            BATTERY_PERCENT = max(min((int)map(BATTERY_RAW, MIN_BATTERY, MAX_BATTERY, 0, 100), 100), 0);
            SENSORS_STABLE = true;
        }
#else
        SENSORS_STABLE = true;
#endif
        if (SENSOR_READING)
        {
            switch (TEMP_SENSOR_TYPE)
            {
            case TEMP_SENSOR_TYPE_BME280:
                CURRENT_TEMP = bme280.readTemperature();
                CURRENT_HUM = bme280.readHumidity();
                break;
            case TEMP_SENSOR_TYPE_BMP280:
                CURRENT_TEMP = bmp280.readTemperature();
                CURRENT_HUM = 0;
                break;
            case TEMP_SENSOR_TYPE_HTU21DF:
                CURRENT_TEMP = htu21df.readTemperature();
                CURRENT_HUM = htu21df.readHumidity();
                break;
            case TEMP_SENSOR_TYPE_SHT31:
                sht31.readBoth(&CURRENT_TEMP, &CURRENT_HUM);
                break;
            case TEMP_SENSOR_TYPE_AHTX0:
                ahtx0_humidity = ahtx0.getHumiditySensor();
                ahtx0_temp = ahtx0.getTemperatureSensor();
                sensors_event_t tempEvent, humEvent;
                if (ahtx0_temp->getEvent(&tempEvent) && ahtx0_humidity->getEvent(&humEvent))
                {
                    CURRENT_TEMP = tempEvent.temperature;
                    CURRENT_HUM = humEvent.relative_humidity;
                }
                else
                {
                    CURRENT_TEMP = NAN;
                    CURRENT_HUM = NAN;
                }
                break;
            default:
                CURRENT_TEMP = 0;
                CURRENT_HUM = 0;
                break;
            }

            CURRENT_TEMP += TEMP_OFFSET;
            CURRENT_HUM += HUM_OFFSET;

            const bool inRange = isfinite(CURRENT_TEMP) && isfinite(CURRENT_HUM) &&
                                 CURRENT_TEMP >= -40.0f && CURRENT_TEMP <= 125.0f &&
                                 CURRENT_HUM >= 0.0f && CURRENT_HUM <= 100.0f;
            const bool suddenJump = hasValidTempHum &&
                                    (fabsf(CURRENT_TEMP - lastValidTemp) > 5.0f ||
                                     fabsf(CURRENT_HUM - lastValidHum) > 20.0f);

            if (inRange && !suddenJump)
            {
                lastValidTemp = CURRENT_TEMP;
                lastValidHum = CURRENT_HUM;
                hasValidTempHum = true;
            }
            else if (hasValidTempHum)
            {
                if (DEBUG_MODE && millis() - lastInvalidTempHumLog > 60000)
                {
                    lastInvalidTempHumLog = millis();
                    DEBUG_PRINTF("Ignoring invalid temp/humidity sample: temp=%.2f hum=%.2f", CURRENT_TEMP, CURRENT_HUM);
                }
                CURRENT_TEMP = lastValidTemp;
                CURRENT_HUM = lastValidHum;
            }
        }
        else
        {
            SENSORS_STABLE = true;
        }
    }

    unsigned long currentMillis_LDR = millis();
    if (currentMillis_LDR - previousMillis_LDR >= interval_LDR)
    {
        previousMillis_LDR = currentMillis_LDR;

        uint16_t LDRVALUE = analogRead(LDR_PIN);
        if (LDR_ON_GROUND)
            LDRVALUE = 1023.0 - LDRVALUE;
        // Send LDR values through median filter to get rid of the remaining spikes and then calculate the average
        LDR_RAW = meanFilterLDR.AddValue(medianFilterLDR.AddValue(LDRVALUE));
        if (bh1750Detected)
        {
            const float lux = bh1750.readLightLevel();
            if (lux >= 0)
                CURRENT_LUX = (roundf(lux * 1000) / 1000);
        }
        else
        {
            CURRENT_LUX = (roundf(photocell.getSmoothedLux() * 1000) / 1000);
        }
        if (AUTO_BRIGHTNESS && !MATRIX_OFF)
        {
            const uint8_t autoBrightness = calculateAutoBrightness();
            if (abs(static_cast<int>(autoBrightness) - BRIGHTNESS) >= 2)
            {
                BRIGHTNESS = autoBrightness;
                DisplayManager.setBrightness(BRIGHTNESS);
            }
        }
    }

    unsigned long currentMillis_SensorStatus = millis();
    if (currentMillis_SensorStatus - previousMillis_SensorStatus >= interval_SensorStatus)
    {
        previousMillis_SensorStatus = currentMillis_SensorStatus;
        printSensorStatus();
    }
}

void PeripheryManager_::pollStartupSensors(uint16_t timeoutMs)
{
#if defined(ESP32_C3) && LD2402_ENABLED
    const unsigned long start = millis();
    while (!LD2402_AVAILABLE && millis() - start < timeoutMs)
    {
        pollLD2402();
        delay(1);
    }
#else
    (void)timeoutMs;
#endif
}

const char *PeripheryManager_::calibrateLD2402()
{
#if defined(ESP32_C3) && LD2402_ENABLED
    static const uint8_t startAutoThreshold[] = {
        0xFD, 0xFC, 0xFB, 0xFA, 0x08, 0x00, 0x09, 0x00,
        0x1E, 0x00, 0x14, 0x00, 0x1E, 0x00, 0x04, 0x03,
        0x02, 0x01};

    if (!LD2402_AVAILABLE)
    {
        setLD2402CalibrationState("unavailable");
        return ld2402CalibrationState;
    }

    if (ld2402CalibrationActive)
    {
        setLD2402CalibrationState("busy");
        return ld2402CalibrationState;
    }

    enterLD2402ConfigMode();
    delay(50);
    sendLD2402Command(startAutoThreshold, sizeof(startAutoThreshold));

    ld2402CalibrationActive = true;
    ld2402CalibrationStartedAt = millis();
    ld2402CalibrationLastQueryAt = 0;
    ld2402CalibrationProgress = 0;
    ld2402CalibrationSavePending = false;
    setLD2402CalibrationState("started");

    if (DEBUG_MODE)
        DEBUG_PRINTLN(F("LD2402 automatic threshold calibration requested"));
    return ld2402CalibrationState;
#else
    return "unavailable";
#endif
}

const char *PeripheryManager_::getLD2402CalibrationState() const
{
#if defined(ESP32_C3) && LD2402_ENABLED
    return ld2402CalibrationState;
#else
    return "unavailable";
#endif
}

unsigned long long PeripheryManager_::readUptime()
{
    static unsigned long lastTime = 0;
    static unsigned long long totalElapsed = 0;

    unsigned long currentTime = millis();
    if (currentTime < lastTime)
    {
        // millis() overflow
        totalElapsed += 4294967295UL - lastTime + currentTime + 1;
    }
    else
    {
        totalElapsed += currentTime - lastTime;
    }
    lastTime = currentTime;

    unsigned long long uptimeSeconds = totalElapsed / 1000;
    return uptimeSeconds;
}

void PeripheryManager_::r2d2(const char *msg)
{
#if defined(ULANZI) && HAS_BUZZER
    for (int i = 0; msg[i] != '\0'; i++)
    {
        char c = msg[i];
        tone(BUZZER_PIN, (c - 'A' + 1) * 50);
        delay(baudRate + 10);
    }
    noTone(BUZZER_PIN);
#endif
}
