#include <MQTTManager.h>
#include "Globals.h"
#include "DisplayManager.h"
#include "ServerManager.h"
#include <ArduinoHA.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "Dictionary.h"
#include "PeripheryManager.h"
#include "UpdateManager.h"
#include "PowerManager.h"

const uint16_t PORT = 1883;

WiFiClient espClient;
HADevice device;
HAMqtt mqtt(espClient, device, 26);

HALight *Matrix, *Indikator1, *Indikator2, *Indikator3 = nullptr;
HASelect *BriMode, *transEffect = nullptr;
HAButton *dismiss, *nextApp, *prevApp, *doUpdate, *ld2402Calibrate = nullptr;
HASwitch *transition = nullptr;
#ifdef ULANZI
HASensor *battery = nullptr;
#endif
HASensor *temperature, *humidity, *illuminance, *uptime, *strength, *version, *ram, *curApp, *myOwnID, *ipAddr, *ld2402Motion, *ld2402Distance = nullptr;
HABinarySensor *btnleft, *btnmid, *btnright, *ld2402Presence = nullptr;
bool connected;
char matID[40], ind1ID[40], ind2ID[40], ind3ID[40], briID[40], btnAID[40], btnBID[40], btnCID[40], appID[40], tempID[40], humID[40], luxID[40], verID[40], ramID[40], upID[40], sigID[40], btnLID[40], btnMID[40], btnRID[40], transID[40], doUpdateID[40], batID[40], myID[40], sSpeed[40], effectID[40], ipAddrID[40], ld2402PresenceID[40], ld2402MotionID[40], ld2402DistanceID[40], ld2402CalibrateID[40];
long previousMillis_Stats;
std::map<String, String> mqttValues;
std::vector<String> topicsToSubscribe;
static constexpr size_t MQTT_INLINE_PAYLOAD_SIZE = 1024;
static constexpr size_t MQTT_MAX_PAYLOAD_SIZE = 4096;
static char mqttPayloadBuffer[MQTT_INLINE_PAYLOAD_SIZE];

static const uint32_t MQTT_MIN_FREE_HEAP = 12000;
static const uint32_t MQTT_MIN_MAX_ALLOC_HEAP = 4096;
#ifdef ESP32_C3
static const uint32_t MQTT_C3_CURRENT_APP_MIN_FREE_HEAP = 20000;
static const uint32_t MQTT_C3_CURRENT_APP_MIN_MAX_ALLOC_HEAP = 6000;
static const unsigned long MQTT_C3_STATS_INTERVAL = 60000;
#endif

#ifdef ESP32_C3
static bool c3LightHaDiscoveryRequested = false;
static bool c3LightHaDiscoveryPending = false;
static uint8_t c3LightHaDiscoveryStep = 0;
static unsigned long c3LightHaDiscoveryLastPublish = 0;
static bool c3AvailabilityWasConnected = false;
static bool c3SubscriptionsPending = false;
static size_t c3SubscriptionIndex = 0;
static unsigned long c3LastSubscriptionMillis = 0;
static constexpr unsigned long C3_SUBSCRIPTION_INTERVAL_MS = 250;
#endif

static const char *ld2402MotionText()
{
    if (LD2402_MOVING_STATE > 0)
        return "moving";
    if (LD2402_MOVING_STATE == 0)
        return "still";
    return "unknown";
}

static bool mqttHasWriteRoom()
{
    if (MQTT_HOST == "" || !mqtt.isConnected())
        return false;

    const uint32_t freeHeap = ESP.getFreeHeap();
    const uint32_t maxAllocHeap = ESP.getMaxAllocHeap();
    if (freeHeap >= MQTT_MIN_FREE_HEAP && maxAllocHeap >= MQTT_MIN_MAX_ALLOC_HEAP)
        return true;

    static unsigned long lastLowHeapLog = 0;
    if (millis() - lastLowHeapLog > 10000)
    {
        lastLowHeapLog = millis();
        DEBUG_PRINTF("Skip MQTT publish: low heap free=%lu max_alloc=%lu bytes", freeHeap, maxAllocHeap);
    }
    return false;
}

static void stopMqttAfterWriteFailure()
{
    connected = false;
#ifdef ESP32_C3
    c3AvailabilityWasConnected = false;
#endif
    espClient.stop();
    if (DEBUG_MODE)
        DEBUG_PRINTLN(F("MQTT publish failed; closing client socket"));
}

static bool mqttPublishRetained(const char *topic, const char *payload)
{
    if (!mqttHasWriteRoom())
        return false;

    if (!mqtt.publish(topic, payload, true))
    {
        stopMqttAfterWriteFailure();
        return false;
    }
    return true;
}

#ifdef ESP32_C3
static int buildHaDeviceJson(char *buffer, size_t length)
{
    return snprintf(buffer, length,
                    "\"availability_topic\":\"%s/status\",\"payload_available\":\"online\",\"payload_not_available\":\"offline\",\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"%s\",\"model\":\"%s\",\"sw_version\":\"%s\"}",
                    MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), HOSTNAME.c_str(), HAmanufacturer, HAmodel, VERSION);
}

static bool publishC3HaConfig(const char *component, const char *objectId, const char *payload)
{
    char topic[256];
    const int length = snprintf(topic, sizeof(topic), "%s/%s/%s_%s/config", HA_PREFIX.c_str(), component, MQTT_PREFIX.c_str(), objectId);
    if (length < 0 || static_cast<size_t>(length) >= sizeof(topic))
    {
        DEBUG_PRINTLN(F("HA discovery topic too long"));
        return false;
    }
    return mqttPublishRetained(topic, payload);
}

static bool clearC3HaConfig(const char *component, const char *objectId)
{
    char topic[256];
    const int length = snprintf(topic, sizeof(topic), "%s/%s/%s_%s/config", HA_PREFIX.c_str(), component, MQTT_PREFIX.c_str(), objectId);
    if (length < 0 || static_cast<size_t>(length) >= sizeof(topic))
    {
        DEBUG_PRINTLN(F("HA discovery topic too long"));
        return false;
    }
    return mqttPublishRetained(topic, "");
}

static void persistBrightnessSlope()
{
    DynamicJsonDocument doc(512);
    if (LittleFS.exists("/dev.json"))
    {
        File readFile = LittleFS.open("/dev.json", "r");
        if (readFile)
        {
            deserializeJson(doc, readFile);
            readFile.close();
        }
    }

    doc["ldr_factor"] = LDR_FACTOR;

    File writeFile = LittleFS.open("/dev.json", "w");
    if (writeFile)
    {
        serializeJsonPretty(doc, writeFile);
        writeFile.close();
    }
}

static void publishC3AvailabilityIfNeeded()
{
    if (!mqtt.isConnected())
    {
        c3AvailabilityWasConnected = false;
        return;
    }

    if (c3AvailabilityWasConnected)
        return;

    char availabilityTopic[100];
    snprintf(availabilityTopic, sizeof(availabilityTopic), "%s/status", MQTT_PREFIX.c_str());
    if (mqttPublishRetained(availabilityTopic, "online"))
    {
        c3AvailabilityWasConnected = true;
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("MQTT availability published: online"));
    }
}

static void publishC3StatsNow()
{
    char statsBuffer[512];
    DisplayManager.getStats(statsBuffer, sizeof(statsBuffer));
    MQTTManager.publish("stats", statsBuffer);
}

static void publishC3HaDiscoveryTick(unsigned long now)
{
    if (!c3LightHaDiscoveryPending || !mqtt.isConnected())
        return;

    if (now - c3LightHaDiscoveryLastPublish < 2500)
        return;

    char deviceJson[420];
    buildHaDeviceJson(deviceJson, sizeof(deviceJson));

    char payload[960];
    bool published = false;

    switch (c3LightHaDiscoveryStep)
    {
    case 0:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Device topic\",\"unique_id\":\"%s_device_topic\",\"state_topic\":\"%s/stats/device_topic\",\"icon\":\"mdi:id-card\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("sensor", "device_topic", payload);
        break;
    case 1:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"BH1750 Illuminance\",\"unique_id\":\"%s_bh1750_lux\",\"state_topic\":\"%s/stats/lux\",\"device_class\":\"illuminance\",\"unit_of_measurement\":\"lx\",\"icon\":\"mdi:sun-wireless\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("sensor", "bh1750_lux", payload);
        break;
    case 2:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"LD2402 Presence\",\"unique_id\":\"%s_ld2402_presence\",\"state_topic\":\"%s/ld2402/status\",\"value_template\":\"{{ 'true' if value_json.presence else 'false' }}\",\"payload_on\":\"true\",\"payload_off\":\"false\",\"device_class\":\"occupancy\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("binary_sensor", "ld2402_presence", payload);
        break;
    case 3:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"LD2402 Motion\",\"unique_id\":\"%s_ld2402_motion\",\"state_topic\":\"%s/ld2402/status\",\"value_template\":\"{{ 'true' if value_json.motion == 'moving' else 'false' }}\",\"payload_on\":\"true\",\"payload_off\":\"false\",\"device_class\":\"motion\",\"icon\":\"mdi:motion-sensor\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("binary_sensor", "ld2402_motion", payload);
        break;
    case 4:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"LD2402 Distance\",\"unique_id\":\"%s_ld2402_distance\",\"state_topic\":\"%s/ld2402/status\",\"value_template\":\"{{ value_json.distance }}\",\"unit_of_measurement\":\"cm\",\"icon\":\"mdi:map-marker-distance\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("sensor", "ld2402_distance", payload);
        break;
    case 5:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"LD2402 Calibrate\",\"unique_id\":\"%s_ld2402_calibrate\",\"command_topic\":\"%s/ld2402/calibrate\",\"payload_press\":\"1\",\"icon\":\"mdi:tune-variant\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("button", "ld2402_calibrate", payload);
        break;
    case 6:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Temperature\",\"unique_id\":\"%s_temperature\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ value_json.temp }}\",\"device_class\":\"temperature\",\"unit_of_measurement\":\"C\",\"icon\":\"mdi:thermometer\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("sensor", "temperature", payload);
        break;
    case 7:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Humidity\",\"unique_id\":\"%s_humidity\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ value_json.hum }}\",\"device_class\":\"humidity\",\"unit_of_measurement\":\"%%\",\"icon\":\"mdi:water-percent\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("sensor", "humidity", payload);
        break;
    case 8:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Current App\",\"unique_id\":\"%s_current_app\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ value_json.app }}\",\"icon\":\"mdi:apps\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("sensor", "current_app", payload);
        break;
    case 9:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Brightness\",\"unique_id\":\"%s_brightness\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ value_json.bri }}\",\"unit_of_measurement\":\"%%\",\"icon\":\"mdi:brightness-6\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("sensor", "brightness", payload);
        break;
    case 10:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Free RAM\",\"unique_id\":\"%s_ram\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ value_json.ram }}\",\"device_class\":\"data_size\",\"unit_of_measurement\":\"B\",\"icon\":\"mdi:memory\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("sensor", "ram", payload);
        break;
    case 11:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Uptime\",\"unique_id\":\"%s_uptime\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ value_json.uptime }}\",\"device_class\":\"duration\",\"unit_of_measurement\":\"s\",\"icon\":\"mdi:timer-outline\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("sensor", "uptime", payload);
        break;
    case 12:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"WiFi Signal\",\"unique_id\":\"%s_wifi_signal\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ value_json.wifi_signal }}\",\"device_class\":\"signal_strength\",\"unit_of_measurement\":\"dB\",\"icon\":\"mdi:wifi\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("sensor", "wifi_signal", payload);
        break;
    case 13:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"IP Address\",\"unique_id\":\"%s_ip_address\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ value_json.ip_address }}\",\"icon\":\"mdi:ip-network\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("sensor", "ip_address", payload);
        break;
    case 14:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Matrix\",\"unique_id\":\"%s_matrix\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ 'ON' if value_json.matrix else 'OFF' }}\",\"command_topic\":\"%s/ha/matrix_power\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"icon\":\"mdi:clock-digital\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("switch", "matrix", payload);
        break;
    case 15:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Indicator 1\",\"unique_id\":\"%s_indicator1\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ 'ON' if value_json.indicator1 else 'OFF' }}\",\"command_topic\":\"%s/ha/indicator1\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"icon\":\"mdi:arrow-top-right-thick\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("switch", "indicator1", payload);
        break;
    case 16:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Indicator 2\",\"unique_id\":\"%s_indicator2\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ 'ON' if value_json.indicator2 else 'OFF' }}\",\"command_topic\":\"%s/ha/indicator2\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"icon\":\"mdi:arrow-right-thick\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("switch", "indicator2", payload);
        break;
    case 17:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Indicator 3\",\"unique_id\":\"%s_indicator3\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ 'ON' if value_json.indicator3 else 'OFF' }}\",\"command_topic\":\"%s/ha/indicator3\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"icon\":\"mdi:arrow-bottom-right-thick\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("switch", "indicator3", payload);
        break;
    case 18:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Next App\",\"unique_id\":\"%s_next_app\",\"command_topic\":\"%s/nextapp\",\"payload_press\":\"1\",\"icon\":\"mdi:arrow-right-bold\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("button", "next_app", payload);
        break;
    case 19:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Previous App\",\"unique_id\":\"%s_previous_app\",\"command_topic\":\"%s/previousapp\",\"payload_press\":\"1\",\"icon\":\"mdi:arrow-left-bold\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("button", "previous_app", payload);
        break;
    case 20:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Dismiss Notification\",\"unique_id\":\"%s_dismiss_notification\",\"command_topic\":\"%s/notify/dismiss\",\"payload_press\":\"1\",\"icon\":\"mdi:bell-off\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("button", "dismiss_notification", payload);
        break;
    case 21:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Brightness Lux Slope\",\"unique_id\":\"%s_brightness_lux_slope\",\"command_topic\":\"%s/brightness/slope\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ value_json.brightness_slope }}\",\"min\":0.2,\"max\":5,\"step\":0.1,\"mode\":\"slider\",\"icon\":\"mdi:chart-bell-curve\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("number", "brightness_lux_slope", payload);
        break;
    case 22:
        published = clearC3HaConfig("button", "start_update");
        break;
    case 23:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Reboot\",\"unique_id\":\"%s_reboot\",\"command_topic\":\"%s/reboot\",\"payload_press\":\"1\",\"icon\":\"mdi:restart\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("button", "reboot", payload);
        break;
    case 24:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Brightness Mode\",\"unique_id\":\"%s_brightness_mode\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ 'Auto' if value_json.auto_brightness else 'Manual' }}\",\"command_topic\":\"%s/ha/brightness_mode\",\"options\":[\"Manual\",\"Auto\"],\"icon\":\"mdi:brightness-auto\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("select", "brightness_mode", payload);
        break;
    case 25:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Transition\",\"unique_id\":\"%s_transition\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ 'ON' if value_json.auto_transition else 'OFF' }}\",\"command_topic\":\"%s/ha/transition\",\"payload_on\":\"ON\",\"payload_off\":\"OFF\",\"icon\":\"mdi:swap-horizontal\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("switch", "transition", payload);
        break;
    case 26:
        snprintf(payload, sizeof(payload),
                 "{\"name\":\"Transition Effect\",\"unique_id\":\"%s_transition_effect\",\"state_topic\":\"%s/stats\",\"value_template\":\"{{ ['Random','Slide','Dim','Zoom','Rotate','Pixelate','Curtain','Ripple','Blink','Reload','Fade'][value_json.transition_effect] }}\",\"command_topic\":\"%s/ha/transition_effect\",\"options\":[\"Random\",\"Slide\",\"Dim\",\"Zoom\",\"Rotate\",\"Pixelate\",\"Curtain\",\"Ripple\",\"Blink\",\"Reload\",\"Fade\"],\"icon\":\"mdi:auto-fix\",%s}",
                 MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), MQTT_PREFIX.c_str(), deviceJson);
        published = publishC3HaConfig("select", "transition_effect", payload);
        break;
    // Replace the original read-only C3 entities with their controllable counterparts.
    case 27:
        published = clearC3HaConfig("binary_sensor", "matrix_power");
        break;
    case 28:
        published = clearC3HaConfig("binary_sensor", "indicator1");
        break;
    case 29:
        published = clearC3HaConfig("binary_sensor", "indicator2");
        break;
    case 30:
        published = clearC3HaConfig("binary_sensor", "indicator3");
        break;
    case 31:
        published = clearC3HaConfig("sensor", "ld2402_motion");
        break;
    default:
        c3LightHaDiscoveryPending = false;
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("ESP32-C3 lightweight Home Assistant discovery complete"));
        return;
    }

    if (!published)
        return;

    if (DEBUG_MODE)
        DEBUG_PRINTF("Published ESP32-C3 HA discovery item %u", c3LightHaDiscoveryStep + 1);

    c3LightHaDiscoveryStep++;
    c3LightHaDiscoveryLastPublish = now;
}
#endif

MQTTManager_ &MQTTManager_::getInstance()
{
    static MQTTManager_ instance;
    return instance;
}

MQTTManager_ &MQTTManager = MQTTManager.getInstance();

void processMqttMessage(const String &strTopic, const String &payloadCopy)
{
    DisplayManager.logC3Heap("mqtt_process_entry");
    if (DEBUG_MODE)
    {
        DEBUG_PRINTF("Processing MQTT message for topic %s", strTopic.c_str());
        DEBUG_PRINTF("Payload: %s", payloadCopy.c_str());
    }

    ++RECEIVED_MESSAGES;

    if (strTopic.equals(MQTT_PREFIX + "/notify"))
    {
        if (payloadCopy[0] != '{' || payloadCopy[payloadCopy.length() - 1] != '}')
        {
            return;
        }
        DisplayManager.generateNotification(0, payloadCopy.c_str());
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/notify/dismiss"))
    {
        DisplayManager.dismissNotify();
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/doupdate"))
    {
        MQTTManager.publish("update/status", "disabled");
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Official firmware update disabled"));
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/apps"))
    {
        DisplayManager.updateAppVector(payloadCopy.c_str());
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/switch"))
    {
        DisplayManager.switchToApp(payloadCopy.c_str());
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/sendscreen"))
    {
        MQTTManager.getInstance().publish("screen", DisplayManager.ledsAsJson().c_str());
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/settings"))
    {
        DisplayManager.setNewSettings(payloadCopy.c_str());
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/r2d2"))
    {
        PeripheryManager.r2d2(payloadCopy.c_str());
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/ld2402/calibrate"))
    {
        PeripheryManager.calibrateLD2402();
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/brightness/slope"))
    {
        const float requestedSlope = payloadCopy.toFloat();
        if (requestedSlope >= 0.2f && requestedSlope <= 5.0f)
        {
            LDR_FACTOR = requestedSlope;
            persistBrightnessSlope();
            char slopeText[16];
            snprintf(slopeText, sizeof(slopeText), "%.2f", LDR_FACTOR);
            MQTTManager.publish("brightness/slope", slopeText);
            if (DEBUG_MODE)
                DEBUG_PRINTF("Brightness lux slope set to %.2f", LDR_FACTOR);
        }
        return;
    }

#ifdef ESP32_C3
    if (strTopic.equals(MQTT_PREFIX + "/ha/matrix_power"))
    {
        DisplayManager.setPower(payloadCopy.equalsIgnoreCase("ON"));
        publishC3StatsNow();
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/ha/indicator1") || strTopic.equals(MQTT_PREFIX + "/ha/indicator2") || strTopic.equals(MQTT_PREFIX + "/ha/indicator3"))
    {
        const bool enabled = payloadCopy.equalsIgnoreCase("ON");
        if (strTopic.endsWith("indicator1"))
            DisplayManager.setIndicator1State(enabled);
        else if (strTopic.endsWith("indicator2"))
            DisplayManager.setIndicator2State(enabled);
        else
            DisplayManager.setIndicator3State(enabled);
        publishC3StatsNow();
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/ha/brightness_mode"))
    {
        AUTO_BRIGHTNESS = payloadCopy.equalsIgnoreCase("Auto");
        if (!AUTO_BRIGHTNESS)
            DisplayManager.setBrightness(BRIGHTNESS);
        saveSettings();
        publishC3StatsNow();
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/ha/transition"))
    {
        AUTO_TRANSITION = payloadCopy.equalsIgnoreCase("ON");
        DisplayManager.setAutoTransition(AUTO_TRANSITION);
        saveSettings();
        publishC3StatsNow();
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/ha/transition_effect"))
    {
        static const char *const effects[] = {"Random", "Slide", "Dim", "Zoom", "Rotate", "Pixelate", "Curtain", "Ripple", "Blink", "Reload", "Fade"};
        for (uint8_t i = 0; i < sizeof(effects) / sizeof(effects[0]); i++)
        {
            if (payloadCopy.equalsIgnoreCase(effects[i]))
            {
                TRANS_EFFECT = i;
                saveSettings();
                publishC3StatsNow();
                break;
            }
        }
        return;
    }
#endif

    if (strTopic.equals(MQTT_PREFIX + "/nextapp"))
    {
        DisplayManager.nextApp();
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/previousapp"))
    {
        DisplayManager.previousApp();
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/rtttl"))
    {
        PeripheryManager.playRTTTLString(payloadCopy.c_str());
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/power"))
    {
        StaticJsonDocument<128> doc;
        DeserializationError error = deserializeJson(doc, payloadCopy.c_str());
        if (error)
        {
            if (DEBUG_MODE)
                DEBUG_PRINTLN(F("Failed to parse json"));
            return;
        }
        if (doc.containsKey("power"))
        {
            DisplayManager.setPower(doc["power"].as<bool>());
        }
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/sleep"))
    {
        StaticJsonDocument<128> doc;
        DeserializationError error = deserializeJson(doc, payloadCopy.c_str());
        if (error)
        {
            if (DEBUG_MODE)
                DEBUG_PRINTLN(F("Failed to parse json"));
            return;
        }
        if (doc.containsKey("sleep"))
        {
            DisplayManager.setPower(false);
            PowerManager.sleep(doc["sleep"].as<uint64_t>());
        }
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/indicator1"))
    {
        DisplayManager.indicatorParser(1, payloadCopy.c_str());
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/indicator2"))
    {
        DisplayManager.indicatorParser(2, payloadCopy.c_str());
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/indicator3"))
    {
        DisplayManager.indicatorParser(3, payloadCopy.c_str());
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/moodlight"))
    {
        DisplayManager.moodlight(payloadCopy.c_str());
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/reboot"))
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN("REBOOT COMMAND RECEIVED");
        delay(1000);
        ESP.restart();
        return;
    }

    if (strTopic.equals(MQTT_PREFIX + "/sound"))
    {
        PeripheryManager.parseSound(payloadCopy.c_str());
        return;
    }

    if (strTopic.startsWith(MQTT_PREFIX + "/custom"))
    {
        String topic_str = strTopic;
        String prefix = MQTT_PREFIX + "/custom/";
        if (topic_str.startsWith(prefix))
        {
            topic_str = topic_str.substring(prefix.length());
            DisplayManager.logC3Heap("mqtt_custom_before_parse");
            if (!DisplayManager.parseCustomPage(topic_str, payloadCopy.c_str(), false))
            {
                DEBUG_PRINTF("MQTT custom app '%s' was not admitted", topic_str.c_str());
            }
            DisplayManager.logC3Heap("mqtt_custom_after_parse");
        }
        return;
    }

    if (mqttValues.find(strTopic) != mqttValues.end())
    {
        mqttValues[strTopic] = payloadCopy;
        if (DEBUG_MODE)
        {
            Serial.print("Updated existing topic: ");
            Serial.println(strTopic);
            Serial.print("New value: ");
            Serial.println(mqttValues[strTopic]);
        }
        return;
    }
}

void onButtonCommand(HAButton *sender)
{
    if (sender == dismiss)
    {
        DisplayManager.dismissNotify();
    }
    else if (sender == nextApp)
    {
        DisplayManager.nextApp();
    }
    else if (sender == prevApp)
    {
        DisplayManager.previousApp();
    }
    else if (sender == doUpdate)
    {
        MQTTManager.publish("update/status", "disabled");
    }
    else if (sender == ld2402Calibrate)
    {
        PeripheryManager.calibrateLD2402();
    }
}

void onSwitchCommand(bool state, HASwitch *sender)
{
    AUTO_TRANSITION = state;
    DisplayManager.setAutoTransition(state);
    saveSettings();
    sender->setState(state);
}

void onSelectCommand(int8_t index, HASelect *sender)
{
    if (sender == BriMode)
    {
        switch (index)
        {
        case 0:
            AUTO_BRIGHTNESS = false;
            Matrix->setBrightness(BRIGHTNESS, true);
            break;
        case 1:
            AUTO_BRIGHTNESS = true;
            break;
        }
    }
    else if (sender == transEffect)
    {
        TRANS_EFFECT = index;
    }
    saveSettings();
    sender->setState(index);
}

void onRGBColorCommand(HALight::RGBColor color, HALight *sender)
{
    if (sender == Matrix)
    {
        TEXTCOLOR_888 = (color.red << 16) | (color.green << 8) | color.blue;
        DisplayManager.setCustomAppColors(TEXTCOLOR_888);
        saveSettings();
    }
    else if (sender == Indikator1)
    {
        DisplayManager.setIndicator1Color((color.red << 16) | (color.green << 8) | color.blue);
    }
    else if (sender == Indikator2)
    {
        DisplayManager.setIndicator2Color((color.red << 16) | (color.green << 8) | color.blue);
    }
    else if (sender == Indikator3)
    {
        DisplayManager.setIndicator3Color((color.red << 16) | (color.green << 8) | color.blue);
    }
    sender->setRGBColor(color); // report color back to the Home Assistant
}

void onStateCommand(bool state, HALight *sender)
{
    if (sender == Matrix)
    {
        DisplayManager.setPower(state);
    }
    else if (sender == Indikator1)
    {
        DisplayManager.setIndicator1State(state);
    }
    else if (sender == Indikator2)
    {
        DisplayManager.setIndicator2State(state);
    }
    else if (sender == Indikator3)
    {
        DisplayManager.setIndicator3State(state);
    }
    sender->setState(state);
}

void onBrightnessCommand(uint8_t brightness, HALight *sender)
{
    sender->setBrightness(brightness);
    if (AUTO_BRIGHTNESS)
        return;
    BRIGHTNESS = brightness;
    saveSettings();
    DisplayManager.setBrightness(brightness);
}

void onNumberCommand(HANumeric number, HANumber *sender)
{
    if (!number.isSet())
    {
        // the reset command was send by Home Assistant
    }
    else
    {
        SCROLL_SPEED = number.toInt8();
        saveSettings();
    }

    sender->setState(number); // report the selected option back to the HA panel
}

void onMqttMessage(const char *topic, const uint8_t *payload, uint16_t length)
{
    DisplayManager.logC3Heap("mqtt_callback_entry");
    if (DEBUG_MODE)
        DEBUG_PRINTF("MQTT message received at topic %s", topic);

    if (length >= MQTT_MAX_PAYLOAD_SIZE)
    {
        if (DEBUG_MODE)
            DEBUG_PRINTF("MQTT payload too large: %u bytes", length);
        return;
    }

    if (length < sizeof(mqttPayloadBuffer))
    {
        memcpy(mqttPayloadBuffer, payload, length);
        mqttPayloadBuffer[length] = '\0';
        DisplayManager.logC3Heap("mqtt_inline_copied");
        processMqttMessage(String(topic), String(mqttPayloadBuffer));
        return;
    }

    // Keep the common small-message buffer in static memory while allowing
    // large custom pages without permanently consuming another 3 KB of RAM.
    String largePayload;
    DisplayManager.logC3Heap("mqtt_large_before_copy");
    if (!largePayload.reserve(length + 1) || !largePayload.concat(reinterpret_cast<const char *>(payload), length))
    {
        DEBUG_PRINTF("MQTT payload allocation failed: %u bytes", length);
        return;
    }
    DisplayManager.logC3Heap("mqtt_large_copied");
    processMqttMessage(String(topic), largePayload);
}

String MQTTManager_::getValueForTopic(const String &topic)
{
    if (mqttValues.find(topic) != mqttValues.end())
    {
        return mqttValues[topic];
    }
    else
    {
        return "N/A"; // Return "N/A" if the topic is not found
    }
}

void onMqttConnected()
{

    if (DEBUG_MODE)
        DEBUG_PRINTLN(F("MQTT Connected"));
#ifdef ESP32_C3
    if (c3LightHaDiscoveryRequested)
    {
        c3LightHaDiscoveryPending = true;
        c3LightHaDiscoveryStep = 0;
        c3LightHaDiscoveryLastPublish = 0;
    }
#endif
    const char *topics[] PROGMEM = {
        "/brightness",
        "/notify/dismiss",
        "/notify",
        "/custom/#",
        "/switch",
        "/settings",
        "/previousapp",
        "/nextapp",
        "/doupdate",
        "/apps",
        "/power",
        "/sleep",
        "/indicator1",
        "/indicator2",
        "/indicator3",
        "/timeformat",
        "/dateformat",
        "/reboot",
        "/moodlight",
        "/sound",
        "/rtttl",
        "/sendscreen",
        "/r2d2",
        "/ld2402/calibrate",
        "/brightness/slope",
        "/ha/matrix_power",
        "/ha/indicator1",
        "/ha/indicator2",
        "/ha/indicator3",
        "/ha/brightness_mode",
        "/ha/transition",
        "/ha/transition_effect"};
#ifdef ESP32_C3
    // ArduinoHA invokes this callback from mqtt.loop(). Sending every
    // subscription here fills the C3 TCP send buffer before it can process
    // broker acknowledgements. Queue them for the regular loop instead.
    for (const char *topic : topics)
    {
        const String fullTopic = MQTT_PREFIX + topic;
        if (std::find(topicsToSubscribe.begin(), topicsToSubscribe.end(), fullTopic) == topicsToSubscribe.end())
        {
            topicsToSubscribe.push_back(fullTopic);
        }
    }
    c3SubscriptionIndex = 0;
    c3LastSubscriptionMillis = 0;
    c3SubscriptionsPending = true;
    return;
#else
    for (const char *topic : topics)
    {
        if (DEBUG_MODE)
            DEBUG_PRINTF("Subscribe to topic %s", topic);
        mqtt.subscribe((MQTT_PREFIX + topic).c_str());
        delay(30);
    }

    for (const auto &topic : topicsToSubscribe)
    {
        mqtt.subscribe(topic.c_str());
        if (DEBUG_MODE)
            Serial.printf("Subscribed to topic %s\n", topic.c_str());
    }
    topicsToSubscribe.clear();

      delay(200);
      if (HA_DISCOVERY)
    {
        myOwnID->setValue(MQTT_PREFIX.c_str());
        version->setValue(VERSION);
    }

#ifndef ESP32_C3
    MQTTManager.publish("stats/effects", DisplayManager.getEffectNames().c_str());
    MQTTManager.publish("stats/transitions", DisplayManager.getTransitionNames().c_str());
    if (!HA_DISCOVERY)
    {
        MQTTManager.publish("stats/device", "online");
    }
#endif
      connected = true;
#endif
  }

bool MQTTManager_::subscribe(const char *topic)
{
    if (mqttValues.find(topic) == mqttValues.end())
    {
        mqttValues[topic] = "N/A";
    }
#ifdef ESP32_C3
    if (mqtt.isConnected() && !c3SubscriptionsPending)
    {
        mqtt.subscribe(topic);
    }
    else if (std::find(topicsToSubscribe.begin(), topicsToSubscribe.end(), topic) == topicsToSubscribe.end())
    {
        topicsToSubscribe.push_back(topic);
    }
#else
    if (mqtt.isConnected())
    {
        mqtt.subscribe(topic);
    }
    else
    {
        topicsToSubscribe.push_back(topic);
    }
#endif
    return true;
}

bool MQTTManager_::isConnected()
{
    if (MQTT_HOST != "")
    {
        return mqtt.isConnected();
    }
    else
    {
        return true;
    }
}

void connect()
{
    if (MQTT_HOST == "")
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("MQTT disabled: host is empty"));
        return;
    }

    mqtt.onMessage(onMqttMessage);
    mqtt.onConnected(onMqttConnected);

#ifdef ESP32_C3
    static char c3AvailabilityTopic[100];
    snprintf(c3AvailabilityTopic, sizeof(c3AvailabilityTopic), "%s/status", MQTT_PREFIX.c_str());
    mqtt.setLastWill(c3AvailabilityTopic, "offline", true);
#else
    if (!HA_DISCOVERY)
    {
        static char topic[50];
        snprintf(topic, sizeof(topic), "%s/stats/device", MQTT_PREFIX.c_str());
        mqtt.setLastWill(topic, "offline", false);
    }
#endif

    if (MQTT_USER == "" || MQTT_PASS == "")
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Connecting to MQTT w/o login"));
        mqtt.begin(MQTT_HOST.c_str(), MQTT_PORT, nullptr, nullptr, HOSTNAME.c_str());
    }
    else
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Connecting to MQTT with login"));
        mqtt.begin(MQTT_HOST.c_str(), MQTT_PORT, MQTT_USER.c_str(), MQTT_PASS.c_str(), HOSTNAME.c_str());
    }
}

void MQTTManager_::sendStats()
{
    if (MQTT_HOST == "" || !mqtt.isConnected())
    {
        return;
    }

    if (HA_DISCOVERY)
    {
        char buffer[8];
#ifdef ULANZI
        snprintf(buffer, 5, "%d", BATTERY_PERCENT);
        battery->setValue(buffer);
#endif
        if (SENSOR_READING)
        {
            snprintf(buffer, sizeof(buffer), "%.*f", TEMP_DECIMAL_PLACES, CURRENT_TEMP);
            temperature->setValue(buffer);
            snprintf(buffer, 5, "%.0f", CURRENT_HUM);
            humidity->setValue(buffer);
        }

        snprintf(buffer, sizeof(buffer), "%.0f", CURRENT_LUX);
        illuminance->setValue(buffer);
        ld2402Presence->setState(LD2402_AVAILABLE && LD2402_PRESENCE, false);
        ld2402Motion->setValue(ld2402MotionText());
        snprintf(buffer, sizeof(buffer), "%u", LD2402_DISTANCE_CM);
        ld2402Distance->setValue(buffer);
        BriMode->setState(AUTO_BRIGHTNESS, false);
        Matrix->setBrightness(BRIGHTNESS);
        Matrix->setState(!MATRIX_OFF, false);
        HALight::RGBColor color;
        color.isSet = true;
        color.red = (TEXTCOLOR_888 >> 16) & 0xFF;
        color.green = (TEXTCOLOR_888 >> 8) & 0xFF;
        color.blue = TEXTCOLOR_888 & 0xFF;
        Matrix->setRGBColor(color);
        int8_t rssiValue = WiFi.RSSI();
        char rssiString[4];
        snprintf(rssiString, sizeof(rssiString), "%d", rssiValue);
        strength->setValue(rssiString);

        char rambuffer[10];
        int freeHeapBytes = ESP.getFreeHeap();
        itoa(freeHeapBytes, rambuffer, 10);
        ram->setValue(rambuffer);
        char uptimeStr[25]; // Buffer for string representation
        sprintf(uptimeStr, "%ld", PeripheryManager.readUptime());
        uptime->setValue(uptimeStr);
        transition->setState(AUTO_TRANSITION, false);
        ipAddr->setValue(ServerManager.myIP.toString().c_str());
    }
    char luxBuffer[16];
    snprintf(luxBuffer, sizeof(luxBuffer), "%.3f", CURRENT_LUX);
    publish("stats/lux", luxBuffer);
    publishLD2402State();
#ifdef ESP32_C3
      char statsBuffer[512];
      DisplayManager.getStats(statsBuffer, sizeof(statsBuffer));
      publish(StatsTopic, statsBuffer);
#else
      char statsBuffer[512];
      DisplayManager.getStats(statsBuffer, sizeof(statsBuffer));
      publish(StatsTopic, statsBuffer);
#endif
  }

void MQTTManager_::setup()
{

#ifdef ESP32_C3
    if (HA_DISCOVERY)
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("ESP32-C3: using lightweight Home Assistant discovery"));
        c3LightHaDiscoveryRequested = true;
        HA_DISCOVERY = false;
    }
#endif

    if (HA_DISCOVERY)
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Starting Homeassistant discovery"));
        mqtt.setDiscoveryPrefix(HA_PREFIX.c_str());
        mqtt.setDataPrefix(MQTT_PREFIX.c_str());
        uint8_t mac[6];
        WiFi.macAddress(mac);
        char macStr[7];
        snprintf(macStr, 7, "%02x%02x%02x", mac[3], mac[4], mac[5]);
        device.setUniqueId(mac, sizeof(mac));
        device.setName(HOSTNAME.c_str());
        device.setSoftwareVersion(VERSION);
        device.setManufacturer(HAmanufacturer);

        device.setModel(HAmodel);
        device.setAvailability(true);
        device.enableSharedAvailability();
        device.enableLastWill();

        IPAddress ip = WiFi.localIP();
        static char configurationUrl[32]; // static!
        sprintf(configurationUrl, "http://%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
        device.setConfigurationUrl(configurationUrl);

        sprintf(matID, HAmatID, macStr);
        Matrix = new HALight(matID, HALight::BrightnessFeature | HALight::RGBFeature);

        Matrix->setIcon(HAmatIcon);
        Matrix->setName(HAmatName);
        Matrix->onStateCommand(onStateCommand);
        Matrix->onBrightnessCommand(onBrightnessCommand);
        Matrix->onRGBColorCommand(onRGBColorCommand);
        Matrix->setCurrentState(true);
        Matrix->setBRIGHTNESS(BRIGHTNESS);

        HALight::RGBColor color;
        color.isSet = true;
        color.red = (TEXTCOLOR_888 >> 16) & 0xFF;  // Die obersten 8 Bits für Rot
        color.green = (TEXTCOLOR_888 >> 8) & 0xFF; // Die mittleren 8 Bits für Grün
        color.blue = TEXTCOLOR_888 & 0xFF;         // Die untersten 8 Bits für Blau
        Matrix->setCurrentRGBColor(color);
        Matrix->setState(MATRIX_OFF, true);

        sprintf(ind1ID, HAi1ID, macStr);
        Indikator1 = new HALight(ind1ID, HALight::RGBFeature);
        Indikator1->setIcon(HAi1Icon);
        Indikator1->setName(HAi1Name);
        Indikator1->onStateCommand(onStateCommand);
        Indikator1->onRGBColorCommand(onRGBColorCommand);

        sprintf(ind2ID, HAi2ID, macStr);
        Indikator2 = new HALight(ind2ID, HALight::RGBFeature);
        Indikator2->setIcon(HAi2Icon);
        Indikator2->setName(HAi2Name);
        Indikator2->onStateCommand(onStateCommand);
        Indikator2->onRGBColorCommand(onRGBColorCommand);

        sprintf(ind3ID, HAi3ID, macStr);
        Indikator3 = new HALight(ind3ID, HALight::RGBFeature);
        Indikator3->setIcon(HAi3Icon);
        Indikator3->setName(HAi3Name);
        Indikator3->onStateCommand(onStateCommand);
        Indikator3->onRGBColorCommand(onRGBColorCommand);

        sprintf(briID, HAbriID, macStr);
        BriMode = new HASelect(briID);
        BriMode->setOptions(HAbriOptions);
        BriMode->onCommand(onSelectCommand);
        BriMode->setIcon(HAbriIcon);
        BriMode->setName(HAbriName);
        BriMode->setState(AUTO_BRIGHTNESS, true);

        sprintf(effectID, HAeffectID, macStr);
        transEffect = new HASelect(effectID);
        transEffect->setOptions(HAeffectOptions);
        transEffect->onCommand(onSelectCommand);
        transEffect->setIcon(HAeffectIcon);
        transEffect->setName(HAeffectName);
        transEffect->setState(TRANS_EFFECT, true);

        sprintf(btnAID, HAbtnaID, macStr);
        dismiss = new HAButton(btnAID);
        dismiss->setIcon(HAbtnaIcon);
        dismiss->setName(HAbtnaName);

        sprintf(doUpdateID, HAdoUpID, macStr);
        doUpdate = new HAButton(doUpdateID);
        doUpdate->setIcon(HAdoUpIcon);
        doUpdate->setName(HAdoUpName);
        doUpdate->onCommand(onButtonCommand);

        sprintf(transID, HAtransID, macStr);
        transition = new HASwitch(transID);
        transition->setIcon(HAtransIcon);
        transition->setName(HAtransName);
        transition->onCommand(onSwitchCommand);

        sprintf(appID, HAappID, macStr);
        curApp = new HASensor(appID);
        curApp->setIcon(HAappIcon);
        curApp->setName(HAappName);

        sprintf(myID, HAIDID, macStr);
        myOwnID = new HASensor(myID);
        myOwnID->setIcon(HAIDIcon);
        myOwnID->setName(HAIDName);

        sprintf(btnBID, HAbtnbID, macStr);
        nextApp = new HAButton(btnBID);
        nextApp->setIcon(HAbtnbIcon);
        nextApp->setName(HAbtnbName);

        sprintf(btnCID, HAbtncID, macStr);
        prevApp = new HAButton(btnCID);
        prevApp->setIcon(HAbtncIcon);
        prevApp->setName(HAbtncName);

        dismiss->onCommand(onButtonCommand);
        nextApp->onCommand(onButtonCommand);
        prevApp->onCommand(onButtonCommand);

        sprintf(tempID, HAtempID, macStr);
        temperature = new HASensor(tempID);
        temperature->setIcon(HAtempIcon);
        temperature->setName(HAtempName);
        temperature->setDeviceClass(HAtempClass);
        temperature->setUnitOfMeasurement(HAtempUnit);

        sprintf(humID, HAhumID, macStr);
        humidity = new HASensor(humID);
        humidity->setIcon(HAhumIcon);
        humidity->setName(HAhumName);
        humidity->setDeviceClass(HAhumClass);
        humidity->setUnitOfMeasurement(HAhumUnit);

#ifdef ULANZI
        sprintf(batID, HAbatID, macStr);
        battery = new HASensor(batID);
        battery->setIcon(HAbatIcon);
        battery->setName(HAbatName);
        battery->setDeviceClass(HAbatClass);
        battery->setUnitOfMeasurement(HAbatUnit);

#endif
        sprintf(luxID, HAluxID, macStr);
        illuminance = new HASensor(luxID);
        illuminance->setIcon(HAluxIcon);
        illuminance->setName(HAluxName);
        illuminance->setDeviceClass(HAluxClass);
        illuminance->setUnitOfMeasurement(HAluxUnit);

        snprintf(ld2402PresenceID, sizeof(ld2402PresenceID), "%s_ld2402_presence", macStr);
        ld2402Presence = new HABinarySensor(ld2402PresenceID);
        ld2402Presence->setName("LD2402 Presence");

        snprintf(ld2402MotionID, sizeof(ld2402MotionID), "%s_ld2402_motion", macStr);
        ld2402Motion = new HASensor(ld2402MotionID);
        ld2402Motion->setName("LD2402 Motion");
        ld2402Motion->setIcon("mdi:motion-sensor");

        snprintf(ld2402DistanceID, sizeof(ld2402DistanceID), "%s_ld2402_distance", macStr);
        ld2402Distance = new HASensor(ld2402DistanceID);
        ld2402Distance->setName("LD2402 Distance");
        ld2402Distance->setIcon("mdi:map-marker-distance");
        ld2402Distance->setUnitOfMeasurement("cm");

        snprintf(ld2402CalibrateID, sizeof(ld2402CalibrateID), "%s_ld2402_calibrate", macStr);
        ld2402Calibrate = new HAButton(ld2402CalibrateID);
        ld2402Calibrate->setName("LD2402 Calibrate");
        ld2402Calibrate->setIcon("mdi:tune-variant");
        ld2402Calibrate->onCommand(onButtonCommand);

        sprintf(verID, HAverID, macStr);
        version = new HASensor(verID);
        version->setName(HAverName);

        sprintf(sigID, HAsigID, macStr);
        strength = new HASensor(sigID);
        strength->setName(HAsigName);
        strength->setDeviceClass(HAsigClass);
        strength->setUnitOfMeasurement(HAsigUnit);

        sprintf(upID, HAupID, macStr);
        uptime = new HASensor(upID);
        uptime->setName(HAupName);
        uptime->setDeviceClass(HAupClass);
        uptime->setUnitOfMeasurement("s");

        sprintf(btnLID, HAbtnLID, macStr);
        btnleft = new HABinarySensor(btnLID);
        btnleft->setName(HAbtnLName);

        sprintf(btnMID, HAbtnMID, macStr);
        btnmid = new HABinarySensor(btnMID);
        btnmid->setName(HAbtnMName);

        sprintf(btnRID, HAbtnRID, macStr);
        btnright = new HABinarySensor(btnRID);
        btnright->setName(HAbtnRName);

        sprintf(ramID, HAramRID, macStr);
        ram = new HASensor(ramID);
        ram->setDeviceClass(HAramClass);
        ram->setIcon(HAramIcon);
        ram->setName(HAramName);
        ram->setUnitOfMeasurement(HAramUnit);

        sprintf(ipAddrID, HAipAddrRID, macStr);
        ipAddr = new HASensor(ipAddrID);
        ipAddr->setName(HAipAddrName);
        ipAddr->setIcon(HAipAddrIcon);
    }
    else
    {
#ifdef ESP32_C3
        if (c3LightHaDiscoveryRequested)
        {
            Serial.println(F("Homeassistant discovery lightweight mode"));
        }
        else
#endif
        {
            Serial.println(F("Homeassistant discovery disabled"));
        }
        mqtt.disableHA();
    }

    connect();
}

void MQTTManager_::tick()
{
    if (MQTT_HOST != "")
    {
        mqtt.loop();
    }
    unsigned long currentMillis_Stats = millis();
#ifdef ESP32_C3
    if (c3SubscriptionsPending && mqtt.isConnected() && currentMillis_Stats - c3LastSubscriptionMillis >= C3_SUBSCRIPTION_INTERVAL_MS)
    {
        if (c3SubscriptionIndex < topicsToSubscribe.size())
        {
            const String &topic = topicsToSubscribe[c3SubscriptionIndex++];
            if (DEBUG_MODE)
                DEBUG_PRINTF("Subscribe to topic %s", topic.c_str());
            mqtt.subscribe(topic.c_str());
            c3LastSubscriptionMillis = currentMillis_Stats;
        }
        else
        {
            topicsToSubscribe.clear();
            c3SubscriptionIndex = 0;
            c3SubscriptionsPending = false;
            publishC3AvailabilityIfNeeded();
            publish("stats/device_topic", MQTT_PREFIX.c_str());
            if (HA_DISCOVERY)
            {
                myOwnID->setValue(MQTT_PREFIX.c_str());
                version->setValue(VERSION);
            }
            connected = true;
        }
    }
    publishC3AvailabilityIfNeeded();
    // Discovery is nonessential and creates transient MQTT buffers. Defer it
    // while a custom GIF holds the C3's LittleFS/decoder working set.
    if (!DisplayManager.showGif)
    {
        publishC3HaDiscoveryTick(currentMillis_Stats);
    }
#endif
#ifdef ESP32_C3
    if ((currentMillis_Stats - previousMillis_Stats >= MQTT_C3_STATS_INTERVAL) && SENSORS_STABLE)
#else
    if ((currentMillis_Stats - previousMillis_Stats >= STATS_INTERVAL) && SENSORS_STABLE)
#endif
    {
        previousMillis_Stats = currentMillis_Stats;
        sendStats();
    }
}

void MQTTManager_::publish(const char *topic, const char *payload)
{
    if (!mqttHasWriteRoom())
        return;

    char result[256];
    const int length = snprintf(result, sizeof(result), "%s/%s", MQTT_PREFIX.c_str(), topic);
    if (length < 0 || static_cast<size_t>(length) >= sizeof(result))
    {
        DEBUG_PRINTLN(F("MQTT topic too long"));
        return;
    }

    if (!mqtt.publish(result, payload, false))
        stopMqttAfterWriteFailure();
}

void MQTTManager_::publishLD2402State()
{
    char luxBuffer[16];
    snprintf(luxBuffer, sizeof(luxBuffer), "%.3f", CURRENT_LUX);

    char payload[224];
    snprintf(payload, sizeof(payload),
             "{\"available\":%s,\"presence\":%s,\"motion\":\"%s\",\"distance\":%u,\"calibration\":\"%s\",\"lux\":%s}",
             LD2402_AVAILABLE ? "true" : "false", LD2402_PRESENCE ? "true" : "false",
             ld2402MotionText(), LD2402_DISTANCE_CM, PeripheryManager.getLD2402CalibrationState(), luxBuffer);
    publish("ld2402/status", payload);
}

void MQTTManager_::rawPublish(const char *prefix, const char *topic, const char *payload)
{
    if (!mqttHasWriteRoom())
        return;
    char result[256];
    const int length = snprintf(result, sizeof(result), "%s/%s", prefix, topic);
    if (length < 0 || static_cast<size_t>(length) >= sizeof(result))
    {
        DEBUG_PRINTLN(F("MQTT topic too long"));
        return;
    }
    if (!mqtt.publish(result, payload, false))
        stopMqttAfterWriteFailure();
}

void MQTTManager_::setCurrentApp(String appName)
{
      static String lastApp = "";

      if (lastApp == appName)
          return;

      if (DEBUG_MODE)
          DEBUG_PRINTF("Publish current app %s", appName.c_str());
      if (HA_DISCOVERY && mqtt.isConnected())
          curApp->setValue(appName.c_str());

#ifdef ESP32_C3
      const uint32_t freeHeap = ESP.getFreeHeap();
      const uint32_t maxAllocHeap = ESP.getMaxAllocHeap();
      static unsigned long lastCurrentAppPublish = 0;
      if (freeHeap < MQTT_C3_CURRENT_APP_MIN_FREE_HEAP || maxAllocHeap < MQTT_C3_CURRENT_APP_MIN_MAX_ALLOC_HEAP)
      {
          if (DEBUG_MODE)
              DEBUG_PRINTF("Skip current app publish on ESP32-C3: free=%lu max_alloc=%lu", freeHeap, maxAllocHeap);
          lastApp = appName;
          return;
      }
      if (millis() - lastCurrentAppPublish < 5000)
      {
          lastApp = appName;
          return;
      }
      lastCurrentAppPublish = millis();
#endif
      publish("stats/currentApp", appName.c_str());
      lastApp = appName;
}

void MQTTManager_::sendButton(byte btn, bool state)
{
    static bool btn0State, btn1State, btn2State;

    switch (btn)
    {
    case 0:
        if (btn0State != state)
        {
            if (HA_DISCOVERY && mqtt.isConnected())
                btnleft->setState(state, false);
            btn0State = state;
            publish(ButtonLeftTopic, state ? State1 : State0);
        }
        break;
    case 1:
        if (btn1State != state)
        {
            if (HA_DISCOVERY && mqtt.isConnected())
                btnmid->setState(state, false);
            btn1State = state;
            publish(ButtonSelectTopic, state ? State1 : State0);
        }

        break;
    case 2:
        if (btn2State != state)
        {
            if (HA_DISCOVERY && mqtt.isConnected())
                btnright->setState(state, false);
            btn2State = state;
            publish(ButtonRightTopic, state ? State1 : State0);
        }
        break;
    default:
        break;
    }
}

void MQTTManager_::setIndicatorState(uint8_t indicator, bool state, uint32_t color)
{
    if (HA_DISCOVERY && mqtt.isConnected())
    {
        HALight::RGBColor c;
        c.isSet = true;
        c.red = (color >> 16) & 0xFF;  // Rote Komponente 8-Bit
        c.green = (color >> 8) & 0xFF; // Grüne Komponente 8-Bit
        c.blue = color & 0xFF;

        switch (indicator)
        {
        case 1:
            Indikator1->setRGBColor(c);
            Indikator1->setState(state);
            break;
        case 2:
            Indikator2->setRGBColor(c);
            Indikator2->setState(state);
            break;
        case 3:
            Indikator3->setRGBColor(c);
            Indikator3->setState(state);
            break;
        default:
            break;
        }
    }
}

void MQTTManager_::beginPublish(const char *topic, unsigned int plength, boolean retained)
{
    if (!mqttHasWriteRoom())
        return;

    if (!mqtt.beginPublish(topic, plength, retained))
        stopMqttAfterWriteFailure();
}

void MQTTManager_::writePayload(const char *data, const uint16_t length)
{
    if (!mqttHasWriteRoom())
        return;

    mqtt.writePayload(data, length);
}

void MQTTManager_::endPublish()
{
    if (!mqttHasWriteRoom())
        return;

    mqtt.endPublish();
}
