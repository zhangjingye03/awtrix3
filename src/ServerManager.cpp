#include "ServerManager.h"
#include "Globals.h"
#include <WebServer.h>
#include <esp-fs-webserver.h>
#include "htmls.h"
#include <Update.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <WiFi.h>
#include "DisplayManager.h"
#include "UpdateManager.h"
#include "PeripheryManager.h"
#include "PowerManager.h"
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include "Games/GameManager.h"
#include <EEPROM.h>
#include <mbedtls/base64.h>

WiFiUDP udp;

unsigned int localUdpPort = 4210;
char incomingPacket[255];

// Pufferdefinition
#define BUFFER_SIZE 64
char dataBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// Aktueller verbundener Client
WiFiClient currentClient = WiFiClient();
#ifdef ESP32_C3
class C3WebServer : public WebServer
{
public:
    using WebServer::WebServer;

    ~C3WebServer() override
    {
        free(_heapReserve);
    }

    void handleClient() override
    {
        if (_currentStatus == HC_NONE)
        {
            // An animated GIF keeps a LittleFS read buffer alive. Do not
            // reserve a second heap block while it is rendering; the RX
            // reserve can be recreated after the GIF page becomes inactive.
            if (DisplayManager.showGif && _heapReserve != nullptr)
            {
                free(_heapReserve);
                _heapReserve = nullptr;
                _reserveDeferred = true;
#ifdef ESP32_C3
                Serial.printf("[%lu] [C3HTTP] reserve released for active GIF free=%u largest=%u\n",
                              millis(), ESP.getFreeHeap(), ESP.getMaxAllocHeap());
#endif
            }
            _currentClient = _server.available();
            if (!_currentClient)
            {
                restoreHeapReserve();
                if (_nullDelay)
                    delay(1);
                return;
            }

            // WiFiClient allocates its 1436-byte RX buffer lazily while
            // parsing the request. Keep this contiguous emergency block out
            // of the fragmented GIF heap, then release it before parsing.
            // This avoids an active named GIF making every HTTP endpoint
            // unavailable solely because WiFiClient::fillBuffer() cannot
            // allocate its fixed RX buffer.
#ifdef ESP32_C3
            Serial.printf("[%lu] [C3HTTP] client accepted free=%u largest=%u reserve=%u\n",
                          millis(), ESP.getFreeHeap(), ESP.getMaxAllocHeap(),
                          _heapReserve == nullptr ? 0 : HeapReserveSize);
#endif
            DisplayManager.prepareForHttpRequest();
            free(_heapReserve);
            _heapReserve = nullptr;
#ifdef ESP32_C3
            Serial.printf("[%lu] [C3HTTP] reserve released free=%u largest=%u\n",
                          millis(), ESP.getFreeHeap(), ESP.getMaxAllocHeap());
#endif
            _currentStatus = HC_WAIT_READ;
            _statusChange = millis();
        }

        bool keepCurrentClient = false;
        bool callYield = false;
        if (_currentClient.connected())
        {
            if (_currentStatus == HC_WAIT_READ)
            {
                if (_currentClient.available())
                {
                    if (_parseRequest(_currentClient))
                    {
                        _currentClient.setTimeout(HTTP_MAX_SEND_WAIT / 1000);
                        _contentLength = CONTENT_LENGTH_NOT_SET;
                        _handleRequest();
                    }
                }
                // A C3 has only one synchronous WebServer client. Do not
                // hold it for the framework's multi-second read timeout:
                // rapid curl/browser polling can otherwise leave a stale
                // half-open client blocking every following request.
                else if (millis() - _statusChange <= 250)
                {
                    keepCurrentClient = true;
                    callYield = true;
                }
            }
        }

        if (!keepCurrentClient)
        {
            // Arduino-ESP32 2.0.x only releases the WiFiClient wrapper here.
            // Explicitly stop first so lwIP releases the per-request pbufs on
            // the C3 instead of retaining them until the peer's close timer.
            _currentClient.stop();
            _currentClient = WiFiClient();
            _currentStatus = HC_NONE;
            _currentUpload.reset();
            _currentRaw.reset();
            restoreHeapReserve();
        }

        if (callYield)
            yield();
    }

private:
    // WiFiClient allocates a 1436-byte RX buffer lazily. Keep only that
    // allocation available for the next request. A larger reserve looked
    // safer, but after a response lwIP still owns several KB of pbuf/socket
    // state; reclaiming 4 KB at that point fragmented the C3 heap enough to
    // prevent the next request from being received.
    static constexpr size_t HeapReserveSize = 1536;
    static constexpr size_t HeapReserveRestoreMinBlock = 4096;
    uint8_t *_heapReserve = nullptr;
    bool _reserveDeferred = false;

    void restoreHeapReserve()
    {
        if (_heapReserve == nullptr)
        {
            const uint32_t freeBefore = ESP.getFreeHeap();
            const uint32_t largestBefore = ESP.getMaxAllocHeap();
            // Do not split the last useful TCP/GIF block in two. The reserve
            // is a recovery aid, not a requirement: with it released the
            // existing largest block is still available to WiFiClient.
            if (DisplayManager.showGif || largestBefore < HeapReserveRestoreMinBlock)
            {
#ifdef ESP32_C3
                if (!_reserveDeferred)
                {
                    Serial.printf("[%lu] [C3HTTP] reserve deferred free=%u largest=%u\n",
                                  millis(), freeBefore, largestBefore);
                }
#endif
                _reserveDeferred = true;
                return;
            }
            _heapReserve = static_cast<uint8_t *>(malloc(HeapReserveSize));
            _reserveDeferred = false;
#ifdef ESP32_C3
            Serial.printf("[%lu] [C3HTTP] reserve %s free=%u largest=%u (before free=%u largest=%u)\n",
                          millis(), _heapReserve == nullptr ? "failed" : "restored",
                          ESP.getFreeHeap(), ESP.getMaxAllocHeap(), freeBefore, largestBefore);
#endif
        }
    }
};
// The normal framework lifecycle handles queued clients correctly now that
// GIF files use a bounded VFS buffer on the ESP32-C3.
WebServer server(80);
#else
WebServer server(80);
#endif
FSWebServer mws(LittleFS, server);

// Erstelle eine Server-Instanz
WiFiServer TCPserver(8080);

static bool webOtaSucceeded = false;
static bool webOtaStarted = false;
static size_t webOtaBytesWritten = 0;
static String webOtaError;

#ifdef ESP32_C3
static String c3IconUploadName;
static size_t c3IconUploadSize = 0;

static bool isValidC3IconName(const char *name)
{
    if (name == nullptr)
        return false;

    const String filename(name);
    if (filename.length() == 0 || filename.length() > 48 || filename.indexOf('/') >= 0 || filename.indexOf('\\') >= 0 || filename.indexOf("..") >= 0)
        return false;

    const String extension = filename.substring(filename.lastIndexOf('.') + 1);
    return extension.equalsIgnoreCase("gif") || extension.equalsIgnoreCase("jpg") || extension.equalsIgnoreCase("jpeg");
}

static void handleC3IconUpload()
{
    WebServerClass *request = mws.getRequest();
    StaticJsonDocument<1536> payload;
    const DeserializationError parseError = deserializeJson(payload, request->arg("plain"));
    const char *name = payload["name"].as<const char *>();
    const char *encoded = payload["data"].as<const char *>();
    const bool start = payload["start"] | false;
    const bool finalChunk = payload["final"] | false;

    if (parseError || !isValidC3IconName(name) || encoded == nullptr)
    {
        DEBUG_PRINTF("[C3HTTP] rejected icon chunk: json=%s name=%s data=%u",
                     parseError.c_str(), name == nullptr ? "<null>" : name,
                     encoded == nullptr ? 0 : static_cast<unsigned int>(strlen(encoded)));
        request->send(400, F("text/plain"), F("Invalid icon upload chunk"));
        return;
    }

    constexpr size_t maxChunkSize = 384;
    constexpr size_t maxIconSize = 32 * 1024;
    const size_t encodedLength = strlen(encoded);
    if (encodedLength == 0 || encodedLength > 4 * ((maxChunkSize + 2) / 3))
    {
        request->send(400, F("text/plain"), F("Invalid icon chunk size"));
        return;
    }

    if (start)
    {
        c3IconUploadName = name;
        c3IconUploadSize = 0;
    }
    else if (c3IconUploadName != name)
    {
        request->send(409, F("text/plain"), F("Icon upload must start with the first chunk"));
        return;
    }

    uint8_t decoded[maxChunkSize];
    size_t writtenLength = 0;
    if (mbedtls_base64_decode(decoded, sizeof(decoded), &writtenLength,
                              reinterpret_cast<const unsigned char *>(encoded), encodedLength) != 0 ||
        writtenLength == 0 || writtenLength > maxChunkSize)
    {
        request->send(400, F("text/plain"), F("Invalid base64 icon chunk"));
        return;
    }

    if (c3IconUploadSize + writtenLength > maxIconSize)
    {
        LittleFS.remove(String("/ICONS/") + c3IconUploadName);
        c3IconUploadName = String();
        c3IconUploadSize = 0;
        request->send(413, F("text/plain"), F("Icon exceeds 32 KB limit"));
        return;
    }

    const String path = String("/ICONS/") + c3IconUploadName;
    File file = LittleFS.open(path, start ? "w" : "a");
    if (!file || file.write(decoded, writtenLength) != writtenLength)
    {
        if (file)
            file.close();
        request->send(500, F("text/plain"), F("Icon write failed"));
        return;
    }
    file.close();
    c3IconUploadSize += writtenLength;

    char response[64];
    snprintf(response, sizeof(response), "{\"ok\":true,\"size\":%u}", static_cast<unsigned int>(c3IconUploadSize));
    request->send(200, F("application/json"), response);

    if (finalChunk)
    {
        DEBUG_PRINTF("[C3HTTP] icon upload complete: %s (%u bytes)", c3IconUploadName.c_str(), static_cast<unsigned int>(c3IconUploadSize));
        c3IconUploadName = String();
        c3IconUploadSize = 0;
    }
}

static void logC3WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
    if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED)
    {
        DEBUG_PRINTF("[C3WIFI] disconnected reason=%u free=%u largest=%u min=%u",
                     info.wifi_sta_disconnected.reason,
                     ESP.getFreeHeap(),
                     heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
                     ESP.getMinFreeHeap());
    }
    else if (event == ARDUINO_EVENT_WIFI_STA_GOT_IP)
    {
        DEBUG_PRINTF("[C3WIFI] got IP %s free=%u largest=%u",
                     WiFi.localIP().toString().c_str(),
                     ESP.getFreeHeap(),
                     heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    }
    else if (event == ARDUINO_EVENT_WIFI_STA_LOST_IP)
    {
        DEBUG_PRINTF("[C3WIFI] lost IP free=%u largest=%u",
                     ESP.getFreeHeap(),
                     heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    }
}
#endif

static bool finalizeWebOta()
{
    if (webOtaSucceeded || webOtaError.length() || !webOtaStarted || !webOtaBytesWritten)
        return webOtaSucceeded;

    if (Update.end(true))
    {
        webOtaSucceeded = true;
        DEBUG_PRINTF("Web OTA success: %u bytes", webOtaBytesWritten);
        return true;
    }

    StreamString error;
    Update.printError(error);
    webOtaError = error.c_str();
    return false;
}

static void sendJsonString(WebServerClass *webserver, const String &json)
{
    webserver->send(200, F("application/json"), json);
}

#ifdef ESP32_C3
static void handleC3MqttConfig()
{
    WebServerClass *request = mws.getRequest();

    if (request->method() == HTTP_GET)
    {
        StaticJsonDocument<512> response;
        response["host"] = MQTT_HOST;
        response["port"] = MQTT_PORT;
        response["user"] = MQTT_USER;
        response["prefix"] = MQTT_PREFIX;
        response["discovery"] = HA_DISCOVERY;
        char payload[512];
        serializeJson(response, payload, sizeof(payload));
        request->send(200, F("application/json"), payload);
        return;
    }

    StaticJsonDocument<512> update;
    const DeserializationError parseError = deserializeJson(update, request->arg("plain"));
    if (parseError || !update["host"].is<const char *>())
    {
        request->send(400, F("text/plain"), F("Invalid MQTT configuration"));
        return;
    }

    File file = LittleFS.open("/DoNotTouch.json", "r");
    StaticJsonDocument<1536> config;
    if (file)
    {
        const DeserializationError configError = deserializeJson(config, file);
        file.close();
        if (configError)
        {
            request->send(500, F("text/plain"), F("Cannot read configuration"));
            return;
        }
    }

    config["Broker"] = update["host"].as<const char *>();
    config["Port"] = update["port"] | 1883;
    config["Username"] = update["user"] | "";
    if (update.containsKey("pass"))
        config["Password"] = update["pass"] | "";
    config["Prefix"] = update["prefix"] | "";
    config["Homeassistant Discovery"] = update["discovery"] | false;

    file = LittleFS.open("/DoNotTouch.json", "w");
    if (!file || serializeJson(config, file) == 0)
    {
        if (file)
            file.close();
        request->send(500, F("text/plain"), F("Cannot save configuration"));
        return;
    }
    file.close();
    request->send(200, F("text/plain"), F("OK"));
}
#endif

void setupWebOtaHandler()
{
    server.on("/api/webota", HTTP_GET, []()
              { server.send(200, F("application/json"), F("{\"status\":\"ready\"}")); });
    server.on(
        "/api/webota", HTTP_POST,
        []()
        {
            // Some ESP32 WebServer versions invoke the request handler without UPLOAD_FILE_END.
            finalizeWebOta();
            if (webOtaSucceeded && !Update.hasError())
            {
                server.client().setNoDelay(true);
                server.send(200, F("text/plain"), F("OK"));
                delay(250);
                ESP.restart();
            }
            else
            {
                String message = F("OTA failed");
                if (webOtaError.length())
                {
                    message += F(": ");
                    message += webOtaError;
                }
                server.send(500, F("text/plain"), message);
            }
        },
        []()
        {
            HTTPUpload &upload = server.upload();
            if (upload.status == UPLOAD_FILE_START)
            {
                webOtaSucceeded = false;
                webOtaStarted = true;
                webOtaBytesWritten = 0;
                webOtaError = "";
                DEBUG_PRINTF("Web OTA start: %s", upload.filename.c_str());
                DisplayManager.clear();
                DisplayManager.resetTextColor();
                DisplayManager.printText(0, 6, "OTA", true, true);
                DisplayManager.show();

                if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH))
                {
                    StreamString error;
                    Update.printError(error);
                    webOtaError = error.c_str();
                    DEBUG_PRINTF("Web OTA begin failed: %s", webOtaError.c_str());
                }
            }
            else if (upload.status == UPLOAD_FILE_WRITE)
            {
                if (!webOtaError.length() && Update.write(upload.buf, upload.currentSize) == upload.currentSize)
                {
                    webOtaBytesWritten += upload.currentSize;
                }
                else if (!webOtaError.length())
                {
                    StreamString error;
                    Update.printError(error);
                    webOtaError = error.c_str();
                    DEBUG_PRINTF("Web OTA write failed: %s", webOtaError.c_str());
                }
            }
            else if (upload.status == UPLOAD_FILE_END)
            {
                finalizeWebOta();
            }
            else if (upload.status == UPLOAD_FILE_ABORTED)
            {
                Update.end();
                webOtaStarted = false;
                webOtaError = F("aborted");
                DEBUG_PRINTLN(F("Web OTA aborted"));
            }
            delay(0);
        });
}

// The getter for the instantiated singleton instance
ServerManager_ &ServerManager_::getInstance()
{
    static ServerManager_ instance;
    return instance;
}

// Initialize the global shared instance
ServerManager_ &ServerManager = ServerManager.getInstance();

void versionHandler()
{
    WebServerClass *webRequest = mws.getRequest();
    webRequest->send(200, F("text/plain"), VERSION);
}

void ServerManager_::erase()
{
    DisplayManager.HSVtext(0, 6, "RESET", true, 0);
    wifi_config_t conf;
    memset(&conf, 0, sizeof(conf)); // Set all the bytes in the structure to 0
    esp_wifi_set_config(WIFI_IF_STA, &conf);
    LittleFS.format();
    delay(200);
    formatSettings();
    delay(200);
}

void saveHandler()
{
    WebServerClass *webRequest = mws.getRequest();
    ServerManager.getInstance().loadSettings();
    webRequest->send(200);
}

void addHandler()
{
#ifdef ESP32_C3
    mws.addHandler("/api/c3/mqtt", HTTP_GET, handleC3MqttConfig);
    mws.addHandler("/api/c3/mqtt", HTTP_POST, handleC3MqttConfig);
    mws.addHandler("/api/c3/icon", HTTP_POST, handleC3IconUpload);
#endif

    mws.addHandler("/api/power", HTTP_POST, []()
                   { DisplayManager.powerStateParse(mws.webserver->arg("plain").c_str()); mws.webserver->send(200,F("text/plain"),F("OK")); });
    mws.addHandler(
        "/api/sleep", HTTP_POST, []()
        { 
            mws.webserver->send(200,F("text/plain"),F("OK"));
            DisplayManager.setPower(false);
            PowerManager.sleepParser(mws.webserver->arg("plain").c_str()); });
    mws.addHandler("/api/loop", HTTP_GET, []()
                   { mws.webserver->send_P(200, "application/json", DisplayManager.getAppsAsJson().c_str()); });
    mws.addHandler("/api/effects", HTTP_GET, []()
                   { mws.webserver->send_P(200, "application/json", DisplayManager.getEffectNames().c_str()); });
    mws.addHandler("/api/transitions", HTTP_GET, []()
                   { mws.webserver->send_P(200, "application/json", DisplayManager.getTransitionNames().c_str()); });
    mws.addHandler("/api/reboot", HTTP_ANY, []()
                   { mws.webserver->send(200,F("text/plain"),F("OK")); delay(200); ESP.restart(); });
    mws.addHandler("/api/rtttl", HTTP_POST, []()
                   { mws.webserver->send(200,F("text/plain"),F("OK")); PeripheryManager.playRTTTLString(mws.webserver->arg("plain").c_str()); });
    mws.addHandler("/api/sound", HTTP_POST, []()
                   { if (PeripheryManager.parseSound(mws.webserver->arg("plain").c_str())){
                    mws.webserver->send(200,F("text/plain"),F("OK")); 
                   }else{
                    mws.webserver->send(404,F("text/plain"),F("FileNotFound"));  
                   }; });

    mws.addHandler("/api/moodlight", HTTP_POST, []()
                   {
                    if (DisplayManager.moodlight(mws.webserver->arg("plain").c_str()))
                    {
                        mws.webserver->send(200, F(F("text/plain")), F("OK"));
                    }
                    else
                    {
                        mws.webserver->send(500, F("text/plain"), F("ErrorParsingJson"));
                    } });
    mws.addHandler("/api/notify", HTTP_POST, []()
                   {
                       if (DisplayManager.generateNotification(1,mws.webserver->arg("plain").c_str()))
                       {
                        mws.webserver->send(200, F("text/plain"), F("OK"));
                       }else{
                        mws.webserver->send(500, F("text/plain"), F("ErrorParsingJson"));
                       } });
    mws.addHandler("/api/nextapp", HTTP_ANY, []()
                   { DisplayManager.nextApp(); mws.webserver->send(200,F("text/plain"),F("OK")); });
    mws.addHandler("/fullscreen", HTTP_GET, []()
                   {
    String fps = mws.webserver->arg("fps");
    if (fps == "") {
#ifdef ESP32_C3
        fps = "5";
#else
        fps = "30";
#endif
    }
#ifdef ESP32_C3
    // Keep framebuffer polling within the capacity of the synchronous C3 server.
    fps = String(constrain(fps.toInt(), 1, 10));
#endif
    String finalHTML = screenfull_html; 
    finalHTML.replace("%%FPS%%", fps);

    mws.webserver->sendHeader("Cache-Control", "no-store");
    mws.webserver->send(200, "text/html", finalHTML.c_str()); });
    mws.addHandler("/screen", HTTP_GET, []()
                   { mws.webserver->sendHeader("Cache-Control", "no-store"); mws.webserver->send(200, "text/html", screen_html); });
    mws.addHandler("/backup", HTTP_GET, []()
                   { mws.webserver->send(200, "text/html", backup_html); });
    mws.addHandler("/api/previousapp", HTTP_POST, []()
                   { DisplayManager.previousApp(); mws.webserver->send(200,F("text/plain"),F("OK")); });
    mws.addHandler("/api/notify/dismiss", HTTP_ANY, []()
                   { DisplayManager.dismissNotify(); mws.webserver->send(200,F("text/plain"),F("OK")); });
    mws.addHandler("/api/apps", HTTP_POST, []()
                   { DisplayManager.updateAppVector(mws.webserver->arg("plain").c_str()); mws.webserver->send(200,F("text/plain"),F("OK")); });
    mws.addHandler(
        "/api/switch", HTTP_POST, []()
        {
        if (DisplayManager.switchToApp(mws.webserver->arg("plain").c_str()))
        {
            mws.webserver->send(200, F("text/plain"), F("OK"));
        }
        else
        {
            mws.webserver->send(500, F("text/plain"), F("FAILED"));
        } });
    mws.addHandler("/api/apps", HTTP_GET, []()
                   { String json = DisplayManager.getAppsWithIcon(); sendJsonString(mws.webserver, json); });
    mws.addHandler("/api/settings", HTTP_POST, []()
                   { DisplayManager.setNewSettings(mws.webserver->arg("plain").c_str()); mws.webserver->send(200,F("text/plain"),F("OK")); });
    mws.addHandler("/api/erase", HTTP_ANY, []()
                   { ServerManager.erase();  mws.webserver->send(200,F("text/plain"),F("OK"));delay(200); ESP.restart(); });
    mws.addHandler("/api/resetSettings", HTTP_ANY, []()
                   { formatSettings();   mws.webserver->send(200,F("text/plain"),F("OK"));delay(200); ESP.restart(); });
    mws.addHandler("/api/reorder", HTTP_POST, []()
                   { DisplayManager.reorderApps(mws.webserver->arg("plain").c_str()); mws.webserver->send(200,F("text/plain"),F("OK")); });
    mws.addHandler("/api/settings", HTTP_GET, []()
                   { String json = DisplayManager.getSettings(); sendJsonString(mws.webserver, json); });
    mws.addHandler("/api/custom", HTTP_POST, []()
                   {
                    DisplayManager.logC3Heap("http_custom_entry");
                    DisplayManager.logC3CustomAdmission("http_handler_entry");
                    String name = mws.webserver->arg("name");
                    String payload = mws.webserver->arg("plain");
                    DisplayManager.logC3Heap("http_custom_copied");
                    DisplayManager.logC3CustomAdmission("http_body_copied");
                    if (DisplayManager.parseCustomPage(name, payload.c_str(), false)){
                        DisplayManager.logC3Heap("http_custom_parsed");
                        DisplayManager.logC3CustomAdmission("http_parse_complete");
                        DisplayManager.logC3CustomAdmission("http_response_send");
                        mws.webserver->send(200,F("text/plain"),F("OK"));
                        DisplayManager.logC3CustomAdmission("http_response_complete");
#ifdef ESP32_C3
                        Serial.printf("[%lu] [C3GIF] http custom complete name=%s remove=%u\n",
                                      millis(), name.c_str(), payload == "{}" || payload.length() == 0);
#endif
                    }else{
                        DisplayManager.logC3CustomAdmission("http_parse_rejected");
                        mws.webserver->send(500,F("text/plain"),F("ErrorParsingJson"));
                    }
                    DisplayManager.logC3Heap("http_custom_complete"); });
    mws.addHandler("/api/stats", HTTP_GET, []()
                   {
                    char statsBuffer[512];
                    DisplayManager.logC3Heap("http_stats_entry");
                    const size_t statsLength = DisplayManager.getStats(statsBuffer, sizeof(statsBuffer));
                    // Let WebServer own the response lifecycle. Writing the
                    // socket directly bypasses its cleanup bookkeeping and
                    // leaves lwIP pbufs fragmented after rapid C3 polling.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
                    mws.webserver->send(200, "application/json", String(statsBuffer, statsLength));
#else
                    mws.webserver->send(200, "application/json", statsBuffer, statsLength);
#endif
                    DisplayManager.logC3Heap("http_stats_complete");
                   });
    mws.addHandler("/api/screen", HTTP_GET, []()
                   {
                    static char screenBuffer[32 * 8 * 9 + 2];
                    const size_t length = DisplayManager.ledsAsJson(screenBuffer, sizeof(screenBuffer));
                    if (length == 0)
                    {
                     mws.webserver->send(500, F("text/plain"), F("ScreenSerializationFailed"));
                     return;
                    }
                    mws.webserver->send(200, F("application/json"), screenBuffer);
                   });
    mws.addHandler("/api/indicator1", HTTP_POST, []()
                   { 
                    if (DisplayManager.indicatorParser(1,mws.webserver->arg("plain").c_str())){
                     mws.webserver->send(200,F("text/plain"),F("OK")); 
                    }else{
                         mws.webserver->send(500,F("text/plain"),F("ErrorParsingJson")); 
                    } });
    mws.addHandler("/api/indicator2", HTTP_POST, []()
                   { 
                    if (DisplayManager.indicatorParser(2,mws.webserver->arg("plain").c_str())){
                     mws.webserver->send(200,F("text/plain"),F("OK")); 
                    }else{
                         mws.webserver->send(500,F("text/plain"),F("ErrorParsingJson")); 
                    } });
    mws.addHandler("/api/indicator3", HTTP_POST, []()
                   { 
                    if (DisplayManager.indicatorParser(3,mws.webserver->arg("plain").c_str())){
                     mws.webserver->send(200,F("text/plain"),F("OK")); 
                    }else{
                         mws.webserver->send(500,F("text/plain"),F("ErrorParsingJson")); 
                    } });
    mws.addHandler("/api/doupdate", HTTP_POST, []()
                   { 
                    if (UpdateManager.checkUpdate(true)){
                        mws.webserver->send(200,F("text/plain"),F("OK"));
                        UpdateManager.updateFirmware();
                    }else{
                        mws.webserver->send(404,F("text/plain"),"NoUpdateFound");    
                    } });
    mws.addHandler("/api/r2d2", HTTP_POST, []()
                   { PeripheryManager.r2d2(mws.webserver->arg("plain").c_str()); mws.webserver->send(200,F("text/plain"),F("OK")); });
}

void ServerManager_::setup()
{
#ifdef ESP32_C3
    WiFi.onEvent(logC3WiFiEvent);
#endif
    esp_wifi_set_max_tx_power(80); // 82 * 0.25 dBm = 20.5 dBm
    esp_wifi_set_ps(WIFI_PS_NONE); // Power Saving deaktivieren
    if (!local_IP.fromString(NET_IP) || !gateway.fromString(NET_GW) || !subnet.fromString(NET_SN) || !primaryDNS.fromString(NET_PDNS) || !secondaryDNS.fromString(NET_SDNS))
        NET_STATIC = false;
    if (NET_STATIC)
    {
        WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
    }
    WiFi.setHostname(HOSTNAME.c_str()); // define hostname
    myIP = mws.startWiFi(AP_TIMEOUT * 1000, HOSTNAME.c_str(), "12345678");
    isConnected = !(myIP == IPAddress(192, 168, 4, 1));
    if (DEBUG_MODE)
        DEBUG_PRINTF("My IP: %d.%d.%d.%d", myIP[0], myIP[1], myIP[2], myIP[3]);
    mws.setAuth(AUTH_USER, AUTH_PASS);
    if (isConnected)
    {
        mws.addOptionBox("Network");
        mws.addOption("Static IP", NET_STATIC);
        mws.addOption("Local IP", NET_IP);
        mws.addOption("Gateway", NET_GW);
        mws.addOption("Subnet", NET_SN);
        mws.addOption("Primary DNS", NET_PDNS);
        mws.addOption("Secondary DNS", NET_SDNS);
        mws.addOptionBox("MQTT");
        mws.addOption("Broker", MQTT_HOST);
        mws.addOption("Port", MQTT_PORT);
        mws.addOption("Username", MQTT_USER);
        mws.addOption("Password", MQTT_PASS);
        mws.addOption("Prefix", MQTT_PREFIX);
        mws.addOption("Homeassistant Discovery", HA_DISCOVERY);
        mws.addOptionBox("Time");
        mws.addOption("NTP Server", NTP_SERVER);
        mws.addOption("Timezone", NTP_TZ);
        mws.addHTML("<p>Find your timezone at <a href='https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv' target='_blank' rel='noopener noreferrer'>posix_tz_db</a>.</p>", "tz_link");
        mws.addOptionBox("Icons");
        mws.addHTML(custom_html, "icon_html");
        mws.addCSS(custom_css);
        mws.addJavascript(custom_script);
        mws.addOptionBox("Auth");
        mws.addOption("Auth Username", AUTH_USER);
        mws.addOption("Auth Password", AUTH_PASS);
        mws.addHandler("/save", HTTP_POST, saveHandler);
        addHandler();
        setupWebOtaHandler();
        udp.begin(localUdpPort);
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Webserver loaded"));
    }
    mws.addHandler("/version", HTTP_GET, versionHandler);
    mws.begin(WEB_PORT);

    if (!MDNS.begin(HOSTNAME))
    {
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Error starting mDNS"));
    }
    else
    {
        MDNS.addService("http", "tcp", 80);
        MDNS.addService("awtrix", "tcp", 80);
        MDNS.addServiceTxt("awtrix", "tcp", "id", uniqueID);
        MDNS.addServiceTxt("awtrix", "tcp", "name", HOSTNAME.c_str());
        MDNS.addServiceTxt("awtrix", "tcp", "type", "awtrix3");
    }

    configTzTime(NTP_TZ.c_str(), NTP_SERVER.c_str());
    tm timeInfo;
    getLocalTime(&timeInfo);
    TCPserver.begin();
    TCPserver.setNoDelay(true);
}

void ServerManager_::tick()
{
    mws.run();

    if (!AP_MODE)
    {
        int packetSize = udp.parsePacket();
        if (packetSize)
        {
            int len = udp.read(incomingPacket, 255);
            if (len > 0)
            {
                incomingPacket[len] = 0;
            }
            if (strcmp(incomingPacket, "FIND_AWTRIX") == 0)
            {
                udp.beginPacket(udp.remoteIP(), 4211);
                if (WEB_PORT != 80)
                {
                    char buffer[128];
                    sprintf(buffer, "%s:%d", HOSTNAME.c_str(), WEB_PORT);
                    udp.printf(buffer);
                }
                else
                {
                    udp.printf(HOSTNAME.c_str());
                }

                udp.endPacket();
            }
        }
    }

    if (!currentClient || !currentClient.connected()) {
        if (TCPserver.hasClient()) {
            if (currentClient) {
                currentClient.stop();
                Serial.println("Vorheriger Client getrennt, um neuen Client zu akzeptieren.");
            }
            currentClient = TCPserver.available();
            Serial.println("Neuer Client verbunden.");
        }
    }

    if (currentClient && currentClient.connected()) {
        while (currentClient.available()) {
            char incomingByte = currentClient.read();            
            if (incomingByte == '\n') {
                dataBuffer[bufferIndex] = '\0';               
                GameManager.ControllerInput(dataBuffer);
                bufferIndex = 0;
            }
            else if (incomingByte != '\r') {
                if (bufferIndex < BUFFER_SIZE - 1) {
                    dataBuffer[bufferIndex++] = incomingByte;
                }
                else {
                    bufferIndex = 0;
                }
            }
        }
    }
}

void ServerManager_::sendTCP(String message)
{
    if (currentClient && currentClient.connected()) {
        currentClient.print(message);
    }
}

void ServerManager_::loadSettings()
{
    if (LittleFS.exists("/DoNotTouch.json"))
    {
        File file = LittleFS.open("/DoNotTouch.json", "r");
        DynamicJsonDocument doc(file.size() * 1.33);
        DeserializationError error = deserializeJson(doc, file);
        if (error)
            return;

        NTP_SERVER = doc["NTP Server"].as<String>();
        NTP_TZ = doc["Timezone"].as<String>();
        MQTT_HOST = doc["Broker"].as<String>();
        MQTT_PORT = doc["Port"].as<uint16_t>();
        MQTT_USER = doc["Username"].as<String>();
        MQTT_PASS = doc["Password"].as<String>();
        MQTT_PREFIX = doc["Prefix"].as<String>();
        MQTT_PREFIX.trim();
        NET_STATIC = doc["Static IP"];
        HA_DISCOVERY = doc["Homeassistant Discovery"];
        NET_IP = doc["Local IP"].as<String>();
        NET_GW = doc["Gateway"].as<String>();
        NET_SN = doc["Subnet"].as<String>();
        NET_PDNS = doc["Primary DNS"].as<String>();
        NET_SDNS = doc["Secondary DNS"].as<String>();
        if (doc["Auth Username"].is<String>())
            AUTH_USER = doc["Auth Username"].as<String>();
        if (doc["Auth Password"].is<String>())
            AUTH_PASS = doc["Auth Password"].as<String>();

        file.close();
        DisplayManager.applyAllSettings();
        if (DEBUG_MODE)
            DEBUG_PRINTLN(F("Webserver configuration loaded"));
        doc.clear();
        return;
    }
    else if (DEBUG_MODE)
        DEBUG_PRINTLN(F("Webserver configuration file not exist"));
    return;
}

void ServerManager_::sendButton(byte btn, bool state)
{
    if (BUTTON_CALLBACK == "")
        return;
    static bool btn0State, btn1State, btn2State;
    String payload;
    switch (btn)
    {
    case 0:
        if (btn0State != state)
        {
            btn0State = state;
            payload = "button=left&state=" + String(state) + "&uid=" + uniqueID;
        }
        break;
    case 1:
        if (btn1State != state)
        {
            btn1State = state;
            payload = "button=middle&state=" + String(state) + "&uid=" + uniqueID;
        }
        break;
    case 2:
        if (btn2State != state)
        {
            btn2State = state;
            payload = "button=right&state=" + String(state) + "&uid=" + uniqueID;
        }
        break;
    default:
        return;
    }
    if (!payload.isEmpty())
    {
        HTTPClient http;
        http.begin(BUTTON_CALLBACK);
        http.addHeader("Content-Type", "application/x-www-form-urlencoded");
        http.POST(payload);
        http.end();
    }
}
