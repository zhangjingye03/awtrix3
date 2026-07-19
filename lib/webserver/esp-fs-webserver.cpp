#include "esp-fs-webserver.h"

static void sendGzipPage(WebServerClass *server, PGM_P content, size_t length)
{
    // A single 12+ KB write exhausts the C3 TCP send buffer. Chunking also
    // yields to the Wi-Fi task while a browser loads the embedded pages.
    // The WebServer chunked-response footer is written directly to WiFiClient
    // and bypasses the C3 retry wrapper. Embedded pages have a known size, so
    // use Content-Length and keep every body write on the safe path.
    server->setContentLength(length);
    server->sendHeader("Content-Encoding", "gzip");
    server->send(200, "text/html", "");

    constexpr size_t chunkSize = 512;
    for (size_t offset = 0; offset < length; offset += chunkSize)
    {
        const size_t remaining = length - offset;
        server->sendContent_P(content + offset, min(chunkSize, remaining));
        delay(0);
    }
}

FSWebServer::FSWebServer(fs::FS &fs, WebServerClass &server)
{
    m_filesystem = &fs;
    webserver = &server;
    m_basePath[0] = '\0';
}

WebServerClass *FSWebServer::getRequest()
{
    return webserver;
}

void FSWebServer::run()
{
    webserver->handleClient();
    if (m_apmode)
        m_dnsServer.processNextRequest();

    unsigned long currentMillis = millis();
    if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval) && !m_apmode)
    {
        Serial.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();
        previousMillis = currentMillis;
    }
}

void FSWebServer::addHandler(const Uri &uri, HTTPMethod method, WebServerClass::THandlerFunction fn)
{
    webserver->on(uri, method, authMiddleware(fn));
}

void FSWebServer::onNotFound(WebServerClass::THandlerFunction fn)
{
    webserver->onNotFound(fn);
}

void FSWebServer::addHandler(const Uri &uri, WebServerClass::THandlerFunction handler)
{

    addHandler(uri, HTTP_ANY, handler);
}

// List all files saved in the selected filesystem
bool FSWebServer::checkDir(char *dirname, uint8_t levels)
{
    if (dirname[0] != '/')
        dirname[0] = '/';
    File root = m_filesystem->open(dirname, "r");
    if (!root)
    {
        DebugPrintln("- failed to open directory\n");
        return false;
    }
    if (!root.isDirectory())
    {
        DebugPrintln(" - not a directory\n");
        return false;
    }
    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            char dir[16];
            strcpy(dir, "/");
            strcat(dir, file.name());
            DebugPrintf("DIR : %s\n", dir);
            checkDir(dir, levels - 1);
        }
        else
        {
            DebugPrintf("  FILE: %s\tSIZE: %d\n", file.name(), file.size());
        }
        file = root.openNextFile();
    }
    return true;
}

bool FSWebServer::begin( int port,const char *path)
{
    DebugPrintln("\nList the files of webserver: ");
    if (path != nullptr)
        strcpy(m_basePath, path);

    m_fsOK = checkDir(m_basePath, 2);

#ifdef INCLUDE_EDIT_HTM
    addHandler("/status", HTTP_GET, std::bind(&FSWebServer::handleStatus, this));
    addHandler("/list", HTTP_GET, std::bind(&FSWebServer::handleFileList, this));
    addHandler("/edit", HTTP_GET, std::bind(&FSWebServer::handleGetEdit, this));
    addHandler("/edit", HTTP_PUT, std::bind(&FSWebServer::handleFileCreate, this));
    addHandler("/edit", HTTP_DELETE, std::bind(&FSWebServer::handleFileDelete, this));
#endif
    webserver->onNotFound(authMiddleware(std::bind(&FSWebServer::handleRequest, this)));
    addHandler("/favicon.ico", HTTP_GET, std::bind(&FSWebServer::replyOK, this));
    addHandler("/", HTTP_GET, std::bind(&FSWebServer::handleIndex, this));
#ifdef INCLUDE_SETUP_HTM
    addHandler("/setup", HTTP_GET, std::bind(&FSWebServer::handleSetup, this));
#endif
    addHandler("/scan", HTTP_GET, std::bind(&FSWebServer::handleScanNetworks, this));
    addHandler("/connect", HTTP_POST, std::bind(&FSWebServer::doWifiConnection, this));
    addHandler("/restart", HTTP_GET, std::bind(&FSWebServer::doRestart, this));
    addHandler("/ipaddress", HTTP_GET, std::bind(&FSWebServer::getIpAddress, this));

    // Captive Portal redirect
    webserver->on("/redirect", HTTP_GET, std::bind(&FSWebServer::captivePortal, this));
    // Windows
    webserver->on("/connecttest.txt", HTTP_GET, std::bind(&FSWebServer::captivePortal, this));
    // Apple
    webserver->on("/hotspot-detect.html", HTTP_GET, std::bind(&FSWebServer::captivePortal, this));
    // Android
    webserver->on("/generate_204", HTTP_GET, std::bind(&FSWebServer::captivePortal, this));
    webserver->on("/gen_204", HTTP_GET, std::bind(&FSWebServer::captivePortal, this));

    // Upload file
    // - first callback is called after the request has ended with all parsed arguments
    // - second callback handles file upload at that location
    webserver->on("/edit", HTTP_POST, std::bind(&FSWebServer::replyOK, this), authMiddleware(std::bind(&FSWebServer::handleFileUpload, this)));

    // OTA update via webbrowser. On ESP32-C3 this is also our firewall-friendly
    // OTA path because ArduinoOTA/espota needs inbound TCP to the PC.
#ifdef ESP32_C3
    m_httpUpdater.setup(webserver);
#else
    m_httpUpdater.setup(webserver, authUser, authPass);
#endif

    webserver->enableCORS(true);

    // Each route supplies its own response size. A global 1024-byte length
    // makes short endpoints such as /api/stats advertise bytes they never
    // send, causing HTTP clients to wait until their socket timeout.
    webserver->setContentLength(CONTENT_LENGTH_NOT_SET);
    webserver->begin(port);

    return true;
}

WebServerClass::THandlerFunction FSWebServer::authMiddleware(WebServerClass::THandlerFunction fn)
{
    if (authUser.isEmpty() || m_apmode)
    {
        return fn;
    }

    return [this, fn]()
    {
        if (!webserver->authenticate(authUser.c_str(), authPass.c_str()))
        {
            return webserver->requestAuthentication();
        }

        fn();
    };
}

void FSWebServer::setCaptiveWebage(const char *url)
{
    m_apWebpage = (char *)realloc(m_apWebpage, sizeof(url));
    strcpy(m_apWebpage, url);
}

IPAddress FSWebServer::setAPmode(const char *ssid, const char *psk)
{
    m_apmode = true;
    WiFi.mode(WIFI_AP_STA);
    WiFi.persistent(false);
    WiFi.softAP(ssid, psk);
    /* Setup the DNS server redirecting all the domains to the apIP */
    m_dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    m_dnsServer.start(53, "*", WiFi.softAPIP());
    return WiFi.softAPIP();
}

IPAddress FSWebServer::startWiFi(uint32_t timeout, const char *apSSID, const char *apPsw)
{
    IPAddress ip;
    m_timeout = timeout;
    WiFi.mode(WIFI_STA);

    const char *_ssid;
    const char *_pass;

    wifi_config_t conf;
    esp_wifi_get_config(WIFI_IF_STA, &conf);
  
    _ssid = reinterpret_cast<const char *>(conf.sta.ssid);
    _pass = reinterpret_cast<const char *>(conf.sta.password);

    char *my_ssid = new char[33];
    strncpy(my_ssid, _ssid, 32);
    my_ssid[32] = '\0';
    _ssid = my_ssid;

    if (strlen(_ssid) && strlen(_pass))
    {
        WiFi.begin(_ssid, _pass, 0, 0, true);
        Serial.print(F("Connecting to "));
        Serial.println(_ssid);

        uint32_t startTime = millis();
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(300);
            Serial.print(".");
            if (WiFi.status() == WL_CONNECTED)
            {
                ip = WiFi.localIP();
                WiFi.persistent(true);
                delete[] my_ssid;
                return ip;
            }
            // If no connection after a while go in Access Point mode
            if (millis() - startTime > m_timeout)
            {
                Serial.println(F("No connection after a while -> go in Access Point mode"));
                break;
            }
        }
    }

    if (apSSID != nullptr && apPsw != nullptr)
        setAPmode(apSSID, apPsw);
    else
        setAPmode("ESP_AP", "123456789");

    WiFi.begin();
    ip = WiFi.softAPIP();
    Serial.print(F("\nAP mode.\nServer IP address: "));
    Serial.println(ip);
    Serial.println();
    delete[] my_ssid;
    return ip;
}

////////////////////////////////  WiFi  /////////////////////////////////////////

/**
 * Redirect to captive portal if we got a request for another domain.
 */
bool FSWebServer::captivePortal()
{
    IPAddress ip = webserver->client().localIP();
    char serverLoc[sizeof("https:://255.255.255.255/") + sizeof(m_apWebpage) + 1];
    snprintf(serverLoc, sizeof(serverLoc), "http://%d.%d.%d.%d%s", ip[0], ip[1], ip[2], ip[3], m_apWebpage);

    // redirect if hostheader not server ip, prevent redirect loops
    if (strcmp(serverLoc, webserver->hostHeader().c_str()))
    {
        webserver->sendHeader(F("Location"), serverLoc, true);
        webserver->send(302, F("text/html"), ""); // Empty content inhibits Content-length header so we have to close the socket ourselves.
        webserver->client().stop();               // Stop is needed because we sent no content length
        return true;
    }
    return false;
}

void FSWebServer::handleRequest()
{
    if (!m_fsOK)
    {
        replyToCLient(ERROR, PSTR(FS_INIT_ERROR));
        return;
    }
#if defined(ESP32)
    String _url = WebServer::urlDecode(webserver->uri());
#elif defined(ESP8266)
    String _url = ESP8266WebServer::urlDecode(webserver->uri());
#endif
    // First try to find and return the requested file from the filesystem,
    // and if it fails, return a 404 page with debug information
    // Serial.print("urlDecode: ");
    // Serial.println(_url);
    if (handleFileRead(_url))
        return;
    else
        replyToCLient(NOT_FOUND, PSTR(FILE_NOT_FOUND));
}

void FSWebServer::getIpAddress()
{
    webserver->send(200, "text/json", WiFi.localIP().toString());
}

void FSWebServer::doRestart()
{
    Serial.println("RESTART");
    webserver->send(200, "text/json", "Going to restart ESP");
    delay(500);
    ESP.restart();
}

void FSWebServer::doWifiConnection()
{
    String ssid, pass;
    bool persistent = true;
    WiFi.mode(WIFI_AP_STA);

    if (webserver->hasArg("ssid"))
    {
        ssid = webserver->arg("ssid");
    }

    if (webserver->hasArg("password"))
    {
        pass = webserver->arg("password");
    }

    if (webserver->hasArg("persistent"))
    {
        String pers = webserver->arg("persistent");
        if (pers.equals("false"))
        {
            persistent = false;
        }
    }

    if (WiFi.status() == WL_CONNECTED)
    {

        IPAddress ip = WiFi.localIP();
        String resp = "ESP is currently connected to a WiFi network.<br><br>"
                      "Actual connection will be closed and a new attempt will be done with <b>";
        resp += ssid;
        resp += "</b> WiFi network.";
        webserver->send(200, "text/plain", resp);

        delay(500);
        Serial.println("Disconnect from current WiFi network");
        WiFi.disconnect();
    }

    if (ssid.length() && pass.length())
    {
        // Try to connect to new ssid
        Serial.print("\nConnecting to ");
        Serial.println(ssid);
        WiFi.begin(ssid.c_str(), pass.c_str(), 0, 0, true);

        uint32_t beginTime = millis();
        while (WiFi.status() != WL_CONNECTED)
        {
            delay(300);
            Serial.print("*.*");
            if (millis() - beginTime > m_timeout)
                break;
        }
        // reply to client
        if (WiFi.status() == WL_CONNECTED)
        {
            // WiFi.softAPdisconnect();
            IPAddress ip = WiFi.localIP();
            Serial.print("\nConnected to Wifi! IP address: ");
            Serial.println(ip);
            webserver->send(200, "text/plain", ip.toString());
            m_apmode = false;
            delay(500);
            ESP.restart();
            // Store current WiFi configuration in flash
            if (persistent)
            {
#if defined(ESP8266)
                struct station_config stationConf;
                wifi_station_get_config_default(&stationConf);
                // Clear previuos configuration
                memset(&stationConf, 0, sizeof(stationConf));
                os_memcpy(&stationConf.ssid, ssid.c_str(), ssid.length());
                os_memcpy(&stationConf.password, pass.c_str(), pass.length());
                wifi_set_opmode(STATION_MODE);
                wifi_station_set_config(&stationConf);
#elif defined(ESP32)
                wifi_config_t stationConf;
                esp_wifi_get_config(WIFI_IF_STA, &stationConf);
                // Clear previuos configuration
                memset(&stationConf, 0, sizeof(stationConf));
                memcpy(&stationConf.sta.ssid, ssid.c_str(), ssid.length());
                memcpy(&stationConf.sta.password, pass.c_str(), pass.length());
                esp_wifi_set_config(WIFI_IF_STA, &stationConf);
#endif
            }
            else
            {
#if defined(ESP8266)
                struct station_config stationConf;
                wifi_station_get_config_default(&stationConf);
                // Clear previuos configuration
                memset(&stationConf, 0, sizeof(stationConf));
                wifi_station_set_config(&stationConf);
#elif defined(ESP32)
                wifi_config_t stationConf;
                esp_wifi_get_config(WIFI_IF_STA, &stationConf);
                // Clear previuos configuration
                memset(&stationConf, 0, sizeof(stationConf));
                esp_wifi_set_config(WIFI_IF_STA, &stationConf);
#endif
            }
        }
        else
            webserver->send(500, "text/plain", "Connection error, maybe the password is wrong?");
    }
    webserver->send(500, "text/plain", "Wrong credentials provided");
}

void FSWebServer::setCrossOrigin()
{
    webserver->sendHeader(F("Access-Control-Allow-Origin"), F("*"));
    webserver->sendHeader(F("Access-Control-Max-Age"), F("600"));
    webserver->sendHeader(F("Access-Control-Allow-Methods"), F("PUT,POST,GET,OPTIONS"));
    webserver->sendHeader(F("Access-Control-Allow-Headers"), F("*"));
};

void FSWebServer::handleScanNetworks()
{
    String jsonList = "[";
    DebugPrint("Scanning WiFi networks...");
#ifdef ESP32_C3
    // The default active scan dwells on every channel long enough to lose AP
    // beacons on the C3. Use a short passive scan while already associated.
    int n = WiFi.scanNetworks(false, true, true, 40);
#else
    int n = WiFi.scanNetworks();
#endif
    DebugPrintln(" complete.");
    if (n == 0)
    {
        DebugPrintln("no networks found");
        webserver->send(200, "text/json", "[]");
        WiFi.scanDelete();
        return;
    }
    else
    {
        DebugPrint(n);
        DebugPrintln(" networks found:");

        for (int i = 0; i < n; ++i)
        {
            String ssid = WiFi.SSID(i);
            int rssi = WiFi.RSSI(i);
#if defined(ESP8266)
            String security = WiFi.encryptionType(i) == AUTH_OPEN ? "none" : "enabled";
#elif defined(ESP32)
            String security = WiFi.encryptionType(i) == WIFI_AUTH_OPEN ? "none" : "enabled";
#endif
            jsonList += "{\"ssid\":\"";
            jsonList += ssid;
            jsonList += "\",\"strength\":\"";
            jsonList += rssi;
            jsonList += "\",\"security\":";
            jsonList += security == "none" ? "false" : "true";
            jsonList += ssid.equals(WiFi.SSID()) ? ",\"selected\": true" : "";
            jsonList += i < n - 1 ? "}," : "}";
        }
        jsonList += "]";
    }
    webserver->send(200, "text/json", jsonList);
    WiFi.scanDelete();
    DebugPrintln(jsonList);
}

#ifdef INCLUDE_SETUP_HTM

void FSWebServer::addDropdownList(const char *label, const char **array, size_t size)
{
    File file = m_filesystem->open("/DoNotTouch.json", "r");
    int sz = file.size() * 1.33;
    int docSize = max(sz, 2048);
    DynamicJsonDocument doc((size_t)docSize);
    if (file)
    {
        // If file is present, load actual configuration
        DeserializationError error = deserializeJson(doc, file);
        if (error)
        {
            DebugPrintln(F("Failed to deserialize file, may be corrupted"));
            DebugPrintln(error.c_str());
            file.close();
            return;
        }
        file.close();
    }
    else
    {
        DebugPrintln(F("File not found, will be created new configuration file"));
    }

    numOptions++;

    // If key is present in json, we don't need to create it.
    if (doc.containsKey(label))
        return;

    JsonObject obj = doc.createNestedObject(label);
    obj["selected"] = array[0]; // first element selected as default
    JsonArray arr = obj.createNestedArray("values");
    for (int i = 0; i < size; i++)
    {
        arr.add(array[i]);
    }

    file = m_filesystem->open("/DoNotTouch.json", "w");
    if (serializeJsonPretty(doc, file) == 0)
    {
        DebugPrintln(F("Failed to write to file"));
    }
    file.close();
}

void FSWebServer::removeWhiteSpaces(String &str)
{
    const char noChars[] = {'\n', '\r', '\t'};
    int pos = -1;
    // Remove non printable characters
    for (int i = 0; i < sizeof(noChars); i++)
    {
        pos = str.indexOf(noChars[i]);
        while (pos > -1)
        {
            str.replace(String(noChars[i]), "");
            pos = str.indexOf(noChars[i]);
        }
    }
    // Remove doubles spaces
    pos = str.indexOf("  ");
    while (pos > -1)
    {
        str.replace("  ", " ");
        pos = str.indexOf("  ");
    }
}

void FSWebServer::handleSetup()
{
    sendGzipPage(webserver, SETUP_HTML, SETUP_HTML_SIZE);
}
#endif

void FSWebServer::handleIndex()
{
    if (m_filesystem->exists("/index.htm"))
    {
        handleFileRead("/index.htm");
    }
    else if (m_filesystem->exists("/index.html"))
    {
        handleFileRead("/index.html");
    }
#ifdef INCLUDE_SETUP_HTM
    else
    {
        handleSetup();
    }
#endif
}

/*
    Read the given file from the filesystem and stream it back to the client
*/
bool FSWebServer::handleFileRead(const String &uri)
{
    String path = m_basePath;
    path = uri;

    DebugPrintln("handleFileRead: " + path);
    if (path.endsWith("/"))
    {
        path += "index.htm";
    }
#ifdef ESP32_C3
    // LittleFS icons are decoded locally by the matrix and uploaded through
    // /api/c3/icon. Sending GIFs through the synchronous WebServer can hold
    // the only HTTP loop in WiFiClient's ten-second EAGAIN retry path.
    if (path.startsWith("/ICONS/"))
    {
        webserver->send(503, F("text/plain"), F("Icon downloads are unavailable on ESP32-C3"));
        return true;
    }
#endif
    String pathWithGz = path + ".gz";

    if (m_filesystem->exists(pathWithGz) || m_filesystem->exists(path))
    {
        if (m_filesystem->exists(pathWithGz))
        {
            path += ".gz";
        }
        const char *contentType = getContentType(path.c_str());
        File file = m_filesystem->open(path, "r");
        if (webserver->streamFile(file, contentType) != file.size())
        {
            DebugPrintln(PSTR("Sent less data than expected!"));
            // webserver->stop();
        }
        file.close();
        return true;
    }
    return false;
}

/*
    Handle a file upload request
*/
void FSWebServer::handleFileUpload()
{
    if (webserver->uri() != "/edit")
    {
        return;
    }
    HTTPUpload &upload = webserver->upload();
    if (upload.status == UPLOAD_FILE_START)
    {
#ifdef ESP32_C3
        // The synchronous C3 server has one client slot. Avoid buffering a
        // browser upload longer than LittleFS and Wi-Fi can safely service.
        webserver->client().setNoDelay(true);
#endif
        String filename = upload.filename;
        String result;
        // Make sure paths always start with "/"
        if (!filename.startsWith("/"))
        {
            filename = "/" + filename;
        }
        checkForUnsupportedPath(filename, result);
        if (result.length() > 0)
        {
            replyToCLient(ERROR, PSTR("INVALID FILENAME"));
            return;
        }

        DebugPrintf_P(PSTR("handleFileUpload Name: %s\n"), filename.c_str());
        m_uploadPath = filename;
        m_uploadFile = m_filesystem->open(m_uploadPath, "w");
        if (!m_uploadFile)
        {
            m_uploadPath = String();
            replyToCLient(ERROR, PSTR("CREATE FAILED"));
            return;
        }
        DebugPrintf_P(PSTR("Upload: START, filename: %s\n"), filename.c_str());
#ifdef ESP32_C3
        DebugPrintf_P(PSTR("C3 upload start: %s\n"), filename.c_str());
#endif
    }
    else if (upload.status == UPLOAD_FILE_WRITE)
    {
#ifdef ESP32_C3
        constexpr size_t c3MaxUploadSize = 32 * 1024;
        if (upload.totalSize > c3MaxUploadSize)
        {
            if (m_uploadFile)
            {
                m_uploadFile.close();
                m_uploadFile = File();
            }
            if (m_uploadPath.length())
            {
                m_filesystem->remove(m_uploadPath);
                m_uploadPath = String();
            }
            replyToCLient(ERROR, PSTR("UPLOAD TOO LARGE"));
            return;
        }
#endif
        if (m_uploadFile)
        {
            size_t bytesWritten = m_uploadFile.write(upload.buf, upload.currentSize);
            if (bytesWritten != upload.currentSize)
            {
                replyToCLient(ERROR, PSTR("WRITE FAILED"));
                return;
            }
#ifdef ESP32_C3
            // Let the Wi-Fi task drain incoming TCP data between LittleFS writes.
            delay(0);
#endif
        }
        DebugPrintf_P(PSTR("Upload: WRITE, Bytes: %d\n"), upload.currentSize);
    }
    else if (upload.status == UPLOAD_FILE_END)
    {
        if (m_uploadFile)
        {
            m_uploadFile.close();
            m_uploadFile = File();
        }
        DebugPrintf_P(PSTR("Upload: END, Size: %d\n"), upload.totalSize);
#ifdef ESP32_C3
        DebugPrintf_P(PSTR("C3 upload complete: %u bytes\n"), upload.totalSize);
#endif
        m_uploadPath = String();
    }
    else if (upload.status == UPLOAD_FILE_ABORTED)
    {
        // The synchronous framework aborts a multipart parse when the peer
        // disconnects or times out. Do not leave a zero-byte/partial icon
        // behind, otherwise a later GIF lookup treats it as a valid asset.
        if (m_uploadFile)
        {
            m_uploadFile.close();
            m_uploadFile = File();
        }
        if (m_uploadPath.length())
        {
            m_filesystem->remove(m_uploadPath);
#ifdef ESP32_C3
            DebugPrintf_P(PSTR("C3 upload aborted; removed partial file: %s\n"), m_uploadPath.c_str());
#endif
            m_uploadPath = String();
        }
    }
}

void FSWebServer::replyOK()
{
    replyToCLient(OK, "");
}

void FSWebServer::replyToCLient(int msg_type = 0, const char *msg = "")
{
    // webserver->sendHeader("Access-Control-Allow-Origin", "*");
    switch (msg_type)
    {
    case OK:
        webserver->send(200, FPSTR(TEXT_PLAIN), "");
        break;
    case CUSTOM:
        webserver->send(200, FPSTR(TEXT_PLAIN), msg);
        break;
    case NOT_FOUND:
        if (webserver->method() == HTTP_OPTIONS)
        {
            webserver->send(204); // preflight CORS OPTIONS requests should return OK status
        }
        else
        {
            webserver->send(404, FPSTR(TEXT_PLAIN), msg);
        }
        break;
    case BAD_REQUEST:
        webserver->send(400, FPSTR(TEXT_PLAIN), msg);
        break;
    case ERROR:
        webserver->send(500, FPSTR(TEXT_PLAIN), msg);
        break;
    }
}

/*
    Checks filename for character combinations that are not supported by FSBrowser (alhtough valid on SPIFFS).
    Returns an empty String if supported, or detail of error(s) if unsupported
*/
void FSWebServer::checkForUnsupportedPath(String &filename, String &error)
{
    if (!filename.startsWith("/"))
    {
        error += PSTR(" !! NO_LEADING_SLASH !! ");
    }
    if (filename.indexOf("//") != -1)
    {
        error += PSTR(" !! DOUBLE_SLASH !! ");
    }
    if (filename.endsWith("/"))
    {
        error += PSTR(" ! TRAILING_SLASH ! ");
    }
    DebugPrintln(filename);
    DebugPrintln(error);
}

const char *FSWebServer::getContentType(const char *filename)
{
    if (webserver->hasArg("download"))
        return PSTR("application/octet-stream");
    else if (strstr(filename, ".htm"))
        return PSTR("text/html");
    else if (strstr(filename, ".html"))
        return PSTR("text/html");
    else if (strstr(filename, ".css"))
        return PSTR("text/css");
    else if (strstr(filename, ".sass"))
        return PSTR("text/css");
    else if (strstr(filename, ".js"))
        return PSTR("application/javascript");
    else if (strstr(filename, ".png"))
        return PSTR("image/png");
    else if (strstr(filename, ".svg"))
        return PSTR("image/svg+xml");
    else if (strstr(filename, ".gif"))
        return PSTR("image/gif");
    else if (strstr(filename, ".jpg"))
        return PSTR("image/jpeg");
    else if (strstr(filename, ".ico"))
        return PSTR("image/x-icon");
    else if (strstr(filename, ".xml"))
        return PSTR("text/xml");
    else if (strstr(filename, ".pdf"))
        return PSTR("application/x-pdf");
    else if (strstr(filename, ".zip"))
        return PSTR("application/x-zip");
    else if (strstr(filename, ".gz"))
        return PSTR("application/x-gzip");
    return PSTR("text/plain");
}

// edit page, in usefull in some situation, but if you need to provide only a web interface, you can disable
#ifdef INCLUDE_EDIT_HTM

/*
    Return the list of files in the directory specified by the "dir" query string parameter.
    Also demonstrates the use of chuncked responses.
*/
void FSWebServer::handleFileList()
{
    if (!webserver->hasArg("dir"))
    {
        return replyToCLient(BAD_REQUEST, "DIR ARG MISSING");
    }

    String path = webserver->arg("dir");
    DebugPrintln("handleFileList: " + path);
    if (path != "/" && !m_filesystem->exists(path))
    {
        return replyToCLient(BAD_REQUEST, "BAD PATH");
    }

    File root = m_filesystem->open(path, "r");
    path = String();

    // The editor can list hundreds of icons and GIFs. Stream the JSON entries
    // instead of growing one large String and fragmenting the C3 heap.
    webserver->setContentLength(CONTENT_LENGTH_UNKNOWN);
    webserver->send(200, "application/json", "");
    webserver->sendContent("[", 1);
    bool first = true;
    if (root.isDirectory())
    {
        File file = root.openNextFile();
        while (file)
        {
            String filename = file.name();
            if (filename.lastIndexOf("/") > -1)
            {
                filename.remove(0, filename.lastIndexOf("/") + 1);
            }
            if (!first)
            {
                webserver->sendContent(",", 1);
            }
            first = false;

            char entry[192];
            const int length = snprintf(entry, sizeof(entry),
                                        "{\"type\":\"%s\",\"size\":\"%u\",\"name\":\"%s\"}",
                                        file.isDirectory() ? "dir" : "file",
                                        static_cast<unsigned int>(file.size()),
                                        filename.c_str());
            if (length > 0 && static_cast<size_t>(length) < sizeof(entry))
            {
                webserver->sendContent(entry, static_cast<size_t>(length));
            }
            file.close();
            file = root.openNextFile();
        }
    }
    root.close();
    webserver->sendContent("]", 1);
    webserver->sendContent("", 0);
}

/*
    Handle the creation/rename of a new file
    Operation      | req.responseText
    ---------------+--------------------------------------------------------------
    Create file    | parent of created file
    Create folder  | parent of created folder
    Rename file    | parent of source file
    Move file      | parent of source file, or remaining ancestor
    Rename folder  | parent of source folder
    Move folder    | parent of source folder, or remaining ancestor
*/
void FSWebServer::handleFileCreate()
{
    String path = webserver->arg("path");
    if (path.isEmpty())
    {
        replyToCLient(BAD_REQUEST, PSTR("PATH ARG MISSING"));
        return;
    }
    if (path == "/")
    {
        replyToCLient(BAD_REQUEST, PSTR("BAD PATH"));
        return;
    }

    String src = webserver->arg("src");
    if (src.isEmpty())
    {
        // No source specified: creation
        DebugPrintf_P(PSTR("handleFileCreate: %s\n"), path.c_str());
        if (path.endsWith("/"))
        {
            // Create a folder
            path.remove(path.length() - 1);
            if (!m_filesystem->mkdir(path))
            {
                replyToCLient(ERROR, PSTR("MKDIR FAILED"));
                return;
            }
        }
        else
        {
            // Create a file
            File file = m_filesystem->open(path, "w");
            if (file)
            {
                file.write(0);
                file.close();
            }
            else
            {
                replyToCLient(ERROR, PSTR("CREATE FAILED"));
                return;
            }
        }
        replyToCLient(CUSTOM, path.c_str());
    }
    else
    {
        // Source specified: rename
        if (src == "/")
        {
            replyToCLient(BAD_REQUEST, PSTR("BAD SRC"));
            return;
        }
        if (!m_filesystem->exists(src))
        {
            replyToCLient(BAD_REQUEST, PSTR("BSRC FILE NOT FOUND"));
            return;
        }

        DebugPrintf_P(PSTR("handleFileCreate: %s from %s\n"), path.c_str(), src.c_str());
        if (path.endsWith("/"))
        {
            path.remove(path.length() - 1);
        }
        if (src.endsWith("/"))
        {
            src.remove(src.length() - 1);
        }
        if (!m_filesystem->rename(src, path))
        {
            replyToCLient(ERROR, PSTR("RENAME FAILED"));
            return;
        }
        replyOK();
    }
}

/*
    Handle a file deletion request
    Operation      | req.responseText
    ---------------+--------------------------------------------------------------
    Delete file    | parent of deleted file, or remaining ancestor
    Delete folder  | parent of deleted folder, or remaining ancestor
*/
void FSWebServer::handleFileDelete()
{

    String path = webserver->arg(0);
    if (path.isEmpty() || path == "/")
    {
        replyToCLient(BAD_REQUEST, PSTR("BAD PATH"));
        return;
    }

    DebugPrintf_P(PSTR("handleFileDelete: %s\n"), path.c_str());
    if (!m_filesystem->exists(path))
    {
        replyToCLient(NOT_FOUND, PSTR(FILE_NOT_FOUND));
        return;
    }
    // deleteRecursive(path);
    File root = m_filesystem->open(path, "r");
    // If it's a plain file, delete it
    if (!root.isDirectory())
    {
        root.close();
        m_filesystem->remove(path);
        replyOK();
    }
    else
    {
        m_filesystem->rmdir(path);
        replyOK();
    }
}

/*
    This specific handler returns the edit.htm (or a gzipped version) from the /edit folder.
    If the file is not present but the flag INCLUDE_FALLBACK_INDEX_HTM has been set, falls back to the version
    embedded in the program code.
    Otherwise, fails with a 404 page with debug information
*/
void FSWebServer::handleGetEdit()
{
#if defined(INCLUDE_EDIT_HTM)
    sendGzipPage(webserver, edit_htm_gz, sizeof(edit_htm_gz));
#else
    replyToCLient(NOT_FOUND, PSTR("FILE_NOT_FOUND"));
#endif
}

/*
    Return the FS type, status and size info
*/
void FSWebServer::handleStatus()
{
    DebugPrintln(PSTR("handleStatus"));

    size_t totalBytes = 1024;
    size_t usedBytes = 0;

#ifdef ESP8266
    FSInfo fs_info;
    m_filesystem->info(fs_info);
    totalBytes = fs_info.totalBytes;
    usedBytes = fs_info.usedBytes;
#elif defined(ESP32)
    totalBytes = LittleFS.totalBytes();
    usedBytes = LittleFS.usedBytes();
#endif

    String json;
    json.reserve(256); // Increased the size to accommodate the SSID
    json = "{\"type\":\"Filesystem\", \"isOk\":";
    if (m_fsOK)
    {
        uint32_t ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP() : WiFi.softAPIP();
        json += PSTR("\"true\", \"totalBytes\":\"");
        json += totalBytes;
        json += PSTR("\", \"usedBytes\":\"");
        json += usedBytes;
        json += PSTR("\", \"mode\":\"");
        json += WiFi.status() == WL_CONNECTED ? "Station" : "Access Point";
        json += PSTR("\", \"ip\":\"");
        json += ip;
        json += PSTR("\", \"ssid\":\"");
        json += WiFi.SSID();
        json += "\"";
    }
    else
    {
        json += "\"false\"";
    }

    json += "}"; // Closing the JSON object
    webserver->send(200, "application/json", json);
}
#endif // INCLUDE_EDIT_HTM
