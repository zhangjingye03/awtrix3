Import("env")

from pathlib import Path


# Arduino-ESP32 2.0.x lazily heap-allocates a WiFiClient RX buffer per TCP
# client. A named GIF also has a LittleFS buffer, so repeated HTTP requests on
# the C3 can fragment the internal heap. Keep a small C3-only BSS pool instead.
framework_dir = Path(env.PioPlatform().get_package_dir("framework-arduinoespressif32"))
wifi_client = framework_dir / "libraries" / "WiFi" / "src" / "WiFiClient.cpp"
source = wifi_client.read_text(encoding="utf-8")

if "c3WiFiClientRxPool" not in source:
    pool = '''#ifdef CONFIG_IDF_TARGET_ESP32C3
static uint8_t c3WiFiClientRxPool[3][512];
static bool c3WiFiClientRxPoolUsed[3] = {};

static uint8_t *acquireC3WiFiClientRxBuffer(size_t size)
{
    if (size > sizeof(c3WiFiClientRxPool[0])) return nullptr;
    for (size_t i = 0; i < 3; ++i) {
        if (!c3WiFiClientRxPoolUsed[i]) {
            c3WiFiClientRxPoolUsed[i] = true;
            return c3WiFiClientRxPool[i];
        }
    }
    return nullptr;
}

static bool releaseC3WiFiClientRxBuffer(uint8_t *buffer)
{
    for (size_t i = 0; i < 3; ++i) {
        if (buffer == c3WiFiClientRxPool[i]) {
            c3WiFiClientRxPoolUsed[i] = false;
            return true;
        }
    }
    return false;
}
#endif

'''
    source = source.replace("class WiFiClientRxBuffer {", pool + "class WiFiClientRxBuffer {", 1)
    source = source.replace(
        """            if(!_buffer){
                _buffer = (uint8_t *)malloc(_size);""",
        """            if(!_buffer){
#ifdef CONFIG_IDF_TARGET_ESP32C3
                _buffer = acquireC3WiFiClientRxBuffer(_size);
#endif
                if (!_buffer) {
                    _buffer = (uint8_t *)malloc(_size);
                }""",
        1,
    )
    source = source.replace(
        """    ~WiFiClientRxBuffer()
    {
        free(_buffer);""",
        """    ~WiFiClientRxBuffer()
    {
#ifdef CONFIG_IDF_TARGET_ESP32C3
        if (releaseC3WiFiClientRxBuffer(_buffer)) return;
#endif
        free(_buffer);""",
        1,
    )

source = source.replace("WiFiClientRxBuffer(int fd, size_t size=1436)", "WiFiClientRxBuffer(int fd, size_t size=512)", 1)

if "c3WiFiClientRxPool" not in source or "size=512" not in source:
    raise RuntimeError("Unable to apply the ESP32-C3 WiFiClient RX-pool patch")

wifi_client.write_text(source, encoding="utf-8")

web_server = framework_dir / "libraries" / "WebServer" / "src" / "WebServer.cpp"
web_source = web_server.read_text(encoding="utf-8")

if "Explicitly release the C3 socket" not in web_source:
    old_cleanup = '''  if (!keepCurrentClient) {
    _currentClient = WiFiClient();'''
    new_cleanup = '''  if (!keepCurrentClient) {
    // Explicitly release the C3 socket and its lwIP pbufs. Assigning an empty
    // WiFiClient only drops the wrapper, which can leave rapid HTTP polling
    // competing with the small GIF/VFS heap for several seconds.
    _currentClient.stop();
    _currentClient = WiFiClient();'''
    if old_cleanup not in web_source:
        raise RuntimeError("Unable to apply the ESP32-C3 WebServer socket-cleanup patch")
    web_server.write_text(web_source.replace(old_cleanup, new_cleanup, 1), encoding="utf-8")

# Arduino-ESP32's const char* send overload copies each response into a
# temporary String. Stats responses are already serialized into a caller-owned
# buffer, so that copy is avoidable and is costly during repeated C3 polling.
web_header = framework_dir / "libraries" / "WebServer" / "src" / "WebServer.h"
header_source = web_header.read_text(encoding="utf-8")
header_marker = "  void send(int code, const char* content_type, const char* content);"
header_addition = header_marker + "\n  void send(int code, const char* content_type, const char* content, size_t contentLength);"
if "void send(int code, const char* content_type, const char* content, size_t contentLength);" not in header_source:
    if header_marker not in header_source:
        raise RuntimeError("Unable to apply the ESP32-C3 WebServer raw-response header patch")
    web_header.write_text(header_source.replace(header_marker, header_addition, 1), encoding="utf-8")

web_source = web_server.read_text(encoding="utf-8")
source_marker = '''void WebServer::send_P(int code, PGM_P content_type, PGM_P content) {'''
source_addition = '''void WebServer::send(int code, const char* content_type, const char* content, size_t contentLength)
{
    String header;
    _prepareHeader(header, code, content_type, contentLength);
    _currentClientWrite(header.c_str(), header.length());
    if (contentLength) {
        sendContent(content, contentLength);
    }
}

'''
if "void WebServer::send(int code, const char* content_type, const char* content, size_t contentLength)" not in web_source:
    if source_marker not in web_source:
        raise RuntimeError("Unable to apply the ESP32-C3 WebServer raw-response source patch")
    web_server.write_text(web_source.replace(source_marker, source_addition + source_marker, 1), encoding="utf-8")
