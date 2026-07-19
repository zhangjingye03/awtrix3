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
