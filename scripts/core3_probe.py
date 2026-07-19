Import("env")

# Arduino-ESP32 3.x uses NetworkClient rather than the Arduino 2.x
# WiFiClient implementation patched by the production C3 target.
print("Core 3 C3 probe: skipping the Arduino 2.x WiFiClient patch")
