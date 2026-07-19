Import("env")

# Arduino-ESP32 3.x uses NetworkClient rather than the Arduino 2.x
# WiFiClient implementation patched by the production C3 target.
from pathlib import Path

print("Core 3 C3 probe: skipping the Arduino 2.x WiFiClient patch")

# FastLED 3.10.3's IDF5 RMT backend starts a transfer asynchronously but
# rewrites the same led_strip pixel buffer on the next frame.  A single 256
# LED matrix can therefore show a partially updated WS2812 frame.  Waiting
# for the transfer (about 8 ms) keeps the buffer ownership unambiguous.
fastled_rmt = Path(env.subst("$PROJECT_LIBDEPS_DIR")) / env.subst("$PIOENV") / "FastLED" / "src" / "platforms" / "esp" / "32" / "rmt_5" / "idf5_rmt.cpp"
async_call = "void RmtController5::showPixels() {\n    mLedStrip->drawAsync();\n}"
sync_call = "void RmtController5::showPixels() {\n    mLedStrip->drawSync();\n}"

if fastled_rmt.exists():
    source = fastled_rmt.read_text(encoding="utf-8")
    if async_call in source:
        fastled_rmt.write_text(source.replace(async_call, sync_call), encoding="utf-8")
        print("Core 3 C3 probe: FastLED RMT5 transfers are synchronous")
    elif sync_call in source:
        print("Core 3 C3 probe: FastLED RMT5 synchronous transfer patch already applied")
    else:
        print("Core 3 C3 probe: FastLED RMT5 showPixels implementation not found")
