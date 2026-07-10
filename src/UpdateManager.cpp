#include <UpdateManager.h>
#include "DisplayManager.h"
#include "Globals.h"

UpdateManager_ &UpdateManager_::getInstance()
{
    static UpdateManager_ instance;
    return instance;
}

UpdateManager_ &UpdateManager = UpdateManager.getInstance();

void update_started()
{
}

void update_finished()
{
}

void update_progress(int cur, int total)
{
    DisplayManager.clear();
    int progress = (cur * 100) / total;
    char progressStr[5];
    snprintf(progressStr, 5, "%d%%", progress);
    DisplayManager.resetTextColor();
    DisplayManager.printText(0, 6, progressStr, true, false);
    DisplayManager.drawProgressBar(0, 7, progress, 0x00FF00, 0xFFFFFF);
    DisplayManager.show();
}

void update_error(int)
{
    DisplayManager.clear();
    DisplayManager.resetTextColor();
    DisplayManager.printText(0, 6, "OTA ONLY", true, true);
    DisplayManager.show();
}

void UpdateManager_::updateFirmware()
{
    if (DEBUG_MODE)
        DEBUG_PRINTLN(F("Official firmware download is disabled in this custom build"));
    update_error(0);
}

bool UpdateManager_::checkUpdate(bool withScreen)
{
    UPDATE_AVAILABLE = false;
    if (DEBUG_MODE)
        DEBUG_PRINTLN(F("Official firmware update check disabled; use ESP OTA or web /update with a local binary"));

    if (withScreen)
    {
        DisplayManager.clear();
        DisplayManager.resetTextColor();
        DisplayManager.printText(0, 6, "OTA ONLY", true, true);
        DisplayManager.show();
        delay(1000);
    }

    return false;
}

void checkUpdateNoReturn()
{
    UpdateManager.getInstance().checkUpdate(false);
}

void UpdateManager_::setup()
{
}
