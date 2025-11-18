#include "./FSHelper.h"

#include <functional>

namespace SlimeVR::Utils {

SlimeVR::Logging::Logger m_Logger("FSHelper");

// This is for WRITE paths (e.g. Configuration::save)
bool ensureDirectory(const char* directory) {
    if (FFat.exists(directory)) {
        auto dir = FFat.open(directory);
        if (!dir) {
            m_Logger.error("Failed to open path: %s", directory);
            return false;
        }
        bool isDirectory = dir.isDirectory();
        dir.close();

        if (isDirectory) {
            return true;
        }

        // Something exists but isn't a directory - remove and recreate
        if (!FFat.remove(directory)) {
            m_Logger.error("Failed to remove non-directory path: %s", directory);
            return false;
        }
    }

    if (!FFat.mkdir(directory)) {
        m_Logger.error("Failed to create directory: %s", directory);
        return false;
    }

    return true;
}

File openFile(const char* path, const char* mode) {
    return File(FFat.open(path, mode));
}

// This is for READ paths (load) – it must be QUIET if nothing exists yet.
void forEachFile(const char* directory, std::function<void(File file)> callback) {
    // If there is no directory yet, just return – no mkdir, no logs.
    if (!FFat.exists(directory)) {
        return;
    }

    auto dir = FFat.open(directory);
    if (!dir || !dir.isDirectory()) {
        dir.close();
        return;
    }

    while (true) {
        fs::File f = dir.openNextFile();
        if (!f) {
            break;
        }

        if (f.isDirectory()) {
            f.close();
            continue;
        }

        // Wrap into our helper File type and hand it off
        callback(File(f));
        // File wrapper will close in destructor / when callback is done
    }
    dir.close();
}

}  // namespace SlimeVR::Utils
