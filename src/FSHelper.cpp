#include "./FSHelper.h"

#include <functional>

namespace SlimeVR::Utils {

SlimeVR::Logging::Logger m_Logger("FSHelper");

bool ensureDirectory(const char* directory) {
    if (!FFat.exists(directory)) {
        if (!FFat.mkdir(directory)) {
            m_Logger.error("Failed to create directory: %s", directory);
            return false;
        }
    }

    auto dir = FFat.open(directory);
    bool isDirectory = dir.isDirectory();
    dir.close();

    if (!isDirectory) {
        if (!FFat.remove(directory)) {
            m_Logger.error("Failed to remove directory: %s", directory);
            return false;
        }

        if (!FFat.mkdir(directory)) {
            m_Logger.error("Failed to create directory: %s", directory);
            return false;
        }
    }

    return true;
}

File openFile(const char* path, const char* mode) {
    return File(FFat.open(path, mode));
}

void forEachFile(const char* directory, std::function<void(File file)> callback) {
    if (!ensureDirectory(directory)) {
        return;
    }

    auto dir = FFat.open(directory);
    while (true) {
        fs::File f = dir.openNextFile();
        if (!f) {
            break;
        }

        if (f.isDirectory()) {
            continue;
        }

        callback(File(f));
    }
    dir.close();
}

}  // namespace SlimeVR::Utils
