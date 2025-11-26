#include "./FSHelper.h"

#include <functional>

namespace SlimeVR::Utils {
SlimeVR::Logging::Logger m_Logger("FSHelper");

bool ensureDirectory(const char* directory) {
	if (!LittleFS.exists(directory)) {
		if (!LittleFS.mkdir(directory)) {
			m_Logger.error("Failed to create directory: %s", directory);
			return false;
		}
	}

	auto dir = LittleFS.open(directory, "r");
	auto isDirectory = dir.isDirectory();
	dir.close();

	if (!isDirectory) {
		if (!LittleFS.remove(directory)) {
			m_Logger.error("Failed to remove directory: %s", directory);
			return false;
		}

		if (!LittleFS.mkdir(directory)) {
			m_Logger.error("Failed to create directory: %s", directory);
			return false;
		}
	}

	return true;
}

File openFile(const char* path, const char* mode) {
	return File(LittleFS.open(path, mode));
}

void forEachFile(const char* directory, std::function<void(File file)> callback) {
	if (!ensureDirectory(directory)) {
		return;
	}

#ifdef ESP32
	auto dir = LittleFS.open(directory);
	while (auto f = dir.openNextFile()) {
		if (f.isDirectory()) {
			continue;
		}

		callback(File(f));
	}

	dir.close();
#else
	auto dir = LittleFS.openDir(directory);
	while (dir.next()) {
		auto fd = dir.openFile("r");
		if (!fd.isFile()) {
			continue;
		}

		callback(File(fd));
	}
#endif
}

}  // namespace SlimeVR::Utils
