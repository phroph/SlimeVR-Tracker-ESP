#ifndef UTILS_FSHELPER_H
#define UTILS_FSHELPER_H

#include <FS.h>
#include <FFat.h>
#include <logging/Logger.h>

#include <functional>

namespace SlimeVR::Utils {

class File {
public:
    File(fs::File file)
        : m_File(file) {}

    ~File() {
        if (m_File) {
            m_File.close();
        }
    }

    const char* name() const { return m_File.name(); }
    size_t size() const { return m_File.size(); }
    bool isDirectory() { return m_File.isDirectory(); }
    bool seek(size_t pos) { return m_File.seek(pos); }
    bool read(uint8_t* buffer, size_t size) { return m_File.read(buffer, size); }
    bool write(const uint8_t* buffer, size_t size) {
        return m_File.write(buffer, size);
    }
    void close() { return m_File.close(); }

private:
    fs::File m_File;
};

bool ensureDirectory(const char* directory);

File openFile(const char* path, const char* mode);

void forEachFile(const char* directory, std::function<void(File file)> callback);

}  // namespace SlimeVR::Utils

#endif
