#include "Configuration.h"

#include <FS.h>
#include <LittleFS.h>

#include <cstdint>
#include <cstring>

#include "../FSHelper.h"
#include "consts.h"
#include "sensors/SensorToggles.h"
#include "utils.h"

#define DIR_CALIBRATIONS "/calibrations"
#define DIR_TEMPERATURE_CALIBRATIONS "/tempcalibrations"
#define DIR_TOGGLES_OLD "/toggles"
#define DIR_TOGGLES "/sensortoggles"

namespace SlimeVR::Configuration {

void Configuration::setup() {
    if (m_Loaded) {
        return;
    }

#ifdef ESP32
    // Try partition names for ESP32 - prioritize simplefs as it's in the partition table
    bool status = false;
    const char* partitionNames[] = {"simplefs", "ffat", "spiffs", "littlefs"};
    const char* usedPartition = nullptr;
    
    for (const char* partitionName : partitionNames) {
        m_Logger.debug("Attempting to mount LittleFS on partition: %s", partitionName);
        // On ESP32, begin() signature: begin(bool formatOnFail, const char* basePath, uint8_t maxOpenFiles, const char* partitionLabel)
        status = LittleFS.begin(false, "/littlefs", 5, partitionName);
        if (status) {
            usedPartition = partitionName;
            m_Logger.info("Successfully mounted LittleFS on partition: %s", partitionName);
            break;
        } else {
            m_Logger.debug("Failed to mount LittleFS on partition: %s", partitionName);
        }
    }
#else
    bool status = LittleFS.begin();
    const char* usedPartition = "default";
#endif

    if (!status) {
        this->m_Logger.warn("Could not mount LittleFS, formatting");

#ifdef ESP32
        // Try to format each partition by attempting to mount it first, then formatting
        status = false;
        for (const char* partitionName : partitionNames) {
            m_Logger.debug("Attempting to format and mount LittleFS on partition: %s", partitionName);
            // Select partition for formatting by attempting to begin with it
            LittleFS.begin(false, "/littlefs", 5, partitionName);
            // Format the selected partition
            if (LittleFS.format()) {
                // Format succeeded, try to mount again
                status = LittleFS.begin(false, "/littlefs", 5, partitionName);
                if (status) {
                    usedPartition = partitionName;
                    m_Logger.info("Formatted and mounted LittleFS on partition: %s", partitionName);
                    break;
                }
            }
        }
#else
        status = LittleFS.format();
        if (status) {
            status = LittleFS.begin();
        }
#endif
        if (!status) {
            this->m_Logger.warn("Could not format or mount LittleFS, aborting");
            return;
        }
    }

    if (LittleFS.exists("/config.bin")) {
        m_Logger.trace("Found configuration file");

        auto file = LittleFS.open("/config.bin", "r");
        if (file) {
            file.read((uint8_t*)&m_Config.version, sizeof(int32_t));

            if (m_Config.version < CURRENT_CONFIGURATION_VERSION) {
                m_Logger.debug(
                    "Configuration is outdated: v%d < v%d",
                    m_Config.version,
                    CURRENT_CONFIGURATION_VERSION
                );

                if (!runMigrations(m_Config.version)) {
                    m_Logger.error(
                        "Failed to migrate configuration from v%d to v%d",
                        m_Config.version,
                        CURRENT_CONFIGURATION_VERSION
                    );
                    file.close();
                    return;
                }
            } else {
                m_Logger.info("Found up-to-date configuration v%d", m_Config.version);
            }

            file.seek(0);
            file.read((uint8_t*)&m_Config, sizeof(DeviceConfig));
            file.close();
        } else {
            m_Logger.error("Failed to open /config.bin for reading");
        }
    } else {
        m_Logger.info("No configuration file found, creating new one");
        m_Config.version = CURRENT_CONFIGURATION_VERSION;
        save();
    }

    loadSensors();

    m_Loaded = true;

    m_Logger.info("Loaded configuration");

#ifdef DEBUG_CONFIGURATION
    print();
#endif
}

void Configuration::save() {
    // Ensure directories exist
    if (!SlimeVR::Utils::ensureDirectory(DIR_CALIBRATIONS)) {
        m_Logger.error("Cannot save calibrations - directory creation failed");
        return;
    }
    if (!SlimeVR::Utils::ensureDirectory(DIR_TOGGLES)) {
        m_Logger.error("Cannot save toggles - directory creation failed");
        return;
    }

    for (size_t i = 0; i < m_Sensors.size(); i++) {
        SensorConfig config = m_Sensors[i];
        if (config.type == SensorConfigType::NONE) {
            continue;
        }

        char path[32];

        // --- Calibration data ---
        sprintf(path, DIR_CALIBRATIONS "/%zu", i);
        m_Logger.trace("Saving sensor config data for %d", (int)i);

        File file = LittleFS.open(path, "w");
        if (!file) {
            m_Logger.error("Failed to open %s for writing", path);
            continue;
        }
        
        size_t written = file.write((uint8_t*)&config, sizeof(SensorConfig));
        file.close();
        
        if (written != sizeof(SensorConfig)) {
            m_Logger.error("Failed to write complete config to %s (wrote %zu of %zu bytes)", 
                path, written, sizeof(SensorConfig));
        }

        // --- Toggle state ---
        sprintf(path, DIR_TOGGLES "/%zu", i);
        m_Logger.trace("Saving sensor toggle state for %d", (int)i);

        SensorToggleState toggleState{};
        if (i < m_SensorToggles.size()) {
            toggleState = m_SensorToggles[i];
        }

        file = LittleFS.open(path, "w");
        if (!file) {
            m_Logger.error("Failed to open %s for writing", path);
            continue;
        }
        
        written = file.write((uint8_t*)&toggleState, sizeof(SensorToggleState));
        file.close();
        
        if (written != sizeof(SensorToggleState)) {
            m_Logger.error("Failed to write complete toggle state to %s (wrote %zu of %zu bytes)", 
                path, written, sizeof(SensorToggleState));
        }
    }

    // Save main config file
    File file = LittleFS.open("/config.bin", "w");
    if (!file) {
        m_Logger.error("Failed to open /config.bin for writing");
        return;
    }
    
    size_t written = file.write((uint8_t*)&m_Config, sizeof(DeviceConfig));
    file.close();
    
    if (written != sizeof(DeviceConfig)) {
        m_Logger.error("Failed to write complete config to /config.bin (wrote %zu of %zu bytes)", 
            written, sizeof(DeviceConfig));
        return;
    }

    m_Logger.debug("Saved configuration");
}

void Configuration::reset() {
    LittleFS.format();

    m_Sensors.clear();
    m_SensorToggles.clear();
    m_Config.version = CURRENT_CONFIGURATION_VERSION;
    
    // Save new default configuration
    save();
}

void Configuration::formatFFat() {
    m_Logger.warn("Formatting LittleFS filesystem - ALL DATA WILL BE LOST!");
    
#ifdef ESP32
    // Try to format each partition by attempting to mount it first, then formatting
    bool success = false;
    const char* partitionNames[] = {"simplefs", "ffat", "spiffs", "littlefs"};
    const char* usedPartition = nullptr;
    
    for (const char* partitionName : partitionNames) {
        m_Logger.debug("Attempting to format LittleFS on partition: %s", partitionName);
        // Select partition for formatting
        LittleFS.begin(false, "/littlefs", 5, partitionName);
        // Format the selected partition
        if (LittleFS.format()) {
            // Remount after format
            success = LittleFS.begin(false, "/littlefs", 5, partitionName);
            if (success) {
                usedPartition = partitionName;
                m_Logger.info("Formatted and mounted LittleFS on partition: %s", partitionName);
                break;
            }
        }
    }
#else
    bool success = LittleFS.format();
    if (success) {
        success = LittleFS.begin();
    }
#endif
    if (!success) {
        m_Logger.error("LittleFS format failed!");
        return;
    }
    
    m_Logger.info("LittleFS formatted successfully. All data cleared.");
    m_Logger.info("  - Configuration files deleted");
    m_Logger.info("  - Calibration data deleted");
    m_Logger.info("  - Toggle states deleted");
    m_Logger.info("  - Corrupted files removed");
    m_Logger.info("  - Wear leveling errors should be resolved");
}

int32_t Configuration::getVersion() const { return m_Config.version; }

size_t Configuration::getSensorCount() const { return m_Sensors.size(); }

SensorConfig Configuration::getSensor(size_t sensorID) const {
    if (sensorID >= m_Sensors.size()) {
        return {};
    }

    return m_Sensors.at(sensorID);
}

void Configuration::setSensor(size_t sensorID, const SensorConfig& config) {
    size_t currentSensors = m_Sensors.size();

    if (sensorID >= currentSensors) {
        m_Sensors.resize(sensorID + 1);
    }

    m_Sensors[sensorID] = config;
}

SensorToggleState Configuration::getSensorToggles(size_t sensorId) const {
    if (sensorId >= m_SensorToggles.size()) {
        return {};
    }

    return m_SensorToggles.at(sensorId);
}

void Configuration::setSensorToggles(size_t sensorId, SensorToggleState state) {
    size_t currentSensors = m_SensorToggles.size();

    if (sensorId >= currentSensors) {
        m_SensorToggles.resize(sensorId + 1);
    }

    m_SensorToggles[sensorId] = state;
}

void Configuration::eraseSensors() {
    m_Sensors.clear();

    SlimeVR::Utils::forEachFile(DIR_CALIBRATIONS, [&](SlimeVR::Utils::File f) {
        const char* fullPath = f.name();
        if (!fullPath || strlen(fullPath) == 0) {
            return;
        }

        // Extract sensor ID from flat filename: "/calibrations_0" -> "0"
        const char* name = strrchr(fullPath, '_');
        if (!name) {
            return;  // Invalid format
        }
        name++;  // Skip '_'
        
        char path[64];
        strncpy(path, fullPath, sizeof(path) - 1);
        path[sizeof(path) - 1] = '\0';

        f.close();

        if (!LittleFS.remove(path)) {
            m_Logger.warn("Failed to remove calibration file: %s", path);
        }
    });

    save();
}

void Configuration::loadSensors() {
	// --- Calibration blobs ---
	SlimeVR::Utils::forEachFile(DIR_CALIBRATIONS, [&](SlimeVR::Utils::File f) {
		SensorConfig sensorConfig;
		f.read((uint8_t*)&sensorConfig, sizeof(SensorConfig));

		const char* name = f.name();
		uint8_t sensorId = strtoul(name, nullptr, 10);
		m_Logger.debug(
			"Found sensor calibration for %s at index %d",
			calibrationConfigTypeToString(sensorConfig.type),
			sensorId
		);

		if (sensorConfig.type == SensorConfigType::BNO0XX) {
			SensorToggleState toggles;
			toggles.setToggle(
				SensorToggles::MagEnabled,
				sensorConfig.data.bno0XX.magEnabled
			);
			setSensorToggles(sensorId, toggles);
		}

		setSensor(sensorId, sensorConfig);
	});

	// --- Toggle state blobs ---
	SlimeVR::Utils::forEachFile(DIR_TOGGLES, [&](SlimeVR::Utils::File f) {
		SensorToggleState sensorToggleState{};
		f.read((uint8_t*)&sensorToggleState, sizeof(SensorToggleState));

		const char* name = f.name();
		uint8_t sensorId = strtoul(name, nullptr, 10);
		m_Logger.debug("Found sensor toggle state at index %d", sensorId);

		setSensorToggles(sensorId, sensorToggleState);
	});
}

bool Configuration::loadTemperatureCalibration(
    uint8_t sensorId,
    GyroTemperatureCalibrationConfig& config
) {
    if (!SlimeVR::Utils::ensureDirectory(DIR_TEMPERATURE_CALIBRATIONS)) {
        return false;
    }

    char path[32];
    sprintf(path, DIR_TEMPERATURE_CALIBRATIONS "/%d", sensorId);

    if (!LittleFS.exists(path)) {
        return false;
    }

    auto f = SlimeVR::Utils::openFile(path, "r");
    if (f.isDirectory()) {
        return false;
    }

    if (f.size() != sizeof(GyroTemperatureCalibrationConfig)) {
        m_Logger.debug(
            "Found incompatible sensor temperature calibration (size mismatch) "
            "sensorId:%d, skipping",
            sensorId
        );
        return false;
    }

    SensorConfigType storedConfigType;
    f.read((uint8_t*)&storedConfigType, sizeof(SensorConfigType));

    if (storedConfigType != config.type) {
        m_Logger.debug(
            "Found incompatible sensor temperature calibration (expected %s, "
            "found %s) sensorId:%d, skipping",
            calibrationConfigTypeToString(config.type),
            calibrationConfigTypeToString(storedConfigType),
            sensorId
        );
        return false;
    }

    f.seek(0);
    f.read((uint8_t*)&config, sizeof(GyroTemperatureCalibrationConfig));
    m_Logger.debug(
        "Found sensor temperature calibration for %s sensorId:%d",
        calibrationConfigTypeToString(config.type),
        sensorId
    );
    return true;
}

bool Configuration::saveTemperatureCalibration(
    uint8_t sensorId,
    const GyroTemperatureCalibrationConfig& config
) {
    if (config.type == SensorConfigType::NONE) {
        return false;
    }

    if (!SlimeVR::Utils::ensureDirectory(DIR_TEMPERATURE_CALIBRATIONS)) {
        m_Logger.error("Cannot save temperature calibration - directory creation failed");
        return false;
    }

    char path[32];
    sprintf(path, DIR_TEMPERATURE_CALIBRATIONS "/%d", sensorId);

    m_Logger.trace("Saving temperature calibration data for sensorId:%d", sensorId);

    File file = LittleFS.open(path, "w");
    if (!file) {
        m_Logger.error("Failed to open %s for writing", path);
        return false;
    }
    
    size_t written = file.write((uint8_t*)&config, sizeof(GyroTemperatureCalibrationConfig));
    file.close();
    
    if (written != sizeof(GyroTemperatureCalibrationConfig)) {
        m_Logger.error("Failed to write complete temperature calibration to %s (wrote %zu of %zu bytes)", 
            path, written, sizeof(GyroTemperatureCalibrationConfig));
        return false;
    }

    m_Logger.debug("Saved temperature calibration data for sensorId:%i", sensorId);
    return true;
}

bool Configuration::runMigrations(int32_t version) { return true; }

void Configuration::print() {
	m_Logger.info("Configuration:");
	m_Logger.info("  Version: %d", m_Config.version);
	m_Logger.info("  %d Sensors:", m_Sensors.size());

	for (size_t i = 0; i < m_Sensors.size(); i++) {
		const SensorConfig& c = m_Sensors[i];
		m_Logger.info("    - [%3d] %s", i, calibrationConfigTypeToString(c.type));

		switch (c.type) {
			case SensorConfigType::NONE:
				break;

			case SensorConfigType::BMI160:
				m_Logger.info(
					"            A_B        : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.bmi160.A_B)
				);

				m_Logger.info("            A_Ainv     :");
				for (uint8_t i = 0; i < 3; i++) {
					m_Logger.info(
						"                         %f, %f, %f",
						UNPACK_VECTOR_ARRAY(c.data.bmi160.A_Ainv[i])
					);
				}

				m_Logger.info(
					"            G_off      : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.bmi160.G_off)
				);
				m_Logger.info("            Temperature: %f", c.data.bmi160.temperature);

				break;

			case SensorConfigType::SFUSION:
				m_Logger.info(
					"            A_B        : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.sfusion.A_B)
				);

				m_Logger.info("            A_Ainv     :");
				for (uint8_t i = 0; i < 3; i++) {
					m_Logger.info(
						"                         %f, %f, %f",
						UNPACK_VECTOR_ARRAY(c.data.sfusion.A_Ainv[i])
					);
				}

				m_Logger.info(
					"            G_off      : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.sfusion.G_off)
				);
				m_Logger.info(
					"            Temperature: %f",
					c.data.sfusion.temperature
				);
				break;

			case SensorConfigType::ICM20948:
				m_Logger.info(
					"            G: %d, %d, %d",
					UNPACK_VECTOR_ARRAY(c.data.icm20948.G)
				);
				m_Logger.info(
					"            A: %d, %d, %d",
					UNPACK_VECTOR_ARRAY(c.data.icm20948.A)
				);
				m_Logger.info(
					"            C: %d, %d, %d",
					UNPACK_VECTOR_ARRAY(c.data.icm20948.C)
				);

				break;

			case SensorConfigType::MPU9250:
				m_Logger.info(
					"            A_B   : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.mpu9250.A_B)
				);

				m_Logger.info("            A_Ainv:");
				for (uint8_t i = 0; i < 3; i++) {
					m_Logger.info(
						"                    %f, %f, %f",
						UNPACK_VECTOR_ARRAY(c.data.mpu9250.A_Ainv[i])
					);
				}

				m_Logger.info(
					"            M_B   : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.mpu9250.M_B)
				);

				m_Logger.info("            M_Ainv:");
				for (uint8_t i = 0; i < 3; i++) {
					m_Logger.info(
						"                    %f, %f, %f",
						UNPACK_VECTOR_ARRAY(c.data.mpu9250.M_Ainv[i])
					);
				}

				m_Logger.info(
					"            G_off  : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.mpu9250.G_off)
				);

				break;

			case SensorConfigType::MPU6050:
				m_Logger.info(
					"            A_B  : %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.mpu6050.A_B)
				);
				m_Logger.info(
					"            G_off: %f, %f, %f",
					UNPACK_VECTOR_ARRAY(c.data.mpu6050.G_off)
				);

				break;

			case SensorConfigType::BNO0XX:
				m_Logger.info("            magEnabled: %d", c.data.bno0XX.magEnabled);

				break;
			case SensorConfigType::RUNTIME_CALIBRATION:
				m_Logger.info("            runtimeCalibration: true");

				break;
		}
	}
}
}  // namespace SlimeVR::Configuration
