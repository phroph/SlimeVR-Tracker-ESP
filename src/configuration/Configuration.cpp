#include "Configuration.h"

#include <FS.h>
#include <FFat.h>

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

    // Mount FFat, format on failure
    bool status = FFat.begin(false);
    if (!status) {
        this->m_Logger.warn("Could not mount FFat, formatting");

        // formatOnFail = true
        status = FFat.begin(true);
        if (!status) {
            this->m_Logger.error("Could not format FFat, aborting");
            return;
        }
    }

    if (FFat.exists("/config.bin")) {
        m_Logger.trace("Found configuration file");

        auto file = FFat.open("/config.bin", "r");

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
    // Make sure directories are there before writing
    SlimeVR::Utils::ensureDirectory(DIR_CALIBRATIONS);
    SlimeVR::Utils::ensureDirectory(DIR_TOGGLES);

    for (size_t i = 0; i < m_Sensors.size(); i++) {
        SensorConfig config = m_Sensors[i];
        if (config.type == SensorConfigType::NONE) {
            continue;
        }

        char path[32];

        // --- Calibration data ---
        sprintf(path, DIR_CALIBRATIONS "/%zu", i);
        m_Logger.trace("Saving sensor config data for %d", (int)i);

        File file = FFat.open(path, "w");
        if (!file) {
            m_Logger.error("Failed to open %s for writing", path);
        } else {
            file.write((uint8_t*)&config, sizeof(SensorConfig));
            file.close();
        }

        // --- Toggle state ---
        sprintf(path, DIR_TOGGLES "/%zu", i);
        m_Logger.trace("Saving sensor toggle state for %d", (int)i);

        SensorToggleState toggleState{};
        if (i < m_SensorToggles.size()) {
            toggleState = m_SensorToggles[i];
        }

        file = FFat.open(path, "w");
        if (!file) {
            m_Logger.error("Failed to open %s for writing", path);
        } else {
            file.write((uint8_t*)&toggleState, sizeof(SensorToggleState));
            file.close();
        }
    }

    {
        File file = FFat.open("/config.bin", "w");
        if (!file) {
            m_Logger.error("Failed to open /config.bin for writing");
        } else {
            file.write((uint8_t*)&m_Config, sizeof(DeviceConfig));
            file.close();
        }
    }

    m_Logger.debug("Saved configuration");
}

void Configuration::reset() {
    FFat.format();  // wipe FS

    m_Sensors.clear();
    m_SensorToggles.clear();
    m_Config.version = 1;
    save();

    m_Logger.debug("Reset configuration");
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
        char path[32];
        sprintf(path, DIR_CALIBRATIONS "/%s", f.name());

        f.close();

        FFat.remove(path);
    });

    save();
}

void Configuration::loadSensors() {
	// --- Calibration blobs ---
	SlimeVR::Utils::forEachFile(DIR_CALIBRATIONS, [&](SlimeVR::Utils::File f) {
		SensorConfig sensorConfig;
		f.read((uint8_t*)&sensorConfig, sizeof(SensorConfig));

		uint8_t sensorId = strtoul(f.name(), nullptr, 10);
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
		if (f.isDirectory()) {
			return;
		}

		SensorToggleState sensorToggleState{};
		f.read((uint8_t*)&sensorToggleState, sizeof(SensorToggleState));

		uint8_t sensorId = strtoul(f.name(), nullptr, 10);
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

    if (!FFat.exists(path)) {
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

    char path[32];
    sprintf(path, DIR_TEMPERATURE_CALIBRATIONS "/%d", sensorId);

    m_Logger.trace("Saving temperature calibration data for sensorId:%d", sensorId);

    File file = FFat.open(path, "w");
    file.write((uint8_t*)&config, sizeof(GyroTemperatureCalibrationConfig));
    file.close();

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
