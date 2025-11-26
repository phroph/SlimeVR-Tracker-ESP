/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 SlimeVR Contributors

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
*/

#include "mag_calibration.h"

#include <FS.h>
#include <LittleFS.h>

#include "FSHelper.h"
#include "logging/Logger.h"
#include "utils.h"

namespace SlimeVR::MagCalibration {

static Logging::Logger logger("MagCalibration");

constexpr const char* MAG_CAL_DIR = "/magcalibrations";
constexpr const char* MAG_CAL_FILE = "/magcalibrations/global";

bool magCalLoad(MagCalibrationConfig& cfg) {
	// Initialize with defaults
	cfg = MagCalibrationConfig();

	if (!LittleFS.exists(MAG_CAL_FILE)) {
		logger.debug("No mag calibration file found");
		return false;
	}

	File file = LittleFS.open(MAG_CAL_FILE, "r");
	if (!file) {
		logger.error("Failed to open mag calibration file for reading");
		return false;
	}

	size_t read = file.read((uint8_t*)&cfg, sizeof(MagCalibrationConfig));
	file.close();

	if (read != sizeof(MagCalibrationConfig)) {
		logger.error("Failed to read complete mag calibration (read %zu of %zu bytes)", read, sizeof(MagCalibrationConfig));
		return false;
	}

	// Validate version
	if (cfg.version != MAG_CAL_VERSION) {
		logger.warn("Mag calibration version mismatch: expected %u, got %u", MAG_CAL_VERSION, cfg.version);
		cfg.valid = false;
		return false;
	}

	if (cfg.valid) {
		logger.info("Loaded mag calibration: offset=[%.3f, %.3f, %.3f], fieldStrength=%.3f",
			cfg.offset[0], cfg.offset[1], cfg.offset[2], cfg.fieldStrength);
	} else {
		logger.debug("Mag calibration file exists but marked invalid");
	}

	return cfg.valid;
}

bool magCalSave(const MagCalibrationConfig& cfg) {
	// Ensure directory exists
	if (!SlimeVR::Utils::ensureDirectory(MAG_CAL_DIR)) {
		logger.error("Cannot save mag calibration - directory creation failed");
		return false;
	}

	File file = LittleFS.open(MAG_CAL_FILE, "w");
	if (!file) {
		logger.error("Failed to open mag calibration file for writing");
		return false;
	}

	size_t written = file.write((uint8_t*)&cfg, sizeof(MagCalibrationConfig));
	file.close();

	if (written != sizeof(MagCalibrationConfig)) {
		logger.error("Failed to write complete mag calibration (wrote %zu of %zu bytes)", written, sizeof(MagCalibrationConfig));
		return false;
	}

	logger.info("Saved mag calibration: offset=[%.3f, %.3f, %.3f], fieldStrength=%.3f",
		cfg.offset[0], cfg.offset[1], cfg.offset[2], cfg.fieldStrength);

	return true;
}

}  // namespace SlimeVR::MagCalibration

