/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2025 Gorbit99 & SlimeVR Contributors

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

#include "magdriver.h"

namespace SlimeVR::Sensors::SoftFusion {

std::vector<MagDefinition> MagDriver::supportedMags{
	MagDefinition{
		.name = "QMC6309",

		.deviceId = 0x7c,

		.whoAmIReg = 0x00,
		.expectedWhoAmI = 0x90,

		.dataWidth = MagDataWidth::SixByte,
		.dataReg = 0x01,

		.setup =
			[](MagInterface& interface) {
				// QMC6309 soft reset sequence
				interface.writeByte(0x0b, 0x80);
				interface.writeByte(0x0b, 0x00);  // Soft reset clear
				delay(10);
				// Configure range/ODR/set-reset: 0x0B = 0x48 (8G, 200Hz, set/reset on)
				interface.writeByte(0x0b, 0x48);
				// Enter Normal mode with OSR1=8, OSR2=2: 0x0A = 0x21
				interface.writeByte(0x0a, 0x21);
				return true;
			},
	},
	MagDefinition{
		.name = "IST8306",

		.deviceId = 0x19,

		.whoAmIReg = 0x00,
		.expectedWhoAmI = 0x06,

		.dataWidth = MagDataWidth::SixByte,
		.dataReg = 0x11,

		.setup =
			[](MagInterface& interface) {
				interface.writeByte(0x32, 0x01);  // Soft reset
				delay(50);
				interface.writeByte(0x30, 0x20);  // Noise suppression: low
				interface.writeByte(0x41, 0x2d);  // Oversampling: 32X
				interface.writeByte(0x31, 0x02);  // Continuous measurement @ 10Hz
				return true;
			},
	},
};

bool MagDriver::init(MagInterface&& interface, bool supports9ByteMags) {
	// Try to detect magnetometer with fault tolerance
	for (auto& mag : supportedMags) {
		try {
			interface.setDeviceId(mag.deviceId);

			logger.info("Trying mag %s!", mag.name);

			// Add a small delay to allow I2C bus to stabilize
			delay(5);

			// Attempt to read WhoAmI register with error handling
			uint8_t whoAmI;
			try {
				whoAmI = interface.readByte(mag.whoAmIReg);
			} catch (...) {
				// If readByte throws an exception, log and continue to next mag
				logger.warn("Failed to read WhoAmI from mag %s (may not be present)", mag.name);
				continue;
			}

			// Check if WhoAmI matches expected value
			if (whoAmI != mag.expectedWhoAmI) {
				logger.debug("Mag %s WhoAmI mismatch: expected 0x%02x, got 0x%02x", mag.name, mag.expectedWhoAmI, whoAmI);
				continue;
			}

			// Check if sensor supports this magnetometer type
			if (!supports9ByteMags && mag.dataWidth == MagDataWidth::NineByte) {
				logger.error("The sensor doesn't support 9-byte mags!");
				continue;
			}

			logger.info("Found mag %s! Initializing", mag.name);

			// Attempt setup with error handling
			bool setupSuccess = false;
			try {
				setupSuccess = mag.setup(interface);
			} catch (...) {
				logger.error("Exception during mag %s setup", mag.name);
				setupSuccess = false;
			}

			if (!setupSuccess) {
				logger.error("Mag %s failed to initialize!", mag.name);
				continue;  // Try next magnetometer instead of returning false
			}

			// Successfully detected and initialized
			detectedMag = mag;
			this->interface = interface;
			logger.info("Mag %s successfully initialized", mag.name);
			return true;
		} catch (...) {
			// Catch any unexpected exceptions to prevent kernel panic
			logger.error("Unexpected error while trying mag %s", mag.name);
			continue;  // Try next magnetometer
		}
	}

	// No magnetometer detected - this is OK, sensor can work without it
	logger.info("No magnetometer detected - sensor will operate in 6DoF mode");
	this->interface = interface;
	return false;  // Return false but don't cause panic - sensor can work without mag
}

void MagDriver::startPolling() const {
	if (!detectedMag) {
		return;
	}

	interface.startPolling(detectedMag->dataReg, detectedMag->dataWidth);
}

void MagDriver::stopPolling() const {
	if (!detectedMag) {
		return;
	}

	interface.stopPolling();
}

const char* MagDriver::getAttachedMagName() const {
	if (!detectedMag) {
		return nullptr;
	}

	return detectedMag->name;
}

}  // namespace SlimeVR::Sensors::SoftFusion
