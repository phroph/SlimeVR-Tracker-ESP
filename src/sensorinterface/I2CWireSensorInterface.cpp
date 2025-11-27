/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Eiren Rain & SlimeVR Contributors

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

#include "I2CWireSensorInterface.h"

#include <optional>

std::optional<uint8_t> activeSCLPin;
std::optional<uint8_t> activeSDAPin;
bool isI2CActive = false;

namespace SlimeVR {
void swapI2C(uint8_t sclPin, uint8_t sdaPin) {
	// If already on the correct pins and I2C is active, no need to swap
	if (sclPin == activeSCLPin && sdaPin == activeSDAPin && isI2CActive) {
		return;
	}

#ifdef ESP32
	// Check if we're switching to the default pins that were initialized in main.cpp
	// If so, and I2C hasn't been marked as active yet, just update state without reinitializing
	if (!isI2CActive && sclPin == PIN_IMU_SCL && sdaPin == PIN_IMU_SDA) {
		// Wire was already initialized in main.cpp, just update our state
		activeSCLPin = sclPin;
		activeSDAPin = sdaPin;
		isI2CActive = true;
		return;
	}

	// CRITICAL: On ESP32, we MUST call Wire.end() before Wire.begin() if switching pins
	// or if Wire might already be initialized. Calling Wire.begin() when already
	// initialized can leave the driver in an invalid state causing NULL TX buffer errors.
	
	// Wait for any pending transactions to complete before ending
	// Wire.flush() doesn't wait for transactions, so we need to ensure the bus is idle
	Wire.flush();
	
	// Wait for any in-flight transactions to complete
	// ESP32 I2C transactions can take up to a few hundred microseconds
	// We need to wait longer to ensure transactions from SensorHub complete
	delay(2);  // 2ms to be safe for any pending I2C transaction
	
	// Disconnect pins from HWI2C before ending (if we had previous pins)
	if (activeSCLPin.has_value()) {
		gpio_set_direction((gpio_num_t)*activeSCLPin, GPIO_MODE_INPUT);
	}
	if (activeSDAPin.has_value()) {
		gpio_set_direction((gpio_num_t)*activeSDAPin, GPIO_MODE_INPUT);
	}
	
	// ALWAYS end Wire before beginning with new pins on ESP32
	// This is critical - even if Wire wasn't initialized by us, ending it is safe
	// and prevents the NULL TX buffer pointer error
	Wire.end();
	
	// Additional delay to ensure I2C driver is fully stopped and cleaned up
	// This is critical to prevent ESP_ERR_INVALID_STATE
	delay(2);
	
	// Initialize I2C with new pins
	// On ESP32, Wire.begin() MUST be called after Wire.end() to avoid invalid state
	Wire.begin(static_cast<int>(sdaPin), static_cast<int>(sclPin), I2C_SPEED);
	Wire.setTimeOut(150);
	
	// Small delay to ensure I2C driver is fully initialized before use
	// This prevents NULL TX buffer pointer errors
	delay(1);  // 1ms to ensure driver is ready
#else
	Wire.begin(static_cast<int>(sdaPin), static_cast<int>(sclPin));
#endif

	activeSCLPin = sclPin;
	activeSDAPin = sdaPin;
	isI2CActive = true;
}

void disconnectI2C() {
	Wire.flush();
	isI2CActive = false;
#ifdef ESP32
	Wire.end();
#endif
}
}  // namespace SlimeVR
