/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2022 TheDevMinerTV, 2025 SlimeVR Contributors

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

#ifndef STATUS_STATUSMANAGER_H
#define STATUS_STATUSMANAGER_H

#include <Arduino.h>
#include <cstddef>
#include "LEDStatus.h"
#include "Status.h"
#include "logging/Logger.h"

namespace SlimeVR::Status {

/**
 * Centralized status manager for LED display.
 *
 * This class tracks all status sources and resolves them to a single LED status
 * based on priority. Components set/unset status flags, and the manager
 * determines what the LED should show.
 *
 * Priority order (highest to lowest):
 * 1. ERROR (IMU failure, hardware fault)
 * 2. LOW_BATTERY
 * 3. STARTUP (loading/initializing)
 * 4. WIFI_CONNECTING
 * 5. SERVER_CONNECTING
 * 6. MAG_CALIBRATING
 * 7. IMU_CALIBRATING
 * 8. NEEDS_REST_CALIBRATION
 * 9. OK (everything working)
 */
class StatusManager {
public:
	/**
	 * Set or clear a status flag.
	 * @param status The status flag to modify
	 * @param value true to set, false to clear
	 */
	void setStatus(Status status, bool value);

	/**
	 * Check if a specific status flag is set.
	 */
	bool hasStatus(Status status);

	/**
	 * Get the raw status bitmask (for legacy compatibility).
	 */
	uint32_t getStatus() { return m_Status; }

	// === New centralized LED status API ===

	/**
	 * Set the magnetometer calibration state.
	 * When true, LED shows MAG_CALIBRATING status.
	 */
	void setMagCalibrating(bool calibrating);

	/**
	 * Check if mag calibration is in progress.
	 */
	bool isMagCalibrating() const { return m_MagCalibrating; }

	/**
	 * Set whether rest calibration is needed.
	 * When true, LED shows NEEDS_REST_CALIBRATION status (after connection).
	 */
	void setNeedsRestCalibration(bool needed);

	/**
	 * Check if rest calibration is needed.
	 */
	bool needsRestCalibration() const { return m_NeedsRestCalibration; }

	/**
	 * Set whether an IMU calibration flow is running (SoftFusion / legacy IMUs).
	 * When true, LED shows IMU_CALIBRATING (unless a higher-priority status is active).
	 */
	void setImuCalibrating(bool calibrating);

	bool isImuCalibrating() const { return m_ImuCalibrating; }

	/**
	 * Post a short-lived LED event (one-shot indication) that is still resolved
	 * through the centralized priority system.
	 *
	 * Event display rule:
	 * - Events are only shown when the persistent resolved status is OK, so they
	 *   do not override error/startup/connection/calibration indicators.
	 *
	 * This is used to replace direct ledManager.blink()/pattern() calls.
	 */
	void pushLedEvent(LEDStatus status, uint32_t durationMs);

	/**
	 * Resolve all status sources and return the highest-priority LED status.
	 * This is the single source of truth for what the LED should display.
	 */
	LEDStatus getResolvedLEDStatus() const;

private:
	struct LedEvent {
		LEDStatus status{LEDStatus::OFF};
		uint32_t untilMs{0};
	};

	static constexpr size_t kMaxLedEvents = 4;

	// Legacy status bitmask (for backward compatibility with existing code)
	uint32_t m_Status = 0;

	// New status flags
	bool m_MagCalibrating = false;
	bool m_ImuCalibrating = false;
	bool m_NeedsRestCalibration = true;  // Default true until rest cal completes

	// Fixed-size ring buffer of short-lived events
	LedEvent m_LedEvents[kMaxLedEvents]{};
	size_t m_LedEventWriteIndex = 0;

	Logging::Logger m_Logger = Logging::Logger("StatusManager");
};

}  // namespace SlimeVR::Status

#endif
