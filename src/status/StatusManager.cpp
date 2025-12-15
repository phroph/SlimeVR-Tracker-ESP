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

#include "StatusManager.h"

namespace SlimeVR::Status {

static inline bool isTimeInFuture(uint32_t nowMs, uint32_t futureMs) {
	// Handles millis() wraparound correctly using signed subtraction.
	return static_cast<int32_t>(futureMs - nowMs) > 0;
}

void StatusManager::setStatus(Status status, bool value) {
	if (value) {
		if (m_Status & status) {
			return;
		}

		m_Logger.trace("Added status %s", statusToString(status));

		m_Status |= status;
	} else {
		if (!(m_Status & status)) {
			return;
		}

		m_Logger.trace("Removed status %s", statusToString(status));

		m_Status &= ~status;
	}
}

bool StatusManager::hasStatus(Status status) {
	return (m_Status & status) == status;
}

void StatusManager::setMagCalibrating(bool calibrating) {
	if (m_MagCalibrating != calibrating) {
		m_Logger.trace(
			"Mag calibrating: %s",
			calibrating ? "true" : "false"
		);
		m_MagCalibrating = calibrating;
	}
}

void StatusManager::setNeedsRestCalibration(bool needed) {
	if (m_NeedsRestCalibration != needed) {
		m_Logger.trace(
			"Needs rest calibration: %s",
			needed ? "true" : "false"
		);
		m_NeedsRestCalibration = needed;
	}
}

void StatusManager::setImuCalibrating(bool calibrating) {
	if (m_ImuCalibrating != calibrating) {
		m_Logger.trace(
			"IMU calibrating: %s",
			calibrating ? "true" : "false"
		);
		m_ImuCalibrating = calibrating;
	}
}

void StatusManager::pushLedEvent(LEDStatus status, uint32_t durationMs) {
	if (durationMs == 0) {
		return;
	}

	const uint32_t now = millis();
	LedEvent& slot = m_LedEvents[m_LedEventWriteIndex % kMaxLedEvents];
	m_LedEventWriteIndex++;

	slot.status = status;
	slot.untilMs = now + durationMs;
}

LEDStatus StatusManager::getResolvedLEDStatus() const {
	// Priority order - check highest priority first

	const uint32_t now = millis();

	// Start with persistent statuses
	LEDStatus resolved = LEDStatus::OK;

	// 1. ERROR (IMU failure or other hardware error)
	if (m_Status & Status::IMU_ERROR) {
		resolved = LEDStatus::ERROR;
	}

	// 2. LOW_BATTERY
	else if (m_Status & Status::LOW_BATTERY) {
		resolved = LEDStatus::LOW_BATTERY;
	}

	// 3. STARTUP (loading/initializing)
	else if (m_Status & Status::LOADING) {
		resolved = LEDStatus::STARTUP;
	}

	// 4. WIFI_CONNECTING
	else if (m_Status & Status::WIFI_CONNECTING) {
		resolved = LEDStatus::WIFI_CONNECTING;
	}

	// 5. SERVER_CONNECTING
	else if (m_Status & Status::SERVER_CONNECTING) {
		resolved = LEDStatus::SERVER_CONNECTING;
	}

	// 6. MAG_CALIBRATING (solid cyan while user rotates device)
	else if (m_MagCalibrating) {
		resolved = LEDStatus::MAG_CALIBRATING;
	}

	// 7. IMU_CALIBRATING (solid orange while running IMU calibration flows)
	else if (m_ImuCalibrating) {
		resolved = LEDStatus::IMU_CALIBRATING;
	}

	// 8. NEEDS_REST_CALIBRATION (pulsing magenta - set device down)
	// Only show this if we're fully connected (past startup/connection phases)
	else if (m_NeedsRestCalibration) {
		resolved = LEDStatus::NEEDS_REST_CALIBRATION;
	}

	// Consider short-lived LED events (one-shots).
	//
	// Design intent:
	// - Events (e.g. CALIBRATION_SAVED) should provide momentary feedback
	//   *without overriding* errors / startup / connection / calibration states.
	// - Therefore, only show events when the persistent resolved status is OK.
	if (resolved == LEDStatus::OK) {
		bool haveEvent = false;
		LEDStatus eventStatus = LEDStatus::OK;
		uint32_t eventUntilMs = 0;

		for (const auto& ev : m_LedEvents) {
			if (ev.untilMs == 0) {
				continue;
			}
			if (!isTimeInFuture(now, ev.untilMs)) {
				continue;
			}

			// Prefer the most recently pushed event.
			// We approximate this by choosing the event with the furthest expiration.
			if (!haveEvent || isTimeInFuture(eventUntilMs, ev.untilMs)) {
				haveEvent = true;
				eventStatus = ev.status;
				eventUntilMs = ev.untilMs;
			}
		}

		if (haveEvent) {
			return eventStatus;
		}
	}

	return resolved;
}

}  // namespace SlimeVR::Status
