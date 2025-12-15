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

#ifndef SLIMEVR_LED_STATUS_H
#define SLIMEVR_LED_STATUS_H

#include <FastLED.h>
#include <cstdint>

namespace SlimeVR::Status {

/**
 * LED Status enumeration - ordered by priority (lowest value = highest priority).
 *
 * The LED system will always display the highest-priority active status.
 * This provides a single, centralized definition of what the LED should show.
 */
enum class LEDStatus : uint8_t {
	// === HIGHEST PRIORITY ===

	// Critical errors (IMU failure, hardware fault)
	ERROR = 0,

	// Battery critically low - user needs to charge immediately
	LOW_BATTERY = 1,

	// System is starting up / initializing
	STARTUP = 2,

	// === CONNECTION STATES ===

	// Connecting to WiFi network
	WIFI_CONNECTING = 10,

	// Connected to WiFi, searching for SlimeVR server
	SERVER_CONNECTING = 11,

	// === CALIBRATION STATES ===

	// Magnetometer calibration in progress (user is rotating device)
	MAG_CALIBRATING = 20,

	// IMU calibration in progress (user is following calibration prompts)
	IMU_CALIBRATING = 22,

	// Rest calibration not yet complete (needs to sit still for 3s)
	NEEDS_REST_CALIBRATION = 21,

	// === NORMAL OPERATION ===

	// Everything is good, running normally
	OK = 100,

	// === MISC / ONE-SHOT EVENTS ===
	// One-shot feedback that should not override connection/calibration/error.
	CALIBRATION_SAVED = 200,

	// === LOWEST PRIORITY ===

	// LED should be off (power saving or disabled)
	OFF = 255,
};

/**
 * LED pattern type - how the LED behaves
 */
enum class LEDPattern : uint8_t {
	OFF,      // LED is off
	SOLID,    // LED is solid on
	BLINK,    // LED blinks N times, then pauses
	PULSE,    // LED pulses slowly (breathing effect)
};

/**
 * LED status configuration - defines how each status is displayed
 */
struct LEDStatusConfig {
	CRGB color;               // LED color
	LEDPattern pattern;       // Pattern type
	uint8_t blinkCount;       // For BLINK: number of blinks per cycle
	uint16_t onTimeMs;        // On duration in ms
	uint16_t offTimeMs;       // Off/gap duration in ms
	uint16_t intervalMs;      // Pause between blink cycles
};

/**
 * Get the display configuration for a given LED status.
 *
 * Colors and patterns:
 * - ERROR:                  Yellow, 5 fast blinks (hardware issue)
 * - LOW_BATTERY:            Red, 1 blink (charge needed)
 * - STARTUP:                White, solid (initializing)
 * - WIFI_CONNECTING:        Blue, 3 blinks (searching for network)
 * - SERVER_CONNECTING:      Green, 2 blinks (connected to WiFi, finding server)
 * - MAG_CALIBRATING:        Cyan, solid (rotate device)
 * - NEEDS_REST_CALIBRATION: Magenta, slow pulse (set device down)
 * - OK:                     Green, very slow blink (standby heartbeat)
 * - OFF:                    Off
 */
inline LEDStatusConfig getLEDStatusConfig(LEDStatus status) {
	switch (status) {
		case LEDStatus::ERROR:
			return {
				CRGB::Yellow,
				LEDPattern::BLINK,
				5,     // 5 blinks
				300,   // 300ms on
				500,   // 500ms off
				1000,  // 1s pause
			};

		case LEDStatus::LOW_BATTERY:
			return {
				CRGB::Red,
				LEDPattern::BLINK,
				1,    // 1 blink
				300,  // 300ms on
				500,  // 500ms off
				300,  // 300ms pause (fast warning)
			};

		case LEDStatus::STARTUP:
			return {
				CRGB::White,
				LEDPattern::SOLID,
				0,
				0,
				0,
				0,
			};

		case LEDStatus::WIFI_CONNECTING:
			return {
				CRGB::Blue,
				LEDPattern::BLINK,
				3,     // 3 blinks
				300,   // 300ms on
				500,   // 500ms off
				3000,  // 3s pause
			};

		case LEDStatus::SERVER_CONNECTING:
			return {
				CRGB::Green,
				LEDPattern::BLINK,
				2,     // 2 blinks
				300,   // 300ms on
				500,   // 500ms off
				3000,  // 3s pause
			};

		case LEDStatus::MAG_CALIBRATING:
			return {
				CRGB::Cyan,
				LEDPattern::SOLID,
				0,
				0,
				0,
				0,
			};

		case LEDStatus::IMU_CALIBRATING:
			return {
				CRGB::Orange,
				LEDPattern::SOLID,
				0,
				0,
				0,
				0,
			};

		case LEDStatus::NEEDS_REST_CALIBRATION:
			return {
				CRGB::Magenta,
				LEDPattern::PULSE,
				0,
				1000,  // 1s on phase
				1000,  // 1s off phase
				0,
			};

		case LEDStatus::OK:
			// Very slow heartbeat blink when everything is good
			return {
				CRGB::Green,
				LEDPattern::BLINK,
				1,      // 1 blink
				300,    // 300ms on
				500,    // 500ms off
				10000,  // 10s pause (minimal power draw)
			};

		case LEDStatus::CALIBRATION_SAVED:
			// One-shot style: caller should post as a timed event so it doesn't repeat.
			return {
				CRGB::Pink,
				LEDPattern::BLINK,
				1,    // 1 blink
				100,  // 100ms on
				0,    // no gap
				1000  // long interval (shouldn't matter if event is short-lived)
			};

		case LEDStatus::OFF:
		default:
			return {
				CRGB::Black,
				LEDPattern::OFF,
				0,
				0,
				0,
				0,
			};
	}
}

/**
 * Get a human-readable name for a LED status (for debugging)
 */
inline const char* getLEDStatusName(LEDStatus status) {
	switch (status) {
		case LEDStatus::ERROR:
			return "ERROR";
		case LEDStatus::LOW_BATTERY:
			return "LOW_BATTERY";
		case LEDStatus::STARTUP:
			return "STARTUP";
		case LEDStatus::WIFI_CONNECTING:
			return "WIFI_CONNECTING";
		case LEDStatus::SERVER_CONNECTING:
			return "SERVER_CONNECTING";
		case LEDStatus::MAG_CALIBRATING:
			return "MAG_CALIBRATING";
		case LEDStatus::IMU_CALIBRATING:
			return "IMU_CALIBRATING";
		case LEDStatus::NEEDS_REST_CALIBRATION:
			return "NEEDS_REST_CALIBRATION";
		case LEDStatus::OK:
			return "OK";
		case LEDStatus::CALIBRATION_SAVED:
			return "CALIBRATION_SAVED";
		case LEDStatus::OFF:
			return "OFF";
		default:
			return "UNKNOWN";
	}
}

}  // namespace SlimeVR::Status

#endif  // SLIMEVR_LED_STATUS_H
