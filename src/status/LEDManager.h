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

#ifndef SLIMEVR_LEDMANAGER_H
#define SLIMEVR_LEDMANAGER_H

#include <Arduino.h>
#include <FastLED.h>

#include "../globals.h"
#include "../logging/Logger.h"
#include "LEDStatus.h"

#define NUM_LEDS 1

namespace SlimeVR {

/**
 * LED Manager - displays system status via LED patterns.
 *
 * Supports two LED types with RGB as priority:
 * 1. Addressable RGB LED (PIN_RGB) - NeoPixel/WS2812 via FastLED (preferred)
 * 2. Generic single-color LED (LED_PIN) - fallback for simpler boards
 *
 * If both are defined, RGB takes priority. If neither is usable, LED is disabled.
 */
class LEDManager {
public:
	void setup();

	/**
	 * Update the LED display based on current system status.
	 * Call this regularly from the main loop.
	 */
	void update();

	/**
	 * Force the LED off (for power saving or shutdown).
	 */
	void off();

	/**
	 * Turn the LED on with a specific color.
	 */
	void on(CRGB color);

	/**
	 * Blocking blink for simple status indication during startup.
	 * Avoid using this during normal operation as it blocks.
	 */
	void blink(unsigned long timeMs, CRGB color);

	/**
	 * Blocking pattern for simple status indication during startup.
	 * Avoid using this during normal operation as it blocks.
	 */
	void pattern(unsigned long onMs, unsigned long offMs, int times, CRGB color);

private:
	// RGB LED support (priority)
#if defined(PIN_RGB) && !defined(DISABLE_RGB_FASTLED)
	static constexpr bool m_HasRGB = true;
#else
	static constexpr bool m_HasRGB = false;
#endif

	// Generic LED fallback
	uint8_t m_Pin = LED_PIN;
	bool m_HasGenericLED = m_Pin >= 0 && m_Pin < LED_OFF;
	bool m_On = LED_INVERTED ? LOW : HIGH;
	bool m_Off = !m_On;

	// Enabled if we have either RGB or generic LED
	bool m_Enabled = m_HasRGB || m_HasGenericLED;

	CRGB m_Leds[NUM_LEDS];
	CRGB m_CurrentColor = CRGB::Black;
	bool m_PowerEnabled = false;

	// Pattern state machine
	enum class PatternStage { OFF, ON, GAP, INTERVAL };
	PatternStage m_Stage = PatternStage::OFF;
	uint8_t m_BlinkCount = 0;
	uint32_t m_StageStartMs = 0;

	// Pulse state (for breathing effect)
	uint32_t m_PulseStartMs = 0;

	// Track current status to detect changes
	Status::LEDStatus m_CurrentStatus = Status::LEDStatus::OFF;

	// Power profile integration
	bool m_UserEnabled = true;

	// Timing
	uint32_t m_LastUpdateMs = 0;

	Logging::Logger m_Logger = Logging::Logger("LEDManager");

	// Internal helpers
	void setLED(CRGB color);
	void updateBlink(const Status::LEDStatusConfig& config);
	void updatePulse(const Status::LEDStatusConfig& config);
};

}  // namespace SlimeVR

#endif
