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

#include "LEDManager.h"

#include "../GlobalVars.h"
#include "../power/PowerProfile.h"
#include "StatusManager.h"

#if defined(ESP32)
#include <esp_log.h>
#endif

namespace SlimeVR {

void LEDManager::setup() {
	if (!m_Enabled) {
		m_Logger.info("No LED configured");
		return;
	}

#if defined(ESP32)
	// Suppress noisy FastLED driver warnings
	esp_log_level_set("ChannelEngineSpi", ESP_LOG_ERROR);
#endif

	// Setup generic LED (fallback) if available and no RGB
	if (m_HasGenericLED && !m_HasRGB) {
		pinMode(m_Pin, OUTPUT);
		digitalWrite(m_Pin, m_Off);
		m_Logger.info("Generic LED initialized on pin %d", m_Pin);
	}

	// Setup RGB LED (priority) if available
#if defined(PIN_RGB) && !defined(DISABLE_RGB_FASTLED)
#ifdef PIN_RGB_POWER
	pinMode(PIN_RGB_POWER, OUTPUT);
	digitalWrite(PIN_RGB_POWER, HIGH);
	m_PowerEnabled = true;
#endif
	FastLED.setBrightness(25);
	FastLED.addLeds<NEOPIXEL, PIN_RGB>(m_Leds, NUM_LEDS);
	m_Leds[0] = CRGB::Black;
	FastLED.show();
	m_Logger.info("RGB LED initialized on pin %d", PIN_RGB);
#endif

	m_LastUpdateMs = millis();

	// Initial update
	update();
}

void LEDManager::setLED(CRGB color) {
	if (!m_Enabled) {
		return;
	}

	// Only update hardware if color changed
	if (m_CurrentColor == color) {
		return;
	}

	m_CurrentColor = color;

	bool isOff = (color == CRGB(CRGB::Black));

	// RGB LED handling (priority)
#if defined(PIN_RGB) && !defined(DISABLE_RGB_FASTLED)
#ifdef PIN_RGB_POWER
	if (isOff && m_PowerEnabled) {
		digitalWrite(PIN_RGB_POWER, LOW);
		m_PowerEnabled = false;
	} else if (!isOff && !m_PowerEnabled) {
		digitalWrite(PIN_RGB_POWER, HIGH);
		m_PowerEnabled = true;
	}
#endif
	m_Leds[0] = color;
	FastLED.show();
#endif

	// Generic LED fallback (only if no RGB)
	if (!m_HasRGB && m_HasGenericLED) {
		digitalWrite(m_Pin, isOff ? m_Off : m_On);
	}
}

void LEDManager::on(CRGB color) {
	setLED(color);
}

void LEDManager::off() {
	setLED(CRGB::Black);
}

void LEDManager::blink(unsigned long timeMs, CRGB color) {
	on(color);
	delay(timeMs);
	off();
}

void LEDManager::pattern(unsigned long onMs, unsigned long offMs, int times, CRGB color) {
	for (int i = 0; i < times; i++) {
		blink(onMs, color);
		delay(offMs);
	}
}

void LEDManager::update() {
	if (!m_Enabled) {
		return;
	}

	// Power profiling / low-power preset may disable LEDs at runtime
	bool userEnabled = Power::g_ledEnabled;
	if (userEnabled != m_UserEnabled) {
		m_UserEnabled = userEnabled;
		if (!userEnabled) {
			off();
			return;
		}
		// Reset state machine when re-enabling
		m_Stage = PatternStage::OFF;
		m_BlinkCount = 0;
	}

	if (!userEnabled) {
		return;
	}

	uint32_t now = millis();

	// Don't update too frequently
	if (now - m_LastUpdateMs < 10) {
		return;
	}
	m_LastUpdateMs = now;

	// Query the centralized status manager for current LED status
	Status::LEDStatus newStatus = statusManager.getResolvedLEDStatus();

	// Detect status changes to reset pattern state
	if (newStatus != m_CurrentStatus) {
		m_CurrentStatus = newStatus;
		m_Stage = PatternStage::OFF;
		m_BlinkCount = 0;
		m_StageStartMs = now;
		m_PulseStartMs = now;

		m_Logger.debug(
			"LED status: %s",
			Status::getLEDStatusName(newStatus)
		);
	}

	// Get the display configuration for current status
	Status::LEDStatusConfig config = Status::getLEDStatusConfig(m_CurrentStatus);

	// Handle different pattern types
	switch (config.pattern) {
		case Status::LEDPattern::OFF:
			off();
			break;

		case Status::LEDPattern::SOLID:
			on(config.color);
			break;

		case Status::LEDPattern::BLINK:
			updateBlink(config);
			break;

		case Status::LEDPattern::PULSE:
			updatePulse(config);
			break;
	}
}

void LEDManager::updateBlink(const Status::LEDStatusConfig& config) {
	uint32_t now = millis();
	uint32_t elapsed = now - m_StageStartMs;

	switch (m_Stage) {
		case PatternStage::OFF:
			// Start a new blink cycle
			on(config.color);
			m_Stage = PatternStage::ON;
			m_BlinkCount = 0;
			m_StageStartMs = now;
			break;

		case PatternStage::ON:
			if (elapsed >= config.onTimeMs) {
				off();
				m_BlinkCount++;
				m_StageStartMs = now;

				if (m_BlinkCount >= config.blinkCount) {
					// Done with blinks, go to interval
					m_Stage = PatternStage::INTERVAL;
				} else {
					// More blinks to go, short gap
					m_Stage = PatternStage::GAP;
				}
			}
			break;

		case PatternStage::GAP:
			if (elapsed >= config.offTimeMs) {
				on(config.color);
				m_Stage = PatternStage::ON;
				m_StageStartMs = now;
			}
			break;

		case PatternStage::INTERVAL:
			if (elapsed >= config.intervalMs) {
				// Restart blink cycle
				m_Stage = PatternStage::OFF;
			}
			break;
	}
}

void LEDManager::updatePulse(const Status::LEDStatusConfig& config) {
	uint32_t now = millis();
	uint32_t cycleTime = config.onTimeMs + config.offTimeMs;
	uint32_t elapsed = (now - m_PulseStartMs) % cycleTime;

	// Calculate brightness using sine wave for smooth pulsing
	float phase = (float)elapsed / (float)cycleTime;  // 0.0 to 1.0
	float sineValue = sin(phase * 3.14159265f);       // 0 to 1 to 0
	uint8_t brightness = (uint8_t)(sineValue * 255.0f);

	// Scale the color by brightness
	CRGB scaledColor = config.color;
	scaledColor.nscale8(brightness);

	// For pulse, bypass the change check and update directly
#if defined(PIN_RGB) && !defined(DISABLE_RGB_FASTLED)
	if (brightness > 10) {
#ifdef PIN_RGB_POWER
		if (!m_PowerEnabled) {
			digitalWrite(PIN_RGB_POWER, HIGH);
			m_PowerEnabled = true;
		}
#endif
		m_Leds[0] = scaledColor;
	} else {
		m_Leds[0] = CRGB::Black;
	}

	// Rate-limit FastLED.show() calls (every ~20ms is smooth enough)
	static uint32_t lastShowMs = 0;
	if (now - lastShowMs >= 20) {
		lastShowMs = now;
		FastLED.show();
	}
#else
	// Generic LED: simple on/off based on brightness threshold
	if (m_HasGenericLED) {
		digitalWrite(m_Pin, brightness > 127 ? m_On : m_Off);
	}
#endif
}

}  // namespace SlimeVR
