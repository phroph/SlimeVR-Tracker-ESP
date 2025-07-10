/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2021 Eiren Rain & SlimeVR contributors

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
#include "batterymonitor.h"

#include "GlobalVars.h"

#if ESP8266 \
	&& (BATTERY_MONITOR == BAT_INTERNAL || BATTERY_MONITOR == BAT_INTERNAL_MCP3021)
ADC_MODE(ADC_VCC);
#endif

void BatteryMonitor::Setup() {
#if BATTERY_MONITOR == BAT_MCP3021 || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
	for (uint8_t i = 0x48; i < 0x4F; i++) {
		if (I2CSCAN::hasDevOnBus(i)) {
			address = i;
			break;
		}
	}
	if (address == 0) {
		m_Logger.error("MCP3021 not found on I2C bus");
	}
#endif
#if BATTERY_MONITOR == BAT_MAX17048 && defined(ESP32)
	analogReadResolution(12);
	analogSetAttenuation(adc_attenuation_t::ADC_11db);
	if (I2CSCAN::hasDevOnBus(0x36)) {
		address = 0x36;
	} else {
		m_Logger.error("MAX17048 not found on I2C bus");
	}
#endif
}

void BatteryMonitor::Loop() {
#if BATTERY_MONITOR == BAT_EXTERNAL || BATTERY_MONITOR == BAT_INTERNAL           \
	|| BATTERY_MONITOR == BAT_MCP3021 || BATTERY_MONITOR == BAT_INTERNAL_MCP3021 \
	|| BATTERY_MONITOR == BAT_MAX17048
	auto now_ms = millis();
	if (now_ms - last_battery_sample >= batterySampleRate) {
		last_battery_sample = now_ms;
		voltage = -1;
#if ESP8266 \
	&& (BATTERY_MONITOR == BAT_INTERNAL || BATTERY_MONITOR == BAT_INTERNAL_MCP3021)
		// Find out what your max measurement is (voltage_3_3).
		// Take the max measurement and check if it was less than 50mV
		// if yes output 5.0V
		// if no output 3.3V - dropvoltage + 0.1V
		auto ESPmV = ESP.getVcc();
		if (ESPmV > voltage_3_3) {
			voltage_3_3 = ESPmV;
		} else {
			// Calculate drop in mV
			ESPmV = voltage_3_3 - ESPmV;
			if (ESPmV < 50) {
				voltage = 5.0F;
			} else {
				voltage = 3.3F - ((float)ESPmV / 1000.0F)
						+ 0.1F;  // we assume 100mV drop on the linear converter
			}
		}
#endif
#if ESP8266 && BATTERY_MONITOR == BAT_EXTERNAL
		voltage = ((float)analogRead(PIN_BATTERY_LEVEL)) * ADCVoltageMax / ADCResolution
				* ADCMultiplier;
#endif
#if defined(ESP32) && BATTERY_MONITOR == BAT_EXTERNAL
		voltage
			= ((float)analogReadMilliVolts(PIN_BATTERY_LEVEL)) / 1000 * ADCMultiplier;
#endif
#if BATTERY_MONITOR == BAT_MCP3021 || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
		if (address > 0) {
			Wire.beginTransmission(address);
			Wire.requestFrom(address, (uint8_t)2);
			auto MSB = Wire.read();
			auto LSB = Wire.read();
			auto status = Wire.endTransmission();
			if (status == 0) {
				float v = (((uint16_t)(MSB & 0x0F) << 6) | (uint16_t)(LSB >> 2));
				v *= ADCMultiplier;
				voltage = (voltage > 0) ? min(voltage, v) : v;
			}
		}
#endif
#if BATTERY_MONITOR == BAT_MAX17048
		m_Logger.info("MAX17048 battery time...");
#if PIN_BATTERY_THERM
		auto Vout
			= analogReadMilliVolts(PIN_BATTERY_THERM) * 2.0;  // 10k voltage divider
		auto Resistance = 10000 / (4095.0 - Vout);
		int index = 0;
		for (int i = 0; i < 50; i++) {
			if (Resistance > B3950_TABLE[i]) {
				index = i;
				break;
			}
		}
		uint8_t temperature;
		if (Resistance - B3950_TABLE[index] > B3950_TABLE[index - 1] - Resistance) {
			temperature = index - 1;
		} else {
			temperature = index;
		}
		m_Logger.info(
			"B3950 Thermistor Sample (V=%f, R=%f, T=%d)",
			Vout / 1000.0,
			Resistance,
			temperature
		);
		uint8_t rcomp0, rcomp;
		I2Cdev::readByte(address, 0x0C, &rcomp0);  // get RCOMP0
		if (temperature > 20.0) {
			rcomp = rcomp0 + static_cast<uint8_t>(lround((temperature - 20) * -0.5));
		} else {
			rcomp = rcomp0 + static_cast<uint8_t>(lround((temperature - 20) * -5.0));
		}
		m_Logger
			.info("MAX17048 RCOMP Computation (RCOMP0=%d, RCOMP=%d)", rcomp0, rcomp);
		I2Cdev::writeByte(address, 0x0C, rcomp);  // push RCOMP
#endif
		if (address > 0) {
			// Cell voltage register address 0x02
			uint16_t data;
			auto status = I2Cdev::readWord(address, 0x02, &data);
			if (status == 1) {
				float v = data * (78.125 / 1000000);
				voltage = (voltage > 0) ? min(voltage, v) : v;
				m_Logger.info("MAX17048 voltage %f", voltage);
			}
		}
		uint16_t data;
		auto status = I2Cdev::readWord(address, 0x04, &data);
		if (status == 1) {
			level = data * (0.01 / 256);
			m_Logger.info("MAX17048 level %f", level);
		}
#else
		if (voltage > 0)  // valid measurement
		{
			// Estimate battery level, 3.2V is 0%, 4.17V is 100% (1.0)
			if (voltage > 3.975f) {
				level = (voltage - 2.920f) * 0.8f;
			} else if (voltage > 3.678f) {
				level = (voltage - 3.300f) * 1.25f;
			} else if (voltage > 3.489f) {
				level = (voltage - 3.400f) * 1.7f;
			} else if (voltage > 3.360f) {
				level = (voltage - 3.300f) * 0.8f;
			} else {
				level = (voltage - 3.200f) * 0.3f;
			}

			level = (level - 0.05f) / 0.95f;  // Cut off the last 5% (3.36V)

			if (level > 1) {
				level = 1;
			} else if (level < 0) {
				level = 0;
			}

#endif
		if (voltage > 0) {
			networkConnection.sendBatteryLevel(voltage, level);
#ifdef BATTERY_LOW_POWER_VOLTAGE
			if (voltage < BATTERY_LOW_POWER_VOLTAGE) {
#if defined(BATTERY_LOW_VOLTAGE_DEEP_SLEEP) && BATTERY_LOW_VOLTAGE_DEEP_SLEEP
				ESP.deepSleep(0);
#else
				statusManager.setStatus(SlimeVR::Status::LOW_BATTERY, true);
#endif
			} else {
				statusManager.setStatus(SlimeVR::Status::LOW_BATTERY, false);
			}
#endif
		}
	}
#endif
}
