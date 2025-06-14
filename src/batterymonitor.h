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
#ifndef SLIMEVR_BATTERYMONITOR_H_
#define SLIMEVR_BATTERYMONITOR_H_

#include <Arduino.h>
#include <I2Cdev.h>
#include <i2cscan.h>

#include "globals.h"
#include "logging/Logger.h"

#if ESP8266
#define ADCResolution 1023.0  // ESP8266 has 10bit ADC
#define ADCVoltageMax 1.0  // ESP8266 input is 1.0 V = 1023.0
#endif
#ifndef ADCResolution
#define ADCResolution 1023.0
#endif
#ifndef ADCVoltageMax
#define ADCVoltageMax 1.0
#endif

#ifndef BATTERY_SHIELD_RESISTANCE
#define BATTERY_SHIELD_RESISTANCE 180.0
#endif
#ifndef BATTERY_SHIELD_R1
#define BATTERY_SHIELD_R1 100.0
#endif
#ifndef BATTERY_SHIELD_R2
#define BATTERY_SHIELD_R2 220.0
#endif

#if BATTERY_MONITOR == BAT_EXTERNAL
// Wemos D1 Mini has an internal Voltage Divider with R1=100K and R2=220K > this
// means, 3.3V analogRead input voltage results in 1023.0 Wemos D1 Mini with Wemos
// Battery Shield v1.2.0 or higher: Battery Shield with J2 closed, has an additional
// 130K resistor. So the resulting Voltage Divider is R1=220K+100K=320K and R2=100K >
// this means, 4.5V analogRead input voltage results in 1023.0 ESP32 Boards may have not
// the internal Voltage Divider. Also ESP32 has a 12bit ADC (0..4095). So R1 and R2 can
// be changed. Diagramm:
//   (Battery)--- [BATTERY_SHIELD_RESISTANCE] ---(INPUT_BOARD)---  [BATTERY_SHIELD_R2]
//   ---(ESP_INPUT)--- [BATTERY_SHIELD_R1] --- (GND)
// SlimeVR Board can handle max 5V > so analogRead of 5.0V input will result in 1023.0
#define ADCMultiplier                                                   \
	(BATTERY_SHIELD_R1 + BATTERY_SHIELD_R2 + BATTERY_SHIELD_RESISTANCE) \
		/ BATTERY_SHIELD_R1
#elif BATTERY_MONITOR == BAT_MCP3021 || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
// Default recommended resistors are 9.1k and 5.1k
#define ADCMultiplier 3.3 / 1023.0 * 14.2 / 9.1
#endif

class BatteryMonitor {
public:
	void Setup();
	void Loop();

	float getVoltage() const { return voltage; }
	float getLevel() const { return level; }

private:
	unsigned long last_battery_sample = 0;
#if BATTERY_MONITOR == BAT_MCP3021 || BATTERY_MONITOR == BAT_INTERNAL_MCP3021 || BATTERY_MONITOR == BAT_MAX17048
	uint8_t address = 0;
	float B3950_TABLE[50] = {
		31.77,
		30.25,
		28.82,
		27.45,
		26.16,
		24.94,
		23.77,
		22.67,
		21.62,
		20.63,
		19.68,
		18.78,
		17.93,
		17.12,
		16.35,
		15.62,
		14.93,
		14.26,
		13.63,
		13.04,
		12.47,
		11.92,
		11.41,
		10.91,
		10.45,
		10.00,
		9.575,
		9.170,
		8.784,
		8.416,
		8.064,
		7.730,
		7.410,
		7.106,
		6.815,
		6.538,
		6.273,
		6.020,
		5.778,
		5.548,
		5.327,
		5.117,
		4.915,
		4.723,
		4.539,
		4.363,
		4.195,
		4.034,
		3.880,
		3.733
	};
#endif
#if BATTERY_MONITOR == BAT_INTERNAL || BATTERY_MONITOR == BAT_INTERNAL_MCP3021
	uint16_t voltage_3_3 = 3000;
#endif
	float voltage = -1;
	float level = -1;

	SlimeVR::Logging::Logger m_Logger = SlimeVR::Logging::Logger("BatteryMonitor");
};

#endif  // SLIMEVR_BATTERYMONITOR_H_
