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

#include <i2cscan.h>

#include "GlobalVars.h"
#include "Wire.h"
#include "batterymonitor.h"
#include "credentials.h"
#include "debugging/TimeTaken.h"
#include "globals.h"
#include "logging/Logger.h"
#include "mag_calibration_manager.h"
#include "ota.h"
#include "serial/serialcommands.h"
#include "status/TPSCounter.h"

#if defined(ESP32)
#include <esp_log.h>
#endif

Timer<> globalTimer;
SlimeVR::Logging::Logger logger("SlimeVR");
SlimeVR::Sensors::SensorManager sensorManager;
SlimeVR::LEDManager ledManager;
SlimeVR::Status::StatusManager statusManager;
SlimeVR::Configuration::Configuration configuration;
SlimeVR::Network::Manager networkManager;
SlimeVR::Network::Connection networkConnection;
SlimeVR::WiFiNetwork wifiNetwork;
SlimeVR::WifiProvisioning wifiProvisioning;

constexpr uint32_t SERIAL_CONNECT_WAIT_MS = 4000;

static void waitForSerialConnection() {
#if defined(USBCON) || defined(ARDUINO_USB_CDC_ON_BOOT)
	const unsigned long start = millis();
	while (!Serial && (millis() - start) < SERIAL_CONNECT_WAIT_MS) {
		delay(10);
	}
	if (!Serial) {
		logger.warn(
			"No USB CDC host detected after %lu ms, continuing anyway",
			static_cast<unsigned long>(SERIAL_CONNECT_WAIT_MS)
		);
	} else {
		delay(50);
	}
#endif
}

#if DEBUG_MEASURE_SENSOR_TIME_TAKEN
SlimeVR::Debugging::TimeTakenMeasurer sensorMeasurer{"Sensors"};
#endif

int sensorToCalibrate = -1;
bool blinking = false;
unsigned long blinkStart = 0;
unsigned long loopTime = 0;
unsigned long lastStatePrint = 0;
bool secondImuActive = false;
BatteryMonitor battery;
TPSCounter tpsCounter;

void setup() {
	Serial.begin(serialBaudRate);

	waitForSerialConnection();

#if defined(ESP32)
	// The ESP-IDF 5.x based Arduino core used by some ESP32-S3 boards can be
	// very chatty (e.g. ChannelEngineSpi warnings about missing SPI hosts).
	// Lower the global IDF log verbosity so our own logs stay readable.
	esp_log_level_set("*", ESP_LOG_ERROR);
#endif

	delay(1000);
	
	Serial.println("=== SlimeVR Boot ===");
	Serial.println("Serial initialized successfully");
	Serial.flush(); // Force flush to ensure output is sent
	Serial.println();
	
	globalTimer = timer_create_default();

	logger.info("SlimeVR v" FIRMWARE_VERSION " starting up...");

	char vendorBuffer[512];
	size_t writtenLength;

	if (strlen(VENDOR_URL) == 0) {
		sprintf(
			vendorBuffer,
			"Vendor: %s, product: %s%n",
			VENDOR_NAME,
			PRODUCT_NAME,
			&writtenLength
		);
	} else {
		sprintf(
			vendorBuffer,
			"Vendor: %s (%s), product: %s%n",
			VENDOR_NAME,
			VENDOR_URL,
			PRODUCT_NAME,
			&writtenLength
		);
	}

	if (strlen(UPDATE_ADDRESS) > 0 && strlen(UPDATE_NAME) > 0) {
		sprintf(
			vendorBuffer + writtenLength,
			", firmware update url: %s, name: %s",
			UPDATE_ADDRESS,
			UPDATE_NAME
		);
	}
	logger.info("%s", vendorBuffer);

	statusManager.setStatus(SlimeVR::Status::LOADING, true);

	ledManager.setup();
	configuration.setup();

	SerialCommands::setUp();
	// Make sure the bus isn't stuck when resetting ESP without powering it down
	// Fixes I2C issues for certain IMUs. Previously this feature was enabled for
	// selected IMUs, now it's enabled for all. If some IMU turned out to be broken by
	// this, check needs to be re-added.
	// Skip clearBus - it's causing pin validation issues on ESP32-S3
	// auto clearResult = I2CSCAN::clearBus(PIN_IMU_SDA, PIN_IMU_SCL);
	// if (clearResult != 0) {
	// 	logger.warn("Can't clear I2C bus, error %d", clearResult);
	// }

	// join I2C bus
	logger.info("Initializing I2C on SDA=%d, SCL=%d", PIN_IMU_SDA, PIN_IMU_SCL);

#if defined(ESP32)
	// On ESP32-class boards, explicitly re-init the Wire bus with the desired pins
	Wire.end();
	Wire.begin(static_cast<int>(PIN_IMU_SDA), static_cast<int>(PIN_IMU_SCL));
	Wire.setTimeOut(150);
#else
	Wire.begin(static_cast<int>(PIN_IMU_SDA), static_cast<int>(PIN_IMU_SCL));
#endif

#ifdef ESP8266
	Wire.setClockStretchLimit(150000L);  // Default stretch limit 150mS
#endif
	Wire.setClock(I2C_SPEED);

	// Wait for IMU to boot
	delay(500);

	sensorManager.setup();

	networkManager.setup();
	OTA::otaSetup(otaPassword);
	battery.Setup();

	// Initialize magnetometer sidecar communication (for Magneto integration)
	SlimeVR::MagCalibration::g_magCalManager.setup();

	statusManager.setStatus(SlimeVR::Status::LOADING, false);

	sensorManager.postSetup();

	loopTime = micros();
	tpsCounter.reset();
}

void loop() {
	tpsCounter.update();
	globalTimer.tick();
	SerialCommands::update();
	OTA::otaUpdate();
	networkManager.update();
#if DEBUG_MEASURE_SENSOR_TIME_TAKEN
	sensorMeasurer.before();
#endif
	sensorManager.update();
#if DEBUG_MEASURE_SENSOR_TIME_TAKEN
	sensorMeasurer.after();
#endif

	battery.Loop();
	ledManager.update();

	// Update magnetometer sidecar communication (Magneto integration) on a
	// best-effort, rate-limited basis *after* IMU/FIFO processing, so any
	// WiFi/UDP stalls can't delay reading the sensor FIFO.
#if ENABLE_MAG_CALIBRATION_MODE
	{
		static uint32_t lastMagUpdateMs = 0;
		constexpr uint32_t MAG_UPDATE_INTERVAL_MS = 20;  // Max ~50 Hz
		uint32_t nowMs = millis();
		if (nowMs - lastMagUpdateMs >= MAG_UPDATE_INTERVAL_MS) {
			lastMagUpdateMs = nowMs;
			SlimeVR::MagCalibration::g_magCalManager.update();
		}
	}
#endif

	I2CSCAN::update();
#ifdef TARGET_LOOPTIME_MICROS
	long elapsed = (micros() - loopTime);
	if (elapsed < TARGET_LOOPTIME_MICROS) {
		long sleepus = TARGET_LOOPTIME_MICROS - elapsed - 100;  // µs to sleep
		long sleepms = sleepus / 1000;  // ms to sleep
		if (sleepms > 0)  // if >= 1 ms
		{
			delay(sleepms);  // sleep ms = save power
			sleepus -= sleepms * 1000;
		}
		if (sleepus > 100) {
			delayMicroseconds(sleepus);
		}
	}
	loopTime = micros();
#endif
#if defined(PRINT_STATE_EVERY_MS) && PRINT_STATE_EVERY_MS > 0
	unsigned long now = millis();
	if (lastStatePrint + PRINT_STATE_EVERY_MS < now) {
		lastStatePrint = now;
		SerialCommands::printState();
	}
#endif
}
