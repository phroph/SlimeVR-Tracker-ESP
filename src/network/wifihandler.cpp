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
#include "network/wifihandler.h"

#include "GlobalVars.h"
#include "globals.h"
#if !ESP8266
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#endif

namespace SlimeVR {

void WiFiNetwork::reportWifiProgress() {
	if (lastWifiReportTime + 1000 < millis()) {
		lastWifiReportTime = millis();
		Serial.print(".");
	}
}

void WiFiNetwork::setStaticIPIfDefined() {
#ifdef WIFI_USE_STATICIP
	const IPAddress ip(WIFI_STATIC_IP);
	const IPAddress gateway(WIFI_STATIC_GATEWAY);
	const IPAddress subnet(WIFI_STATIC_SUBNET);
	WiFi.config(ip, gateway, subnet);
#endif
}

bool WiFiNetwork::isConnected() const {
	return wifiState == WiFiReconnectionStatus::Success;
}

void WiFiNetwork::setWiFiCredentials(const char* SSID, const char* pass) {
	wifiProvisioning.stopProvisioning();
	tryConnecting(false, SSID, pass);
	retriedOnG = false;
	// Reset state, will get back into provisioning if can't connect
	hadWifi = false;
	wifiState = WiFiReconnectionStatus::ServerCredAttempt;
}

IPAddress WiFiNetwork::getAddress() { return WiFi.localIP(); }

void WiFiNetwork::setUp() {
	wifiHandlerLogger.info("Setting up WiFi");
#if ESP32S3
    WiFi.setAutoReconnect(false);  // we control reconnects ourselves
	WiFi.disconnect(true, false);
#endif
	WiFi.persistent(true);
	WiFi.mode(WIFI_STA);
	WiFi.hostname("SlimeVR FBT Tracker");
	wifiHandlerLogger.info(
		"Loaded credentials for SSID '%s' and pass length %d",
		getSSID().c_str(),
		getPassword().length()
	);

	wifiState = WiFiReconnectionStatus::NotSetup;
    hadWifi = false;
    retriedOnG = false;
    wifiConnectionTimeout = millis();

#if ESP8266
#if POWERSAVING_MODE == POWER_SAVING_NONE
	WiFi.setSleepMode(WIFI_NONE_SLEEP);
#elif POWERSAVING_MODE == POWER_SAVING_MINIMUM
	WiFi.setSleepMode(WIFI_MODEM_SLEEP);
#elif POWERSAVING_MODE == POWER_SAVING_MODERATE
	WiFi.setSleepMode(WIFI_MODEM_SLEEP, 10);
#elif POWERSAVING_MODE == POWER_SAVING_MAXIMUM
	WiFi.setSleepMode(WIFI_LIGHT_SLEEP, 10);
#error "MAX POWER SAVING NOT WORKING YET, please disable!"
#endif
#else
#if POWERSAVING_MODE == POWER_SAVING_NONE
	WiFi.setSleep(WIFI_PS_NONE);
#elif POWERSAVING_MODE == POWER_SAVING_MINIMUM
	WiFi.setSleep(WIFI_PS_MIN_MODEM);
#elif POWERSAVING_MODE == POWER_SAVING_MODERATE \
	|| POWERSAVING_MODE == POWER_SAVING_MAXIMUM
	wifi_config_t conf;
	if (esp_wifi_get_config(WIFI_IF_STA, &conf) == ESP_OK) {
		conf.sta.listen_interval = 10;
		esp_wifi_set_config(WIFI_IF_STA, &conf);
		WiFi.setSleep(WIFI_PS_MAX_MODEM);
	} else {
		wifiHandlerLogger.error("Unable to get WiFi config, power saving not enabled!");
	}
#endif
#endif
}

void WiFiNetwork::onConnected() {
	wifiState = WiFiReconnectionStatus::Success;
	wifiProvisioning.stopProvisioning();
	statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, false);
	hadWifi = true;
	wifiHandlerLogger.info(
		"Connected successfully to SSID '%s', IP address %s",
		getSSID().c_str(),
		WiFi.localIP().toString().c_str()
	);
	// Reset it, in case we just connected with server creds
}

String WiFiNetwork::getSSID() {
#if ESP8266
	return WiFi.SSID();
#else
	// Necessary, because without a WiFi.begin(), ESP32 is not kind enough to load the
	// SSID on its own, for whatever reason
	wifi_config_t wifiConfig;
	esp_wifi_get_config((wifi_interface_t)ESP_IF_WIFI_STA, &wifiConfig);
	return {reinterpret_cast<char*>(wifiConfig.sta.ssid)};
#endif
}

String WiFiNetwork::getPassword() {
#if ESP8266
	return WiFi.psk();
#else
	// Same as above
	wifi_config_t wifiConfig;
	esp_wifi_get_config((wifi_interface_t)ESP_IF_WIFI_STA, &wifiConfig);
	return {reinterpret_cast<char*>(wifiConfig.sta.password)};
#endif
}

WiFiNetwork::WiFiReconnectionStatus WiFiNetwork::getWiFiState() { return wifiState; }

void WiFiNetwork::upkeep() {
	wifiProvisioning.upkeepProvisioning();

	const auto now = millis();
	const auto status = WiFi.status();

	// 1. Physically connected: keep logical state in sync and send RSSI
	if (status == WL_CONNECTED) {
		if (!isConnected()) {
			static uint32_t connectedAt = millis();
			if (millis() - connectedAt < WiFiGraceAfterConnectMs) {
				return;
			}
			onConnected();  // sets wifiState = Success
		}

		if (now - lastRssiSample >= 2000) {
			lastRssiSample = now;
			uint8_t signalStrength = WiFi.RSSI();
			networkConnection.sendSignalStrength(signalStrength);
		}
		return;
	}

	// 2. We *were* connected (state machine says Success) but link is now lost
	if (wifiState == WiFiReconnectionStatus::Success) {
		statusManager.setStatus(SlimeVR::Status::WIFI_CONNECTING, true);
		wifiHandlerLogger.warn(
			"Connection to WiFi lost (wl_status=%d), restarting WiFi state machine",
			static_cast<int>(status)
		);
		wifiState = WiFiReconnectionStatus::NotSetup;
		retriedOnG = false;
		hadWifi = true;
		return;
	}

	// 3. First run: kick off initial attempt from upkeep, not from setUp()
	if (wifiState == WiFiReconnectionStatus::NotSetup) {
		wifiHandlerLogger.debug("Initial WiFi connect using saved credentials");
		trySavedCredentials();  // sets wifiState = SavedAttempt and wifiConnectionTimeout
		return;
	}

	// 4. While an attempt is in progress, wait up to WiFiTimeoutSeconds unless we connect
	if (wifiState == WiFiReconnectionStatus::SavedAttempt
		|| wifiState == WiFiReconnectionStatus::HardcodeAttempt
		|| wifiState == WiFiReconnectionStatus::ServerCredAttempt) {

		const uint32_t timeoutMs
			= static_cast<uint32_t>(WiFiTimeoutSeconds * 1000);

		if ((now - wifiConnectionTimeout) < timeoutMs && status != WL_CONNECTED) {
			// Still within this attempt's timeout window and not connected yet
			reportWifiProgress();
			return;
		}
	}

	// 5. If we got here, the current attempt either:
	//    - timed out (now - wifiConnectionTimeout >= timeoutMs), or
	//    - finished with some final non-connected status.
	switch (wifiState) {
		case WiFiReconnectionStatus::SavedAttempt:
			// Try saved credentials again (G-mode fallback on ESP8266),
			// otherwise move on to hardcoded.
			if (!trySavedCredentials()) {
				tryHardcodedCredentials();
			}
			break;

		case WiFiReconnectionStatus::HardcodeAttempt:
			// Try hardcoded credentials (incl. G-mode on ESP8266 once),
			// otherwise mark as failed.
			if (!tryHardcodedCredentials()) {
				wifiState = WiFiReconnectionStatus::Failed;
			}
			break;

		case WiFiReconnectionStatus::ServerCredAttempt:
			if (!tryServerCredentials()) {
				wifiState = WiFiReconnectionStatus::Failed;
			}
			break;

		case WiFiReconnectionStatus::Failed: {
			const uint32_t timeoutMs
				= static_cast<uint32_t>(WiFiTimeoutSeconds * 1000);

			// All credential paths failed: optionally fall back to SmartConfig
			if (!hadWifi && !WiFi.smartConfigDone()
				&& (now - wifiConnectionTimeout) >= timeoutMs) {
				wifiHandlerLogger.error(
					"Can't connect from any credentials, last wl_status=%d (%s).",
					static_cast<int>(status),
					statusToReasonString(status)
				);
				wifiConnectionTimeout = now;
				wifiProvisioning.startProvisioning();
			}
			break;
		}

		case WiFiReconnectionStatus::NotSetup:
		case WiFiReconnectionStatus::Success:
			// Should have been handled in earlier branches
			break;
	}
}

const char* WiFiNetwork::statusToReasonString(wl_status_t status) {
	switch (status) {
		case WL_DISCONNECTED:
			return "Timeout";
#ifdef ESP8266
		case WL_WRONG_PASSWORD:
			return "Wrong password";
		case WL_CONNECT_FAILED:
			return "Connection failed";
#elif ESP32S3
		case WL_CONNECT_FAILED:
			return "Wrong password";
#endif
        case WL_SCAN_COMPLETED:return "Scan completed";
        case WL_CONNECTED:     return "Connected";
		case WL_NO_SSID_AVAIL:
			return "SSID not found";
		default:
			return "Unknown";
	}
}

WiFiNetwork::WiFiFailureReason WiFiNetwork::statusToFailure(wl_status_t status) {
	switch (status) {
		case WL_DISCONNECTED:
			return WiFiFailureReason::Timeout;
#ifdef ESP8266
		case WL_WRONG_PASSWORD:
			return WiFiFailureReason::WrongPassword;
#elif ESP32S3
		case WL_CONNECT_FAILED:
			return WiFiFailureReason::WrongPassword;
#endif

		case WL_NO_SSID_AVAIL:
			return WiFiFailureReason::SSIDNotFound;
		default:
			return WiFiFailureReason::Unknown;
	}
}

void WiFiNetwork::showConnectionAttemptFailed(const char* type) const {
	auto status = WiFi.status();
	wifiHandlerLogger.error(
		"Can't connect from %s credentials, wl_status=%d (%s).",
		type,
		static_cast<int>(status),
		statusToReasonString(status)
	);
}

bool WiFiNetwork::trySavedCredentials() {
	if (getSSID().length() == 0) {
		wifiHandlerLogger.debug("Skipping saved credentials attempt on 0-length SSID..."
		);
		wifiState = WiFiReconnectionStatus::HardcodeAttempt;
		return false;
	}

	if (wifiState == WiFiReconnectionStatus::SavedAttempt) {
		showConnectionAttemptFailed("saved");

		if (WiFi.status() != WL_DISCONNECTED) {
			return false;
		}

		if (retriedOnG) {
			return false;
		}

		retriedOnG = true;
		wifiHandlerLogger.debug("Trying saved credentials with PHY Mode G...");
		return tryConnecting(true);
	}

	retriedOnG = false;

	wifiState = WiFiReconnectionStatus::SavedAttempt;
	return tryConnecting();
}

bool WiFiNetwork::tryHardcodedCredentials() {
#if defined(WIFI_CREDS_SSID) && defined(WIFI_CREDS_PASSWD)
	if (wifiState == WiFiReconnectionStatus::HardcodeAttempt) {
		showConnectionAttemptFailed("hardcoded");

		if (WiFi.status() != WL_DISCONNECTED) {
			return false;
		}

		if (retriedOnG) {
			return false;
		}

		retriedOnG = true;
		wifiHandlerLogger.debug("Trying hardcoded credentials with PHY Mode G...");
		// Don't need to save hardcoded credentials
		WiFi.persistent(false);
		auto result = tryConnecting(true, WIFI_CREDS_SSID, WIFI_CREDS_PASSWD);
		WiFi.persistent(true);
		return result;
	}

	retriedOnG = false;

	wifiState = WiFiReconnectionStatus::HardcodeAttempt;
	// Don't need to save hardcoded credentials
	WiFi.persistent(false);
	auto result = tryConnecting(false, WIFI_CREDS_SSID, WIFI_CREDS_PASSWD);
	WiFi.persistent(true);
	return result;
#else
	wifiState = WiFiReconnectionStatus::HardcodeAttempt;
	return false;
#endif
}

bool WiFiNetwork::tryServerCredentials() {
	if (WiFi.status() != WL_DISCONNECTED) {
		return false;
	}

	if (retriedOnG) {
		return false;
	}

	retriedOnG = true;

	return tryConnecting(true);
}

bool WiFiNetwork::tryConnecting(bool phyModeG, const char* SSID, const char* pass) {
#if ESP8266
	if (phyModeG) {
		WiFi.setPhyMode(WIFI_PHY_MODE_11G);
		if constexpr (USE_ATTENUATION) {
			WiFi.setOutputPower(20.0 - ATTENUATION_G);
		}
	} else {
		WiFi.setPhyMode(WIFI_PHY_MODE_11N);
		if constexpr (USE_ATTENUATION) {
			WiFi.setOutputPower(20.0 - ATTENUATION_N);
		}
	}
#else
	if (phyModeG) {
		return false;
	}
#endif

	setStaticIPIfDefined();
	if (SSID == nullptr) {
		WiFi.begin();
	} else {
		WiFi.begin(SSID, pass);
	}
	wifiConnectionTimeout = millis();
	return true;
}

}  // namespace SlimeVR
