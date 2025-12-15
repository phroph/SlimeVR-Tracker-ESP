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

#include "mag_calibration_manager.h"

#include <Arduino.h>
#include <WiFi.h>
#include <cstring>
#include <cmath>

#include "GlobalVars.h"
#include "status/StatusManager.h"

namespace SlimeVR::MagCalibration {

// Global instance
MagCalibrationManager g_magCalManager;

MagCalibrationManager::MagCalibrationManager()
	: m_sidecarIP(0, 0, 0, 0)  // Will be set from server IP or config
	, m_sidecarIPConfigured(false)
{
	// Generate tracker ID from MAC address
	uint8_t mac[6];
	WiFi.macAddress(mac);
	snprintf(m_trackerId, sizeof(m_trackerId), "tracker_%02x%02x%02x%02x%02x%02x",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void MagCalibrationManager::setup() {
	// Load existing calibration
	magCalLoad(m_calConfig);

	if (m_calConfig.valid) {
		m_logger.info("Mag calibration loaded: valid=true");
	} else {
		m_logger.info("No valid mag calibration found");
	}

	// Initialize UDP receiver (can be called before WiFi is connected)
	if (!m_udp.begin(7779)) {  // Listen on port 7779
		m_logger.error("Failed to start UDP receiver for mag calibration");
	} else {
		m_logger.info("Mag calibration UDP receiver started on port 7779");
	}

	// Load sidecar IP: use MAG_SIDECAR_IP if defined, otherwise use SlimeVR server IP
	#ifdef MAG_SIDECAR_IP
		if (m_sidecarIP.fromString(MAG_SIDECAR_IP)) {
			m_sidecarIPConfigured = true;
			m_logger.info("Mag sidecar IP configured from MAG_SIDECAR_IP: %s", m_sidecarIP.toString().c_str());
		} else {
			m_logger.warn("Failed to parse MAG_SIDECAR_IP, will use server IP");
		}
	#endif
	// If not explicitly configured, will use server IP (updated in update())

	// Check if we should enter calibration mode on boot. This decision is based
	// only on local state (no valid calibration + boot button held) and does
	// NOT depend on WiFi being connected, so that the user gets immediate
	// feedback (LED + logging) even before networking is up.
	if (shouldEnterCalibrationMode()) {
		m_logger.info("Entering mag calibration mode on boot");
		enterCalibrationMode();
	}
}

void MagCalibrationManager::update() {
	// Only process if WiFi is connected
	if (WiFi.status() != WL_CONNECTED) {
		return;
	}

	// If we've detected that mag sidecar network calls are too slow, completely
	// disable further processing here to ensure we never impact the IMU/FIFO
	// loop. The tracker will continue to function without mag sidecar support.
	if (m_networkDisabled) {
		return;
	}

	// Handle runtime button presses for starting / stopping calibration.
	updateButton();

	// Update sidecar IP from server IP if not explicitly configured
	if (!m_sidecarIPConfigured) {
		IPAddress serverIP = networkConnection.getServerHost();
		if (serverIP != IPAddress(0, 0, 0, 0) && serverIP != IPAddress(255, 255, 255, 255)) {
			if (m_sidecarIP != serverIP) {
				m_sidecarIP = serverIP;
				m_helloSent = false;  // Need to re-send hello to new IP
				m_logger.info("Mag sidecar IP set to server IP: %s", m_sidecarIP.toString().c_str());
			}
		}
	}

	// Send hello message to register with sidecar (required by Magneto protocol)
	// Must be sent before telemetry will be accepted
	if (!m_helloSent && m_sidecarIP != IPAddress(0, 0, 0, 0)) {
		sendHello();
	}

	// Check if we should enter calibration mode (first time WiFi connects)
	if (m_state == MagCalState::IDLE && shouldEnterCalibrationMode()) {
		m_logger.info("Entering mag calibration mode (WiFi connected)");
		enterCalibrationMode();
	}

	// Drain queued samples and telemetry (non-blocking, batched)
	drainCalibSampleQueue();
	drainTelemetryQueue();

	receiveAndProcessPackets();

	if (m_state != MagCalState::IDLE && m_state != MagCalState::COMPLETE) {
		updateStateMachine();
	}
}

bool MagCalibrationManager::shouldEnterCalibrationMode() const {
#if ENABLE_MAG_CALIBRATION_MODE
	// Enter calibration mode at boot if the dedicated button is held down.
	// This is treated as an explicit user request and takes precedence over
	// any existing saved calibration.
	//
	// Runtime re-calibration is still available via long-press handling in
	// updateButton(), but holding the button during boot will always force
	// a calibration run for this session.
	if (isMagCalButtonHeldOnBoot()) {
		return true;
	}
#endif
	return false;
}

void MagCalibrationManager::enterCalibrationMode() {
#if ENABLE_MAG_CALIBRATION_MODE
	m_logger.info("Entering mag calibration mode");
	m_state = MagCalState::COLLECTING;
	m_collectionStartTime = millis();
	m_samplesSent = 0;
	m_lastSampleTime = 0;

	// Clear any leftover queued samples from a previous run so that each
	// calibration session starts with a clean buffer and sample counter.
	m_calibQueueHead = 0;
	m_calibQueueTail = 0;
	m_calibQueueSize = 0;

	// Notify the centralized status manager that mag calibration is active.
	// The LED will show the appropriate pattern based on priority.
	statusManager.setMagCalibrating(true);

	m_logger.info("Mag calibration mode: collecting samples...");
#endif
}

void MagCalibrationManager::updateButton() {
#ifdef MAG_CAL_BUTTON_PIN
	uint32_t nowMs = millis();

	if (!m_buttonInitialized) {
		pinMode(MAG_CAL_BUTTON_PIN, INPUT_PULLUP);
		m_buttonLastLevel = digitalRead(MAG_CAL_BUTTON_PIN);
		m_buttonLastChangeMs = nowMs;
		m_buttonInitialized = true;
	}

	bool level = digitalRead(MAG_CAL_BUTTON_PIN);
	if (level != m_buttonLastLevel) {
		// Debounce
		if (nowMs - m_buttonLastChangeMs >= BUTTON_DEBOUNCE_MS) {
			m_buttonLastChangeMs = nowMs;
			m_buttonLastLevel = level;

			// Active-low button: LOW = pressed, HIGH = released
			if (level == LOW) {
				// Button press start
				m_buttonPressStartMs = nowMs;
			} else {
				// Button released
				if (m_buttonPressStartMs != 0) {
					uint32_t pressDuration = nowMs - m_buttonPressStartMs;
					m_buttonPressStartMs = 0;

					// Long press: start a new calibration run (from IDLE / COMPLETE / ERROR)
					if (pressDuration >= BUTTON_LONG_PRESS_MS) {
						if (m_state == MagCalState::IDLE
							|| m_state == MagCalState::COMPLETE
							|| m_state == MagCalState::ERROR) {
							m_logger.info(
								"Mag cal button long-press detected, starting "
								"magnetometer calibration"
							);
							resetCalibration();
							enterCalibrationMode();
						}
					}
					// Short press during COLLECTING: finish sample collection early
					else if (
						pressDuration >= BUTTON_SHORT_PRESS_MS
						&& m_state == MagCalState::COLLECTING
					) {
						m_logger.info(
							"Mag cal button short-press detected, finishing "
							"magnetometer calibration sample collection"
						);
						sendCalibDone();
						m_state = MagCalState::WAITING_FOR_RESULT;
					}
				}
			}
		}
	}
#endif
}

void MagCalibrationManager::applyYawBias(const float q_vqf[4], float q_final[4]) const {
	// Only apply yaw bias if:
	// 1. Yaw consensus is enabled at compile time
	// 2. We have received at least one valid yaw_update from the sidecar
	// 3. The bias is non-zero (within epsilon)
#ifndef ENABLE_YAW_CONSENSUS
	// Yaw consensus disabled at compile time, pass through
	memcpy(q_final, q_vqf, 4 * sizeof(float));
	return;
#endif

	if (!m_yawBiasValid || fabs(m_yawBiasRad) < 1e-6f) {
		// No valid bias or zero bias, pass through
		memcpy(q_final, q_vqf, 4 * sizeof(float));
		return;
	}

	// Sidecar sends yawBiasRad = angle (tracker→global).
	// Tracker applies rotation of −yawBiasRad around world-up to correct its heading.
	// World-up in world frame is [0, 1, 0]
	float halfAngle = -m_yawBiasRad * 0.5f;
	float c = cosf(halfAngle);
	float s = sinf(halfAngle);

	// Quaternion representing rotation around Y axis: [c, 0, s, 0] (w, x, y, z)
	float q_yawCorr[4] = {c, 0.0f, s, 0.0f};

	// Multiply: q_final = q_yawCorr * q_vqf
	q_final[0] = q_yawCorr[0] * q_vqf[0] - q_yawCorr[1] * q_vqf[1] - q_yawCorr[2] * q_vqf[2] - q_yawCorr[3] * q_vqf[3];
	q_final[1] = q_yawCorr[0] * q_vqf[1] + q_yawCorr[1] * q_vqf[0] + q_yawCorr[2] * q_vqf[3] - q_yawCorr[3] * q_vqf[2];
	q_final[2] = q_yawCorr[0] * q_vqf[2] - q_yawCorr[1] * q_vqf[3] + q_yawCorr[2] * q_vqf[0] + q_yawCorr[3] * q_vqf[1];
	q_final[3] = q_yawCorr[0] * q_vqf[3] + q_yawCorr[1] * q_vqf[2] - q_yawCorr[2] * q_vqf[1] + q_yawCorr[3] * q_vqf[0];
}

void MagCalibrationManager::sendTelemetry(const float quat[4], const float magCal[3], float magQuality) {
	// Queue telemetry instead of sending immediately (non-blocking)
	// This is called from IMU read path, so we must not block
	if (m_state == MagCalState::IDLE || m_state == MagCalState::COMPLETE) {
		// Check if queue has space
		if (m_telemetryQueueSize < MAX_QUEUED_TELEMETRY) {
			QueuedTelemetry& entry = m_telemetryQueue[m_telemetryQueueTail];
			memcpy(entry.quat, quat, 4 * sizeof(float));
			memcpy(entry.magCal, magCal, 3 * sizeof(float));
			entry.magQuality = magQuality;
			entry.timestamp = millis();
			m_telemetryQueueTail = (m_telemetryQueueTail + 1) % MAX_QUEUED_TELEMETRY;
			m_telemetryQueueSize++;
		} else {
			// Queue full - drop oldest entry (FIFO)
			m_telemetryQueueHead = (m_telemetryQueueHead + 1) % MAX_QUEUED_TELEMETRY;
			// Now add new entry
			QueuedTelemetry& entry = m_telemetryQueue[m_telemetryQueueTail];
			memcpy(entry.quat, quat, 4 * sizeof(float));
			memcpy(entry.magCal, magCal, 3 * sizeof(float));
			entry.magQuality = magQuality;
			entry.timestamp = millis();
			m_telemetryQueueTail = (m_telemetryQueueTail + 1) % MAX_QUEUED_TELEMETRY;
			// Size stays the same (we dropped one, added one)
		}
	}
}

void MagCalibrationManager::feedMagSample(float mx, float my, float mz) {
	// Queue sample instead of sending immediately (non-blocking)
	// This is called from IMU read path, so we must not block
	if (m_state != MagCalState::COLLECTING) {
		// During normal operation this is expected (we continuously feed raw mag
		// samples but only record them when a calibration run is active). To
		// help debug cases where no samples ever reach the sidecar, log a
		// single warning the first time we see mag samples while not in
		// COLLECTING, then stay quiet to avoid spam.
		static bool warnedOnce = false;
		if (!warnedOnce) {
			m_logger.warn(
				"feedMagSample called while not COLLECTING (state=%d); "
				"mag data will not be recorded for calibration",
				static_cast<int>(m_state)
			);
			warnedOnce = true;
		}
		return;
	}

	uint32_t now = millis();
	uint32_t elapsed = now - m_lastSampleTime;
	if (elapsed >= SAMPLE_INTERVAL_MS) {
		m_lastSampleTime = now;
		
		// Queue sample instead of sending
		if (m_calibQueueSize < MAX_QUEUED_CALIB_SAMPLES) {
			QueuedCalibSample& sample = m_calibSampleQueue[m_calibQueueTail];
			sample.mx = mx;
			sample.my = my;
			sample.mz = mz;
			sample.timestamp = now;
			m_calibQueueTail = (m_calibQueueTail + 1) % MAX_QUEUED_CALIB_SAMPLES;
			m_calibQueueSize++;
		} else {
			// Queue full - drop sample. This can happen if the IMU loop is
			// producing samples faster than we can drain them over WiFi (for
			// example on slower networks). To avoid log spam, rate-limit the
			// warning instead of printing every drop.
			static uint32_t droppedCount = 0;
			if ((droppedCount++ % 50u) == 0u) {
				m_logger.warn("Mag sample queue full, dropping sample");
			}
		}
	}
}

void MagCalibrationManager::resetCalibration() {
	m_calConfig = MagCalibrationConfig();
	magCalSave(m_calConfig);  // Save invalid calibration to clear it
	m_logger.info("Mag calibration reset");
}

void MagCalibrationManager::startCalibrationManual() {
#if ENABLE_MAG_CALIBRATION_MODE
	// Mimic long-press behaviour: clear existing calibration and start a new run
	m_logger.info("Mag calibration: manual START requested (serial)");
	resetCalibration();
	enterCalibrationMode();
#else
	m_logger.warn("Mag calibration: manual START requested but ENABLE_MAG_CALIBRATION_MODE=0");
#endif
}

void MagCalibrationManager::finishCalibrationManual() {
#if ENABLE_MAG_CALIBRATION_MODE
	if (m_state == MagCalState::COLLECTING) {
		m_logger.info("Mag calibration: manual STOP requested (serial), finishing sample collection");
		sendCalibDone();
		m_state = MagCalState::WAITING_FOR_RESULT;
	} else {
		m_logger.warn(
			"Mag calibration: manual STOP requested (serial) but not in COLLECTING state (state=%d)",
			static_cast<int>(m_state)
		);
	}
#else
	m_logger.warn("Mag calibration: manual STOP requested but ENABLE_MAG_CALIBRATION_MODE=0");
#endif
}

void MagCalibrationManager::sendHello() {
	if (m_networkDisabled) {
		return;
	}
	// Send hello message to register with sidecar
	// Required by Magneto protocol before sending telemetry
	char buffer[256];
	snprintf(buffer, sizeof(buffer),
		"{\"type\":\"hello\",\"trackerId\":\"%s\",\"fwVersion\":\"" FIRMWARE_VERSION "\",\"supportsMagCalibration\":true,\"supportsYawCorrection\":true}",
		m_trackerId);

	m_udp.beginPacket(m_sidecarIP, m_sidecarPort);
	m_udp.write((uint8_t*)buffer, strlen(buffer));
	m_udp.endPacket();

	m_helloSent = true;
	m_logger.info("Sent hello to sidecar at %s:%d (trackerId: %s)", 
		m_sidecarIP.toString().c_str(), m_sidecarPort, m_trackerId);
}

void MagCalibrationManager::sendCalibSample(float mx, float my, float mz, uint32_t timestamp) {
	if (m_networkDisabled) {
		return;
	}
	char buffer[256];
	snprintf(buffer, sizeof(buffer),
		"{\"type\":\"calib_sample\",\"trackerId\":\"%s\",\"timestamp\":%u,\"mx\":%.6f,\"my\":%.6f,\"mz\":%.6f}",
		m_trackerId, timestamp, mx, my, mz);

	m_udp.beginPacket(m_sidecarIP, m_sidecarPort);
	m_udp.write((uint8_t*)buffer, strlen(buffer));
	m_udp.endPacket();
}

void MagCalibrationManager::sendCalibDone() {
	if (m_networkDisabled) {
		// We previously detected that UDP calls were too slow and disabled mag
		// sidecar networking for this boot. Log explicitly so it's clear why no
		// calib_done is going out even though the user requested STOP.
		m_logger.warn(
			"Not sending calib_done: mag sidecar networking is disabled for "
			"this session (maxNetworkCallTimeUs=%u)",
			m_maxNetworkCallTimeUs
		);
		return;
	}
	char buffer[128];
	snprintf(buffer, sizeof(buffer),
		"{\"type\":\"calib_done\",\"trackerId\":\"%s\"}",
		m_trackerId);

	m_udp.beginPacket(m_sidecarIP, m_sidecarPort);
	m_udp.write((uint8_t*)buffer, strlen(buffer));
	m_udp.endPacket();

	m_logger.info(
		"Sent calib_done to sidecar (samplesSent=%u, queuedSamples=%u)",
		m_samplesSent,
		static_cast<unsigned int>(m_calibQueueSize)
	);
}

void MagCalibrationManager::sendTelemetryPacket(const float quat[4], const float magCal[3], float magQuality, uint32_t timestamp) {
	if (m_networkDisabled) {
		return;
	}
	char buffer[512];
	snprintf(buffer, sizeof(buffer),
		"{\"type\":\"telemetry\",\"trackerId\":\"%s\",\"timestamp\":%u,\"quat\":[%.6f,%.6f,%.6f,%.6f],\"magCal\":[%.6f,%.6f,%.6f],\"magQuality\":%.3f}",
		m_trackerId, timestamp,
		quat[0], quat[1], quat[2], quat[3],
		magCal[0], magCal[1], magCal[2],
		magQuality);

	m_udp.beginPacket(m_sidecarIP, m_sidecarPort);
	m_udp.write((uint8_t*)buffer, strlen(buffer));
	m_udp.endPacket();
}

void MagCalibrationManager::receiveAndProcessPackets() {
	if (m_networkDisabled) {
		return;
	}
	int packetSize = m_udp.parsePacket();
	if (packetSize <= 0) {
		return;
	}

	char buffer[512];
	int len = m_udp.read(buffer, sizeof(buffer) - 1);
	if (len <= 0) {
		return;
	}
	buffer[len] = '\0';

	// Simple JSON parsing: look for "type" field
	if (strstr(buffer, "\"type\":\"calib_result\"") != nullptr) {
		handleCalibResult(buffer, len);
	} else if (strstr(buffer, "\"type\":\"yaw_update\"") != nullptr) {
#ifdef ENABLE_MAG_CALIBRATION_MODE
		handleYawUpdate(buffer, len);
#endif
	}
}

void MagCalibrationManager::handleCalibResult(const char* json, size_t len) {
	// Simple JSON parsing (no external library)
	// Look for: "success":true, "offset":[bx,by,bz], "matrix":[[...]], "fieldStrength":value

	if (m_state != MagCalState::WAITING_FOR_RESULT) {
		return;  // Not expecting a result
	}

	// Check for success
	if (strstr(json, "\"success\":true") == nullptr) {
		m_logger.error("Calibration result: success=false");
		m_state = MagCalState::ERROR;
		return;
	}

	// Parse offset [bx, by, bz]
	float offset[3] = {0.0f, 0.0f, 0.0f};
	const char* offsetStart = strstr(json, "\"offset\":[");
	if (offsetStart) {
		sscanf(offsetStart, "\"offset\":[%f,%f,%f]", &offset[0], &offset[1], &offset[2]);
	}

	// Parse matrix [[a11,a12,a13],[a21,a22,a23],[a31,a32,a33]]
	float matrix[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
	const char* matrixStart = strstr(json, "\"matrix\":[[");
	if (matrixStart) {
		sscanf(matrixStart,
			"\"matrix\":[[%f,%f,%f],[%f,%f,%f],[%f,%f,%f]]",
			&matrix[0][0], &matrix[0][1], &matrix[0][2],
			&matrix[1][0], &matrix[1][1], &matrix[1][2],
			&matrix[2][0], &matrix[2][1], &matrix[2][2]);
	}

	// Parse fieldStrength
	float fieldStrength = 0.0f;
	const char* fsStart = strstr(json, "\"fieldStrength\":");
	if (fsStart) {
		sscanf(fsStart, "\"fieldStrength\":%f", &fieldStrength);
	}

	// Fill calibration config
	memcpy(m_calConfig.offset, offset, sizeof(offset));
	memcpy(m_calConfig.matrix, matrix, sizeof(matrix));
	m_calConfig.fieldStrength = fieldStrength;
	m_calConfig.version = MAG_CAL_VERSION;
	m_calConfig.valid = true;

	// Save to flash
	if (magCalSave(m_calConfig)) {
		m_logger.info("Mag calibration saved successfully");
		m_state = MagCalState::COMPLETE;
		// Calibration finished - notify status manager
		statusManager.setMagCalibrating(false);
	} else {
		m_logger.error("Failed to save mag calibration");
		m_state = MagCalState::ERROR;
		statusManager.setMagCalibrating(false);
	}
}

void MagCalibrationManager::handleYawUpdate(const char* json, size_t len) {
#ifndef ENABLE_MAG_CALIBRATION_MODE
	// Mag sidecar support disabled
	return;
#endif

	// Parse trackerId first to check if this message is for us
	const char* trackerIdStart = strstr(json, "\"trackerId\":\"");
	if (!trackerIdStart) {
		m_logger.warn("Yaw update missing trackerId, ignoring");
		return;
	}

	// Extract trackerId from JSON (simple parsing: "trackerId":"value")
	char msgTrackerId[32] = {0};
	if (sscanf(trackerIdStart, "\"trackerId\":\"%31[^\"]\"", msgTrackerId) != 1) {
		m_logger.warn("Yaw update: failed to parse trackerId, ignoring");
		return;
	}

	// Check if this message is for this tracker
	if (strcmp(msgTrackerId, m_trackerId) != 0) {
		// Not for us, ignore silently
		return;
	}

	// Parse yawBiasRad
	const char* yawStart = strstr(json, "\"yawBiasRad\":");
	if (!yawStart) {
		m_logger.warn("Yaw update missing yawBiasRad, ignoring");
		return;
	}

	float yawBias = 0.0f;
	if (sscanf(yawStart, "\"yawBiasRad\":%f", &yawBias) != 1) {
		m_logger.warn("Yaw update: failed to parse yawBiasRad, ignoring");
		return;
	}

	// Parse valid flag
	bool valid = false;
	const char* validStart = strstr(json, "\"valid\":");
	if (validStart) {
		if (strstr(validStart, "\"valid\":true") != nullptr) {
			valid = true;
		} else if (strstr(validStart, "\"valid\":false") != nullptr) {
			valid = false;
		}
	}

	if (!valid) {
		// Invalid yaw update: clear the bias
		m_yawBiasRad = 0.0f;
		m_yawBiasValid = false;
		m_logger.debug("Yaw update: valid=false, clearing yaw bias");
		return;
	}

	// Clamp to safety limits
	if (yawBias > MAX_YAW_BIAS_RAD) {
		yawBias = MAX_YAW_BIAS_RAD;
		m_logger.warn("Yaw bias clamped to max: %.6f rad (%.2f deg)", yawBias, yawBias * 180.0f / 3.14159265359f);
	} else if (yawBias < -MAX_YAW_BIAS_RAD) {
		yawBias = -MAX_YAW_BIAS_RAD;
		m_logger.warn("Yaw bias clamped to min: %.6f rad (%.2f deg)", yawBias, yawBias * 180.0f / 3.14159265359f);
	}

	m_yawBiasRad = yawBias;
	m_yawBiasValid = true;
	m_logger.debug("Yaw bias updated: %.6f rad (%.2f deg)", m_yawBiasRad, m_yawBiasRad * 180.0f / 3.14159265359f);
}

void MagCalibrationManager::updateStateMachine() {
	uint32_t now = millis();

	switch (m_state) {
		case MagCalState::COLLECTING: {
			// Check timeout
			if (now - m_collectionStartTime > COLLECTION_TIMEOUT_MS) {
				m_logger.warn("Calibration collection timeout");
				sendCalibDone();
				m_state = MagCalState::WAITING_FOR_RESULT;
				break;
			}

			// If we've collected a large, high-quality dataset, auto-submit to the
			// sidecar. This gives us an effective target range of
			// [MIN_SAMPLES, MAX_SAMPLES] samples, but still allows the user to
			// finish early via STOP.
			if (m_samplesSent >= MAX_SAMPLES) {
				m_logger.info(
					"Collected %u samples (>= MAX_SAMPLES=%u), auto-submitting calib_done",
					m_samplesSent,
					MAX_SAMPLES
				);
				sendCalibDone();
				m_state = MagCalState::WAITING_FOR_RESULT;
				break;
			}

			// Send samples at fixed rate (handled by caller)
			break;
		}

		case MagCalState::WAITING_FOR_RESULT: {
			// Check timeout
			uint32_t waitStart = m_collectionStartTime + COLLECTION_TIMEOUT_MS;
			if (now - waitStart > RESULT_TIMEOUT_MS) {
				m_logger.error("Calibration result timeout");
				m_state = MagCalState::ERROR;
				// Timed out - notify status manager
				statusManager.setMagCalibrating(false);
			}
			break;
		}

		case MagCalState::COMPLETE:
		case MagCalState::ERROR:
		case MagCalState::IDLE:
			// No action needed
			break;
	}
}

bool MagCalibrationManager::isMagCalButtonHeldOnBoot() const {
	// Original implementation used a blocking 5s loop with delay(), which can
	// completely stall the timing-critical IMU/FIFO loop when called from the
	// main thread. To guarantee non-blocking behaviour, we now treat "held on
	// boot" as simply "button is currently pressed at boot" and avoid any
	// waiting here.

#ifdef MAG_CAL_BUTTON_PIN
	pinMode(MAG_CAL_BUTTON_PIN, INPUT_PULLUP);
	// Active-low: LOW means pressed
	return digitalRead(MAG_CAL_BUTTON_PIN) == LOW;
#else
	// No button defined, return false (safe default)
	return false;
#endif
}

const char* MagCalibrationManager::getTrackerId() const {
	return m_trackerId;
}

void MagCalibrationManager::drainCalibSampleQueue() {
	// Send queued calibration samples (non-blocking, batched)
	if (m_state == MagCalState::COLLECTING && m_calibQueueSize > 0) {
		// Send up to N samples per update to avoid blocking
		constexpr size_t MAX_SENDS_PER_UPDATE = 5;
		for (size_t i = 0; i < MAX_SENDS_PER_UPDATE && m_calibQueueSize > 0; i++) {
			const QueuedCalibSample& sample = m_calibSampleQueue[m_calibQueueHead];
			uint32_t startTime = micros();
			sendCalibSample(sample.mx, sample.my, sample.mz, sample.timestamp);
			uint32_t elapsed = micros() - startTime;
			recordNetworkCallTime(elapsed);
			m_samplesSent++;
			// Log once when we've reached the recommended minimum, but do not
			// automatically stop collection. The user (or timeout) will decide
			// when to finish.
			if (m_samplesSent == MIN_SAMPLES) {
				m_logger.info(
					"Collected %u samples (recommended minimum), continue "
					"moving for extra coverage or send STOP to finish.",
					m_samplesSent
				);
			}
			m_calibQueueHead = (m_calibQueueHead + 1) % MAX_QUEUED_CALIB_SAMPLES;
			m_calibQueueSize--;
		}
	}
}

void MagCalibrationManager::drainTelemetryQueue() {
	// Send queued telemetry with rate limiting (non-blocking)
	if ((m_state == MagCalState::IDLE || m_state == MagCalState::COMPLETE) && m_telemetryQueueSize > 0) {
		uint32_t now = millis();
		// Rate limit: only send if enough time has passed
		if (now - m_lastTelemetrySend >= TELEMETRY_INTERVAL_MS) {
			// Send oldest telemetry entry
			const QueuedTelemetry& entry = m_telemetryQueue[m_telemetryQueueHead];
			uint32_t startTime = micros();
			sendTelemetryPacket(entry.quat, entry.magCal, entry.magQuality, entry.timestamp);
			uint32_t elapsed = micros() - startTime;
			recordNetworkCallTime(elapsed);
			m_lastTelemetrySend = now;
			m_telemetryQueueHead = (m_telemetryQueueHead + 1) % MAX_QUEUED_TELEMETRY;
			m_telemetryQueueSize--;
		}
	}
}

void MagCalibrationManager::recordNetworkCallTime(uint32_t timeUs) {
	m_totalNetworkCalls++;
	m_totalNetworkTimeUs += timeUs;
	if (timeUs > m_maxNetworkCallTimeUs) {
		m_maxNetworkCallTimeUs = timeUs;
	}
	
	// If a single network call takes too long, it can starve the IMU FIFO and
	// cause overruns. If we ever see a call taking more than ~2ms, permanently
	// disable mag sidecar networking for this boot to guarantee the tracker
	// loop stays responsive. The device will continue operating without mag
	// sidecar support.
	if (timeUs > 2000 && !m_networkDisabled) {
		m_networkDisabled = true;
		m_logger.error(
			"Mag sidecar network call took %u us (max so far %u us). "
			"Disabling mag sidecar networking for this session to protect "
			"IMU/FIFO timing.",
			timeUs,
			m_maxNetworkCallTimeUs
		);
	}
}

}  // namespace SlimeVR::MagCalibration

