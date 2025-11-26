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

#ifndef SLIMEVR_MAG_CALIBRATION_MANAGER_H
#define SLIMEVR_MAG_CALIBRATION_MANAGER_H

#include <WiFiUdp.h>
#include <cstdint>

#include "mag_calibration.h"
#include "logging/Logger.h"

namespace SlimeVR::MagCalibration {

// Compile-time flag to enable mag calibration mode (MagSidecar support)
#ifndef ENABLE_MAG_CALIBRATION_MODE
#define ENABLE_MAG_CALIBRATION_MODE 1
#endif

// Compile-time flag to enable yaw consensus (requires ENABLE_MAG_CALIBRATION_MODE)
// Define ENABLE_YAW_CONSENSUS in platformio.ini build_flags to enable yaw consensus
// Defaults to disabled (undefined) for safety

enum class MagCalState {
	IDLE,
	COLLECTING,
	WAITING_FOR_RESULT,
	COMPLETE,
	ERROR
};

class MagCalibrationManager {
public:
	MagCalibrationManager();
	~MagCalibrationManager() = default;

	// Initialize the manager (call after WiFi is connected)
	void setup();

	// Update loop (call regularly)
	void update();

	// Check if we should enter calibration mode at boot
	bool shouldEnterCalibrationMode() const;

	// Enter calibration mode
	void enterCalibrationMode();

	// Get current state
	MagCalState getState() const { return m_state; }

	// Get calibration config (for applying to mag readings)
	const MagCalibrationConfig& getCalibration() const { return m_calConfig; }

	// Apply yaw bias to quaternion (returns corrected quaternion)
	// Only applies if yawBiasValid is true and yaw consensus is enabled
	void applyYawBias(const float q_vqf[4], float q_final[4]) const;
	
	// Check if yaw bias is valid
	bool isYawBiasValid() const { return m_yawBiasValid; }

	// Send telemetry to sidecar (call periodically during normal operation)
	// IMPORTANT: quat should be the RAW VQF quaternion (before yaw correction),
	// not the yaw-corrected quaternion sent to SlimeVR server.
	// The sidecar needs raw orientation to compute yaw errors correctly.
	void sendTelemetry(const float quat[4], const float magCal[3], float magQuality);

	// Feed mag sample during calibration (call when in COLLECTING state)
	void feedMagSample(float mx, float my, float mz);

	// Reset calibration (for re-calibration)
	void resetCalibration();

private:
	// UDP communication
	void sendCalibSample(float mx, float my, float mz, uint32_t timestamp);
	void sendCalibDone();
	void sendTelemetryPacket(const float quat[4], const float magCal[3], float magQuality, uint32_t timestamp);
	void receiveAndProcessPackets();

	// JSON parsing (simple, no external library)
	void handleCalibResult(const char* json, size_t len);
	void handleYawUpdate(const char* json, size_t len);

	// State machine
	void updateStateMachine();

	// Helper: check if button is held on boot
	bool isMagCalButtonHeldOnBoot() const;

	// Helper: get tracker ID string
	const char* getTrackerId() const;

	MagCalState m_state = MagCalState::IDLE;
	MagCalibrationConfig m_calConfig;

	WiFiUDP m_udp;
	IPAddress m_sidecarIP;
	uint16_t m_sidecarPort = 7778;
	bool m_sidecarIPConfigured = false;  // Track if explicitly configured

	// Calibration collection state
	uint32_t m_collectionStartTime = 0;
	uint32_t m_samplesSent = 0;
	uint32_t m_lastSampleTime = 0;
	static constexpr uint32_t MIN_SAMPLES = 1000;
	static constexpr uint32_t COLLECTION_TIMEOUT_MS = 90000;  // 90 seconds
	static constexpr uint32_t RESULT_TIMEOUT_MS = 10000;      // 10 seconds
	static constexpr uint32_t SAMPLE_INTERVAL_MS = 20;         // 50 Hz

	// Yaw bias
	float m_yawBiasRad = 0.0f;
	bool m_yawBiasValid = false;  // Set true once we receive at least one valid yaw_update
	static constexpr float MAX_YAW_BIAS_RAD = 20.0f * (3.14159265359f / 180.0f);  // ±20 degrees

	// Tracker ID (will be set from configuration or MAC address)
	char m_trackerId[32];

	// Queue structures for deferred network sending (non-blocking IMU path)
	struct QueuedCalibSample {
		float mx, my, mz;
		uint32_t timestamp;
	};
	
	struct QueuedTelemetry {
		float quat[4];
		float magCal[3];
		float magQuality;
		uint32_t timestamp;
	};
	
	static constexpr size_t MAX_QUEUED_CALIB_SAMPLES = 50;
	static constexpr size_t MAX_QUEUED_TELEMETRY = 10;
	
	QueuedCalibSample m_calibSampleQueue[MAX_QUEUED_CALIB_SAMPLES];
	size_t m_calibQueueHead = 0;
	size_t m_calibQueueTail = 0;
	size_t m_calibQueueSize = 0;
	
	QueuedTelemetry m_telemetryQueue[MAX_QUEUED_TELEMETRY];
	size_t m_telemetryQueueHead = 0;
	size_t m_telemetryQueueTail = 0;
	size_t m_telemetryQueueSize = 0;
	
	// Rate limiting for telemetry
	uint32_t m_lastTelemetrySend = 0;
	static constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;  // 10 Hz max
	
	// Performance monitoring
	uint32_t m_maxNetworkCallTimeUs = 0;
	uint32_t m_totalNetworkCalls = 0;
	uint32_t m_totalNetworkTimeUs = 0;
	
	// Queue draining helpers
	void drainCalibSampleQueue();
	void drainTelemetryQueue();
	
	// Performance monitoring helper
	void recordNetworkCallTime(uint32_t timeUs);

	Logging::Logger m_logger = Logging::Logger("MagCalManager");
};

// Global instance
extern MagCalibrationManager g_magCalManager;

}  // namespace SlimeVR::MagCalibration

#endif

