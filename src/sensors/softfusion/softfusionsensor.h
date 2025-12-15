/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Tailsy13 & SlimeVR Contributors

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

#pragma once

#include <PinInterface.h>

#include <algorithm>
#include <cstdint>
#include <cstring>

#include "../../GlobalVars.h"
#include "../../sensorinterface/SensorInterface.h"
#include "../RestCalibrationDetector.h"
#include "../sensor.h"
#include "../../mag_calibration.h"
#include "../../mag_calibration_manager.h"
#include "../../power/PowerProfile.h"
#include "TempGradientCalculator.h"
#include "imuconsts.h"
#include "motionprocessing/types.h"
#include "sensors/SensorFusion.h"
#include "sensors/softfusion/magdriver.h"

namespace SlimeVR::Sensors {

template <typename SensorType, template <typename IMU> typename Calibrator>
class SoftFusionSensor : public Sensor {
	using Consts = IMUConsts<SensorType>;
	using RawSensorT = typename Consts::RawSensorT;

	using Calib = Calibrator<SensorType>;
	static constexpr auto UpsideDownCalibrationInit = Calib::HasUpsideDownCalibration;

	float lastReadTemperature = 0;
	uint32_t lastTempPollTime = micros();
	
	// Store last calibrated mag reading for telemetry
	float m_lastMagCal[3] = {0.0f, 0.0f, 0.0f};
	// Store last *raw* mag reading in body frame for sidecar telemetry /
	// calibration, to avoid any conflation with VQF corrections.
	float m_lastMagRaw[3] = {0.0f, 0.0f, 0.0f};
	bool m_hasMagData = false;
	// Track whether a magnetometer was actually detected on this sensor. Some
	// board variants wire LSM6DSV without an external mag; in that case we
	// should never attempt aux reads in the motion loop to avoid sensor hub
	// timeout spam.
	bool m_magPresent = false;

	bool detected() const {
		const auto value
			= m_sensor.m_RegisterInterface.readReg(SensorType::Regs::WhoAmI::reg);
		if constexpr (requires { SensorType::Regs::WhoAmI::values.size(); }) {
			for (auto possible : SensorType::Regs::WhoAmI::values) {
				if (value == possible) {
					return true;
				}
			}
			// this assumes there are only 2 values in the array
			m_Logger.error(
				"Sensor not detected, expected reg 0x%02x = [0x%02x, 0x%02x] but got "
				"0x%02x",
				SensorType::Regs::WhoAmI::reg,
				SensorType::Regs::WhoAmI::values[0],
				SensorType::Regs::WhoAmI::values[1],
				value
			);
			return false;
		} else {
			if (value == SensorType::Regs::WhoAmI::value) {
				return true;
			}
			m_Logger.error(
				"Sensor not detected, expected reg 0x%02x = 0x%02x but got 0x%02x",
				SensorType::Regs::WhoAmI::reg,
				SensorType::Regs::WhoAmI::value,
				value
			);
			return false;
		}
	}

	void sendData() final {
		Sensor::sendData();
		sendTempIfNeeded();
	}

	void sendTempIfNeeded() {
		uint32_t now = micros();
		constexpr float maxSendRateHz = 2.0f;
		constexpr uint32_t sendInterval = 1.0f / maxSendRateHz * 1e6;
		uint32_t elapsed = now - m_lastTemperaturePacketSent;
		if (elapsed >= sendInterval) {
			m_lastTemperaturePacketSent = now - (elapsed - sendInterval);
			networkConnection.sendTemperature(sensorId, lastReadTemperature);
		}
	}

	TemperatureGradientCalculator tempGradientCalculator{[&](float gradient) {
		m_fusion.updateBiasForgettingTime(
			calibrator.getZROChange() / std::fabs(gradient)
		);
	}};

	void processAccelSample(const RawSensorT xyz[3], const sensor_real_t timeDelta) {
		sensor_real_t accelData[]
			= {static_cast<sensor_real_t>(xyz[0]),
			   static_cast<sensor_real_t>(xyz[1]),
			   static_cast<sensor_real_t>(xyz[2])};

		calibrator.scaleAccelSample(accelData);

		m_fusion.updateAcc(accelData, calibrator.getAccelTimestep());

		calibrator.provideAccelSample(xyz);
	}

	void processGyroSample(const RawSensorT xyz[3], const sensor_real_t timeDelta) {
		sensor_real_t gyroData[]
			= {static_cast<sensor_real_t>(xyz[0]),
			   static_cast<sensor_real_t>(xyz[1]),
			   static_cast<sensor_real_t>(xyz[2])};
		calibrator.scaleGyroSample(gyroData);
		m_fusion.updateGyro(gyroData, calibrator.getGyroTimestep());

		calibrator.provideGyroSample(xyz);
	}

	void
	processTempSample(const int16_t rawTemperature, const sensor_real_t timeDelta) {
		if constexpr (!Consts::DirectTempReadOnly) {
			const float scaledTemperature
				= SensorType::TemperatureBias
				+ static_cast<float>(rawTemperature)
					  * (1.0 / SensorType::TemperatureSensitivity);

			lastReadTemperature = scaledTemperature;
			if (toggles.getToggle(SensorToggles::TempGradientCalibrationEnabled)) {
				tempGradientCalculator.feedSample(
					lastReadTemperature,
					calibrator.getTempTimestep()
				);
			}

			calibrator.provideTempSample(lastReadTemperature);
		}
	}

	void processMagSample() {
		if constexpr (Consts::SupportsMags) {
			// Skip entirely if MagDriver did not find an attached magnetometer
			// for this sensor instance.
			if (!m_magPresent) {
				return;
			}
			int16_t magRaw[3] = {0, 0, 0};

			// When we're actively collecting calibration samples for the mag
			// sidecar, we want to be absolutely certain we're looking at the
			// QMC6309's true hardware output, not whatever the streaming sensor
			// hub path might be doing on a given board revision. In that mode,
			// prefer direct aux register reads when available.
			bool inCalMode = false;
#if ENABLE_MAG_CALIBRATION_MODE
			using SlimeVR::MagCalibration::MagCalState;
			if (SlimeVR::MagCalibration::g_magCalManager.getState()
				== MagCalState::COLLECTING) {
				inCalMode = true;
			}
#endif

			if (inCalMode && requires { m_sensor.readAux(uint8_t{0}); }) {
				uint8_t xL = m_sensor.readAux(0x01);
				uint8_t xH = m_sensor.readAux(0x02);
				uint8_t yL = m_sensor.readAux(0x03);
				uint8_t yH = m_sensor.readAux(0x04);
				uint8_t zL = m_sensor.readAux(0x05);
				uint8_t zH = m_sensor.readAux(0x06);
				magRaw[0] = static_cast<int16_t>((xH << 8) | xL);
				magRaw[1] = static_cast<int16_t>((yH << 8) | yL);
				magRaw[2] = static_cast<int16_t>((zH << 8) | zL);
			} else {
				// Normal tracking mode: favour the LSM6DSV sensor hub streaming
				// path when it's implemented, with a direct-aux fallback for
				// other IMUs.
				if constexpr (requires { m_sensor.readMagFromSensorHub(magRaw); }) {
					if (!m_sensor.readMagFromSensorHub(magRaw)) {
						return;  // No new data or error
					}
				} else if constexpr (requires { m_sensor.readAux(uint8_t{0}); }) {
					uint8_t xL = m_sensor.readAux(0x01);
					uint8_t xH = m_sensor.readAux(0x02);
					uint8_t yL = m_sensor.readAux(0x03);
					uint8_t yH = m_sensor.readAux(0x04);
					uint8_t zL = m_sensor.readAux(0x05);
					uint8_t zH = m_sensor.readAux(0x06);
					magRaw[0] = static_cast<int16_t>((xH << 8) | xL);
					magRaw[1] = static_cast<int16_t>((yH << 8) | yL);
					magRaw[2] = static_cast<int16_t>((zH << 8) | zL);
				} else {
					return;  // Sensor doesn't support any mag read path
				}
			}

#ifdef DEBUG_MAG_RAW
			// Periodic debug logging of raw magnetometer samples. To avoid
			// impacting FIFO timing, this is heavily rate-limited and only
			// performs a direct status read when aux access is available.
			static uint32_t dbgCount = 0;
			if ((dbgCount++ % 20u) == 0u) {
				if constexpr (requires { m_sensor.readAux(uint8_t{0}); }) {
					uint8_t qmcStatus = m_sensor.readAux(0x09);  // Status (DRDY, OVFL, etc.)
					m_Logger.info(
						"MAGDBG: status=0x%02x raw=(%d,%d,%d)",
						static_cast<unsigned int>(qmcStatus),
						static_cast<int>(magRaw[0]),
						static_cast<int>(magRaw[1]),
						static_cast<int>(magRaw[2])
					);
				} else {
					m_Logger.info(
						"MAGDBG: raw=(%d,%d,%d)",
						static_cast<int>(magRaw[0]),
						static_cast<int>(magRaw[1]),
						static_cast<int>(magRaw[2])
					);
				}
			}
#endif

			// Convert to float (sensor frame)
			float m_sens[3] = {
				static_cast<float>(magRaw[0]),
				static_cast<float>(magRaw[1]),
				static_cast<float>(magRaw[2])
			};

			// TODO: Apply sensor-to-body frame transform if needed
			// For now, assume sensor frame = body frame
			float m_body_raw[3];
			memcpy(m_body_raw, m_sens, sizeof(m_body_raw));

			// Apply mag calibration
			float m_body_cal[3];
			const auto& magCal = SlimeVR::MagCalibration::g_magCalManager.getCalibration();
			SlimeVR::MagCalibration::magCalApply(magCal, m_body_raw, m_body_cal);

			// Store both raw and calibrated mag. Raw is used for sidecar telemetry
			// and calibration; calibrated is used locally for VQF / IMU fusion.
			memcpy(m_lastMagRaw, m_body_raw, sizeof(m_lastMagRaw));
			memcpy(m_lastMagCal, m_body_cal, sizeof(m_lastMagCal));
			m_hasMagData = true;

			// During calibration mode, send *raw* body-frame magnetometer samples
			// to the sidecar (queued, non-blocking). This ensures the calibration
			// process sees the actual hardware measurements, without any fusion
			// or soft-iron correction applied.
			SlimeVR::MagCalibration::g_magCalManager.feedMagSample(
				m_body_raw[0],
				m_body_raw[1],
				m_body_raw[2]
			);

			// Feed calibrated mag data to VQF
			m_fusion.updateMag(m_body_cal, SensorType::MagTs);
		}
	}

public:
	static constexpr auto TypeID = SensorType::Type;
	static constexpr uint8_t Address = SensorType::Address;

	SoftFusionSensor(
		uint8_t id,
		RegisterInterface& registerInterface,
		float rotation,
		SlimeVR::SensorInterface* sensorInterface = nullptr,
		PinInterface* intPin = nullptr,
		uint8_t = 0
	)
		: Sensor(
			SensorType::Name,
			SensorType::Type,
			id,
			registerInterface,
			rotation,
			sensorInterface
		)
		, m_fusion(
			  SensorType::SensorVQFParams,
			  SensorType::GyrTs,
			  SensorType::AccTs,
			  SensorType::MagTs
		  )
		, m_sensor(registerInterface, m_Logger) {}
	~SoftFusionSensor() override = default;

	void checkSensorTimeout() {
		uint32_t now = millis();
		constexpr uint32_t sensorTimeoutMillis = 2e3;  // 2 seconds
		if (m_lastRotationUpdateMillis + sensorTimeoutMillis > now) {
			return;
		}

		working = false;
		m_status = SensorStatus::SENSOR_ERROR;
		m_Logger.error(
			"Sensor timeout I2C Address 0x%02x delaytime: %d ms",
			addr,
			now - m_lastRotationUpdateMillis
		);
		networkConnection.sendSensorError(
			this->sensorId,
			static_cast<uint8_t>(PacketErrorCode::WATCHDOG_TIMEOUT)
		);
	}

	void motionLoop() final {
		calibrator.tick();

		// read fifo updating fusion
		uint32_t now = micros();

		if constexpr (Consts::DirectTempReadOnly) {
			uint32_t tempElapsed = now - lastTempPollTime;
			if (tempElapsed >= Consts::DirectTempReadTs * 1e6) {
				lastTempPollTime
					= now
					- (tempElapsed
					   - static_cast<uint32_t>(Consts::DirectTempReadTs * 1e6));
				lastReadTemperature = m_sensor.getDirectTemp();

				calibrator.provideTempSample(lastReadTemperature);

				if (toggles.getToggle(SensorToggles::TempGradientCalibrationEnabled)) {
					tempGradientCalculator.feedSample(
						lastReadTemperature,
						Consts::DirectTempReadTs
					);
				}
			}
		}

		if (toggles.getToggle(SensorToggles::TempGradientCalibrationEnabled)) {
			tempGradientCalculator.tick();
		}

		// Decouple FIFO/fusion update rate from network send rate:
		// - We should read/drain the FIFO frequently enough to avoid overruns at high ODR.
		// - We only publish/send fused quaternions at the chosen send interval.
		constexpr uint32_t targetPollIntervalMicros = 2000;  // ~500 Hz FIFO service
		uint32_t elapsed = now - m_lastPollTime;
		if (elapsed >= targetPollIntervalMicros) {
			m_lastPollTime = now - (elapsed - targetPollIntervalMicros);

			m_sensor.bulkRead({
				[&](const auto sample[3], float AccTs) { processAccelSample(sample, AccTs); },
				[&](const auto sample[3], float GyrTs) { processGyroSample(sample, GyrTs); },
				[&](int16_t sample, float TempTs) { processTempSample(sample, TempTs); },
			});

			if (m_fusion.isUpdated()) {
				hadData = true;
				m_lastRotationUpdateMillis = millis();
			} else {
				checkSensorTimeout();
			}
		}

		// Send new fusion values when time is up (rate-limited, configurable).
		now = micros();
		constexpr float defaultMaxSendRateHz = 100.0f;
		float maxSendRateHz = defaultMaxSendRateHz;
		if (SlimeVR::Power::g_maxRotationSendRateHz > 0.0f) {
			maxSendRateHz = std::min(maxSendRateHz, SlimeVR::Power::g_maxRotationSendRateHz);
		}
		if (maxSendRateHz < 1.0f) {
			maxSendRateHz = 1.0f;
		}
		const uint32_t sendInterval = static_cast<uint32_t>(1.0f / maxSendRateHz * 1e6f);
		elapsed = now - m_lastRotationPacketSent;
		if (elapsed >= sendInterval) {
			// Only publish when fusion has actually advanced since the last publish.
			if (!m_fusion.isUpdated()) {
				return;
			}

			m_lastRotationPacketSent = now - (elapsed - sendInterval);

			// Process magnetometer data if available (uses last mag sample for this frame).
			if constexpr (Consts::SupportsMags) {
				processMagSample();
			}

			// Get quaternion from VQF (body→world orientation)
			Quat q_vqf = m_fusion.getQuaternionQuat();

			// Convert to array format (used for both telemetry and yaw correction)
			float q_vqf_array[4] = {q_vqf.w, q_vqf.x, q_vqf.y, q_vqf.z};

			// Send telemetry to sidecar (raw quaternion, before yaw correction)
			// This is queued and sent asynchronously, so it doesn't block IMU processing
			if constexpr (Consts::SupportsMags) {
				// Only send telemetry when we actually have magnetometer data in use.
				if (m_hasMagData) {
					// Magnetometer quality heuristic:
					// - Use VQF's magnetic disturbance detection as the primary signal.
					// - When a disturbance is detected, report quality 0.0.
					// - When the field is undisturbed, report quality 1.0.
					//
					// This keeps the heuristic simple and robust while exposing meaningful
					// information to the sidecar without conflating it with other sensors.
					float magQuality = 1.0f;
					if (m_fusion.getMagDistDetected()) {
						magQuality = 0.0f;
					}

					// Send raw quaternion (before yaw correction) and *raw* body-frame
					// magnetometer samples to the sidecar. This keeps Magneto / yaw
					// consensus completely unaware of any internal VQF corrections or
					// our own mag calibration matrix, preventing double-correction of
					// heading over time.
					SlimeVR::MagCalibration::g_magCalManager.sendTelemetry(
						q_vqf_array, m_lastMagRaw, magQuality);
				}
			}

			// Apply yaw bias correction from MagSidecar (if enabled and valid)
			// The sidecar sends yawBiasRad = angle (tracker→global).
			// We apply rotation of −yawBiasRad around world-up to correct heading.
			// applyYawBias() handles the guard checks internally (ENABLE_YAW_CONSENSUS, yawBiasValid).
			float q_final_array[4];
			SlimeVR::MagCalibration::g_magCalManager.applyYawBias(q_vqf_array, q_final_array);
			Quat q_final(q_final_array[1], q_final_array[2], q_final_array[3], q_final_array[0]);

			setFusedRotation(q_final);
			setAcceleration(m_fusion.getLinearAccVec());
			m_fusion.clearUpdated();

#if defined(DEBUG_SENSOR)
			// Low-rate fusion diagnostics useful for high-motion tuning.
			// Kept behind DEBUG_SENSOR and rate-limited to avoid perturbing timing.
			{
				static uint32_t lastVqfLogMs = 0;
				uint32_t nowMs = millis();
				if (nowMs - lastVqfLogMs >= 2000) {
					lastVqfLogMs = nowMs;
					sensor_real_t restDev[2]{0, 0};
					m_fusion.getRelativeRestDeviations(restDev);
					m_Logger.info(
						"VQF: rest=%u restDev(gyr=%.2f acc=%.2f) magDist=%u magRef(norm=%.2f dip=%.2f)",
						static_cast<unsigned int>(m_fusion.getRestDetected() ? 1u : 0u),
						static_cast<double>(restDev[0]),
						static_cast<double>(restDev[1]),
						static_cast<unsigned int>(m_fusion.getMagDistDetected() ? 1u : 0u),
						static_cast<double>(m_fusion.getMagRefNorm()),
						static_cast<double>(m_fusion.getMagRefDip())
					);
				}
			}
#endif
			optimistic_yield(100);
		}

		if (calibrationDetector.update(m_fusion)) {
			markRestCalibrationComplete();
		}
	}

	void motionSetup() final {
		if (!detected()) {
			m_status = SensorStatus::SENSOR_ERROR;
			return;
		}

		SlimeVR::Configuration::SensorConfig sensorCalibration
			= configuration.getSensor(sensorId);

		toggles = configuration.getSensorToggles(sensorId);

		// If no compatible calibration data is found, the calibration data will just be
		// zero-ed out
		if (calibrator.calibrationMatches(sensorCalibration)) {
			calibrator.assignCalibration(sensorCalibration);
		} else if (sensorCalibration.type == SlimeVR::Configuration::SensorConfigType::NONE) {
			m_Logger.warn(
				"No calibration data found for sensor %d, ignoring...",
				sensorId
			);
			m_Logger.info("Calibration is advised");
		} else {
			m_Logger.warn(
				"Incompatible calibration data found for sensor %d, ignoring...",
				sensorId
			);
			m_Logger.info("Please recalibrate");
		}

		calibrator.begin();

		bool initResult = false;

		if constexpr (Calib::HasMotionlessCalib) {
			typename SensorType::MotionlessCalibrationData calibData;
			std::memcpy(
				&calibData,
				calibrator.getMotionlessCalibrationData(),
				sizeof(calibData)
			);
			initResult = m_sensor.initialize(calibData);
		} else {
			initResult = m_sensor.initialize();
		}

		if (!initResult) {
			m_Logger.error("Sensor failed to initialize!");
			m_status = SensorStatus::SENSOR_ERROR;
			return;
		}

		m_status = SensorStatus::SENSOR_OK;
		working = true;

		calibrator.checkStartupCalibration();

		if constexpr (Consts::SupportsMags) {
			bool magDetected = magDriver.init(
				SoftFusion::MagInterface{
					.readByte
					= [&](uint8_t address) { return m_sensor.readAux(address); },
					.writeByte
					= [&](uint8_t address, uint8_t value) {
						  m_sensor.writeAux(address, value);
					  },
					.setDeviceId
					= [&](uint8_t deviceId) { m_sensor.setAuxId(deviceId); },
					.startPolling
					= [&](uint8_t dataReg, SoftFusion::MagDataWidth dataWidth
					  ) { m_sensor.startAuxPolling(dataReg, dataWidth); },
					.stopPolling = [&]() { m_sensor.stopAuxPolling(); },
				},
				Consts::Supports9ByteMag
			);

			m_magPresent = magDetected;

			bool magEnabled = toggles.getToggle(SensorToggles::MagEnabled);
			// Log the exact two-bit state that SlimeVR Server derives MagnetometerStatus from:
			// - magSupported (bit 1)
			// - magEnabled   (bit 0)
			// See SlimeVR-Server: `SensorConfig.magStatus`.
			m_Logger.info(
				"Mag status (server bits): supported=%u enabled=%u",
				static_cast<unsigned int>(magDetected ? 1u : 0u),
				static_cast<unsigned int>((magDetected && magEnabled) ? 1u : 0u)
			);
			if (magDetected && magEnabled) {
				magDriver.startPolling();
				m_Logger.info("Magnetometer %s enabled", magDriver.getAttachedMagName());
			} else if (magDetected && !magEnabled) {
				// Important for diagnosing server-side yaw behavior (StayAligned):
				// the server will only skip yaw correction when it thinks the
				// magnetometer is ENABLED, not merely present.
				m_Logger.info(
					"Magnetometer %s detected but disabled by toggle",
					magDriver.getAttachedMagName()
				);
			} else if (!magDetected) {
				m_Logger.info("No magnetometer detected");
			}
		}

		toggles.onToggleChange([&](SensorToggles toggle, bool value) {
			if (toggle == SensorToggles::MagEnabled) {
				if (value) {
					magDriver.startPolling();
				} else {
					magDriver.stopPolling();
				}
			}
		});
	}

	void startCalibration(int calibrationType) final {
		calibrator.startCalibration(calibrationType);
	}

	[[nodiscard]] bool isFlagSupported(SensorToggles toggle) const final {
		// Advertise magnetometer support to the server when this IMU actually
		// has an auxiliary mag interface AND a magnetometer was detected. This allows
		// SlimeVR Server to:
		//  - Show "Magnetometer: Supported" in the UI
		//  - Send SetConfigFlag(MagEnabled) to enable/disable mag usage
		//
		// IMPORTANT: StayAligned on the server *skips yaw correction* when it thinks
		// the tracker has an enabled magnetometer. If we report "supported" when no
		// magnetometer is physically present, the server may incorrectly skip yaw
		// correction and tracking can drift/overshoot. Therefore we only report
		// magnetometer support when a magnetometer was actually detected at runtime.
		if (toggle == SensorToggles::MagEnabled) {
			if constexpr (Consts::SupportsMags) {
				return m_magPresent;
			} else {
				return false;
			}
		}

		return toggle == SensorToggles::CalibrationEnabled
			|| toggle == SensorToggles::TempGradientCalibrationEnabled;
	}

	SensorStatus getSensorState() final { return m_status; }

	SensorFusion m_fusion;
	SensorType m_sensor;
	Calib calibrator{m_fusion, m_sensor, sensorId, m_Logger, toggles};

	SensorStatus m_status = SensorStatus::SENSOR_OFFLINE;
	uint32_t m_lastPollTime = micros();
	uint32_t m_lastRotationUpdateMillis = 0;
	uint32_t m_lastRotationPacketSent = 0;
	uint32_t m_lastTemperaturePacketSent = 0;
	uint32_t m_lastMagSampleTime = 0;

	RestCalibrationDetector calibrationDetector;

	SoftFusion::MagDriver magDriver;

	static bool checkPresent(const RegisterInterface& imuInterface) {
		I2Cdev::readTimeout = 100;
		auto value = imuInterface.readReg(SensorType::Regs::WhoAmI::reg);
		I2Cdev::readTimeout = I2CDEV_DEFAULT_READ_TIMEOUT;
		if constexpr (requires { SensorType::Regs::WhoAmI::values.size(); }) {
			for (auto possible : SensorType::Regs::WhoAmI::values) {
				if (value == possible) {
					return true;
				}
			}
			return false;
		} else {
			if (value == SensorType::Regs::WhoAmI::value) {
				return true;
			}
			return false;
		}
	}

	const char* getAttachedMagnetometer() const final {
		return magDriver.getAttachedMagName();
	}
};

}  // namespace SlimeVR::Sensors
