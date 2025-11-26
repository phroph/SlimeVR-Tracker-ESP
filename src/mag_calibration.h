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

#ifndef SLIMEVR_MAG_CALIBRATION_H
#define SLIMEVR_MAG_CALIBRATION_H

#include <cstdint>
#include <cstring>

namespace SlimeVR::MagCalibration {

constexpr uint32_t MAG_CAL_VERSION = 1;

struct MagCalibrationConfig {
	float offset[3];      // hard-iron bias in BODY frame [bx, by, bz]
	float matrix[3][3];   // 3x3 soft-iron correction matrix A
	float fieldStrength;  // average |A(m_raw - offset)|, optional
	uint32_t version;
	bool valid;

	MagCalibrationConfig()
		: offset{0.0f, 0.0f, 0.0f}
		, matrix{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}}
		, fieldStrength(0.0f)
		, version(MAG_CAL_VERSION)
		, valid(false)
	{
	}
};

// Load calibration from NVS/config
bool magCalLoad(MagCalibrationConfig& cfg);

// Save calibration to NVS/config
bool magCalSave(const MagCalibrationConfig& cfg);

// Apply calibration: out = matrix * (raw - offset)
inline void magCalApply(
	const MagCalibrationConfig& cfg,
	const float raw[3],  // m_raw_body
	float out[3]          // m_cal_body
) {
	if (!cfg.valid) {
		// No calibration, pass through
		memcpy(out, raw, 3 * sizeof(float));
		return;
	}

	// Subtract hard-iron offset
	float corrected[3] = {
		raw[0] - cfg.offset[0],
		raw[1] - cfg.offset[1],
		raw[2] - cfg.offset[2]
	};

	// Apply soft-iron matrix: out = matrix * corrected
	out[0] = cfg.matrix[0][0] * corrected[0] + cfg.matrix[0][1] * corrected[1] + cfg.matrix[0][2] * corrected[2];
	out[1] = cfg.matrix[1][0] * corrected[0] + cfg.matrix[1][1] * corrected[1] + cfg.matrix[1][2] * corrected[2];
	out[2] = cfg.matrix[2][0] * corrected[0] + cfg.matrix[2][1] * corrected[1] + cfg.matrix[2][2] * corrected[2];
}

}  // namespace SlimeVR::MagCalibration

#endif


