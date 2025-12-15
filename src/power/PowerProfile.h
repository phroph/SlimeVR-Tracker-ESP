#pragma once

#include <cstdint>

namespace SlimeVR::Power {

// Runtime knobs intended for power profiling / baseline measurements.
// These are deliberately simple globals so they can be toggled from serial
// commands without plumbing a full config system.

// When false, we still read IMUs and run fusion, but we do not transmit UDP data.
extern bool g_udpSendEnabled;

// When false, suppress LED updates (and force LED off).
extern bool g_ledEnabled;

// Max rotation send rate cap (Hz). Sensors may clamp further.
// 0 means "use sensor default".
extern float g_maxRotationSendRateHz;

// ESP32 WiFi power tuning (best-effort).
// - g_wifiPsMode: -1 = default, else one of WIFI_PS_NONE(0)/MIN_MODEM(1)/MAX_MODEM(2)
// - g_wifiMaxTxPowerQdbm: -1 = default, else quarter-dBm (qdbm) where 4 = 1 dBm.
extern int g_wifiPsMode;
extern int8_t g_wifiMaxTxPowerQdbm;

}  // namespace SlimeVR::Power
