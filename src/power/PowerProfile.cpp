#include "PowerProfile.h"

namespace SlimeVR::Power {

bool g_udpSendEnabled = true;
bool g_ledEnabled = true;
float g_maxRotationSendRateHz = 0.0f;
int g_wifiPsMode = -1;
int8_t g_wifiMaxTxPowerQdbm = -1;

}  // namespace SlimeVR::Power
