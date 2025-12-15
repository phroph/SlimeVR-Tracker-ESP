/*
	SlimeVR Code is placed under the MIT license
	Copyright (c) 2024 Gorbit99 & SlimeVR Contributors

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

#include <algorithm>
#include <array>
#include <cstdint>

#include "lsm6ds-common.h"
#include "vqf.h"
#include "../magdriver.h"

namespace SlimeVR::Sensors::SoftFusion::Drivers {

// Driver uses acceleration range at 4g
// and gyroscope range at 1000dps
//
// ODR profiles:
// - Balanced: Gyro 480 Hz, Accel 240 Hz
// - Max:      Gyro 960 Hz, Accel 480 Hz
//
// Enable max profile by compiling with -DLSM6DSV_PROFILE_MAX=1
#ifndef LSM6DSV_PROFILE_MAX
#define LSM6DSV_PROFILE_MAX 0
#endif

struct LSM6DSV : LSM6DSOutputHandler {
	static constexpr uint8_t Address = 0x6a;
	static constexpr auto Name = "LSM6DSV";
	static constexpr auto Type = SensorTypeID::LSM6DSV;

	static constexpr float GyrFreq = LSM6DSV_PROFILE_MAX ? 960.0f : 480.0f;
	static constexpr float AccFreq = LSM6DSV_PROFILE_MAX ? 480.0f : 240.0f;
	static constexpr float MagFreq = 120;
	static constexpr float TempFreq = 60;

	static constexpr float GyrTs = 1.0 / GyrFreq;
	static constexpr float AccTs = 1.0 / AccFreq;
	static constexpr float MagTs = 1.0 / MagFreq;
	static constexpr float TempTs = 1.0 / TempFreq;

	static constexpr float GyroSensitivity = 1000 / 35.0f;
	// Datasheet: ±4 g sensitivity is 0.122 mg/LSB.
	static constexpr float AccelSensitivity = 1000 / 0.122f;

	static constexpr float TemperatureBias = 25.0f;
	static constexpr float TemperatureSensitivity = 256.0f;

	static constexpr float TemperatureZROChange = 16.667f;

	// LSM6DSV-specific VQF baseline tuning for high-motion VR:
	// - Slightly faster accel inclination correction than VQF defaults.
	// - Rest detection thresholds tuned to avoid false-rest during small motion.
	static constexpr VQFParams SensorVQFParams = VQFParams{
		.tauAcc = 2.0f,
		.tauMag = 6.0f,
		.restMinT = 2.0f,
		.restThGyr = 0.6f,
		.restThAcc = 0.06f,
	};

	// I2C Master configuration bit masks
	static constexpr uint8_t MASTER_CFG_RST_MASTER   = 1u << 7;
	static constexpr uint8_t MASTER_CFG_WRITE_ONCE   = 1u << 6;
	static constexpr uint8_t MASTER_CFG_START_CONFIG = 1u << 5;
	static constexpr uint8_t MASTER_CFG_PASS_THROUGH = 1u << 4;
	static constexpr uint8_t MASTER_CFG_MASTER_ON    = 1u << 2;
	static constexpr uint8_t MASTER_CFG_AUX_SENS_ON  = 0x03; // bits 1:0

	struct Regs {
		struct WhoAmI {
			static constexpr uint8_t reg = 0x0f;
			static constexpr uint8_t value = 0x70;
		};
		struct HAODRCFG {
			static constexpr uint8_t reg = 0x62;
			static constexpr uint8_t value = (0b00);  // 1st ODR table
		};
		struct Ctrl1XLODR {
			static constexpr uint8_t reg = 0x10;
			// ODR table is selected by HAODRCFG. We encode accel ODR in the low nibble.
			// 0b0111 = 240 Hz, 0b1000 = 480 Hz (for HAODR table 1)
			static constexpr uint8_t value = LSM6DSV_PROFILE_MAX ? (0b0011000) : (0b0010111);
		};
		struct Ctrl2GODR {
			static constexpr uint8_t reg = 0x11;
			// 0b1000 = 480 Hz, 0b1001 = 960 Hz (for HAODR table 1)
			static constexpr uint8_t value = LSM6DSV_PROFILE_MAX ? (0b0011001) : (0b0011000);
		};
		struct Ctrl3C {
			static constexpr uint8_t reg = 0x12;
			static constexpr uint8_t valueSwReset = 1;
			static constexpr uint8_t value = (1 << 6) | (1 << 2);  // BDU = 1, IF_INC =
																   // 1
		};
		struct Ctrl6GFS {
			static constexpr uint8_t reg = 0x15;
			static constexpr uint8_t value = (0b0011);  // 1000dps
		};
		struct Ctrl8XLFS {
			static constexpr uint8_t reg = 0x17;
			static constexpr uint8_t value = (0b01);  // 4g
		};
		struct FifoCtrl3BDR {
			static constexpr uint8_t reg = 0x09;
			// Upper nibble: gyro BDR, lower nibble: accel BDR
			// Balanced: gyro 480 (0b1000), accel 240 (0b0111) => 0x87
			// Max:      gyro 960 (0b1001), accel 480 (0b1000) => 0x98
			static constexpr uint8_t value = LSM6DSV_PROFILE_MAX ? 0b10011000 : 0b10000111;
		};
	struct FifoCtrl4Mode {
		static constexpr uint8_t reg = 0x0a;
		static constexpr uint8_t value = (0b110110);  // continuous mode,
													  // temperature at 60Hz
	};

	// Auxiliary I2C master registers for magnetometer support (MSDA/MSCL)
	// These registers are on the embedded function page and require FUNC_CFG_ACCESS to access
	struct FuncCfgAccess {
		static constexpr uint8_t reg = 0x01;  // Main page: Access embedded function registers
		// FUNC_CFG_ACCESS bits:
		// Bit 7: EMB_FUNC_REG_ACCESS (embedded function registers)
		// Bit 6: SHUB_REG_ACCESS (sensor hub/I2C master registers) - THIS IS WHAT WE NEED!
		// Bit 5: FSM_WR_CTRL_EN
		// Bit 4: SW_POR
		// Bit 3: SPI2_RESET
		// Bit 2: OIS_CTRL_FROM_UI
		// Bits 1-0: Reserved (must be 0)
		static constexpr uint8_t valueEmbedded = 0x80;  // Enable embedded function access (bit 7)
		static constexpr uint8_t valueSensorHub = 0x40;  // Enable sensor hub access (bit 6)
		static constexpr uint8_t valueMain = 0x00;  // Return to main page
	};
	
	// Embedded function page registers (accessed after FUNC_CFG_ACCESS = 0x80)
	struct I2CMasterConfig {
		static constexpr uint8_t reg = 0x14;  // MASTER_CONFIG: I2C master configuration
	};
	struct I2CMasterAddr {
		static constexpr uint8_t reg = 0x15;  // SLV0_ADD: Slave 0 I2C address
	};
	struct I2CMasterSubAddr {
		static constexpr uint8_t reg = 0x16;  // SLV0_SUBADD: Slave 0 sub-address (register)
	};
	struct I2CMasterSlvConfig {
		static constexpr uint8_t reg = 0x17;  // SLV0_CONFIG: Slave 0 configuration
	};
	struct I2CMasterDataWr {
		static constexpr uint8_t reg = 0x21;  // DATAWRITE_SLV0: Data write register
	};
	struct I2CMasterStatus {
		static constexpr uint8_t reg = 0x22;  // STATUS_MASTER: Master status register (embedded page)
	};
	struct I2CMasterStatusMainPage {
		static constexpr uint8_t reg = 0x49;  // EMB_FUNC_STATUS_MAINPAGE: May contain sensor hub status on main page
	};
	struct I2CMasterDataRd {
		static constexpr uint8_t reg = 0x02;  // SENSOR_HUB_1: Data read register (first byte)
	};

	static constexpr uint8_t FifoStatus = 0x1b;
	static constexpr uint8_t FifoData = 0x78;
};

	LSM6DSV(RegisterInterface& registerInterface, SlimeVR::Logging::Logger& logger)
		: LSM6DSOutputHandler(registerInterface, logger) {}

	bool initialize() {
		// perform initialization step
		m_RegisterInterface.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::valueSwReset);
		delay(20);
		m_RegisterInterface.writeReg(Regs::HAODRCFG::reg, Regs::HAODRCFG::value);
		m_RegisterInterface.writeReg(Regs::Ctrl1XLODR::reg, Regs::Ctrl1XLODR::value);
		m_RegisterInterface.writeReg(Regs::Ctrl2GODR::reg, Regs::Ctrl2GODR::value);
		m_RegisterInterface.writeReg(Regs::Ctrl3C::reg, Regs::Ctrl3C::value);
		m_RegisterInterface.writeReg(Regs::Ctrl6GFS::reg, Regs::Ctrl6GFS::value);
		m_RegisterInterface.writeReg(Regs::Ctrl8XLFS::reg, Regs::Ctrl8XLFS::value);
		m_RegisterInterface.writeReg(
			Regs::FifoCtrl3BDR::reg,
			Regs::FifoCtrl3BDR::value
		);
		m_RegisterInterface.writeReg(
			Regs::FifoCtrl4Mode::reg,
			Regs::FifoCtrl4Mode::value
		);
		m_Logger.info(
			"LSM6DSV profile: %s (gyro=%.0fHz accel=%.0fHz) regs: CTRL1_XL=0x%02x CTRL2_G=0x%02x FIFO_CTRL3=0x%02x",
			LSM6DSV_PROFILE_MAX ? "MAX" : "BALANCED",
			GyrFreq,
			AccFreq,
			Regs::Ctrl1XLODR::value,
			Regs::Ctrl2GODR::value,
			Regs::FifoCtrl3BDR::value
		);
		return true;
	}

	void bulkRead(DriverCallbacks<int16_t>&& callbacks) {
		LSM6DSOutputHandler::template bulkRead<Regs>(
			std::move(callbacks),
			GyrTs,
			AccTs,
			TempTs
		);
	}

	// Comprehensive diagnostic function to read and log all available diagnostic information
	void dumpDiagnostics() {
		m_Logger.info("=== LSM6DSV Comprehensive Diagnostics ===");
		
		// === MAIN PAGE REGISTERS ===
		m_Logger.info("--- Main Page Registers ---");
		
		// FUNC_CFG_ACCESS (0x01) - Current page access state
		uint8_t funcCfgAccess = m_RegisterInterface.readReg(0x01);
		m_Logger.info("FUNC_CFG_ACCESS (0x01): 0x%02x", funcCfgAccess);
		m_Logger.info("  EMB_FUNC_REG_ACCESS (bit 7): %d", (funcCfgAccess >> 7) & 0x01);
		m_Logger.info("  SHUB_REG_ACCESS (bit 6): %d", (funcCfgAccess >> 6) & 0x01);
		m_Logger.info("  FSM_WR_CTRL_EN (bit 5): %d", (funcCfgAccess >> 5) & 0x01);
		m_Logger.info("  SW_POR (bit 4): %d", (funcCfgAccess >> 4) & 0x01);
		m_Logger.info("  SPI2_RESET (bit 3): %d", (funcCfgAccess >> 3) & 0x01);
		m_Logger.info("  OIS_CTRL_FROM_UI (bit 2): %d", (funcCfgAccess >> 2) & 0x01);
		
		// PIN_CTRL (0x02) - Pin control register
		uint8_t pinCtrl = m_RegisterInterface.readReg(0x02);
		m_Logger.info("PIN_CTRL (0x02): 0x%02x", pinCtrl);
		m_Logger.info("  OIS_PU_DIS (bit 5): %d (0=OCS_Aux/SDO_Aux pull-up enabled)", (pinCtrl >> 5) & 0x01);
		m_Logger.info("  SDO_PU_EN (bit 4): %d (1=SDO pull-up enabled)", (pinCtrl >> 4) & 0x01);
		m_Logger.info("  IBHR_POR_EN (bit 1): %d (must be 1)", (pinCtrl >> 1) & 0x01);
		m_Logger.info("  Bit 0: %d (must be 0)", pinCtrl & 0x01);
		
		// IF_CFG (0x03) - Interface configuration
		uint8_t ifCfg = m_RegisterInterface.readReg(0x03);
		m_Logger.info("IF_CFG (0x03): 0x%02x", ifCfg);
		m_Logger.info("  SHUB_PU_EN (bit 7): %d (0=aux I2C pull-up disabled, 1=enabled on MSDA/MSCL)", (ifCfg >> 7) & 0x01);
		m_Logger.info("    NOTE: Controls internal pull-ups (30-50kΩ) on auxiliary I2C lines (MSDA/MSCL)");
		m_Logger.info("    If sensor hub fails, try enabling this bit even with external pull-ups");
		m_Logger.info("  SDA_PU_EN (bit 6): %d (0=SDA pull-up disconnected)", (ifCfg >> 6) & 0x01);
		m_Logger.info("  ASF_CTRL (bit 5): %d", (ifCfg >> 5) & 0x01);
		m_Logger.info("  H_LACTIVE (bit 4): %d (0=interrupt active high)", (ifCfg >> 4) & 0x01);
		m_Logger.info("  PP_OD (bit 3): %d (0=push-pull mode)", (ifCfg >> 3) & 0x01);
		m_Logger.info("  SIM (bit 2): %d (0=4-wire SPI)", (ifCfg >> 2) & 0x01);
		m_Logger.info("  I2C_I3C_disable (bit 0): %d (0=I2C enabled - CRITICAL!)", ifCfg & 0x01);
		
		// WHO_AM_I (0x0F) - Device ID
		uint8_t whoAmI = m_RegisterInterface.readReg(Regs::WhoAmI::reg);
		m_Logger.info("WHO_AM_I (0x0F): 0x%02x (expected 0x%02x)", whoAmI, Regs::WhoAmI::value);
		
		// CTRL registers (0x10-0x19)
		m_Logger.info("--- Control Registers ---");
		uint8_t ctrl1 = m_RegisterInterface.readReg(0x10);
		uint8_t ctrl2 = m_RegisterInterface.readReg(0x11);
		uint8_t ctrl3 = m_RegisterInterface.readReg(0x12);
		uint8_t ctrl4 = m_RegisterInterface.readReg(0x13);
		uint8_t ctrl5 = m_RegisterInterface.readReg(0x14);
		uint8_t ctrl6 = m_RegisterInterface.readReg(0x15);
		uint8_t ctrl7 = m_RegisterInterface.readReg(0x16);
		uint8_t ctrl8 = m_RegisterInterface.readReg(0x17);
		uint8_t ctrl9 = m_RegisterInterface.readReg(0x18);
		uint8_t ctrl10 = m_RegisterInterface.readReg(0x19);
		m_Logger.info("CTRL1 (0x10): 0x%02x (Accel ODR/OP_MODE)", ctrl1);
		m_Logger.info("CTRL2 (0x11): 0x%02x (Gyro ODR/OP_MODE)", ctrl2);
		m_Logger.info("CTRL3 (0x12): 0x%02x (BDU=%d, IF_INC=%d, SW_RESET=%d)", 
			ctrl3, (ctrl3 >> 6) & 0x01, (ctrl3 >> 2) & 0x01, ctrl3 & 0x01);
		m_Logger.info("CTRL4 (0x13): 0x%02x", ctrl4);
		m_Logger.info("CTRL5 (0x14): 0x%02x", ctrl5);
		m_Logger.info("CTRL6 (0x15): 0x%02x (Gyro FS)", ctrl6);
		m_Logger.info("CTRL7 (0x16): 0x%02x", ctrl7);
		m_Logger.info("CTRL8 (0x17): 0x%02x (Accel FS)", ctrl8);
		m_Logger.info("CTRL9 (0x18): 0x%02x", ctrl9);
		m_Logger.info("CTRL10 (0x19): 0x%02x", ctrl10);
		
		// Status registers
		m_Logger.info("--- Status Registers ---");
		uint8_t ctrlStatus = m_RegisterInterface.readReg(0x1A);
		uint8_t fifoStatus1 = m_RegisterInterface.readReg(0x1B);
		uint8_t fifoStatus2 = m_RegisterInterface.readReg(0x1C);
		uint8_t allIntSrc = m_RegisterInterface.readReg(0x1D);
		uint8_t statusReg = m_RegisterInterface.readReg(0x1E);
		m_Logger.info("CTRL_STATUS (0x1A): 0x%02x", ctrlStatus);
		m_Logger.info("FIFO_STATUS1 (0x1B): 0x%02x", fifoStatus1);
		m_Logger.info("FIFO_STATUS2 (0x1C): 0x%02x", fifoStatus2);
		m_Logger.info("ALL_INT_SRC (0x1D): 0x%02x", allIntSrc);
		m_Logger.info("STATUS_REG (0x1E): 0x%02x (XLDA=%d, GDA=%d, TDA=%d)", 
			statusReg, (statusReg & 0x01), (statusReg >> 1) & 0x01, (statusReg >> 2) & 0x01);
		
		// Sensor hub status on main page
		uint8_t statusMasterMain = m_RegisterInterface.readReg(0x48);
		uint8_t embFuncStatusMain = m_RegisterInterface.readReg(0x49);
		m_Logger.info("STATUS_MASTER_MAINPAGE (0x48): 0x%02x", statusMasterMain);
		m_Logger.info("EMB_FUNC_STATUS_MAINPAGE (0x49): 0x%02x", embFuncStatusMain);
		
		// Internal frequency
		uint8_t internalFreq = m_RegisterInterface.readReg(0x4F);
		m_Logger.info("INTERNAL_FREQ_FINE (0x4F): 0x%02x", internalFreq);
		
		// === EMBEDDED FUNCTION PAGE (SENSOR HUB) REGISTERS ===
		m_Logger.info("--- Embedded Function Page (Sensor Hub) Registers ---");
		
		// Switch to embedded function page
		switchToEmbeddedPage();
		
		// Verify we're on the right page
		uint8_t funcCfgAccessEmbedded = m_RegisterInterface.readReg(0x01);
		m_Logger.info("FUNC_CFG_ACCESS (on embedded page): 0x%02x", funcCfgAccessEmbedded);
		
		// MASTER_CONFIG (0x14)
		uint8_t masterConfig = m_RegisterInterface.readReg(Regs::I2CMasterConfig::reg);
		m_Logger.info("MASTER_CONFIG (0x14): 0x%02x", masterConfig);
		m_Logger.info("  RST_MASTER_REGS (bit 7): %d (0=normal)", (masterConfig >> 7) & 0x01);
		m_Logger.info("  WRITE_ONCE (bit 6): %d (0=write each cycle, 1=write once - REQUIRED for read transactions!)", (masterConfig >> 6) & 0x01);
		m_Logger.info("  START_CONFIG (bit 5): %d (0=DRDY trigger, 1=INT2 trigger)", (masterConfig >> 5) & 0x01);
		m_Logger.info("  PASS_THROUGH_MODE (bit 4): %d (0=disabled)", (masterConfig >> 4) & 0x01);
		m_Logger.info("  RESERVED BIT (bit 3): %d (0=valid - CRITICAL!)", (masterConfig >> 3) & 0x01);
		m_Logger.info("  MASTER_ON (bit 2): %d (1=enabled - CRITICAL!)", (masterConfig >> 2) & 0x01);
		m_Logger.info("  AUX_SENS_ON[1:0] (bits 2-1): %d (00=1 sensor, 01=2, 10=3, 11=4)", (masterConfig) & 0x03);
		
		// Slave 0 configuration
		uint8_t slv0Add = m_RegisterInterface.readReg(Regs::I2CMasterAddr::reg);
		uint8_t slv0SubAdd = m_RegisterInterface.readReg(Regs::I2CMasterSubAddr::reg);
		uint8_t slv0Config = m_RegisterInterface.readReg(Regs::I2CMasterSlvConfig::reg);
		m_Logger.info("SLV0_ADD (0x15): 0x%02x (I2C addr=0x%02x, rw_0=%d)", 
			slv0Add, (slv0Add >> 1), slv0Add & 0x01);
		m_Logger.info("SLV0_SUBADD (0x16): 0x%02x (register address)", slv0SubAdd);
		m_Logger.info("SLV0_CONFIG (0x17): 0x%02x", slv0Config);
		m_Logger.info("  SHUB_ODR[2:0] (bits 7-5): %d (100=120Hz)", (slv0Config >> 5) & 0x07);
		m_Logger.info("  BATCH_EXT_SENS_0_EN (bit 3): %d", (slv0Config >> 3) & 0x01);
		m_Logger.info("  Slave0_numop[2:0] (bits 2-0): %d (number of read operations)", slv0Config & 0x07);
		
		// Slave 1-3 (for completeness, even if not used)
		uint8_t slv1Add = m_RegisterInterface.readReg(0x18);
		uint8_t slv1SubAdd = m_RegisterInterface.readReg(0x19);
		uint8_t slv1Config = m_RegisterInterface.readReg(0x1A);
		m_Logger.info("SLV1_ADD (0x18): 0x%02x, SLV1_SUBADD (0x19): 0x%02x, SLV1_CONFIG (0x1A): 0x%02x", 
			slv1Add, slv1SubAdd, slv1Config);
		
		// STATUS_MASTER (0x22) - Master status register
		uint8_t statusMaster = m_RegisterInterface.readReg(Regs::I2CMasterStatus::reg);
		m_Logger.info("STATUS_MASTER (0x22): 0x%02x", statusMaster);
		m_Logger.info("  SENS_HUB_ENDOP (bit 0): %d (1=cycle completed)", statusMaster & 0x01);
		m_Logger.info("  SLAVE0_NACK (bit 1): %d (1=NACK error)", (statusMaster >> 1) & 0x01);
		m_Logger.info("  SLAVE1_NACK (bit 2): %d (1=NACK error)", (statusMaster >> 2) & 0x01);
		m_Logger.info("  SLAVE2_NACK (bit 3): %d (1=NACK error)", (statusMaster >> 3) & 0x01);
		m_Logger.info("  SLAVE3_NACK (bit 4): %d (1=NACK error)", (statusMaster >> 4) & 0x01);
		m_Logger.info("  WRITE_ONCE_DONE (bit 5): %d", (statusMaster >> 5) & 0x01);
		m_Logger.info("  SHUB_PASS_THROUGH (bit 6): %d", (statusMaster >> 6) & 0x01);
		m_Logger.info("  SHUB_OP (bit 7): %d (1=operation in progress)", (statusMaster >> 7) & 0x01);
		
		// SENSOR_HUB data registers (read first few bytes)
		m_Logger.info("--- Sensor Hub Data Registers ---");
		for (int i = 0; i < 6; i++) {
			uint8_t hubData = m_RegisterInterface.readReg(0x02 + i);
			m_Logger.info("SENSOR_HUB_%d (0x%02x): 0x%02x", i+1, 0x02+i, hubData);
		}
		
		// Switch back to main page
		switchToMainPage();
		
		// === MODE DETECTION & SENSOR HUB ANALYSIS ===
		m_Logger.info("--- Mode Detection & Sensor Hub Analysis ---");
		m_Logger.info("Mode is hardware-determined by SDx/SCx pin connections:");
		m_Logger.info("  Mode 1: SDx/SCx connected to Vdd_IO or GND (sensor hub NOT available)");
		m_Logger.info("  Mode 2: SDx/SCx available as MSDA/MSCL (sensor hub I2C master) - REQUIRED!");
		m_Logger.info("  Mode 3: SDx/SCx available as auxiliary SPI (sensor hub NOT available)");
		m_Logger.info("Current configuration suggests:");
		if ((masterConfig & (1 << 2)) != 0) {
			m_Logger.info("  - Sensor hub I2C master is ENABLED (Mode 2 likely active)");
		} else {
			m_Logger.info("  - Sensor hub I2C master is DISABLED (Mode 2 may not be active)");
		}
		if ((ifCfg & 0x01) == 0) {
			m_Logger.info("  - I2C interface is ENABLED");
		} else {
			m_Logger.info("  - I2C interface is DISABLED (ERROR!)");
		}
		
		// === CRITICAL ANALYSIS ===
		m_Logger.info("--- Critical Sensor Hub Analysis ---");
		m_Logger.info("Sensor Hub Configuration Status:");
		m_Logger.info("  MASTER_ON: %s", ((masterConfig & (1 << 2)) != 0) ? "ENABLED ✓" : "DISABLED ✗");
		m_Logger.info("  WRITE_ONCE: %s (1=required for read transactions)", 
			((masterConfig >> 6) & 0x01) != 0 ? "ENABLED ✓" : "DISABLED ✗");
		m_Logger.info("  START_CONFIG: %s (0=DRDY trigger, 1=INT2 trigger)", 
			((masterConfig >> 5) & 0x01) == 0 ? "DRDY trigger ✓" : "INT2 trigger");
		// AUX_SENS_ON encoding: 00=1 sensor, 01=2 sensors, 10=3 sensors, 11=4 sensors
		uint8_t auxSensOnValue = (masterConfig >> 1) & 0x03;
		uint8_t numSensors = auxSensOnValue == 0 ? 1 : (auxSensOnValue + 1);
		m_Logger.info("  AUX_SENS_ON: %d sensor(s) configured (raw value: %d)", numSensors, auxSensOnValue);
		m_Logger.info("  SLV0_ADD: 0x%02x (I2C addr=0x%02x) %s", 
			slv0Add, (slv0Add >> 1),
			((slv0Add & 0xFE) != 0x00) ? "✓" : "✗ NOT CONFIGURED");
		m_Logger.info("  SLV0_CONFIG: 0x%02x (Slave0_numop=%d) %s", 
			slv0Config, (slv0Config & 0x07),
			((slv0Config & 0x07) != 0) ? "✓" : "✗ ZERO READ OPERATIONS!");
		m_Logger.info("  STATUS_MASTER: 0x%02x %s", 
			statusMaster,
			(statusMaster != 0x00) ? "✓ (sensor hub active)" : "✗ (sensor hub NOT active)");
		
		m_Logger.info("Auxiliary I2C Bus (MSDA/MSCL) Configuration:");
		m_Logger.info("  SHUB_PU_EN: %s (0=internal pull-ups disabled, 1=enabled)", 
			((ifCfg >> 7) & 0x01) != 0 ? "ENABLED" : "DISABLED");
		m_Logger.info("    Internal pull-ups: 30-50kΩ (if enabled)");
		m_Logger.info("    External pull-ups: 4.7kΩ (recommended)");
		if (((ifCfg >> 7) & 0x01) == 0) {
			m_Logger.info("    NOTE: If sensor hub fails, try enabling SHUB_PU_EN (IF_CFG bit 7)");
		}
		
		m_Logger.info("Accelerometer/Gyro Status:");
		m_Logger.info("  STATUS_REG: 0x%02x (XLDA=%d, GDA=%d, TDA=%d) %s",
			statusReg, (statusReg & 0x01), (statusReg >> 1) & 0x01, (statusReg >> 2) & 0x01,
			((statusReg & 0x03) != 0) ? "✓ (data ready)" : "✗ (no data ready)");
		
		// Determine if sensor hub should be working
		bool configOk = ((masterConfig & (1 << 2)) != 0) && 
		                ((slv0Add & 0xFE) != 0x00) && 
		                ((slv0Config & 0x07) != 0) &&
		                ((statusReg & 0x03) != 0);
		
		if (configOk && statusMaster == 0x00) {
			m_Logger.error("=== SENSOR HUB DIAGNOSIS: CONFIGURATION CORRECT BUT NOT TRIGGERING ===");
			m_Logger.error("All software configuration appears correct:");
			m_Logger.error("  ✓ MASTER_ON is enabled");
			m_Logger.error("  ✓ WRITE_ONCE is %s", ((masterConfig >> 6) & 0x01) != 0 ? "enabled" : "disabled");
			m_Logger.error("  ✓ SLV0_ADD is configured (0x%02x)", slv0Add);
			m_Logger.error("  ✓ SLV0_CONFIG has non-zero read operations (%d)", slv0Config & 0x07);
			m_Logger.error("  ✓ Accelerometer/Gyro are generating DRDY signals");
			m_Logger.error("  ✗ BUT STATUS_MASTER remains 0x00 (sensor hub cycle never starts)");
			m_Logger.error("");
			m_Logger.error("This strongly indicates a HARDWARE issue:");
			m_Logger.error("  1. LSM6DSV may NOT be in Mode 2 (SDx/SCx pins not configured for Mode 2)");
			m_Logger.error("     - Verify SDx/SCx are NOT connected to Vdd_IO or GND");
			m_Logger.error("     - Verify SDx/SCx are available as MSDA/MSCL pins");
			m_Logger.error("  2. MSDA/MSCL pins may not be connected to magnetometer");
			m_Logger.error("     - Verify MSDA -> QMC6309 SDA");
			m_Logger.error("     - Verify MSCL -> QMC6309 SCL");
			m_Logger.error("  3. Missing or insufficient pull-up resistors on MSDA/MSCL");
			m_Logger.error("     - Need 4.7kΩ pull-ups to 3.3V on both MSDA and MSCL");
			m_Logger.error("     - SHUB_PU_EN=%d (try enabling IF_CFG bit 7 if external pull-ups are weak)", (ifCfg >> 7) & 0x01);
			m_Logger.error("  4. Sensor hub hardware may not be functional");
			m_Logger.error("     - Try a different LSM6DSV device if available");
			m_Logger.error("");
			m_Logger.error("NOTE: Mode 2 is HARDWARE-DETERMINED at power-on/reset.");
			m_Logger.error("      Software cannot change the mode - it depends on pin connections.");
		} else if (!configOk) {
			m_Logger.warn("Sensor hub configuration incomplete - this is expected if no magnetometer detected");
		} else {
			m_Logger.info("Sensor hub appears to be functioning correctly");
		}
		
		m_Logger.info("=== End Diagnostics ===");
	}

// Add these members inside LSM6DSV:

private:
    // 7-bit I2C address of the external magnetometer (0 if unset)
    uint8_t aux7bitAddr_ = 0;
    bool sensorHubInitialized_ = false;

    // --- Small helpers ---

    void switchToEmbeddedPage() {
        // Sensor hub / master registers: FUNC_CFG_ACCESS bit 6
        m_RegisterInterface.writeReg(Regs::FuncCfgAccess::reg,
                                     Regs::FuncCfgAccess::valueSensorHub);
        delayMicroseconds(50);
    }

    void switchToMainPage() {
        m_RegisterInterface.writeReg(Regs::FuncCfgAccess::reg,
                                     Regs::FuncCfgAccess::valueMain);
        delayMicroseconds(50);
    }

    // One-time master-reset + bring MASTER_CONFIG into a known state.
    bool enableAuxI2CMaster() {
        switchToEmbeddedPage();

        uint8_t cfg = m_RegisterInterface.readReg(Regs::I2CMasterConfig::reg);

        if (!sensorHubInitialized_) {
			m_RegisterInterface.writeReg(Regs::I2CMasterConfig::reg, cfg | MASTER_CFG_RST_MASTER);
			delay(1);
			m_RegisterInterface.writeReg(Regs::I2CMasterConfig::reg, cfg & ~MASTER_CFG_RST_MASTER);
			delay(1);
			cfg = m_RegisterInterface.readReg(Regs::I2CMasterConfig::reg);
        }

		// Clear START_CONFIG, PASS_THROUGH, AUX_SENS_ON[1:0] and ensure reserved bit 3 = 0
		cfg &= ~(MASTER_CFG_START_CONFIG |
			MASTER_CFG_PASS_THROUGH |
			MASTER_CFG_AUX_SENS_ON |
			(1u << 3)); // reserved

		// Enable WRITE_ONCE and MASTER_ON
		// Also enable AUX sensors (bits 1:0). Without this, the sensor hub may never run.
		cfg |= MASTER_CFG_WRITE_ONCE | MASTER_CFG_MASTER_ON | MASTER_CFG_AUX_SENS_ON;

        m_RegisterInterface.writeReg(Regs::I2CMasterConfig::reg, cfg);
        delay(1);
        uint8_t rb = m_RegisterInterface.readReg(Regs::I2CMasterConfig::reg);

        switchToMainPage();

        if (rb != cfg) {
            m_Logger.error("enableAuxI2CMaster: MASTER_CONFIG write mismatch (0x%02x != 0x%02x)",
                           rb, cfg);
            return false;
        }

        sensorHubInitialized_ = true;
        m_Logger.debug("enableAuxI2CMaster: MASTER_CONFIG=0x%02x", cfg);
        return true;
    }

    // Configure a single slave-0 read of `numBytes` from `subAddr` for addr7.
    bool configureSingleRead(uint8_t addr7, uint8_t subAddr, uint8_t numBytes) {
        if (addr7 == 0) {
            m_Logger.error("configureSingleRead: aux7bitAddr_ not set");
            return false;
        }

        switchToEmbeddedPage();

        // Re-arm the sensor hub for a single transaction by setting
        // WRITE_ONCE | MASTER_ON | AUX_SENS_ON while preserving existing config bits.
        uint8_t masterCfg = m_RegisterInterface.readReg(Regs::I2CMasterConfig::reg);
        masterCfg |= (MASTER_CFG_WRITE_ONCE | MASTER_CFG_MASTER_ON | MASTER_CFG_AUX_SENS_ON);
        m_RegisterInterface.writeReg(Regs::I2CMasterConfig::reg, masterCfg);

        // SLV0_ADD: bits 7-1 = 7-bit addr, bit 0 = 1 for read
        uint8_t slvAdd = (addr7 & 0x7F) << 1 | 0x01;
        m_RegisterInterface.writeReg(Regs::I2CMasterAddr::reg, slvAdd);
        m_RegisterInterface.writeReg(Regs::I2CMasterSubAddr::reg, subAddr);

        // SHUB_ODR = 120 Hz, Slave0_numop = numBytes (3-bit, saturate at 7)
        if (numBytes == 0) numBytes = 1;
        if (numBytes > 7)  numBytes = 7;
        uint8_t slvCfg = (0b100u << 5) | (numBytes & 0x07u);

        m_RegisterInterface.writeReg(Regs::I2CMasterSlvConfig::reg, slvCfg);

		// Kick configuration (some ST parts require START_CONFIG to latch SLV settings)
		masterCfg = m_RegisterInterface.readReg(Regs::I2CMasterConfig::reg);
		m_RegisterInterface.writeReg(
			Regs::I2CMasterConfig::reg,
			static_cast<uint8_t>(masterCfg | MASTER_CFG_START_CONFIG)
		);

        switchToMainPage();
        return true;
    }

    // Configure a pure write: address + reg + DATAWRITE_SLV0.
    bool configureSingleWrite(uint8_t addr7, uint8_t subAddr, uint8_t value) {
        if (addr7 == 0) {
            m_Logger.error("configureSingleWrite: aux7bitAddr_ not set");
            return false;
        }

        switchToEmbeddedPage();

        // Re-arm the sensor hub for a single transaction by setting
        // WRITE_ONCE | MASTER_ON | AUX_SENS_ON while preserving existing config bits.
        uint8_t masterCfg = m_RegisterInterface.readReg(Regs::I2CMasterConfig::reg);
        masterCfg |= (MASTER_CFG_WRITE_ONCE | MASTER_CFG_MASTER_ON | MASTER_CFG_AUX_SENS_ON);
        m_RegisterInterface.writeReg(Regs::I2CMasterConfig::reg, masterCfg);

        // SLV0_ADD: bits 7-1 = addr, bit 0 = 0 (write)
        uint8_t slvAdd = (addr7 & 0x7F) << 1;
        m_RegisterInterface.writeReg(Regs::I2CMasterAddr::reg, slvAdd);
        m_RegisterInterface.writeReg(Regs::I2CMasterSubAddr::reg, subAddr);
        m_RegisterInterface.writeReg(Regs::I2CMasterDataWr::reg, value);

        // SHUB_ODR = 120 Hz, Slave0_numop = 0 → write only
        uint8_t slvCfg = (0b100u << 5);
        m_RegisterInterface.writeReg(Regs::I2CMasterSlvConfig::reg, slvCfg);

		// Kick configuration latch
		masterCfg = m_RegisterInterface.readReg(Regs::I2CMasterConfig::reg);
		m_RegisterInterface.writeReg(
			Regs::I2CMasterConfig::reg,
			static_cast<uint8_t>(masterCfg | MASTER_CFG_START_CONFIG)
		);

        switchToMainPage();
        return true;
    }

    // Poll STATUS_MASTER until SENS_HUB_ENDOP or error or timeout.
    bool waitForSensorHub(const char *tag, uint32_t timeoutMs, uint8_t &statusOut) {
        switchToEmbeddedPage();

        const uint32_t stepMs = 2;
        statusOut = 0;
        for (uint32_t t = 0; t < timeoutMs; t += stepMs) {
            delay(stepMs);
            statusOut = m_RegisterInterface.readReg(Regs::I2CMasterStatus::reg);

            // SENS_HUB_ENDOP
            if (statusOut & 0x01) {
                switchToMainPage();
                return true;
            }

            // Any NACK bit set (SLAVE0..3_NACK)
            if (statusOut & 0x1E) {
                switchToMainPage();
                return true; // caller will inspect NACK
            }
        }

        m_Logger.error("%s: sensor hub timeout, STATUS_MASTER=0x%02x", tag, statusOut);
        switchToMainPage();
        return false;
    }

public:
    // ---------------------------------------------------------------------
    // PUBLIC API used by MagDriver (MagInterface)
    // ---------------------------------------------------------------------

    // Read magnetometer data from sensor hub registers (SENSOR_HUB_1-6)
    // Returns true if data was read successfully, false otherwise
    bool readMagFromSensorHub(int16_t mag[3]) {
        if (!sensorHubInitialized_) {
            return false;
        }

        // Check if sensor hub has completed a read cycle
        switchToEmbeddedPage();
        uint8_t status = m_RegisterInterface.readReg(Regs::I2CMasterStatus::reg);
        switchToMainPage();

        // Check if operation completed (SENS_HUB_ENDOP bit)
        if (!(status & 0x01)) {
            return false;  // No new data
        }

        // Check for NACK errors
        if (status & 0x1E) {
            return false;  // Error reading from mag
        }

        // Read 6 bytes from SENSOR_HUB_1-6 (registers 0x02-0x07 on main page)
        uint8_t rawData[6];
        m_RegisterInterface.readBytes(0x02, 6, rawData);

        // QMC6309 data format: 16-bit little-endian
        mag[0] = (int16_t)((rawData[1] << 8) | rawData[0]);  // X
        mag[1] = (int16_t)((rawData[3] << 8) | rawData[2]);  // Y
        mag[2] = (int16_t)((rawData[5] << 8) | rawData[4]);  // Z

#ifdef DEBUG_MAG_RAW
        // Periodically log raw magnetometer data coming from the LSM6DSV
        // sensor hub, so we can verify that the aux I2C + QMC6309 path is
        // returning sane, varying values during calibration sweeps.
        static uint32_t dbgCount = 0;
        if ((dbgCount++ % 20u) == 0u) {
            m_Logger.info("MAGHUB raw: x=%d y=%d z=%d", mag[0], mag[1], mag[2]);
        }
#endif

        return true;
    }

    void setAuxId(uint8_t deviceId) {
        aux7bitAddr_ = deviceId & 0x7F;

        if (!enableAuxI2CMaster()) {
            return;
        }

        switchToEmbeddedPage();
        uint8_t slvAdd = (aux7bitAddr_ << 1); // default to write
        m_RegisterInterface.writeReg(Regs::I2CMasterAddr::reg, slvAdd);
        switchToMainPage();
    }

    uint8_t readAux(uint8_t reg) {
        if (!sensorHubInitialized_ && !enableAuxI2CMaster()) {
            return 0;
        }

        if (!configureSingleRead(aux7bitAddr_, reg, 1)) {
            return 0;
        }

        uint8_t status = 0;
        if (!waitForSensorHub("readAux", 20, status)) {
            return 0;
        }

        if (status & 0x1E) {
            // SLAVE0_NACK or similar
            m_Logger.error("readAux: NACK talking to 0x%02x reg 0x%02x (STATUS_MASTER=0x%02x)",
                           aux7bitAddr_, reg, status);
            // Clear config so we don't keep banging the bus
            switchToEmbeddedPage();
            m_RegisterInterface.writeReg(Regs::I2CMasterSlvConfig::reg, 0x00);
            switchToMainPage();
            return 0;
        }

        switchToEmbeddedPage();
        uint8_t value = m_RegisterInterface.readReg(Regs::I2CMasterDataRd::reg);
        // Leave master running but disable single-shot config
        m_RegisterInterface.writeReg(Regs::I2CMasterSlvConfig::reg, 0x00);
        switchToMainPage();
		// Per-register aux reads can be extremely chatty when the magnetometer
		// path is active (multiple registers per sample). To avoid log spam and
		// timing pressure on the FIFO path, keep this disabled by default and
		// only enable it for deep sensor-hub debugging.
#ifdef DEBUG_AUX_IO_VERBOSE
		m_Logger.debug("readAux: 0x%02x -> 0x%02x", reg, value);
#endif
        return value;
    }

    // Returns true on success, false on sensor hub / I2C failure
    bool writeAux(uint8_t reg, uint8_t value) {
        if (!sensorHubInitialized_ && !enableAuxI2CMaster()) {
            return false;
        }

        if (!configureSingleWrite(aux7bitAddr_, reg, value)) {
            return false;
        }

        uint8_t status = 0;
        if (!waitForSensorHub("writeAux", 20, status)) {
            return false;
        }

        if (status & 0x1E) {
            m_Logger.error("writeAux: NACK talking to 0x%02x reg 0x%02x (STATUS_MASTER=0x%02x)",
                           aux7bitAddr_, reg, status);
            return false;
        }

        // Clear single-write config
        switchToEmbeddedPage();
        m_RegisterInterface.writeReg(Regs::I2CMasterSlvConfig::reg, 0x00);
        switchToMainPage();

        m_Logger.debug("writeAux: 0x%02x = 0x%02x", reg, value);
        return true;
    }

    void startAuxPolling(uint8_t dataReg, MagDataWidth dataWidth) {
        if (!sensorHubInitialized_ && !enableAuxI2CMaster()) {
            return;
        }
        if (aux7bitAddr_ == 0) {
            return;
        }

        switchToEmbeddedPage();

        // Disable MASTER_ON while we reconfigure
        uint8_t masterConfig = m_RegisterInterface.readReg(Regs::I2CMasterConfig::reg);
        m_RegisterInterface.writeReg(Regs::I2CMasterConfig::reg, masterConfig & ~MASTER_CFG_MASTER_ON);
        delay(1);

        // Configure slave for continuous reads
        uint8_t slvAdd = (aux7bitAddr_ << 1) | 0x01; // read mode
        m_RegisterInterface.writeReg(Regs::I2CMasterAddr::reg, slvAdd);
        m_RegisterInterface.writeReg(Regs::I2CMasterSubAddr::reg, dataReg);

        // Configure ODR and bytes to read
        uint8_t numBytes = (dataWidth == MagDataWidth::SixByte) ? 6u : 6u;
        if (numBytes > 7) numBytes = 7;
        uint8_t slvCfg = (0b100u << 5) | (numBytes & 0x07u); // 120 Hz, N bytes
        m_RegisterInterface.writeReg(Regs::I2CMasterSlvConfig::reg, slvCfg);
        
        // Re-enable MASTER_ON with WRITE_ONCE cleared for continuous operation
        masterConfig = m_RegisterInterface.readReg(Regs::I2CMasterConfig::reg);
        masterConfig &= ~MASTER_CFG_WRITE_ONCE;
        masterConfig |= MASTER_CFG_MASTER_ON | MASTER_CFG_AUX_SENS_ON;
        m_RegisterInterface.writeReg(Regs::I2CMasterConfig::reg, masterConfig);
        
        switchToMainPage();
    }

    void stopAuxPolling() {
        if (!sensorHubInitialized_) {
            return;
        }

        switchToEmbeddedPage();
        m_RegisterInterface.writeReg(Regs::I2CMasterSlvConfig::reg, 0x00);
        switchToMainPage();

        m_Logger.info("stopAuxPolling: SLV0_CONFIG=0");
    }

    // Slimmed-down, focused diagnostics just for the sensor hub path
    void dumpSensorHubState(const char *tag = nullptr) {
        if (tag) {
            m_Logger.info("=== LSM6DSV Sensor Hub State (%s) ===", tag);
        } else {
            m_Logger.info("=== LSM6DSV Sensor Hub State ===");
        }

        uint8_t ifCfg      = m_RegisterInterface.readReg(0x03);
        uint8_t statusReg  = m_RegisterInterface.readReg(0x1E); // accel/gyro DRDY
        uint8_t shMain     = m_RegisterInterface.readReg(0x48); // STATUS_MASTER on main page?
        uint8_t embStatus  = m_RegisterInterface.readReg(0x49);

        m_Logger.info("IF_CFG=0x%02x (I2C_I3C_disable=%d, SHUB_PU_EN=%d)",
                      ifCfg, (ifCfg & 0x01), (ifCfg >> 7) & 0x01);
        m_Logger.info("STATUS_REG=0x%02x (XLDA=%d, GDA=%d, TDA=%d)",
                      statusReg, statusReg & 1, (statusReg >> 1) & 1, (statusReg >> 2) & 1);
        m_Logger.info("STATUS_MASTER_MAINPAGE=0x%02x, EMB_FUNC_STATUS_MAINPAGE=0x%02x",
                      shMain, embStatus);

        switchToEmbeddedPage();
        uint8_t masterCfg  = m_RegisterInterface.readReg(Regs::I2CMasterConfig::reg);
        uint8_t slvAdd     = m_RegisterInterface.readReg(Regs::I2CMasterAddr::reg);
        uint8_t slvSub     = m_RegisterInterface.readReg(Regs::I2CMasterSubAddr::reg);
        uint8_t slvCfg     = m_RegisterInterface.readReg(Regs::I2CMasterSlvConfig::reg);
        uint8_t stMaster   = m_RegisterInterface.readReg(Regs::I2CMasterStatus::reg);
        uint8_t hub1       = m_RegisterInterface.readReg(Regs::I2CMasterDataRd::reg);

        switchToMainPage();

        uint8_t addr7 = (slvAdd >> 1) & 0x7F;
        uint8_t rw    = slvAdd & 0x01;

        m_Logger.info("MASTER_CONFIG=0x%02x (MASTER_ON=%d, WRITE_ONCE=%d, AUX_SENS_ON=%d)",
                      masterCfg,
                      (masterCfg >> 2) & 1,
                      (masterCfg >> 6) & 1,
                      (masterCfg >> 1) & 0x03);
        m_Logger.info("SLV0_ADD=0x%02x (addr7=0x%02x, rw=%d)", slvAdd, addr7, rw);
        m_Logger.info("SLV0_SUBADD=0x%02x", slvSub);
        m_Logger.info("SLV0_CONFIG=0x%02x (SHUB_ODR=%u, numop=%u)",
                      slvCfg,
                      (slvCfg >> 5) & 0x07u,
                      slvCfg & 0x07u);
        m_Logger.info("STATUS_MASTER=0x%02x (ENDOP=%d, NACK bits=0x%x)",
                      stMaster,
                      stMaster & 0x01,
                      (stMaster >> 1) & 0x0F);
        m_Logger.info("SENSOR_HUB_1=0x%02x", hub1);
        m_Logger.info("aux7bitAddr_ (cached) = 0x%02x", aux7bitAddr_);
        m_Logger.info("=== End Sensor Hub State ===");
    }
};

}  // namespace SlimeVR::Sensors::SoftFusion::Drivers
