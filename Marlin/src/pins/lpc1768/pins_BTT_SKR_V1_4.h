/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * BigTreeTech SKR 1.4 pin assignments
 */

#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "BTT SKR V1.4" //modedy by DA
#endif

#ifndef BOARD_CUSTOM_BUILD_FLAGS
  #define BOARD_CUSTOM_BUILD_FLAGS -DLPC_PINCFG_UART3_P4_28
#endif

#define USES_DIAG_PINS

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  //#define I2C_EEPROM                            // EEPROM on I2C-0
  //#define SDCARD_EEPROM_EMULATION
#endif

#if ENABLED(I2C_EEPROM)
  #define MARLIN_EEPROM_SIZE              0x8000  // 32K
#elif ENABLED(SDCARD_EEPROM_EMULATION)
  #define MARLIN_EEPROM_SIZE               0x800  // 2K
#endif

//
// Servos
//
#define SERVO0_PIN                         P2_00

//
// TMC StallGuard DIAG pins
// Edited by DA 06.05.2021 for custom delta, mechanic max stops, TMC mins stops
//
#define X_DIAG_PIN                         P1_29  // X-STOP
#define Y_DIAG_PIN                         P1_28  // Y-STOP
#define Z_DIAG_PIN                         P1_27  // Z-STOP

//
// Limit Switches
// Edited by DA 06.05.2021 for custom delta, mechanic max stops, TMC mins stops
// HOME_DIR = 1; DELTA always homes to max
//
#if X_HOME_DIR > 0
    #define X_MAX_PIN                      P1_26  		// E0DET
    #define X_MIN_PIN                      X_DIAG_PIN  	// XMIN P1_29
#else
	#define X_MAX_PIN                        X_DIAG_PIN  	// XMIN P1_29
  #define X_MIN_PIN                        P1_26  		// E0DET
#endif

#if Y_HOME_DIR > 0
  #define Y_MAX_PIN                        P1_25  // E1DET
  #define Y_MIN_PIN                        Y_DIAG_PIN  	//P1_28
#else
	#define Y_MAX_PIN                        Y_DIAG_PIN  	//P1_28
  #define Y_MIN_PIN                        P1_25  // E1DET
#endif

#if Z_HOME_DIR > 0
  #define Z_MAX_PIN                        P1_00  // PWRDET
  #define Z_MIN_PIN                        Z_DIAG_PIN  	//P1_27
#else
	#define Z_MAX_PIN                        Z_DIAG_PIN  	//P1_27
  #define Z_MIN_PIN                        P1_00  // PWRDET
#endif

//
// Steppers
//
#define X_STEP_PIN                         P2_02
#define X_DIR_PIN                          P2_06
#define X_ENABLE_PIN                       P2_01
#ifndef X_CS_PIN
  #define X_CS_PIN                         P1_10
#endif

#define Y_STEP_PIN                         P0_19
#define Y_DIR_PIN                          P0_20
#define Y_ENABLE_PIN                       P2_08
#ifndef Y_CS_PIN
  #define Y_CS_PIN                         P1_09
#endif

#define Z_STEP_PIN                         P0_22
#define Z_DIR_PIN                          P2_11
#define Z_ENABLE_PIN                       P0_21
#ifndef Z_CS_PIN
  #define Z_CS_PIN                         P1_08
#endif

#define E0_STEP_PIN                        P2_13
#define E0_DIR_PIN                         P0_11
#define E0_ENABLE_PIN                      P2_12
#ifndef E0_CS_PIN
  #define E0_CS_PIN                        P1_04
#endif

#define E1_STEP_PIN                        P1_15
#define E1_DIR_PIN                         P1_14
#define E1_ENABLE_PIN                      P1_16
#ifndef E1_CS_PIN
  #define E1_CS_PIN                        P1_01
#endif

#define TEMP_1_PIN                      P0_23_A0  // A0 (T0) - (67) - TEMP_1_PIN
#define TEMP_BED_PIN                    P0_25_A2  // A2 (T2) - (69) - TEMP_BED_PIN

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Z_HARDWARE_SERIAL  Serial1


  #define X_SERIAL_TX_PIN                  P1_10
  #define X_SERIAL_RX_PIN        X_SERIAL_TX_PIN

  #define Y_SERIAL_TX_PIN                  P1_09
  #define Y_SERIAL_RX_PIN        Y_SERIAL_TX_PIN

  #define Z_SERIAL_TX_PIN                  P1_08
  #define Z_SERIAL_RX_PIN        Z_SERIAL_TX_PIN

  #define E0_SERIAL_TX_PIN                 P1_04
  #define E0_SERIAL_RX_PIN      E0_SERIAL_TX_PIN

  #define E1_SERIAL_TX_PIN                 P1_01
  #define E1_SERIAL_RX_PIN      E1_SERIAL_TX_PIN

  // Reduce baud rate to improve software serial reliability
  #define TMC_BAUD_RATE                    19200
#endif

/**       ------                ------
 *  1.30 | 1  2 | 0.28    0.17 | 1  2 | 0.15
 *  1.18 | 3  4 | 1.19    3.26 | 3  4 | 0.16
 *  1.20   5  6 | 1.21    3.25   5  6 | 0.18
 *  1.22 | 7  8 | 1.23    1.31 | 7  8 | RESET
 *   GND | 9 10 | 5V       GND | 9 10 | --
 *        ------                ------
 *         EXP1                  EXP2
 */
#define EXP1_01_PIN                        P1_30
#define EXP1_02_PIN                        P0_28
#define EXP1_03_PIN                        P1_18
#define EXP1_04_PIN                        P1_19
#define EXP1_05_PIN                        P1_20
#define EXP1_06_PIN                        P1_21
#define EXP1_07_PIN                        P1_22
#define EXP1_08_PIN                        P1_23

#define EXP2_01_PIN                        P0_17
#define EXP2_02_PIN                        P0_15
#define EXP2_03_PIN                        P3_26
#define EXP2_04_PIN                        P0_16
#define EXP2_05_PIN                        P3_25
#define EXP2_06_PIN                        P0_18
#define EXP2_07_PIN                        P1_31
#define EXP2_08_PIN                        -1     // RESET

#if HAS_ADC_BUTTONS
  #error "ADC BUTTONS do not work unmodified on SKR 1.4, The ADC ports cannot take more than 3.3v."
#endif

//
// NeoPixel LED
//
#ifndef NEOPIXEL_PIN
  #define NEOPIXEL_PIN                     P1_24 //Powered neopixel socket TODO:Neopixels on frame
  #define NEOPIXEL2_PIN                    P0_10 //Unpowered neopixel socket TODO:Neopixels on effector
#endif

/**
 * Special pins
 *   P1_30  (37) (NOT 5V tolerant)
 *   P1_31  (49) (NOT 5V tolerant)
 *   P0_27  (57) (Open collector)
 *   P0_28  (58) (Open collector)
 */

//
// Include common SKR pins
//
#include "pins_BTT_SKR_common.h"
