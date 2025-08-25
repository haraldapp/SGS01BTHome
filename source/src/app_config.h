/********************************************************************************************************
 * @file    app_config.h
 *
 * @brief   Project configuration
 *
 * @author  haraldapp
 * @date    03,2025
 *
 * @par     Copyright (c) 2025, haraldapp, https://github.com/haraldapp
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *              http://www.apache.org/licenses/LICENSE-2.0
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *******************************************************************************************************/
#ifndef __APP_CONFIG_H__INCLUDED__
#define __APP_CONFIG_H__INCLUDED__

#define VERSION_STR "V1.0"

#if defined(APP_DEBUG_ENABLE) && (APP_DEBUG_ENABLE)
#define VERSION_STR_BUILD "debug"
#else
#define VERSION_STR_BUILD ""
#endif

#define BTHOME_ADV_INTERVAL 		(ADV_INTERVAL_1S * 8)  // 8 sec, ADV mode noconn (max. 8s)
#define BTHOME_CONN_ADV_INTERVAL 	(ADV_INTERVAL_1S * 3)  // 3 sec, ADV mode direct
#define BLE_CONNECTION_TIMEOUT_SEC  (4*60) // 4 min

// App modules
#define APP_BATTERY_CHECK	1   // Battery measure and check
#define APP_MCU_SERIAL 		1   // Third party MCU, serial UART with Tuya protocol
#define APP_BLE_ATT         1   // BLE GATT

// App features
#define BLE_APP_PM_ENABLE				1
#define PM_DEEPSLEEP_RETENTION_ENABLE	1
#define BLE_APP_SECURITY_ENABLE      	1 // ACL Slave device SMP, strongly recommended enabled
#define BLE_OTA_SERVER_ENABLE			1
#define BLE_ATT_CRYPTKEY_CHANGE_ENABLE	1 // Allow to change BTHome encryption key

// RF Power Level
#define RF_POWER_LEVEL_DEFAULT 3 // dbm

// Deep save register
#define USED_DEEP_ANA_REG	DEEP_ANA_REG0	// u8, can save 8 bit info when deep
#define	LOW_BATT_FLG		BIT(0)			// if 1: low battery
#define CONN_DEEP_FLG		BIT(1)			// if 1: conn deep, 0: ADV deep

// System Clock
#define CLOCK_SYS_CLOCK_HZ	16000000

// Watchdog
#define MODULE_WATCHDOG_ENABLE		0
#define WATCHDOG_INIT_TIMEOUT		500  //ms

// DEBUG  Configuration
#ifndef APP_DEBUG_ENABLE
#define APP_DEBUG_ENABLE                    0  // set global for debug build
#endif
#if (APP_DEBUG_ENABLE)
#define APP_LOG_EN							1
#define APP_PM_LOG_EN						1  // power management
#define APP_BLE_LOG_EN						1  // BLE
#define APP_FLASH_LOG_EN					1
#define APP_FLASH_DEBUG_EN					1
#define APP_FLASH_PROT_LOG_EN				0
#define APP_ATT_LOG_EN						1  // BLE GAP/GATT attributes
#define APP_SMP_LOG_EN						1  // BLE security manager
#define APP_KEY_LOG_EN						1
#define APP_BLE_EVENT_LOG_EN				1  // controller event log
#define APP_HOST_EVENT_LOG_EN				1  // host event log
#define APP_OTA_LOG_EN						1
#define APP_BATTERY_CHECK_LOG_EN			1
#define APP_SERIAL_LOG_EN					1
#define APP_SERIAL_DEBUG_EN					1
#define APP_BTHOME_LOG_EN					1
#define APP_DPDATA_LOG_EN					0
#endif

// DEBUG Setup
#if (APP_DEBUG_ENABLE)
#define DEBUG_SWS_PIN		GPIO_PA7
#define DEBUG_INFO_TX_PIN	GPIO_PC3
#endif

// battery check
#define GPIO_VBAT_DETECT				GPIO_PB4 // TL825x: GPIO pin needed to check supply voltage
#define ADC_INPUT_PCHN 					B4P		 // corresponding ADC_InputPchTypeDef
#define APP_BATTERY_LOW_MV				2600 // low battery voltage mV
#define APP_BATTERY_CRITICAL_MV			2200 // critical battery voltage mV
#define APP_BATTERY_CHECK_INTERVAL_SEC  120  // 2 minutes
#define APP_BATTERY_FAIL_DELAY_SEC		30   // battery critical - delayed stop

// UART Serial
#define UART_TX_PIN		UART_TX_PB1
#define UART_RX_PIN		UART_RX_PB7
#define UART_BAUDRATE	9600

// Idle/WakeUp Pins
#define MODULE_WAKEUP_PIN	GPIO_PB5 // high to wake up module to receive notifications
#define MCU_WAKEUP_PIN		GPIO_PD2 // high to wake up MCU and send commands


#endif // #ifndef __APP_CONFIG_H__INCLUDED__
