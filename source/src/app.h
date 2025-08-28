/********************************************************************************************************
 * @file    app.h
 *
 * @brief   Project header file
 *
 * @author  haraldapp
 * @date    05,2025
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
#ifndef __APP_H__INCLUDED__
#define __APP_H__INCLUDED__

#include "app_config.h"

#include "types.h"
#include "compiler.h"

#ifndef _attribute_optimize_size_
#define _attribute_optimize_size_  __attribute__((optimize("-Os")))
#endif

// <stdint.h>
#ifndef INT8_MIN
#define INT8_MIN         (-127 - 1)
#define INT16_MIN        (-32767 - 1)
#define INT32_MIN        (-2147483647 - 1)
#define INT8_MAX         127
#define INT16_MAX        32767
#define INT32_MAX        2147483647
#define UINT8_MAX        0xff
#define UINT16_MAX       0xffff
#define UINT32_MAX       0xffffffff
#endif

// some BTHome types
enum { // subset of used value types
	VT_PID = 0x00, // packet id
	VT_BATTERY_PERCENT = 0x01, // battery u8 percent
	VT_TEMPERATURE = 0x02, // temperature i16 0.01 degree
	VT_HUMIDITY = 0x03, // humidity u16 0.01 percent
	VT_VOLTAGE = 0x0C, // voltage u16 0.001 volt
	VT_MOISTURE = 0x14, // moisture u16 0.01 percent
	VT_BINARY_BATTERY = 0x15, // false = normal, true = low
	VT_BINARY_PROBLEM = 0x26, // false = ok, true = problemlow
	VT_TEXT = 0x53, // textlen, ascii text
	VT_NONE = 0xFF
};

// return value for app_xxx_loop functions
#define APP_PM_DEFAULT				0
#define APP_PM_DISABLE_DEEPSLEEP	1
#define APP_PM_DISABLE_SLEEP		2 // stay awake

// app.c
_attribute_no_inline_ void app_init_normal(void);
_attribute_ram_code_ void app_init_deepRetn(void);
_attribute_no_inline_ void app_main_loop(void);
u32 app_sec_time(void); // seconds timer for longer intervals
bool app_sec_time_exceeds(u32 ref, u32 sec);
enum { APP_NOTIFY_NONE=0, APP_NOTIFY_DPDATA, APP_NOTIFY_PRODUCTID, APP_NOTIFY_BATTERYVOLTAGE, APP_NOTIFY_BATTERYLOW,
	   APP_NOTIFY_FACTORYRESET, APP_NOTIFY_REBOOT,
	   APP_NOTIFY_CONNSTATE, APP_NOTIFY_BUTTONPRESS };
void app_notify(u8 evt, const u8 *data, u16 datalen);

// app_debug.c
void app_debug_init(void);
#if (APP_DEBUG_ENABLE)
#define DEBUGSTR(en,info) if(en){tlk_printf("%s\n", info);} // (enable, info)
#define DEBUGFMT(en,fmt,...) if(en){tlk_printf(fmt, ##__VA_ARGS__);putchar('\n');} // tlkapi_printf(enable, fmt, ...)
#define DEBUGHEXBUF(en,info,buf,len) if(en){tlkapi_send_str_data(info,(u8 *)(buf),len);putchar('\n');} // tlkapi_send_string_data(enable, info, data*, datalen)
#define DEBUGOUT(c) putchar(c)
void DEBUGOUTHEX(u8 u);
void DEBUGOUTSTR(const char *txt);
void DEBUGOUTINT(int val, int digits);
int putchar(int c);
int tlk_printf(const char *format, ...);
void tlkapi_send_str_data (char *str, u8 *pData, u32 data_len);
#define tlkapi_printf(en, fmt, ...)	if(en){tlk_printf(fmt, ##__VA_ARGS__);}
#else
#define DEBUGSTR(...)		((void)0)
#define DEBUGFMT(...)		((void)0)
#define DEBUGHEXBUF(...)	((void)0)
#define DEBUGOUT(c)			((void)0)
#define DEBUGOUTHEX(c)		((void)0)
#define DEBUGOUTSTR(c)		((void)0)
#define DEBUGOUTINT(...)	((void)0)
// #define tlkapi_printf(...)	((void)0)
#endif

// app_flash.c
void app_flash_init_normal(void);
_attribute_ram_code_ void app_flash_init_deepRetn(void);
void app_flash_init_mac_address(u8 *mac_public, u8 *mac_random_static);
u32 app_flash_get_mac_storage_sector(void);
u32 app_flash_get_smp_storage_sector(void);
u32 app_flash_get_app_config_sector(void);
#define APP_STATE_LOWBAT 0x01
u8 app_flash_get_persist_state();
void app_flash_set_persist_state(u8 state, u8 mask);
void app_init_deepsleep_retention_sram(void);
bool isAppMemValid(const u8* pMem, u8 len); // check if not 00 or FF
void app_config_init(void);
void app_config_reset(void);
void app_config_flush(void);
const u8* app_config_get_bthome_key(void);
u32 app_config_get_pincode(void);
void app_config_set_pincode(u32 pin);
void app_config_get_key(u8 *key);
void app_config_set_key(const u8 *key);
u8 app_config_create_key(const u8 *randbase);
void app_config_delete_key(void);
signed char app_config_get_power_level(void);
void app_config_set_power_level(signed char level_dbm);
enum {DEVMODE_DEFAULT=0, DEVMODE_MEASURE_NOCONN=0, DEVMODE_MEASURE_CONN, DEVMODE_LAST};
void app_config_set_mode(u8 mode);
u8 app_config_get_mode(void);
enum {DATAFORMAT_DEFAULT=0, DATAFORMAT_BTHOME_V1=1, DATAFORMAT_BTHOME_V2=2, DATAFORMAT_XIAOMI=4};
void app_config_set_dataformat(u8 mode);
u8 app_config_get_dataformat(void);

// app_battery.c
#if (APP_BATTERY_CHECK)
void app_battery_init_normal(void);
_attribute_ram_code_ void app_battery_init_deepRetn(void);
u8 app_battery_loop(void);
int app_battery_check(u16 alarm_voltage_mv);
void app_battery_check_delayed(void);
#endif

// app_ble.c
void app_ble_init_normal(void);
_attribute_ram_code_ void app_ble_init_deepRetn(void);
u8 app_ble_loop(void);
void app_ble_init_device_name(const char* devname);
u8 app_ble_get_security_level(void);
u8 app_ble_device_connected(void);
void app_ble_device_disconnect(void);
void app_ble_device_reset_conn_timeout(void);
void app_ble_device_disconnect_restart(void);
u8 app_ble_device_bond(void);
void app_ble_delete_bond(void);
enum { APP_BLE_CMD_NONE=0, APP_BLE_CMD_DELETEBOND=0x01 };
void app_ble_async_command(u8 cmd);
enum {BLE_ADV_MODE_None=0, BLE_ADV_MODE_Conn, BLE_ADV_MODE_SensorData };
void app_ble_setup_adv(u8 adv_mode);
int app_ble_set_sensor_data(u8 vt, int val, char digits);
void app_ble_set_sensor_data_changed(void);
void app_ble_set_powerlevel(signed char level_dbm);

// app_att.c
#if (APP_BLE_ATT)
void app_ble_att_setup_devinfo(const u8 *devname, u8 devnamelen, u16 appearance);
void app_ble_att_setup_config(void);
void app_ble_att_init(void);
u8 app_ble_att_get_factoryreset(u8 newval);
void app_ble_att_set_battery_data(u8 level);
void app_ble_att_set_bthome_data(const u8 *data, u8 len);
void app_ble_att_set_xiaomi_data(const u8 *data, u8 len);
#endif

// app_serial_mcu.c
#if (APP_MCU_SERIAL)
void mcu_wakeup_init(void);
_attribute_ram_code_ void mcu_wakeup_init_deepRetn(void);
_attribute_ram_code_ u8 module_wakeup_status();
void app_serial_init_normal(void);
void app_serial_init_deepRetn(void);
u8 app_serial_loop(void);
u8 app_serial_rxtx_busy(void);
enum {
	MCU_CMD_SEQ_NONE=0, MCU_CMD_SEQ_INIT, MCU_CMD_SEQ_START_MEASURE,
	MCU_CMD_SEQ_START_CONNECT, MCU_CMD_SEQ_UPDATE_CONNECT,
	MCU_CMD_SEQ_CHECKSTAT
};
void app_serial_cmd_seq_start(u8 cmd_seq, u32 delay);
u8 app_serial_cmd_seq_stat(void);
#endif

#endif // #ifndef __APP_H__INCLUDED__
