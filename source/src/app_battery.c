/********************************************************************************************************
 * @file    app_battery.c
 *
 * @brief   Battery voltage ADC
 *
 * @author  haraldapp
 * @date    07,2024
 *
 * @par     Copyright (c) 2024, haraldapp, https://github.com/haraldapp
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
#include "app_config.h"

#if (APP_BATTERY_CHECK)  // component enabled
#include "app.h"
#include "gpio.h"

#ifndef GPIO_VBAT_DETECT
#error "GPIO_VBAT_DETECT not defined in app_config.h"
#endif
#ifndef ADC_INPUT_PCHN
#error "ADC_INPUT_PCHN not defined in app_config.h"
#endif

#ifndef APP_BATTERY_CRITICAL_MV
#define APP_BATTERY_CRITICAL_MV	2200 // critical battery voltage mV
#endif

#ifndef APP_BATTERY_CRITICAL_THRESHOLD
#define APP_BATTERY_CRITICAL_THRESHOLD 200 // mV
#endif

#ifndef APP_BATTERY_CHECK_LOG_EN
#define APP_BATTERY_CHECK_LOG_EN 0
#endif

// we use files from the BLE SDK vendor section "inline"
// The SDK sample code for battery check is pretty good -
// only an interface to read the battery voltage is missing
// and an info about expected time check needs would be good
#define APP_BATT_CHECK_ENABLE 1
#include "vendor/common/battery_check.h"
#include "vendor/common/battery_check.c"

_attribute_data_retention_	u32	app_battery_check_time_sec = 0;
_attribute_data_retention_	u32	app_battery_fail_delay_sec = 0;

static inline u16 app_battery_voltage()
{
	return batt_vol_mv; // battery_check.c
}

void app_battery_init_normal(void) // battery_check.c
{
	int bat_ok; u8 app_state; u16 check_mv=APP_BATTERY_CRITICAL_MV;
	app_state=app_flash_get_persist_state();
	if (app_state & APP_STATE_LOWBAT)
	{
		check_mv+=APP_BATTERY_CRITICAL_THRESHOLD;
	}
	bat_ok=app_battery_check(check_mv);
	if (bat_ok)
	{
		app_flash_set_persist_state(0, APP_STATE_LOWBAT); // reset low battery state
		volatile u16 bat_v=app_battery_voltage();
		app_notify(APP_NOTIFY_BATTERYVOLTAGE, (u8*)&bat_v, 2 );
		app_battery_check_time_sec=app_sec_time(); // battery check interval
	}
	else
	{
		DEBUGFMT(APP_BATTERY_CHECK_LOG_EN, "[BAT] The battery voltage is lower than %dmV - shut down", check_mv);
		app_flash_set_persist_state(APP_STATE_LOWBAT, APP_STATE_LOWBAT);
		cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);  // deepsleep
	}
}

_attribute_ram_code_ void app_battery_init_deepRetn(void)
{
	// ADC setting will be lost during deep sleep
	battery_clear_adc_setting_flag();
}

u8 app_battery_loop(void)
{
	u8 low_bat_state;
	// running on low bat - delayed stop
	low_bat_state=(app_flash_get_persist_state()&APP_STATE_LOWBAT);
	if (low_bat_state && app_sec_time_exceeds(app_battery_fail_delay_sec,APP_BATTERY_FAIL_DELAY_SEC))
	{
		cpu_sleep_wakeup(DEEPSLEEP_MODE, PM_WAKEUP_PAD, 0);  // deep sleep
	}
	// periodic battery check
	if (!low_bat_state && app_sec_time_exceeds(app_battery_check_time_sec,APP_BATTERY_CHECK_INTERVAL_SEC))
	{
		int bat_ok=app_battery_check(APP_BATTERY_CRITICAL_MV);
		volatile u16 bat_v=app_battery_voltage();
		if (bat_ok)
		{
			DEBUGFMT(APP_BATTERY_CHECK_LOG_EN, "[BAT] Measure %u mV", bat_v);
			app_battery_check_time_sec=app_sec_time(); // next check time
		}
		else
		{
			DEBUGFMT(APP_BATTERY_CHECK_LOG_EN, "[BAT] The battery voltage is lower than %dmV - delayed shut down", APP_BATTERY_CRITICAL_MV);
			app_flash_set_persist_state(APP_STATE_LOWBAT, APP_STATE_LOWBAT);
			app_notify(APP_NOTIFY_BATTERYLOW, 0, 0 );
			app_battery_fail_delay_sec=app_sec_time(); // start timer for delayed stop
		}
		app_notify(APP_NOTIFY_BATTERYVOLTAGE, (u8*)&bat_v, 2 );
	}
	return APP_PM_DEFAULT;
}

int app_battery_check(u16 alarm_voltage_mv)
{
	return app_battery_power_check(alarm_voltage_mv); // battery_check.c
}

#endif // #if (APP_BATTERY_CHECK)








