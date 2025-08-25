/********************************************************************************************************
 * @file    main.c
 *
 * @brief   Main entry point
 *
 * @author  haraldapp
 * @date    01,2025
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


#include "app_config.h"

#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "common/compiler.h"

#include "app.h"

#ifndef APP_LOG_EN
#define APP_LOG_EN 0
#endif
#ifndef APP_PM_LOG_EN
#define APP_PM_LOG_EN 0
#endif
#ifndef APP_DPDATA_LOG_EN
#define APP_DPDATA_LOG_EN 0
#endif


// BLE stack RX/TX FIFO (must)
#define RX_FIFO_SIZE	64 // CAL_LL_ACL_RX_BUF_SIZE(maxRxOct): maxRxOct + 22, then 16 byte align
#define RX_FIFO_NUM		8 // recommended values: 4, 8, 16
#define TX_FIFO_SIZE	40 // CAL_LL_ACL_TX_BUF_SIZE(maxTxOct):  maxTxOct + 10, then 4 byte align
#define TX_FIFO_NUM		16 // values: 8, 16, 32
_attribute_data_retention_  u8 		 	blt_rxfifo_b[RX_FIFO_SIZE * RX_FIFO_NUM] = {0};
_attribute_data_retention_	my_fifo_t	blt_rxfifo = {RX_FIFO_SIZE, RX_FIFO_NUM, 0, 0, blt_rxfifo_b, };
_attribute_data_retention_  u8 		 	blt_txfifo_b[TX_FIFO_SIZE * TX_FIFO_NUM] = {0};
_attribute_data_retention_	my_fifo_t	blt_txfifo = {TX_FIFO_SIZE, TX_FIFO_NUM, 0,	0, blt_txfifo_b, };

// App states
enum { PM_MODE_NONE=0, PM_MODE_ALIVE, PM_MODE_SLEEP, PM_MODE_DEEPSLEEP };
static _attribute_data_retention_ u8 app_pm_mode = PM_MODE_NONE;

enum { APP_STATE_NONE, APP_STATE_INIT, APP_STATE_CONNPAIR, APP_STATE_MEASURE };
#define APP_STATE_PAIR_TIMEOUT 59 // 59 sec
static _attribute_data_retention_ u8 app_state = APP_STATE_NONE;
static u32 app_state_clock = 0;

enum { DEVICETYPE_None=0, DEVICETYPE_Unknown, DEVICETYPE_SGS01 };
static _attribute_data_retention_ u8 app_device_type = DEVICETYPE_None;
#if (APP_LOG_EN)
static const char *dbg_device_type_name[]={"", "<unknown>", "SGS01"};
#endif

//
// One second timer (for longer intervals)
//
_attribute_data_retention_	u32	app_sec_time_tick   = 0;
_attribute_data_retention_	u32	app_sec_time_cnt   = 0;

u32 app_sec_time(void)
{
	return app_sec_time_cnt;
}

bool app_sec_time_exceeds(u32 ref, u32 sec)
{
	return ((app_sec_time() - ref) > sec);
}

static inline void app_sec_time_update(void)
{
	while (clock_time_exceed(app_sec_time_tick,1000000))
	{
		app_sec_time_tick+=CLOCK_16M_SYS_TIMER_CLK_1S;
		app_sec_time_cnt++;
	}
}

//
// Power Management
//
void app_set_pm_mode(u8 pm_mode)
{
	if (pm_mode == app_pm_mode)   return;
	u8 suspend_mask=SUSPEND_DISABLE;
	if (pm_mode == PM_MODE_ALIVE)       suspend_mask=SUSPEND_DISABLE;
	if (pm_mode == PM_MODE_SLEEP)		suspend_mask=(SUSPEND_ADV | SUSPEND_CONN);
	if (pm_mode == PM_MODE_DEEPSLEEP)	suspend_mask=(SUSPEND_ADV | SUSPEND_CONN | DEEPSLEEP_RETENTION_ADV | DEEPSLEEP_RETENTION_CONN);
	bls_pm_setSuspendMask(suspend_mask);
    #if (APP_PM_LOG_EN)
	static const char *pm_mode_name[]={"", "stay alive", "sleep adv", "deepsleep adv"};
	DEBUGFMT(APP_PM_LOG_EN, "|APP] PM %s", pm_mode_name[pm_mode]);
    #endif
    app_pm_mode=pm_mode;
}

// PM statistics
#if (APP_PM_LOG_EN)
_attribute_data_retention_	u32	app_start_work_time_tick = 0;
_attribute_data_retention_	u32	app_start_sleep_time_tick = 0;

static void app_pm_stat_work()
{
	if (app_start_work_time_tick && app_start_sleep_time_tick)
	{
		u32 work_us=(app_start_sleep_time_tick-app_start_work_time_tick)/16;
		u32 sleep_us=(clock_time()-app_start_sleep_time_tick)/16;
	    DEBUGFMT(APP_PM_LOG_EN, "|APP] PM Stat: work %u us, sleep %u us", work_us, sleep_us);
		app_start_work_time_tick = 0; app_start_sleep_time_tick = 0;
	}
	if (!app_start_work_time_tick)
		app_start_work_time_tick=clock_time() | 1;
}

static void app_pm_stat_sleep()
{
    if (app_start_work_time_tick && !app_start_sleep_time_tick)
    	app_start_sleep_time_tick=clock_time()|1;
}

static _attribute_ram_code_ _attribute_no_inline_ int app_pm_suspend_enter_cb(void)
{
	// run/sleep time statistics
	if (app_pm_mode == PM_MODE_DEEPSLEEP)
	{
       // DEBUGSTR(APP_PM_LOG_EN, "|APP] PM Suspend enter");
       app_pm_stat_sleep();
	}
	return 1;
}
#endif // #if (APP_PM_LOG_EN)

//
// App working states
//

static u8 app_toogle_state(u8 newstate)
{
	if (newstate == APP_STATE_MEASURE || (newstate == APP_STATE_NONE && app_state == APP_STATE_CONNPAIR))
	{
		DEBUGSTR(APP_LOG_EN, "|APP] Switch to AppState measure");
		app_ble_setup_adv(BLE_ADV_MODE_BTHome);
		#if (APP_MCU_SERIAL)
		app_serial_cmd_seq_start(MCU_CMD_SEQ_START_MEASURE, 60000);
		#endif
		app_state=APP_STATE_MEASURE;
		return 1;
	}
	if (newstate == APP_STATE_CONNPAIR || (newstate == APP_STATE_NONE && app_state == APP_STATE_MEASURE))
	{
		if (app_state != APP_STATE_CONNPAIR)
		{
			DEBUGSTR(APP_LOG_EN, "|APP] Switch to AppState conn/pair");
			app_ble_setup_adv(BLE_ADV_MODE_Conn);
			#if (APP_MCU_SERIAL)
			app_serial_cmd_seq_start(MCU_CMD_SEQ_START_CONNECT, 60000);
			#endif
		}
		else
		{
			DEBUGSTR(APP_LOG_EN, "|APP] Update AppState connect/pair");
			#if (APP_MCU_SERIAL)
			app_serial_cmd_seq_start(MCU_CMD_SEQ_UPDATE_CONNECT, 60000);
			#endif
		}
		app_state = APP_STATE_CONNPAIR; app_state_clock = app_sec_time();
		return 1;
	}
	return 0;
}

static u8 app_handle_state()
{
	#if (APP_MCU_SERIAL)
	if (app_serial_cmd_seq_stat()!=0)
		return APP_PM_DISABLE_SLEEP; // mcu serial cmd sequence busy
	#endif
	if (app_state == APP_STATE_INIT)
	{
		u8 bond=app_ble_device_bond();
		if (bond)
			app_toogle_state(APP_STATE_MEASURE); // go direct to measure mode
		else
			app_toogle_state(APP_STATE_CONNPAIR);
		return APP_PM_DISABLE_SLEEP;
	}
	if (app_state == APP_STATE_CONNPAIR)
	{
		if (app_sec_time_exceeds(app_state_clock,APP_STATE_PAIR_TIMEOUT))
		{
			if (app_ble_device_connected())
			{
				app_toogle_state(APP_STATE_CONNPAIR); // send state to mcu (keep LED blinking)
			}
			else
			{
				DEBUGSTR(APP_LOG_EN, "|APP] Conn/Pairing timeout");
				app_toogle_state(APP_STATE_MEASURE);
			}
		}
		return APP_PM_DISABLE_SLEEP;
	}
	#if (APP_MCU_SERIAL)
	if (module_wakeup_status()!=0)   return APP_PM_DISABLE_SLEEP;
	#endif
	return APP_PM_DEFAULT;
}


//
// App interface
//

// initialization when power on or wake up from DeepSleep (called from main.c)
_attribute_no_inline_ void app_init_normal(void)
{
    // basic hardware
	random_generator_init(); // mandatory, must be first
	// debug init
	app_debug_init();
	#if (APP_LOG_EN)
	static const char dbg_version[] = {VERSION_STR VERSION_STR_BUILD}; // app_config.h
	DEBUGFMT(APP_LOG_EN, "-----------------");
    DEBUGFMT(APP_LOG_EN, "|APP] Version %s", dbg_version);
    DEBUGSTR(APP_LOG_EN, "|APP] Init start");
	#endif
	#if (APP_PM_LOG_EN)
    app_pm_stat_work();
	#endif
    // some short delay from MCU startup and show led on
    u32 init_delay=clock_time();
	while (!clock_time_exceed(init_delay,100000)) ;
	// Flash init, load calibration
	app_flash_init_normal();
	// Battery init and check
	#if (APP_BATTERY_CHECK)
	app_battery_init_normal();
    #endif
	// Read app config from flash (must: after battery check)
	app_config_init();
	// BLE init (must: after battery check)
	app_ble_init_normal();
	// MCU serial init
    #if (APP_MCU_SERIAL)
	app_serial_init_normal();
    #endif
	// Power management
	blc_ll_initPowerManagement_module();
    app_init_deepsleep_retention_sram(); // app_flash.c: setup retention size 16k/32k
	blc_pm_setDeepsleepRetentionThreshold(95, 95);
	blc_pm_setDeepsleepRetentionEarlyWakeupTiming(270);
	#if (APP_MCU_SERIAL)
	mcu_wakeup_init(); // init pins for MCU/module wake up
	#endif
	app_pm_mode=PM_MODE_NONE; app_set_pm_mode(PM_MODE_ALIVE); // set power management mode (alive/sleep/deepsleep)
    #if (APP_PM_LOG_EN)
    bls_pm_registerFuncBeforeSuspend(&app_pm_suspend_enter_cb);
    #endif
	// Check for controller or host initialization error
	u32 error_controller = blc_contr_checkControllerInitialization();
	u32 error_host = blc_host_checkHostInitialization();
	if(error_controller != INIT_SUCCESS || error_host != INIT_SUCCESS)
	{
		DEBUGFMT(APP_LOG_EN, "[APP] INIT ERROR 0x%04x, 0x%04x", error_controller, error_host);
		while(1);
	}
    DEBUGSTR(APP_LOG_EN, "|APP] Init end");
    // start
	#if (APP_MCU_SERIAL)
    app_state = APP_STATE_INIT; app_state_clock=0;
    app_serial_cmd_seq_start(MCU_CMD_SEQ_INIT, 300000); // delay 300ms
	#else
    app_state = APP_STATE_CONNPAIR; app_state_clock=clock_time();
	#endif
	irq_enable();
}

// initialization when wake up from deepSleep_retention mode (called from main.c)
_attribute_ram_code_ void app_init_deepRetn(void)
{
	// basic
	blc_app_loadCustomizedParameters_deepRetn();
	blc_ll_initBasicMCU(); // mandatory
	blc_ll_recoverDeepRetention();
	#if (APP_MCU_SERIAL)
	mcu_wakeup_init_deepRetn();
	#endif
	// debug
	app_debug_init();
	DEBUGFMT(APP_PM_LOG_EN, "|APP] Init deepRetn %u sec", app_sec_time_cnt);
	// Flash
	app_flash_init_deepRetn();
	// BLE
	app_ble_init_deepRetn();
	// MCU serial (first - BLE takes some time)
    #if (APP_MCU_SERIAL)
	app_serial_init_deepRetn();
    #endif
	// Battery
	#if (APP_BATTERY_CHECK)
	app_battery_init_deepRetn();
	#endif
	// PM
	#if (APP_PM_LOG_EN)
	app_pm_stat_work();
	#endif
	app_set_pm_mode(PM_MODE_ALIVE); // set power management mode (alive/sleep/deepsleep)
	irq_enable();
}

// main loop (called from main.c)
_attribute_no_inline_ void app_main_loop(void)
{
	u8 pm_flags=APP_PM_DEFAULT;
	// SDK
	blt_sdk_main_loop();
	// one second timer (for longer intervals)
	app_sec_time_update();
	// component loops
	//   battery
	#if (APP_BATTERY_CHECK)
	pm_flags|=app_battery_loop();
	#endif
	//   BLE
	pm_flags|=app_ble_loop();
	//   serial
    #if (APP_MCU_SERIAL)
	pm_flags |= app_serial_loop();
    #endif
	//   app running states
	pm_flags |= app_handle_state();
	// set PM mode (SDK handles sleep)
	u8 pm_mode=PM_MODE_DEEPSLEEP;
	if (pm_flags & APP_PM_DISABLE_DEEPSLEEP)	pm_mode=PM_MODE_SLEEP;
	if (pm_flags & APP_PM_DISABLE_SLEEP)		pm_mode=PM_MODE_ALIVE;
 	app_set_pm_mode(pm_mode);
	// write changed configuration to flash
 	u8 rxtx_busy=0;
	#if (APP_MCU_SERIAL)
 	rxtx_busy=app_serial_rxtx_busy();
	#endif
 	if (!rxtx_busy)   app_config_flush();
}


//
// button handler
//

_attribute_data_retention_	int app_user_button_state  = -1;

static void app_handle_user_button(int val)
{
	if (val < 0)   return;
	// retrieve button presses from value changes
	if (app_user_button_state >= 0 && app_user_button_state != val)
		app_notify(APP_NOTIFY_BUTTONPRESS, 0, 0);
	app_user_button_state = val;
}

//
// data handler
//

static const u8 pid_sgs01[8]={'g','v','y','g','g','3','m','8'};

enum {
	DPTYPE_RAW=0,		// datalen 1...255
	DPTYPE_BOOL=1,		// datalen 1
	DPTYPE_VALUE=2,		// datalen 4
	DPTYPE_STRING=3,	// datalen 0...255
	DPTYPE_ENUM=4,		// datalen 1
};
typedef struct _attribute_packed_ { u8 dpid; u8 dptype; u8 vt_bthome; u8 digits; } dp_def_t;

#define VT_USER        0xF0
#define VT_USER_BUTTON 0xFE // special button handler

// Tuya DP to BTHome data definitions
static const dp_def_t sgs01_dp_def[]= { // SGS01 Tuya data points -> BTHome data
	{3,  2, VT_MOISTURE, 0 },			// Soil Moisture: 1%
	{5,  2, VT_TEMPERATURE, 1 },		// Temperature: 0.1 °C
//	{6,  4, VT_xxx, 0 },				// ?: values 0
//	{9,  4, VT_xxx, 0 }, 				// Temperature Unit: 0=°C, 1=°F
	{9,  4, VT_USER_BUTTON, 0 },		// supposing: short button press will change temp unit
//	{14, 4, VT_xxx, 0 },				// ? values:2
	{15, 2, VT_BATTERY_PERCENT, 0 },	// Battery Level: 1%
	{0, 0, 0, 0}
};

typedef struct _attribute_packed_ { u8 dpid; u8 dptype; u8 dplen_h; u8 dplen_l; } dp_header_t;

int get_val_be(const u8 *data, u8 len)
{
	int val=0;
	if (len==1) {
		val=data[0];
	} else if (len==2) {
		u16 v=data[0]; v<<=8; v|=data[1]; val=v;
	} else if (len==4) {
		val=data[0]; val<<=8; val|=data[1];	val<<=8;
		val|=data[2]; val<<=8; val|=data[3];
	}
	return val;
}

static void set_dp_data(const dp_def_t *dpdef, const u8 *data, u16 datalen)
{
	if (datalen > 1)
	{
		data++; datalen--; // flags 0x01 = report to cloud+panel
	}
	while (datalen >= 3)
	{
		const dp_header_t *hdr=(const dp_header_t *)data; // DP-ID, DP-Type, DataLen, Data
		data+=sizeof(dp_header_t); datalen-=sizeof(dp_header_t);
		const u8 *dpdata=data;
		u16 dplen=hdr->dplen_h; dplen<<=8; dplen|=hdr->dplen_l;	if (dplen>datalen)   break;
        data+=dplen; datalen-=dplen;
		if (dplen>4)   continue; // DP data size not implemented
		for (u8 u=0; dpdef[u].dpid; u++)
		{   // find DP to BTHome data mapping
			if (dpdef[u].dpid != hdr->dpid)   continue;
			if (dpdef[u].dptype != hdr->dptype)   continue;
            int val = get_val_be(dpdata, dplen);
            if (dpdef[u].vt_bthome < VT_USER)
			   app_ble_set_bthome_data(dpdef[u].vt_bthome, val, dpdef[u].digits);
            else if (dpdef[u].vt_bthome == VT_USER_BUTTON)
               app_handle_user_button(val);
			break;
		}
	}
}

#if (APP_DPDATA_LOG_EN)
static void DEBUG_DPDATA(const u8 *data, u16 datalen)
{
	if (datalen > 1)
	{
		DEBUGFMT(APP_DPDATA_LOG_EN, "|APP] DP Data: flags=%02X", *data);
		data++; datalen--;
	}
	while (datalen>=3)
	{
		const dp_header_t *hdr=(const dp_header_t *)data; // DP-ID, DP-Type, DataLen, Data
		data+=sizeof(dp_header_t); datalen-=sizeof(dp_header_t);
		const u8 *dpdata=data; u16 dplen=hdr->dplen_h; dplen<<=8; dplen|=hdr->dplen_l;	if (dplen>datalen)   break;
        data+=dplen; datalen-=dplen;
    	DEBUGFMT(APP_DPDATA_LOG_EN,    "|APP] DP Data: dpid=%02X, dptype=%02X, dplen=%u", hdr->dpid, hdr->dptype, dplen);
    	DEBUGHEXBUF(APP_DPDATA_LOG_EN, "|APP]          data=%s", dpdata, dplen);
	}
}
#endif


void app_notify(u8 evt, const u8 *data, u16 datalen)
{
	switch (evt)
	{
		case APP_NOTIFY_PRODUCTID:
		{
			u8 device_type=DEVICETYPE_Unknown;
			if (memcmp(data,pid_sgs01,8) == 0)	device_type=DEVICETYPE_SGS01;
		    DEBUGFMT(APP_LOG_EN, "|APP] Device type %s", dbg_device_type_name[device_type]);
		    if (device_type==DEVICETYPE_SGS01)   app_ble_init_device_name("SGS01");
			app_device_type=device_type;
		} break;
		case APP_NOTIFY_DPDATA:
			#if (APP_DPDATA_LOG_EN)
			DEBUG_DPDATA(data, datalen);
			#endif
			if (app_device_type == DEVICETYPE_SGS01)
				set_dp_data(sgs01_dp_def, data, datalen);
		    break;
		case APP_NOTIFY_BATTERYVOLTAGE: // data: u16 (mV)
			app_ble_set_bthome_data(VT_VOLTAGE, *(const u16 *)data, 3);
			break;
		case APP_NOTIFY_BATTERYLOW:
			app_ble_set_bthome_data(VT_BATTERY_PERCENT, 0, 0); // report bat level 0%
			app_ble_set_bthome_data(VT_BINARY_BATTERY, 1, 0); // report low bat
			break;
		case APP_NOTIFY_FACTORYRESET:
		    DEBUGSTR(APP_LOG_EN, "|APP] Factory Reset");
			app_ble_device_disconnect();
			app_config_reset(); // first
			#if (APP_BLE_ATT)
			app_ble_att_setup_config(); // set new config values
			#endif
			app_ble_delete_bond();
		    app_state = APP_STATE_INIT; app_state_clock=0;
			break;
		case APP_NOTIFY_REBOOT:
		    DEBUGSTR(APP_LOG_EN, "|APP] Reboot");
			start_reboot();
			break;
		case APP_NOTIFY_CONNSTATE:
		{
			u8 state_new=data[0], state_old=data[1];
		    if (app_state == APP_STATE_CONNPAIR && state_new==0 && state_old!=0)
		    	app_toogle_state(APP_STATE_CONNPAIR); // hold conn state on disconnect
		} break;
		case APP_NOTIFY_BUTTONPRESS:
		    DEBUGSTR(APP_LOG_EN, "|APP] Button press");
		    app_toogle_state(0);
			break;
	}
}



