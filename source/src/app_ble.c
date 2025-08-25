/********************************************************************************************************
 * @file    app_ble.c
 *
 * @brief   Bluetooth LE handler
 *
 * @author  haraldapp
 * @date    08,2024
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
#include "app.h"
#include "compiler.h"

#include "app_config.h"

#include "drivers.h"
#include "stack/ble/ble.h"
#include "stack/ble/ble_common.h"
#if (BLE_OTA_SERVER_ENABLE)
#include "stack/ble/service/ota/ota.h"
#include "stack/ble/service/ota/ota_server.h"
#endif

#include "crypt/ccm.h" // encryption

#ifndef APP_BLE_LOG_EN
#define APP_BLE_LOG_EN 0
#endif
#ifndef APP_BLE_EVENT_LOG_EN
#define APP_BLE_EVENT_LOG_EN 0
#endif
#ifndef APP_SMP_LOG_EN
#define APP_SMP_LOG_EN 0
#endif
#ifndef APP_OTA_LOG_EN
#define APP_OTA_LOG_EN 0
#endif
#ifndef BLE_OTA_SERVER_ENABLE
#define BLE_OTA_SERVER_ENABLE 0
#endif

// Connection mode
#define 	BLE_CONN_ADV_INTERVAL_MIN			ADV_INTERVAL_30MS
#define 	BLE_CONN_ADV_INTERVAL_MAX			ADV_INTERVAL_35MS
#define		BLE_ADV_FLAGS                       0x05 // limited discoverable, BR/EDR not supported
#define		BLE_DEVICE_ADDRESS_TYPE 			BLE_DEVICE_ADDRESS_PUBLIC

// BTHome data mode
#ifndef BTHOME_ADV_INTERVAL
#define BTHOME_ADV_INTERVAL 		(ADV_INTERVAL_1S * 6) // 6 sec
#endif
#ifndef BTHOME_CONN_ADV_INTERVAL
#define BTHOME_CONN_ADV_INTERVAL 	(ADV_INTERVAL_1S * 3) // 3 sec, ADV mode direct
#endif

// States
_attribute_data_retention_	own_addr_type_t	ble_own_address_type = OWN_ADDRESS_PUBLIC;
_attribute_data_retention_  u8 ble_mac_public[6], ble_mac_random_static[6];
enum {BLE_OTA_NONE = 0, BLE_OTA_WORK, BLE_OTA_WAIT, BLE_OTA_EXTENDED};
_attribute_data_retention_	u8 ble_ota_is_working = BLE_OTA_NONE;
_attribute_data_retention_	u8 ble_adv_mode = BLE_ADV_MODE_None;
_attribute_data_retention_	u8 ble_async_cmd = APP_BLE_CMD_NONE;

//
// BTHome data
//
enum { BTH_FLAG_PID=0x01, BTH_FLAG_BAT=0x02,
	   BTH_FLAG_TEMP=0x04, BTH_FLAG_VOLT=0x08,
	   BTH_FLAG_MOIST=0x10,
	   BTH_FLAG_CHANGED=0x080,
	   BTH_FLAGS_DATAVALID=0x7F,
};

_attribute_data_retention_ struct {
	u8    flags;
	u8    pid; 				// VT_PID VD_UINT digits=0
	u8    batterypercent;	// VT_BATTERY_PERCENT VD_UINT digits=0
	short temperature;		// VT_TEMPERATURE VD_INT digits=2
	u16   voltage;			// VT_VOLTAGE VD_UINT digits=3
	u16   moisture;			// VT_MOISTURE VD_UINT digits=2
} bthome_data = {0, 0, 0, 0, 0, 0};

_attribute_data_retention_ u32 bthome_data_sendcount = 0;

void bthome_increment_packetid()
{
	if ((bthome_data.flags&BTH_FLAG_PID)==0)   bthome_data.pid=0;
	bthome_data.pid++; bthome_data.flags|=(BTH_FLAG_PID|BTH_FLAG_CHANGED);
}

static int bthome_adjust_digits(int val, char digits, char dest_digits)
{
	while (digits > dest_digits && val != 0) {
		val /= 10; digits--;
	}
	while (digits < dest_digits) {
		if ( val < INT32_MIN/10 )  { val=INT32_MIN; break; }
		if ( val > INT32_MAX/10 )  { val=INT32_MAX; break; }
		val *= 10; digits++;
	}
	return val;
}

int app_ble_set_bthome_data(u8 vt, int val, char digits)
{
	if (vt==VT_BATTERY_PERCENT) {
		val=bthome_adjust_digits(val, digits, 0);
		if (val<0 || val>100)    return -1;
		if ((bthome_data.flags&BTH_FLAG_BAT)!=0 && val==bthome_data.batterypercent)   return 0;
		DEBUGFMT(APP_BLE_LOG_EN, "[BLE] Data battery %u %%", val);
		bthome_data.batterypercent=(u8)val;
		bthome_data.flags|=BTH_FLAG_BAT|BTH_FLAG_CHANGED;
		#if (APP_BLE_ATT)
		app_ble_att_set_battery_data((u8)val); // GATT value and push notification
		#endif
		return 1;
	}
	if (vt==VT_TEMPERATURE) {
		val=bthome_adjust_digits(val, digits, 2);
		if (val<INT16_MIN || val>INT16_MAX)    return -1;
		if ((bthome_data.flags&BTH_FLAG_TEMP)!=0 && val==bthome_data.temperature)   return 0;
		DEBUGFMT(APP_BLE_LOG_EN, "[BLE] Data temperature %d.%02u C", val/100, abs(val)%100);
		bthome_data.temperature=(short)val;
		bthome_data.flags|=BTH_FLAG_TEMP|BTH_FLAG_CHANGED;
		return 1;
	}
	if (vt==VT_VOLTAGE) {
		val=bthome_adjust_digits(val, digits, 3);
		if (val<0 || val>UINT16_MAX)    return -1;
		if ((bthome_data.flags&BTH_FLAG_VOLT)!=0 && val==bthome_data.voltage)   return 0;
		DEBUGFMT(APP_BLE_LOG_EN, "[BLE] Data voltage %d mV", val);
		bthome_data.voltage=(u16)val;
		bthome_data.flags|=BTH_FLAG_VOLT|BTH_FLAG_CHANGED;
		return 1;
	}
	if (vt==VT_MOISTURE) {
		val=bthome_adjust_digits(val, digits, 2);
		if (val<0 || val>100*100)    return -1;
		if ((bthome_data.flags&BTH_FLAG_MOIST)!=0 && val==bthome_data.moisture)   return 0;
		DEBUGFMT(APP_BLE_LOG_EN, "[BLE] Data moisture %d.%02u %%", val/100, abs(val)%100);
		bthome_data.moisture=(u16)val;
		bthome_data.flags|=BTH_FLAG_MOIST|BTH_FLAG_CHANGED;
		return 1;
	}
	return -2;
}

//
// BLE adv data and scan response data
//
#define DT_SERVICEDATA_UUID16 0x16

#define GAP_APPEARANCE_GENERIC_SENSOR 0x0540 // 1344, Generic Sensor

#define BTHOME_ADV_UUID16 			 0xFCD2 // 16-bit UUID Service Data BTHome V2
#define BTHOME_ADV_FLAG_ENCRYPTED	 0x01
#define BTHOME_ADV_FLAG_TRIGGERBASED 0x04
#define BTHOME_ADV_VERSION 			 2

_attribute_data_retention_ u8 ble_scanRsp [] = {
	 13, DT_COMPLETE_LOCAL_NAME,			'U', 'N', 'K', 'W', 'N', '-', '?', '?', '?', '?', '?', '?'
};

_attribute_data_retention_ u8 ble_advDataConn[] = {
	 2,	 DT_FLAGS, 								BLE_ADV_FLAGS,	// 0x05 BLE limited discoverable mode and BR/EDR not supported
	 3,  DT_APPEARANCE, 						U16_LO(GAP_APPEARANCE_GENERIC_SENSOR), U16_HI(GAP_APPEARANCE_GENERIC_SENSOR),
	 3,  DT_INCOMPLETE_LIST_16BIT_SERVICE_UUID,	0x0F, 0x18,	// incomplete list of service class UUIDs (0x180F Battery)
};

const u8 ble_advDataError[] = {
	 2,	 DT_FLAGS, 								BLE_ADV_FLAGS,	// 0x05 BLE limited discoverable mode and BR/EDR not supported
	 13, DT_SERVICEDATA_UUID16, U16_LO(BTHOME_ADV_UUID16), U16_HI(BTHOME_ADV_UUID16),
	     (BTHOME_ADV_VERSION<<5),
	     VT_BINARY_PROBLEM, 0x01,
	     VT_TEXT, 5, 'E', 'r', 'r', 'o', 'r'
};

_attribute_data_retention_ u8 ble_advDataBTHomeLen = 0;
_attribute_data_retention_ u8 ble_advDataBTHome[31]; // max.

static inline u8 hex_digit(u8 h)
{
	static char *c_hex="0123456789ABCDEF";
	return c_hex[h & 0x0F];
}

_attribute_optimize_size_ static void ble_setup_adv_localname(const char* devname, const u8* macrev, u8 *adv, u8 advlen)
{
	u8 u, n, type, len=0;
	for (u=0; ; u+=len)
	{
		if (u >= advlen)   return;
		len=adv[u++]; if (len==0)   return;
		type=adv[u]; if (type == DT_COMPLETE_LOCAL_NAME)   break;
	}
	for (n=u+1; n<6+1 && devname; n++)
	{
		char c=*devname;
		if (c)   devname++;
		else     c='-';
		adv[n]=c;
	}
	for (n=u+len-1; n>u && macrev; n-=2)
	{
		u8 h=*macrev; macrev++;
		if (adv[n] != '?')   break;
		adv[n]=hex_digit(h);
		if (adv[n-1] != '?')   break;
		adv[n-1]=hex_digit(h>>4);
	}
	#if (APP_BLE_ATT)
	app_ble_att_setup_devinfo(&adv[u+1], len-1, GAP_APPEARANCE_GENERIC_SENSOR); // gatt values
	#endif
}

_attribute_optimize_size_ void app_ble_init_device_name(const char* devname)
{
	ble_setup_adv_localname(devname, 0, ble_scanRsp, sizeof(ble_scanRsp));
}

typedef struct _attribute_packed_ _bthome_nonce_t { // for encryption
    u8  mac[6];
    u16 uuid16;
    u8  flags;
	u32 cnt32;
} bthome_nonce_t;

_attribute_optimize_size_ static int ble_build_adv_bthome()
{
	if (ble_advDataBTHomeLen>0 && (bthome_data.flags&BTH_FLAG_CHANGED)==0)
		return 0; // no change
	u8 u=0;
	// adv flags
	ble_advDataBTHome[u++]=2; // len
	ble_advDataBTHome[u++]=DT_FLAGS; // type flags
	ble_advDataBTHome[u++]=BLE_ADV_FLAGS; // adv flags
	ble_advDataBTHomeLen=u;
	if ((bthome_data.flags&BTH_FLAGS_DATAVALID)==0)
		return 1; // no bthome data
	// adv bthome data
	const u8 *encrypt_key=app_config_get_bthome_key();
	u8 bth_infoflags=(BTHOME_ADV_VERSION<<5); // info flags: bit0: encrypted, bit 5..7: BTHome protocol version
	if (encrypt_key)   bth_infoflags |= BTHOME_ADV_FLAG_ENCRYPTED;
	bthome_increment_packetid();
	u8 len_ofs=u; ble_advDataBTHome[u++]=4; // len: AD type + UUID16 + BTHome flags
	ble_advDataBTHome[u++]=DT_SERVICEDATA_UUID16; // =0x16: AD type "Service Data 16-bit UUID"
	ble_advDataBTHome[u++]=(u8)BTHOME_ADV_UUID16; // =0xFCD2: BTHome V2
	ble_advDataBTHome[u++]=(u8)(BTHOME_ADV_UUID16>>8);
	ble_advDataBTHome[u++]=bth_infoflags; // BTHome info
	u8 data_ofs = u;
	if (bthome_data.flags&BTH_FLAG_PID) {
		ble_advDataBTHome[u++]=VT_PID; // 0x00
		ble_advDataBTHome[u++]=bthome_data.pid;
	}
	if (bthome_data.flags&BTH_FLAG_BAT) {
		if (u >= sizeof(ble_advDataBTHome))   return -1;
		ble_advDataBTHome[u++]=VT_BATTERY_PERCENT; // 0x01
		ble_advDataBTHome[u++]=bthome_data.batterypercent;
	}
	if (bthome_data.flags&BTH_FLAG_TEMP) {
		if (u >= sizeof(ble_advDataBTHome))   return -1;
		ble_advDataBTHome[u++]=VT_TEMPERATURE; // 0x02
		ble_advDataBTHome[u++]=(u8)(bthome_data.temperature&0xFF);
		ble_advDataBTHome[u++]=(u8)(bthome_data.temperature>>8);
	}
	if (bthome_data.flags&BTH_FLAG_VOLT) {
		if (u >= sizeof(ble_advDataBTHome))   return -1;
		ble_advDataBTHome[u++]=VT_VOLTAGE; // 0x0C
		ble_advDataBTHome[u++]=(u8)(bthome_data.voltage&0xFF);
		ble_advDataBTHome[u++]=(u8)(bthome_data.voltage>>8);
	}
	if (bthome_data.flags&BTH_FLAG_MOIST) {
		if (u >= sizeof(ble_advDataBTHome))   return -1;
		ble_advDataBTHome[u++]=VT_MOISTURE; // 0x14
		ble_advDataBTHome[u++]=(u8)(bthome_data.moisture&0xFF);
		ble_advDataBTHome[u++]=(u8)(bthome_data.moisture>>8);
	}
	u8 data_len = u - data_ofs;
	// att data (not encrypted)
	#if (APP_BLE_ATT)
	app_ble_att_set_bthome_data(ble_advDataBTHome+data_ofs, data_len);
	#endif
	// encrypt
	if (encrypt_key) {
		u8 m, buf[20]; bthome_nonce_t nonce; u32 tag;
		// copy data
		if (data_len > sizeof(buf))   return -1; // length error
		if (u+4+4 >= sizeof(ble_advDataBTHome))   return -1; // advertising length error (encryption adds mic+tag)
		memcpy(buf, &ble_advDataBTHome[data_ofs], data_len);
		// build nonce (iv)
		for (m=0; m<6; m++)   nonce.mac[m]=ble_mac_public[5-m];
		nonce.uuid16=BTHOME_ADV_UUID16; nonce.flags=bth_infoflags; nonce.cnt32=bthome_data_sendcount;
		// encrypt
		aes_ccm_encrypt_and_tag( encrypt_key, // key
           (u8 *)&nonce, sizeof(nonce), // iv
		   NULL, 0, // opt.: add data
		   buf, data_len, // in
		   &ble_advDataBTHome[data_ofs], // data out
		   (u8 *)&tag, 4); // tag out
		// append counter + tag (mic)
		ble_advDataBTHome[u++]=(u8)(nonce.cnt32&0xFF);
		ble_advDataBTHome[u++]=(u8)(nonce.cnt32>>8);
		ble_advDataBTHome[u++]=(u8)(nonce.cnt32>>16);
		ble_advDataBTHome[u++]=(u8)(nonce.cnt32>>24);
		ble_advDataBTHome[u++]=(u8)(tag&0xFF);
		ble_advDataBTHome[u++]=(u8)(tag>>8);
		ble_advDataBTHome[u++]=(u8)(tag>>16);
		ble_advDataBTHome[u++]=(u8)(tag>>24);
		data_len = u - data_ofs;
	}
	ble_advDataBTHome[len_ofs]+=data_len; // add adata length
	ble_advDataBTHomeLen=u;
	bthome_data.flags&=(~BTH_FLAG_CHANGED); // reset changed flag
	return 1;
}


//
// BLE stack states / callbacks
//
enum { DEV_CONN_STATE_NONE=0,
	   DEV_CONN_STATE_CONNECTED=BIT(0), DEV_CONN_STATE_ENCRYPTED=BIT(1), DEV_CONN_STATE_SECURED=BIT(2),
	   DEV_CONN_STATE_REBOOT_ON_DISCONNECT=BIT(7)};
_attribute_data_retention_ u8 ble_device_connection_state = DEV_CONN_STATE_NONE;
_attribute_data_retention_ u8 ble_security_level = No_Security;
_attribute_data_retention_ u32 ble_connection_timeout = 0; // sec
_attribute_data_retention_ u8 ble_rf_power_level = RF_POWER_P3p01dBm;

static bool inline isIrkValid(const u8* pIrk)
{	// check, if IRK is valid (16 Bytes)
	return isAppMemValid(pIrk, 16); // check if not 00 or FF
}

_attribute_optimize_size_ void app_ble_set_powerlevel(signed char level_dbm)
{
	static const struct {signed char level; u8 rf;} level2rf[] = {
		{9, RF_POWER_P8p97dBm},	{8, RF_POWER_P8p13dBm}, {7, RF_POWER_P7p02dBm},
		{6, RF_POWER_P6p14dBm}, {5, RF_POWER_P5p13dBm},	{4, RF_POWER_P3p94dBm},
		{3, RF_POWER_P3p01dBm},	{2, RF_POWER_P1p99dBm}, {1, RF_POWER_P0p90dBm},
		{0, RF_POWER_P0p04dBm}, {-1, RF_POWER_N0p97dBm}, {-3, RF_POWER_N3p03dBm},
		{-5, RF_POWER_N5p03dBm}, {-10, RF_POWER_N9p89dBm}, {-127, RF_POWER_N19p27dBm} // lowest
	};
	u8 u, rf=RF_POWER_P10p01dBm; // highest
	for (u=0; u<sizeof(level2rf)/sizeof(level2rf[0]); u++) {
		if (level2rf[u].level<level_dbm)   break;
		rf=level2rf[u].rf;
	}
	ble_rf_power_level=rf;
	rf_set_power_level_index(ble_rf_power_level); // RF driver
}

_attribute_optimize_size_ void ble_set_conn_state(u8 state)
{
	u8 state_old = ble_device_connection_state;
	if (state==DEV_CONN_STATE_NONE || state==DEV_CONN_STATE_CONNECTED)
		ble_device_connection_state=state;
	if (ble_device_connection_state & DEV_CONN_STATE_CONNECTED)
	{ 	// add conn state info
		ble_device_connection_state |= state;
	}
	if (ble_device_connection_state != state_old)
	{
		u8 n[2]; n[0]=ble_device_connection_state; n[1]=state_old;
	    app_notify(APP_NOTIFY_CONNSTATE, n, 2);
	}
}

// callback adv prepare (set by bls_set_advertise_prepare)
_attribute_ram_code_ int ble_advertise_prepare_handler(rf_packet_adv_t * p)
{
	(void) p;
	if (ble_adv_mode == BLE_ADV_MODE_BTHome)
		bthome_data_sendcount++;
	return 1; // = 1 ready to send ADV packet, = 0 not send ADV
}

// callback function of LinkLayer Event BLT_EV_FLAG_SUSPEND_ENTER
void  ble_task_sleep_enter (u8 e, u8 *p, int n)
{
	(void)e;(void)p;(void)n;
	// must: resetted? by pm before enter sleep
	bls_pm_setWakeupSource(PM_WAKEUP_PAD | PM_WAKEUP_TIMER);
}

// callback function of LinkLayer Event BLT_EV_FLAG_CONNECT
void ble_task_connect (u8 e, u8 *p, int n)
{
	(void)e;(void)p;(void)n;
	#if (APP_BLE_EVENT_LOG_EN)
	tlk_contr_evt_connect_t *pConnEvt = (tlk_contr_evt_connect_t *)p;
	DEBUGHEXBUF(APP_BLE_EVENT_LOG_EN, "[BLE] evt connect, intA & advA: %s", pConnEvt->initA, 12);
	#endif
	bls_l2cap_requestConnParamUpdate (CONN_INTERVAL_10MS, CONN_INTERVAL_10MS, 99, CONN_TIMEOUT_4S); // 1 sec
	ble_connection_timeout = app_sec_time(); if (ble_connection_timeout<1)   ble_connection_timeout=1;
	ble_set_conn_state(DEV_CONN_STATE_CONNECTED);
}

// callback function of LinkLayer Event BLT_EV_FLAG_TERMINATE
void ble_task_terminate(u8 e, u8 *p, int n) //*p is terminate reason
{
	(void)e;(void)n;
	#if (APP_BLE_EVENT_LOG_EN)
	tlk_contr_evt_terminate_t *pEvt = (tlk_contr_evt_terminate_t *)p;
	u8 reason=pEvt->terminate_reason; const char *dbg_reason="";
	if (reason == HCI_ERR_CONN_TIMEOUT)					dbg_reason="conn timeout";
	else if (reason == HCI_ERR_REMOTE_USER_TERM_CONN)	dbg_reason="user term conn";
	else if (reason == HCI_ERR_CONN_TERM_MIC_FAILURE)	dbg_reason="mic failure";
	DEBUGFMT(APP_BLE_EVENT_LOG_EN, "[BLE] evt disconnect, reason 0x%02x %s", pEvt->terminate_reason, dbg_reason);
    #endif
	u8 factoryreset=app_ble_att_get_factoryreset(0);
	u8 flags_reboot=(DEV_CONN_STATE_CONNECTED|DEV_CONN_STATE_REBOOT_ON_DISCONNECT);
	if (factoryreset == 0x02)
		ble_device_connection_state |= DEV_CONN_STATE_REBOOT_ON_DISCONNECT;
	if (factoryreset == 0x03)
		app_notify(APP_NOTIFY_FACTORYRESET, 0, 0);
	if ((ble_device_connection_state & flags_reboot) == flags_reboot)
		app_notify(APP_NOTIFY_REBOOT, 0, 0);
	ble_connection_timeout = 0;
	ble_ota_is_working = BLE_OTA_NONE;
	ble_set_conn_state(DEV_CONN_STATE_NONE);
}

// callback function of LinkLayer Event BLT_EV_FLAG_SUSPEND_EXIT
_attribute_optimize_size_ void ble_task_suspend_exit(u8 e, u8 *p, int n)
{
	(void)e;(void)p;(void)n;
	rf_set_power_level_index(ble_rf_power_level); // restore rf power level
}

// callback function (LinkLayer Event BLT_EV_FLAG_DATA_LENGTH_EXCHANGE)
_attribute_optimize_size_ void ble_task_dle_exchange(u8 e, u8 *p, int n)
{
	#if (APP_BLE_EVENT_LOG_EN)
	tlk_contr_evt_dataLenExg_t* pEvt = (tlk_contr_evt_dataLenExg_t*)p;
	DEBUGHEXBUF(APP_BLE_EVENT_LOG_EN, "[BLE] evt DLE exchange %s", (u8*)&pEvt->connEffectiveMaxRxOctets, 4);
	#endif
}
// callback function (Host Events)
_attribute_optimize_size_ int ble_host_event_callback(u32 h, u8 *para, int n)
{
	u8 event = (h & 0xFF);
	switch(event)
	{
		case GAP_EVT_SMP_PAIRING_BEGIN:
		{
			gap_smp_pairingBeginEvt_t *pEvt = (gap_smp_pairingBeginEvt_t *)para;
			DEBUGFMT(APP_SMP_LOG_EN, "[BLE] SMP paring begin: conn %u, secure %u, tk-method %u", pEvt->connHandle, pEvt->secure_conn, pEvt->tk_method);
			u32 pincode=app_config_get_pincode();
			blc_smp_manualSetPinCode_for_debug(pEvt->connHandle,pincode); // using fix pincode
		} break;
		case GAP_EVT_SMP_PAIRING_SUCCESS:
		{
			gap_smp_pairingSuccessEvt_t *pEvt = (gap_smp_pairingSuccessEvt_t *)para;
			DEBUGHEXBUF(APP_SMP_LOG_EN, "[BLE] SMP paring success: %s", pEvt, sizeof(gap_smp_pairingSuccessEvt_t));
			u8 security_level=app_ble_get_security_level();
			if (security_level==Authenticated_Pairing_with_Encryption && pEvt->bonding==1)
			{
				#if (APP_BLE_ATT)
				u8 ok=0; smp_param_save_t bondInfo; bls_smp_param_loadByIndex(0, &bondInfo);
				if(isIrkValid(bondInfo.peer_irk))
					ok=app_config_create_key(0);
				if (ok)
					app_ble_att_setup_config();
				bthome_increment_packetid(); // rebuild BTHome adv data
				#endif
			}
		} break;
		case GAP_EVT_SMP_PAIRING_FAIL:
		{
			#if (APP_SMP_LOG_EN)
			gap_smp_pairingFailEvt_t *pEvt = (gap_smp_pairingFailEvt_t *)para;
			DEBUGHEXBUF(APP_SMP_LOG_EN, "[BLE] SMP paring fail: %s", pEvt, sizeof(gap_smp_pairingFailEvt_t));
			#endif
		} break;
		case GAP_EVT_SMP_CONN_ENCRYPTION_DONE:
		{	// gap_smp_connEncDoneEvt_t *pEvt = (gap_smp_connEncDoneEvt_t *)para;
			DEBUGSTR(APP_SMP_LOG_EN, "[BLE] evt SMP encryption done");
			ble_set_conn_state(DEV_CONN_STATE_ENCRYPTED);
		} break;
		case GAP_EVT_SMP_SECURITY_PROCESS_DONE:
		{	// gap_smp_securityProcessDoneEvt_t *pEvt = (gap_smp_securityProcessDoneEvt_t *)para;
			DEBUGSTR(APP_SMP_LOG_EN, "[BLE] evt security done");
			ble_set_conn_state(DEV_CONN_STATE_SECURED);
		} break;
		case GAP_EVT_SMP_TK_DISPLAY:
		{	// u32 *pinCode = (u32*) para;
			DEBUGFMT(APP_SMP_LOG_EN, "[BLE] evt TK display: %u", *(u32*)para);
		} break;
		case GAP_EVT_SMP_TK_REQUEST_PASSKEY:
		{	// para = NULL
			DEBUGSTR(APP_SMP_LOG_EN, "[BLE] evt TK request passkey");
		} break;
		case GAP_EVT_SMP_TK_REQUEST_OOB:
		{	// para = NULL
			DEBUGSTR(APP_SMP_LOG_EN, "[BLE] evt TK request OOB");
		} break;
		case GAP_EVT_SMP_TK_NUMERIC_COMPARE:
		{	// u32 *pinCode = (u32*) para; blc_smp_setNumericComparisonResult(1);
			#if (APP_SMP_LOG_EN)
			u32 pinCode = MAKE_U32(para[3], para[2], para[1], para[0]);
			DEBUGFMT(APP_SMP_LOG_EN, "[BLE] evt TK compare: %u", pinCode);
			#endif
		} break;
		case GAP_EVT_ATT_EXCHANGE_MTU:
		{
			#if (APP_SMP_LOG_EN)
			gap_gatt_mtuSizeExchangeEvt_t *pEvt = (gap_gatt_mtuSizeExchangeEvt_t *)para;
			DEBUGHEXBUF(APP_HOST_EVENT_LOG_EN, "[BLE] MTU exchange %s", pEvt, sizeof(gap_gatt_mtuSizeExchangeEvt_t));
			#endif
		} break;
		case GAP_EVT_GATT_HANDLE_VALUE_CONFIRM:
		{	// para = NULL
			DEBUGSTR(APP_SMP_LOG_EN, "[BLE] evt value confirm");
		} break;
		default:
			break;
	}
	return 0;
}

#if (BLE_OTA_SERVER_ENABLE)
// callback function for OTA start
void app_enter_ota_mode(void)
{
	DEBUGSTR(APP_OTA_LOG_EN, "[APP] OTA start");
	if(ble_ota_is_working != BLE_OTA_EXTENDED)
		ble_ota_is_working = BLE_OTA_WORK;
	bls_pm_setManualLatency(0);
	blc_ota_setOtaProcessTimeout( 5*60 ); // 5 min
	app_ble_device_reset_conn_timeout();
}

// callback function for OTA end
_attribute_optimize_size_ void app_ota_end_result(int result)
{
	DEBUGFMT(APP_OTA_LOG_EN, "[APP] OTA end: result %d", result);
	if (result != 0)
	{
		DEBUGSTR(APP_BLE_LOG_EN, "[APP] OTA failed");
		app_ble_device_reset_conn_timeout();
	}
	ble_ota_is_working = BLE_OTA_NONE;
}

// must: -> referenced by BLE stack OTA server
_attribute_ble_data_retention_	_attribute_aligned_(4)	flash_prot_op_callback_t flash_prot_op_cb = NULL;
#endif

// setup adv for different states
_attribute_optimize_size_ void app_ble_setup_adv(u8 adv_mode)
{
	u8 adv_enable=BLC_ADV_DISABLE; ble_sts_t adv_param_ret=BLE_SUCCESS; smp_param_save_t bondInfo;
	u8 bond_number = blc_smp_param_getCurrentBondingDeviceNumber();  // get bonded device number
	bls_smp_param_loadByIndex(bond_number-1, &bondInfo); // get the latest bonding device
	if(bond_number > 0 && isIrkValid(bondInfo.peer_irk))
	{
		blc_ll_addDeviceToResolvingList(bondInfo.peer_id_adrType, bondInfo.peer_id_addr, bondInfo.peer_irk, NULL);
		blc_ll_setAddressResolutionEnable(1);
	}
	else
		blc_ll_setAddressResolutionEnable(0);
	if (adv_mode == BLE_ADV_MODE_Conn && bond_number > 0)
	{   // ADV direct
		DEBUGSTR(APP_BLE_LOG_EN, "[BLE] Start ADVdirect");
		adv_param_ret = bls_ll_setAdvParam(
			BLE_CONN_ADV_INTERVAL_MIN, BLE_CONN_ADV_INTERVAL_MAX,
			ADV_TYPE_CONNECTABLE_DIRECTED_LOW_DUTY, ble_own_address_type,
			bondInfo.peer_addr_type,  bondInfo.peer_addr,
			BLT_ENABLE_ADV_ALL,	ADV_FP_NONE);
		bls_ll_setScanRspData((u8 *)ble_scanRsp,sizeof(ble_scanRsp));
		bls_ll_setAdvData((u8 *)ble_advDataConn, sizeof(ble_advDataConn));
		bls_ll_setAdvDuration(0, 0); // disable (adv duration is handled by app.c)
		adv_enable=BLC_ADV_ENABLE;
	}
	if (adv_mode == BLE_ADV_MODE_Conn && bond_number == 0)
	{   // ADV undirected
		DEBUGSTR(APP_BLE_LOG_EN, "[BLE] Start ADVind");
		adv_param_ret = bls_ll_setAdvParam(
			BLE_CONN_ADV_INTERVAL_MIN, BLE_CONN_ADV_INTERVAL_MAX,
			ADV_TYPE_CONNECTABLE_UNDIRECTED, ble_own_address_type,
			0, NULL, BLT_ENABLE_ADV_ALL, ADV_FP_NONE);
		blc_ll_clearResolvingList();
		bls_ll_setScanRspData((u8 *)ble_scanRsp,sizeof(ble_scanRsp));
		bls_ll_setAdvData((u8 *)ble_advDataConn, sizeof(ble_advDataConn));
		bls_ll_setAdvDuration(0, 0); // disable (adv duration is handled by app.c)
		adv_enable=BLC_ADV_ENABLE;
	}
	if (adv_mode == BLE_ADV_MODE_BTHome)
	{  // ADV with BTHome data
		u8 devmode=app_config_get_mode();
		enum {DEVMODE_DEFAULT=0, DEVMODE_MEASURE_NOCONN=0, DEVMODE_MEASURE_CONN, DEVMODE_LAST};
		if (bond_number > 0 && devmode == DEVMODE_MEASURE_CONN)
		{   // note: direct adv
			DEBUGSTR(APP_BLE_LOG_EN, "[BLE] Start ADVind BTHome");
			adv_param_ret = bls_ll_setAdvParam(
					BTHOME_CONN_ADV_INTERVAL, BTHOME_CONN_ADV_INTERVAL+(BTHOME_ADV_INTERVAL/10),
					ADV_TYPE_CONNECTABLE_UNDIRECTED, ble_own_address_type,
					bondInfo.peer_addr_type,  bondInfo.peer_addr,
					BLT_ENABLE_ADV_ALL,	ADV_FP_NONE);
		}
		else // DEVMODE_MEASURE_NOCONN
		{
			DEBUGSTR(APP_BLE_LOG_EN, "[BLE] Start ADVnoconn BTHome");
			adv_param_ret = bls_ll_setAdvParam(
					BTHOME_ADV_INTERVAL, BTHOME_ADV_INTERVAL+(BTHOME_ADV_INTERVAL/10),
					ADV_TYPE_NONCONNECTABLE_UNDIRECTED,
					ble_own_address_type,
					0,  NULL, BLT_ENABLE_ADV_ALL, ADV_FP_NONE);
		}
		ble_build_adv_bthome();
		bls_ll_setScanRspData((u8 *)ble_scanRsp,sizeof(ble_scanRsp));
		bls_ll_setAdvData(ble_advDataBTHome, ble_advDataBTHomeLen);
		bls_ll_setAdvDuration(0, 0); // disable adv duration
		bls_set_advertise_prepare(ble_advertise_prepare_handler); // ll_adv.h
		bthome_data_sendcount = 0;
		adv_enable=BLC_ADV_ENABLE;
	}
	if (adv_param_ret!=BLE_SUCCESS)
	{
		DEBUGFMT(APP_BLE_LOG_EN, "[BLE] ERROR: ADV param 0x%x", adv_param_ret);
		adv_enable=BLC_ADV_DISABLE;
	}
	bls_ll_setAdvEnable(adv_enable);
	rf_set_power_level_index(ble_rf_power_level);
	ble_adv_mode = adv_mode;
}

// setup security for different states
_attribute_optimize_size_ void app_ble_setup_smp_security(void)
{
	ble_security_level = No_Security;
	#if (BLE_APP_SECURITY_ENABLE)
	io_capability_t cap;
	if (app_config_get_pincode() == 0) {
		ble_security_level=Unauthenticated_Pairing_with_Encryption;
		cap=IO_CAPABILITY_NO_INPUT_NO_OUTPUT;
	} else {
		ble_security_level=Authenticated_Pairing_with_Encryption;
		cap=IO_CAPABILITY_DISPLAY_ONLY;
	}
	blc_att_setRxMtuSize(65);
	blc_smp_setSecurityLevel(ble_security_level);
	blc_smp_enableSecureConnections(1);
	blc_smp_setSecurityParameters(Bondable_Mode, 1, 0, 0, cap);
	blc_smp_peripheral_init();
	blc_smp_configSecurityRequestSending(SecReq_IMM_SEND, SecReq_PEND_SEND, 1000); // send security request delayed
	#else
	blc_smp_setSecurityLevel(No_Security);
	#endif
	#if (APP_BLE_ATT)
	app_ble_att_setup_config();
	#endif
}

u8 app_ble_get_security_level(void)
{
	return ble_security_level;
}


//
// Interface
//

// initialization when power on
_attribute_optimize_size_ void app_ble_init_normal(void)
{
	//
	// Controller Initialization
	//
	// MAC
	app_flash_init_mac_address(ble_mac_public, ble_mac_random_static);
	DEBUGFMT(APP_LOG_EN, "[BLE] Public MAC Address %02X:%02X:%02X:%02X:%02X:%02X",
		ble_mac_public[5], ble_mac_public[4], ble_mac_public[3],
		ble_mac_public[2], ble_mac_public[1], ble_mac_public[0]);
    // MAC address type
	#if (BLE_DEVICE_ADDRESS_TYPE == BLE_DEVICE_ADDRESS_PUBLIC)
		ble_own_address_type = OWN_ADDRESS_PUBLIC;
	#elif (BLE_DEVICE_ADDRESS_TYPE == BLE_DEVICE_ADDRESS_RANDOM_STATIC)
		ble_own_address_type = OWN_ADDRESS_RANDOM;
		blc_ll_setRandomAddr(ble_mac_random_static);
	#endif
    // BLE controller basics
	blc_ll_initBasicMCU(); // mandatory
	blc_ll_initStandby_module(ble_mac_public);		// mandatory
	blc_ll_initAdvertising_module(ble_mac_public);	// legacy advertising module: mandatory for BLE slave
	blc_ll_initConnection_module();					// connection module: mandatory for BLE slave/master
	blc_ll_initSlaveRole_module();					// slave module: mandatory for BLE slave,
	//
	// Host Initialization
	//
	// GAP initialization (must be done first)
	blc_gap_peripheral_init();    //gap initialization
	blc_l2cap_register_handler(blc_l2cap_packet_receive);  	//l2cap initialization
	// GATT profile initialization
	#if (APP_BLE_ATT)
	app_ble_att_init();
	#endif
	blc_att_setRxMtuSize(MTU_SIZE_SETTING); // set MTU size, default MTU is 23 if not call this API
	// SMP initialization (may involve flash write/erase - check abttery before)
	bls_smp_configPairingSecurityInfoStorageAddr(app_flash_get_smp_storage_sector()); // must before init
	blc_smp_param_setBondingDeviceMaxNumber(1);
	app_ble_setup_smp_security(); // init smp
	// Host events (GAP/SMP/GATT/ATT): register callback
	blc_gap_registerHostEventHandler(ble_host_event_callback);
	blc_gap_setEventMask(0xFFFFFFFF); // enable all events
	//
	// OTA server
	//
	#if (BLE_OTA_SERVER_ENABLE)
	#if (APP_OTA_LOG_EN)
	// blc_debug_addStackLog(STK_LOG_OTA_FLOW);
	#endif
	blc_ota_initOtaServer_module();
	// blc_ota_setOtaProcessTimeout(30);   //OTA process timeout:  30 seconds
	// blc_ota_setOtaDataPacketTimeout(4);	//OTA data packet timeout:  4 seconds
	// bls_ota_clearNewFwDataArea();
	blc_ota_registerOtaStartCmdCb(app_enter_ota_mode);
	blc_ota_registerOtaResultIndicationCb(app_ota_end_result);
	#endif
	// ADV setup
	ble_setup_adv_localname(0, ble_mac_public, ble_scanRsp, sizeof(ble_scanRsp));
	app_ble_setup_adv(BLE_ADV_MODE_Conn);
	app_ble_set_powerlevel(app_config_get_power_level());
	// Host callbacks
	bls_app_registerEventCallback (BLT_EV_FLAG_CONNECT, &ble_task_connect);
	bls_app_registerEventCallback (BLT_EV_FLAG_TERMINATE, &ble_task_terminate);
	bls_app_registerEventCallback (BLT_EV_FLAG_SUSPEND_ENTER, &ble_task_sleep_enter);
	bls_app_registerEventCallback (BLT_EV_FLAG_SUSPEND_EXIT, &ble_task_suspend_exit);
	bls_app_registerEventCallback (BLT_EV_FLAG_DATA_LENGTH_EXCHANGE, &ble_task_dle_exchange);
}

// initialization when wake up from deepSleep_retention mode
_attribute_ram_code_ void app_ble_init_deepRetn(void)
{
	rf_set_power_level_index(ble_rf_power_level); // not stored during deep sleep
}

// ble main loop
u8 app_ble_loop(void)
{
	// check for BTHome value changes and update adv data
	if (ble_adv_mode == BLE_ADV_MODE_BTHome)
	{
		int ret=ble_build_adv_bthome();
		if (ret > 0)
		{   // data changed
			bls_ll_setAdvData(ble_advDataBTHome, ble_advDataBTHomeLen);
			bls_ll_setAdvEnable(BLC_ADV_ENABLE);
		}
		if (ret < 0)
		{   // adv data error
			bls_ll_setAdvData((u8*)ble_advDataError, sizeof(ble_advDataError));
			bls_ll_setAdvEnable(BLC_ADV_ENABLE);
		}
	}
	// connection timeout
	if (ble_device_connection_state!=DEV_CONN_STATE_NONE && ble_connection_timeout!=0 &&
		ble_ota_is_working == BLE_OTA_NONE &&
		app_sec_time_exceeds(ble_connection_timeout,BLE_CONNECTION_TIMEOUT_SEC))
	{
		DEBUGSTR(APP_BLE_LOG_EN, "[BLE] Connection timeout");
		bls_ll_terminateConnection(0x08); // 0x08: timeout
		ble_connection_timeout = 0;
	}
	// async commands
	if ((ble_async_cmd & APP_BLE_CMD_DELETEBOND)!=0 && ble_device_connection_state==DEV_CONN_STATE_NONE)
	{   // delete bond after connection terminated
		ble_async_cmd &= (~APP_BLE_CMD_DELETEBOND);
		app_ble_delete_bond();
		app_config_delete_key(); // delete encryption key
	}
	if (ble_ota_is_working != BLE_OTA_NONE)
		return APP_PM_DISABLE_SLEEP;
	return APP_PM_DEFAULT;
}

u8 app_ble_device_connected(void)
{
	return ble_device_connection_state;
}

u8 app_ble_device_connected_secure(void)
{
	u8 secureflags=DEV_CONN_STATE_CONNECTED|DEV_CONN_STATE_ENCRYPTED|DEV_CONN_STATE_SECURED;
	return ((ble_device_connection_state & secureflags)==secureflags)?1:0;
}

void app_ble_device_disconnect(void)
{
	bls_ll_terminateConnection(0x13); // 0x13: remote user terminated connection
}

void app_ble_device_disconnect_restart(void)
{
	ble_set_conn_state(DEV_CONN_STATE_REBOOT_ON_DISCONNECT);
	}

void app_ble_device_reset_conn_timeout(void)
{
	if (!ble_connection_timeout)   return;
	ble_connection_timeout = app_sec_time();
	if (ble_connection_timeout<1)   ble_connection_timeout=1;
}

u8 app_ble_device_bond(void)
{
	u8 bond_number=0;
	#if (BLE_APP_SECURITY_ENABLE)
	bond_number = blc_smp_param_getCurrentBondingDeviceNumber();
    #endif
	return bond_number;
}

void app_ble_delete_bond(void)
{
	bls_ll_terminateConnection(0x13); // 0x13: remote user terminated connection
	bls_smp_eraseAllPairingInformation();
	app_ble_setup_smp_security();
	bthome_increment_packetid(); // rebuild BTHome data
	#if (APP_LOG_EN)
	u8 bond_number = blc_smp_param_getCurrentBondingDeviceNumber();
	DEBUGFMT(APP_LOG_EN, "|APP] Delete bond %u",bond_number);
	#endif
}

void app_ble_async_command(u8 cmd)
{
	ble_async_cmd |= cmd; // processed in loop
}








