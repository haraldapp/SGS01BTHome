/********************************************************************************************************
 * @file    app_flash.c
 *
 * @brief   Flash configuration
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
#include "app.h"

// 512k flash layout
//    SMP Storage:    0x74000
//    MAC:            0x76000
//    Calibration:    0x77000
//    FW SígnKey:     0x77180
//    Master Pairing: 0x78000 (unused)
//    App Config.:    0x7C000

#ifndef APP_FLASH_LOG_EN
#define APP_FLASH_LOG_EN 0
#endif
#ifndef APP_FLASH_DEBUG_EN
#define APP_FLASH_DEBUG_EN 0
#endif

#ifndef APP_DEEP_ANA_REG
#define APP_DEEP_ANA_REG DEEP_ANA_REG0 // used deep ana reg (can store u8 data when down/deep sleeping)
#endif

#define CFG_ADR_APP_512K_FLASH 0x7c000
#define CFG_ADR_APP_1M_FLASH 0xfa000

_attribute_data_retention_	unsigned int flash_sector_app_config = CFG_ADR_APP_512K_FLASH;

// we use files from the BLE SDK vendor section "inline"
#include "vendor/common/ble_flash.h"
#include "vendor/common/ble_flash.c"

void app_flash_init_normal(void)
{
    // auto detect flash size and set default SDK flash sectors
	blc_readFlashSize_autoConfigCustomFlashSector();
	#if (APP_FLASH_DEBUG_EN)
	static const char *dbg_fs[]={"64k","128k","256k","512k","1M","2M","4M","8M"};
	const char *fs="?"; if (blc_flash_capacity>=0x10 && blc_flash_capacity<=0x17) fs=dbg_fs[blc_flash_capacity-0x10];
    DEBUGFMT(APP_FLASH_DEBUG_EN, "[FLS] Flash type: MID %06X Size %s", blc_flash_mid, fs);
	#endif
	// load calibration data
	blc_app_loadCustomizedParameters_normal();
	// flash config sector
	flash_sector_app_config = CFG_ADR_APP_512K_FLASH;
	if (blc_flash_capacity == FLASH_SIZE_1M)   flash_sector_app_config = CFG_ADR_APP_1M_FLASH;
    DEBUGFMT(APP_FLASH_DEBUG_EN, "[FLS] Flash init: MAC at %X", flash_sector_mac_address );
    DEBUGFMT(APP_FLASH_DEBUG_EN, "[FLS] Flash init: CONFIG at %X", flash_sector_app_config );
}

_attribute_ram_code_ void app_flash_init_deepRetn(void)
{
	blc_app_loadCustomizedParameters_deepRetn(); // ble_flash.c
}

//
// flash layout
//
void app_flash_init_mac_address(u8 *mac_public, u8 *mac_random_static)
{
	blc_initMacAddress(flash_sector_mac_address, mac_public, mac_random_static);
}

u32 app_flash_get_mac_storage_sector(void)
{
	return flash_sector_mac_address;
}

u32 app_flash_get_smp_storage_sector(void)
{
	return flash_sector_smp_storage;
}

u32 app_flash_get_app_config_sector(void)
{
	return flash_sector_app_config;
}

//
// persist state (saved in deep ana reg)
//
u8 app_flash_get_persist_state()
{
	return analog_read(APP_DEEP_ANA_REG);
}

void app_flash_set_persist_state(u8 state, u8 mask)
{
	u8 s=analog_read(APP_DEEP_ANA_REG);
	s&=(~mask); s|=(state & mask);
	analog_write(USED_DEEP_ANA_REG, s);
}

//
// deep sleep retention
//
extern u32 _retention_size_; // from(!): boot.link / cstartup_825x.S

void app_init_deepsleep_retention_sram(void)
{
	extern u32 _retention_size_;
	u32 ret_size=((u32)&_retention_size_);
    DEBUGFMT(APP_FLASH_DEBUG_EN, "[FLS] Retention RAM size %u", ret_size);
	if (ret_size < 0x4000)
	{
		blc_pm_setDeepsleepRetentionType(DEEPSLEEP_MODE_RET_SRAM_LOW16K); //retention size < 16k, use 16k deep retention
	    DEBUGSTR(APP_FLASH_DEBUG_EN, "[FLS] DEEPSLEEP_MODE_RET_SRAM_LOW16K");
	}
	else if (ret_size < 0x8000)
	{
		blc_pm_setDeepsleepRetentionType(DEEPSLEEP_MODE_RET_SRAM_LOW32K); //retention size < 32k and >16k, use 32k deep retention
	    DEBUGSTR(APP_FLASH_DEBUG_EN, "[FLS] DEEPSLEEP_MODE_RET_SRAM_LOW32K");
	}
	else
	{
		// retention size > 32k, overflow. deep retention size setting err
		DEBUGFMT(APP_FLASH_LOG_EN, "[FLS] deep retention size overflow err");
		while(1);
	}
}

asm(".equ __PM_DEEPSLEEP_RETENTION_ENABLE,	1"); // must: referenced by boot.link !
asm(".global __PM_DEEPSLEEP_RETENTION_ENABLE");

//
// app config
//
// note:
//  -write data to flash memory can only set bits from 1 to 0
//  - writing any new data to flash needs to erase the flash sector before (all bytes set to 0xff)
//  - flash operations taking a long time
//      erase sector:     56 ms (!)
//      write 1 byte:     0.2 ms
//      write 16 bytes:   0.21 ms
//      write 256 bytes:  1.3 ms

#define APP_CFG_MAGIC 0x70706168
#define APP_CFG_VERSION 1

typedef struct _attribute_packed_ _appconfig_v0_t {
	u32 magic; // magic to check if config is valid
	u16 version; // =0
	u16 reserved1; // reserved for future use
	u8  bth_key_init[16];
} appconfig_v0_t;

typedef struct _attribute_packed_ _appconfig_v1_t {
	u32 magic; // magic to check if config is valid
	u16 version; // =1
	u16 reserved1; // reserved for future use
	u8  bth_key_init[16];
	// config values
	u8  bth_key_gatt[16];
	u32 pincode;
	u8  powerlevel; // dbm + 30
	u8  mode;
	u8  dataformat;
	u8  reserved2;
} appconfig_v1_t;

#define appconfig_t appconfig_v1_t

#define APP_CFG_DEFAULT_U8  0xFF
#define APP_CFG_DEFAULT_U16 0xFFFF
#define APP_CFG_DEFAULT_U32 0xFFFFFFFF
_attribute_data_retention_	appconfig_t app_config;

enum { APP_CFG_DIRTY_NO=0, APP_CFG_DIRTY_WRITE=BIT(0), APP_CFG_DIRTY_ERASE=BIT(1), APP_CFG_DIRTY_ALL=BIT(0)|BIT(1) };
_attribute_data_retention_  u8 app_config_dirty = APP_CFG_DIRTY_NO;

enum { APP_CFG_BTH_KEY_NONE=0, APP_CFG_BTH_KEY_INIT, APP_CFG_BTH_KEY_GATT };
_attribute_data_retention_	u8 app_config_bth_key_type = APP_CFG_BTH_KEY_NONE;

static void config_set_val(u8 *dest, const u8 *src, u8 len)
{
	u8 u, val_old, val_new;
	for (u=0; u<len; u++)
	{
		val_old=dest[u]; val_new=src[u]; if (val_old == val_new)   continue;
		dest[u]=val_new; app_config_dirty |= APP_CFG_DIRTY_WRITE;
		if (((~val_old) & (val_new)) !=0)   app_config_dirty |= APP_CFG_DIRTY_ERASE; // bits set - need erase
	}
}

static u8 config_isdefault_val(const u8 *src, u8 len)
{
	for (u8 u=0; u<len; u++)
		if (src[u] != APP_CFG_DEFAULT_U8)
			return 0;
	return 1;
}

bool isAppMemValid(const u8* pMem, u8 len)
{
    u8 u, cnt00=0, cntFF=0;
    for (u=0; u<len; u++, pMem++)
    {
    	if (*pMem == 0x00)		cnt00++;
    	else if (*pMem == 0xFF)	cntFF++;
    }
    return (cnt00!=16) && (cntFF!=16);
}

static bool inline isKeyValid(const u8* pKey)
{
	return isAppMemValid(pKey, 16);
}

static void config_update_keytype(void)
{
	if (isKeyValid(app_config.bth_key_gatt))
		app_config_bth_key_type = APP_CFG_BTH_KEY_GATT;
	else if (isKeyValid(app_config.bth_key_init))
		app_config_bth_key_type = APP_CFG_BTH_KEY_INIT;
	else
		app_config_bth_key_type = APP_CFG_BTH_KEY_NONE;
}

void app_config_init(void)
{
	if (!flash_sector_app_config)   return;
	// read config sector
    app_config_dirty = APP_CFG_DIRTY_NO;
	unsigned char *pcfg=(unsigned char *)&app_config; u8 len_org=0;
	memset(pcfg, APP_CFG_DEFAULT_U8, sizeof(app_config));
	flash_read_page(flash_sector_app_config, sizeof(app_config), pcfg);
	if (app_config.magic != APP_CFG_MAGIC)
	{	// reset default
	    DEBUGSTR(APP_FLASH_LOG_EN, "[FLS] Flash config reset (invalid magic)");
		memset(pcfg, APP_CFG_DEFAULT_U8, sizeof(app_config));
		app_config.magic = APP_CFG_MAGIC;
		app_config.version = APP_CFG_VERSION;
		app_config_dirty = APP_CFG_DIRTY_ALL;
	}
	else if (app_config.version < APP_CFG_VERSION)
	{	// update version
	    DEBUGFMT(APP_FLASH_LOG_EN, "[FLS] Flash update version %u -> %u", app_config.version, APP_CFG_VERSION);
	    if (app_config.version==0)
	    	len_org=sizeof(appconfig_v0_t);
	    else if (app_config.version==1)
	    	len_org=sizeof(appconfig_v1_t);
	    if (len_org>0 && len_org<sizeof(app_config))
	    	memset(pcfg+len_org, APP_CFG_DEFAULT_U8, sizeof(app_config)-len_org);
		app_config_dirty = APP_CFG_DIRTY_ALL;
	}
	app_config_flush();
	config_update_keytype(); // BTHome key type
	#if (APP_FLASH_LOG_EN)
	static const char *dbg_keytype[]={"no key", "persist", "GATT"};
    const u8* key=app_config_get_bthome_key();
    DEBUGFMT(APP_FLASH_LOG_EN, "[FLS] Cfg: pincode %u (0x%08X)", app_config_get_pincode(), app_config.pincode);
    DEBUGFMT(APP_FLASH_LOG_EN, "[FLS] Cfg: keytype <%s>", dbg_keytype[app_config_bth_key_type]);
    if (key)   DEBUGHEXBUF(APP_FLASH_LOG_EN, "[FLS] Cfg: key %s", key, 16);
    DEBUGFMT(APP_FLASH_LOG_EN, "[FLS] Cfg: powerlevel %u (0x%02X)", app_config_get_power_level(), app_config.powerlevel);
    DEBUGFMT(APP_FLASH_LOG_EN, "[FLS] Cfg: mode %u (0x%02X)", app_config_get_mode(), app_config.mode);
    DEBUGFMT(APP_FLASH_LOG_EN, "[FLS] Cfg: format %u (0x%02X)", app_config_get_dataformat(), app_config.dataformat);
	#endif
}

void app_config_reset(void)
{
	unsigned char *pcfg=(unsigned char *)&app_config;
	unsigned char *pcfguser=app_config.bth_key_gatt;
	memset(pcfguser, APP_CFG_DEFAULT_U8, sizeof(app_config)-(pcfguser-pcfg));
	app_config_dirty = APP_CFG_DIRTY_ALL;
}

void app_config_flush(void)
{
	if (!app_config_dirty || !flash_sector_app_config)   return;
	if (app_config_dirty & APP_CFG_DIRTY_ERASE)
	{
	    DEBUGSTR(APP_FLASH_DEBUG_EN, "[FLS] Flash erase config sector");
	    flash_erase_sector(flash_sector_app_config);
	}
	if (app_config_dirty & APP_CFG_DIRTY_WRITE)
	{
	    DEBUGSTR(APP_FLASH_DEBUG_EN, "[FLS] Flash write config");
		flash_write_page(flash_sector_app_config, sizeof(app_config), (u8 *)&app_config);
	}
	app_config_dirty = APP_CFG_DIRTY_NO;
}

const u8* app_config_get_bthome_key(void)
{
	if (app_config_bth_key_type == APP_CFG_BTH_KEY_GATT)
		return app_config.bth_key_gatt;
	if (app_config_bth_key_type == APP_CFG_BTH_KEY_INIT)
		return app_config.bth_key_init;
	return 0;
}

u32 app_config_get_pincode(void)
{
	if (app_config.pincode == APP_CFG_DEFAULT_U32)   return 0;
	return app_config.pincode;
}

void app_config_set_pincode(u32 pin)
{
	if (pin==0 && app_config.pincode == APP_CFG_DEFAULT_U32)   return;
	config_set_val((u8*)&app_config.pincode, (u8*)&pin, 4);
}

void app_config_get_key(u8 *key)
{
	if (isKeyValid(app_config.bth_key_gatt))
		memcpy(key, app_config.bth_key_gatt, 16);
	else
		memset(key, 0, 16);
}

void app_config_set_key(const u8 *key)
{
	config_set_val(app_config.bth_key_gatt, key, 16);
	config_update_keytype();
}

u8 app_config_create_key(const u8 *randbase)
{
	u8 u, key[16], keyrand[16];
	if (!config_isdefault_val(app_config.bth_key_gatt,16))   return 0; // already have key
    DEBUGSTR(APP_FLASH_LOG_EN, "[FLS] Cfg: create key");
	if (randbase)   memcpy(key, randbase, 16);
	else            memset(key, 0, 16);
	generateRandomNum(16, keyrand);
	for (u=0; u<16; u++)   key[u]^=keyrand[u];
	config_set_val(app_config.bth_key_gatt, key, 16);
	config_update_keytype();
	return 1;
}

void app_config_delete_key(void)
{
    DEBUGSTR(APP_FLASH_LOG_EN, "[FLS] Cfg: delete key");
    u8 key[16]; memset(key, APP_CFG_DEFAULT_U8, 16);
	config_set_val(app_config.bth_key_gatt, key, 16);
	config_update_keytype();
}
#ifndef RF_POWER_LEVEL_DEFAULT
#define RF_POWER_LEVEL_DEFAULT 3 // dbm
#endif

signed char app_config_get_power_level(void)
{
	if (app_config.powerlevel == APP_CFG_DEFAULT_U8 )   return RF_POWER_LEVEL_DEFAULT;
	return ((signed char)app_config.powerlevel) - 30;
}

void app_config_set_power_level(signed char level_dbm)
{
	if (level_dbm > 30)   level_dbm=30;
	if (level_dbm < -30)   level_dbm=-30;
	volatile u8 powerlevel=(u8)(level_dbm + 30);
	config_set_val((u8*)&app_config.powerlevel, (u8*)&powerlevel, 1);
}

u8 app_config_get_mode(void)
{
	if (app_config.mode == APP_CFG_DEFAULT_U8 )   return DEVMODE_DEFAULT;
	return app_config.mode;
}

void app_config_set_mode(u8 mode)
{
	if (mode >= DEVMODE_LAST)   mode=DEVMODE_LAST-1;
	config_set_val((u8*)&app_config.mode, (u8*)&mode, 1);
}

u8 app_config_get_dataformat(void)
{
	if (app_config.dataformat == APP_CFG_DEFAULT_U8 )   return DATAFORMAT_DEFAULT;
	return app_config.dataformat;
}

void app_config_set_dataformat(u8 datafmt)
{
	config_set_val((u8*)&app_config.dataformat, (u8*)&datafmt, 1);
}















