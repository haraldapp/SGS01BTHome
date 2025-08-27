/********************************************************************************************************
 * @file    app_att.c
 *
 * @brief   BLE attributes
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
#include "app_config.h"

#if (APP_BLE_ATT)  // component enabled

#include "app.h"
#include "compiler.h"
// #include "tl_common.h"
#include "stack/ble/ble.h"

#ifndef APP_ATT_LOG_EN
#define APP_ATT_LOG_EN 0
#endif
#ifndef BLE_OTA_SERVER_ENABLE
#define BLE_OTA_SERVER_ENABLE 0
#endif
#ifndef BLE_ATT_CRYPTKEY_CHANGE_ENABLE
#define BLE_ATT_CRYPTKEY_CHANGE_ENABLE 1
#endif
#ifndef BLE_ATT_CUSTOMCONFIG
#define BLE_ATT_CUSTOMCONFIG 0
#endif

// helpers
_attribute_optimize_size_ static u8 hex_add(char *buf, u8 val)
{
	u8 u=0; static const char *c_hex="0123456789ABCDEF";
	buf[u++]=c_hex[(val>>4)&0x0F]; buf[u++]=c_hex[val&0x0F];
	return u;
}

static u8 val_in_ccc(u8* ccc)
{
	if (!app_ble_device_connected())	return 0;
	if (blc_ll_getTxFifoNumber() >= 9)	return 0;
	if (ccc[0]==0 && ccc[1]==0)			return 0;
	return 1;
}

_attribute_optimize_size_ static u32 get_u32(const u8* b)
{
	if (!b)   return 0;
	u32 v=b[3]; v<<=8; v|=b[2]; v<<=8; v|=b[1]; v<<=8; v|=b[0];
	return v;
}

_attribute_optimize_size_ static void set_u32(u8* b, u32 val)
{
	if (!b)   return;
	b[0]=(u8)(val&0xFF); val>>=8;
	b[1]=(u8)(val&0xFF); val>>=8;
	b[2]=(u8)(val&0xFF); val>>=8;
	b[3]=(u8)(val&0xFF);
}

static inline int userActionCB(void *p)
{
	app_ble_device_reset_conn_timeout();
	return 0;
}


// common UUID
static const u16 att_primaryServiceUUID = GATT_UUID_PRIMARY_SERVICE; // 0x2800
static const u16 att_characterUUID = GATT_UUID_CHARACTER; // 0x2803
static const u16 att_userdesc_UUID	= GATT_UUID_CHAR_USER_DESC; // 0x2901
static const u16 att_clientCharacterCfgUUID = GATT_UUID_CLIENT_CHAR_CFG; // 0x2902

// ATT profile define
typedef enum
{
	ATT_H_START = 0,
	// GAP
	GenericAccess_PS_H, 					// UUID: 2800, 	VALUE: uuid 1800
	GenericAccess_DeviceName_CD_H,			// UUID: 2803, 	VALUE:  			Prop: Read | Notify
	GenericAccess_DeviceName_DP_H,			// UUID: 2A00,   VALUE: device name
	GenericAccess_Appearance_CD_H,			// UUID: 2803, 	VALUE:  			Prop: Read
	GenericAccess_Appearance_DP_H,			// UUID: 2A01,	VALUE: appearance
	CONN_PARAM_CD_H,						// UUID: 2803, 	VALUE:  			Prop: Read
	CONN_PARAM_DP_H,						// UUID: 2A04,   VALUE: connParameter
	// GATT
	GenericAttribute_PS_H,					// UUID: 2800, 	VALUE: uuid 1801
	GenericAttribute_ServiceChanged_CD_H,	// UUID: 2803, 	VALUE:  			Prop: Indicate
	GenericAttribute_ServiceChanged_DP_H,   // UUID:	2A05,	VALUE: service change
	GenericAttribute_ServiceChanged_CCB_H,	// UUID: 2902,	VALUE: serviceChangeCCC
	// Device Information
	DeviceInformation_PS_H,					// UUID: 2800, 	VALUE: uuid 180A
	DeviceInformation_ModName_CD_H,			// UUID: 2803, 	VALUE:  			Prop: Read
	DeviceInformation_ModName_DP_H,			// UUID: 2A24,	VALUE: Model Number String
	DeviceInformation_SerialN_CD_H,			// UUID: 2803, 	VALUE:  			Prop: Read
	DeviceInformation_SerialN_DP_H,			// UUID: 2A25,	VALUE: Serial Number String
	DeviceInformation_FirmRev_CD_H,			// UUID: 2803, 	VALUE:  			Prop: Read
	DeviceInformation_FirmRev_DP_H,			// UUID: 2A26,	VALUE: Firmware Revision String
	DeviceInformation_HardRev_CD_H,			// UUID: 2803, 	VALUE:  			Prop: Read
	DeviceInformation_HardRev_DP_H,			// UUID: 2A27,	VALUE: Hardware Revision String
	DeviceInformation_SoftRev_CD_H,			// UUID: 2803, 	VALUE:  			Prop: Read
	DeviceInformation_SoftRev_DP_H,			// UUID: 2A28,	VALUE: Software Revision String
	DeviceInformation_ManName_CD_H,			// UUID: 2803, 	VALUE:  			Prop: Read
	DeviceInformation_ManName_DP_H,			// UUID: 2A29,	VALUE: Manufacturer Name String
	// Battery Service
	BATT_PS_H, 								// UUID: 2800, 	VALUE: UUID 180f
	BATT_LEVEL_INPUT_CD_H,					// UUID: 2803, 	VALUE:  			Prop: Read | Notify
	BATT_LEVEL_INPUT_DP_H,					// UUID: 2A19 	VALUE: batVal
	BATT_LEVEL_INPUT_CCB_H,					// UUID: 2902, 	VALUE: batValCCC
	// Custom Device Configuration
	CustomConfig_PS_H,						// service
	CustomConfig_Pincode_CD_H,				// prop
	CustomConfig_Pincode_DP_H,				// value
	CustomConfig_Pincode_DESC_H,            // desc
	CustomConfig_EncryptKey_CD_H,			// prop
	CustomConfig_EncryptKey_DP_H,			// value
	CustomConfig_EncryptKey_DESC_H,         // desc
	#if (BLE_ATT_CUSTOMCONFIG)
	CustomConfig_PowerLevel_CD_H,			// prop
	CustomConfig_PowerLevel_DP_H,			// value
	CustomConfig_DeviceMode_CD_H,			// prop
	CustomConfig_DeviceMode_DP_H,			// value
	CustomConfig_DeviceMode_DESC_H,			// desc
	CustomConfig_DataFormat_CD_H,			// prop
	CustomConfig_DataFormat_DP_H,			// value
	CustomConfig_DataFormat_DESC_H,			// desc
	#endif
	CustomConfig_BTHomeData_CD_H,			// prop
	CustomConfig_BTHomeData_DP_H,			// value
	CustomConfig_BTHomeData_CCB_H,			// ccc
	CustomConfig_BTHomeData_DESC_H,         // desc
	CustomConfig_FactoryReset_CD_H,			// prop
	CustomConfig_FactoryReset_DP_H,			// value
	CustomConfig_FactoryReset_DESC_H,		// desc
	// OTA
	#if (BLE_OTA_SERVER_ENABLE)
	OTA_PS_H, 								// UUID: 2800, 	VALUE: Telink OTA UUID
	OTA_CMD_OUT_CD_H,						// UUID: 2803, 	VALUE: Prop: read | write_without_rsp | Notify
	OTA_CMD_OUT_DP_H,						// UUID: Telink OTA UUID, VALUE: otaData
	OTA_CMD_INPUT_CCB_H,					// UUID: 2902, 	VALUE: otaDataCCC
	OTA_CMD_OUT_DESC_H,						// UUID: 2901, 	VALUE: otaName "OTA"
	#endif
	ATT_END_H,
} ATT_HANDLE;



// GAP device name / appearance
static const u16 att_gapServiceGenericAccessUUID = SERVICE_UUID_GENERIC_ACCESS; // 0x1800
static const u16 att_devNameUUID = GATT_UUID_DEVICE_NAME; // 0x2A00
static const u16 att_devAppearanceUUID = GATT_UUID_APPEARANCE; // 0x2A01

#define GATT_UUID_PERI_CONN_PARAM 		 0x2a04

_attribute_data_retention_ static u8 att_devName_val[12]={ 0 };
_attribute_data_retention_ static u16 att_devAppearance_val = GAP_APPEARE_UNKNOWN;

static const u8 att_devNameChar_def[5] = {
	CHAR_PROP_READ,
	U16_LO(GenericAccess_DeviceName_DP_H), U16_HI(GenericAccess_DeviceName_DP_H),
	U16_LO(GATT_UUID_DEVICE_NAME), U16_HI(GATT_UUID_DEVICE_NAME)
};
static const u8 att_devAppearanceCharVal_def[5] = {
	CHAR_PROP_READ,
	U16_LO(GenericAccess_Appearance_DP_H), U16_HI(GenericAccess_Appearance_DP_H),
	U16_LO(GATT_UUID_APPEARANCE), U16_HI(GATT_UUID_APPEARANCE)
};

void app_ble_att_setup_devinfo(const u8 *devname, u8 devnamelen, u16 appearance)
{
	if (devnamelen>sizeof(att_devName_val))   devnamelen=sizeof(att_devName_val);
	memset(att_devName_val, ' ', sizeof(att_devName_val));
	memcpy(att_devName_val, devname, devnamelen);
	bls_att_setDeviceName(att_devName_val, devnamelen);
	att_devAppearance_val=appearance;
}

// GATT: service change
static const u16 att_gattServiceUUID = SERVICE_UUID_GENERIC_ATTRIBUTE; // 0x1801
static const u16 att_serviceChangeUUID = GATT_UUID_SERVICE_CHANGE; // 0x2A05

_attribute_data_retention_	static u16 att_serviceChange_val[2] = {0};
_attribute_data_retention_	static u8 att_serviceChange_ccc[2] = {0,0};

static const u8 att_serviceChangeChar_def[5] = {
	CHAR_PROP_INDICATE,
	U16_LO(GenericAttribute_ServiceChanged_DP_H), U16_HI(GenericAttribute_ServiceChanged_DP_H),
	U16_LO(GATT_UUID_SERVICE_CHANGE), U16_HI(GATT_UUID_SERVICE_CHANGE)
};

static const u16 att_periConnParamUUID = GATT_UUID_PERI_CONN_PARAM; // 0x2A04  uuid.h

typedef struct _attribute_packed_
{
  u16 intervalMin; // Minimum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms)
  u16 intervalMax; // Maximum value for the connection event (interval. 0x0006 - 0x0C80 * 1.25 ms)
  u16 latency; // Number of LL latency connection events (0x0000 - 0x03e8)
  u16 timeout; // Connection Timeout (0x000A - 0x0C80 * 10 ms)
} gap_periConnectParams_t;

static const gap_periConnectParams_t att_periConnParameters_val = {20, 40, 0, 1000};

static const u8 att_periConnParamChar_def[5] = {
	CHAR_PROP_READ,
	U16_LO(CONN_PARAM_DP_H), U16_HI(CONN_PARAM_DP_H),
	U16_LO(GATT_UUID_PERI_CONN_PARAM), U16_HI(GATT_UUID_PERI_CONN_PARAM)
};

// Device information
#define CHARACTERISTIC_UUID_MODEL_NUMBER		0x2A24 // Model Number String
#define CHARACTERISTIC_UUID_SERIAL_NUMBER		0x2A25 // Serial Number String
#define CHARACTERISTIC_UUID_FIRMWARE_REV		0x2A26 // Firmware Revision String
static const u16 att_devInfoModelUUID = CHARACTERISTIC_UUID_MODEL_NUMBER;
#define CHARACTERISTIC_UUID_HARDWARE_REV		0x2A27 // Hardware Revision String
#define CHARACTERISTIC_UUID_SOFTWARE_REV		0x2A28 // Software Revision String
#define CHARACTERISTIC_UUID_MANUFACTURER		0x2A29 // Manufacturer Name String

static const u16 att_devInfoServiceUUID = SERVICE_UUID_DEVICE_INFORMATION; // 0x180A
static const u16 att_devInfoSerialUUID = CHARACTERISTIC_UUID_SERIAL_NUMBER; // 0x2A25
static const u16 att_devInfoFirmwareRevUUID = CHARACTERISTIC_UUID_FIRMWARE_REV; // 0x2A26
static const u16 att_devInfoHardwareRevUUID = CHARACTERISTIC_UUID_HARDWARE_REV; // 0x2A27
static const u16 att_devInfoSoftwareRevUUID = CHARACTERISTIC_UUID_SOFTWARE_REV; // 0x2A28
static const u16 att_devInfoManufacturerUUID = CHARACTERISTIC_UUID_MANUFACTURER; // 0x2A29

static const u8 att_ModelStr_val[] = {"SGS01-BTHome"};
_attribute_data_retention_ static char att_SerialStr_val[21] = {"000000-000000-0000000"}; // set from flash id
static const u8 att_FirmStr_val[] = {"github.com/haraldapp"};
static const u8 att_HardStr_val[4] = {"V1.0"};
static const u8 att_SoftStr_val[] = {VERSION_STR VERSION_STR_BUILD}; // app_config.h
static const u8 att_ManStr_val[] = {"DIY.home"};

//TODO: may get hardware revision from MCU information
//TODO: may add software revision from MCU information

static const u8 att_ModChar_def[5] = {
	CHAR_PROP_READ,
	U16_LO(DeviceInformation_ModName_DP_H), U16_HI(DeviceInformation_ModName_DP_H),
	U16_LO(CHARACTERISTIC_UUID_MODEL_NUMBER), U16_HI(CHARACTERISTIC_UUID_MODEL_NUMBER)
};
static const u8 att_SerialChar_def[5] = {
	CHAR_PROP_READ,
	U16_LO(DeviceInformation_SerialN_DP_H), U16_HI(DeviceInformation_SerialN_DP_H),
	U16_LO(CHARACTERISTIC_UUID_SERIAL_NUMBER), U16_HI(CHARACTERISTIC_UUID_SERIAL_NUMBER)
};
static const u8 att_FirmChar_def[5] = {
	CHAR_PROP_READ,
	U16_LO(DeviceInformation_FirmRev_DP_H), U16_HI(DeviceInformation_FirmRev_DP_H),
	U16_LO(CHARACTERISTIC_UUID_FIRMWARE_REV), U16_HI(CHARACTERISTIC_UUID_FIRMWARE_REV)
};
static const u8 att_HardChar_def[5] = {
	CHAR_PROP_READ,
	U16_LO(DeviceInformation_HardRev_DP_H), U16_HI(DeviceInformation_HardRev_DP_H),
	U16_LO(CHARACTERISTIC_UUID_HARDWARE_REV), U16_HI(CHARACTERISTIC_UUID_HARDWARE_REV)
};
static const u8 att_SoftChar_def[5] = {
	CHAR_PROP_READ,
	U16_LO(DeviceInformation_SoftRev_DP_H), U16_HI(DeviceInformation_SoftRev_DP_H),
	U16_LO(CHARACTERISTIC_UUID_SOFTWARE_REV), U16_HI(CHARACTERISTIC_UUID_SOFTWARE_REV)
};
static const u8 att_ManChar_def[5] = {
	CHAR_PROP_READ,
	U16_LO(DeviceInformation_ManName_DP_H), U16_HI(DeviceInformation_ManName_DP_H),
	U16_LO(CHARACTERISTIC_UUID_MANUFACTURER), U16_HI(CHARACTERISTIC_UUID_MANUFACTURER)
};

static void app_ble_att_setup_serial(void)
{
	char *s=att_SerialStr_val;
	memset(s, ' ', sizeof(att_SerialStr_val));
	// SoC ID
	s+=hex_add(s, REG_ADDR8(0x7f));
	s+=hex_add(s, REG_ADDR8(0x7e));
	s+=hex_add(s, REG_ADDR8(0x7d));
	*s='-'; s++;
	// Flash ID
	unsigned int mid=flash_read_mid();
	s+=hex_add(s, (u8)(mid>>16));
	s+=hex_add(s, (u8)(mid>>8));
	s+=hex_add(s, (u8)(mid));
	*s='-'; s++;
	// Flash UID
	u8 u, buf[22]; memset(buf, '0', 7);
	flash_read_uid(FLASH_READ_UID_CMD_GD_PUYA_ZB_TH, buf);
	for (u=0; u<7 && buf[u]>' '; u++) { *s=buf[u]; s++; }
	#if (APP_ATT_LOG_EN)
	memcpy(buf, att_SerialStr_val, 21); buf[21]=0;
    DEBUGFMT(APP_ATT_LOG_EN, "[ATT] Setup serial %s", buf);
	#endif
}

// Battery Service
static const u16 att_batServiceUUID	= SERVICE_UUID_BATTERY; // 0x180F
static const u16 att_batCharUUID = CHARACTERISTIC_UUID_BATTERY_LEVEL; // 0x2A19

_attribute_data_retention_ static u8 att_bat_ccc[2] = {0,0};
_attribute_data_retention_ static u8 att_bat_val[1] = {99};

static const u8 att_batCharVal_def[5] = {
	CHAR_PROP_READ | CHAR_PROP_NOTIFY,
	U16_LO(BATT_LEVEL_INPUT_DP_H), U16_HI(BATT_LEVEL_INPUT_DP_H),
	U16_LO(CHARACTERISTIC_UUID_BATTERY_LEVEL), U16_HI(CHARACTERISTIC_UUID_BATTERY_LEVEL)
};

// Custom configuration
//  Service: DE8A5AAC-A99B-C315-0C80-60D4CBB51225
//   Att Pincode:      0ffb7104-860c-49ae-8989-1f946d5f6c03
//   Att EncryptKey:   eb0fb41b-af4b-4724-a6f9-974f55aba81a
//   Att PowerLevel:   0x2A07
//   Att DeviceMode:   9546a800-d32e-4573-81e1-d597c5e1da74
//   Att DeviceMode:   9546a801-d32e-4573-81e1-d597c5e1da74
//   Att BTHome data:  d52246df-98ac-4d21-be1b-70d5f66a5ddb
//   Att FactoryReset: b0a7e40f-2b87-49db-801c-eb3686a24bdb
#define CHARACTERISTIC_UUID_POWER_LEVEL	0x2A07

#define CUSTOM_SERVICE_UUID 0x25,0x12,0xB5,0xCB,0xD4,0x60,0x80,0x0C,0x15,0xC3,0x9B,0xA9,0xAC,0x5A,0x8A,0xDE
#define CUSTOM_ATT_PINCODE_UUID 0x03,0x6C,0x5F,0x6D,0x94,0x1F,0x89,0x89,0xAE,0x49,0x0C,0x86,0x04,0x71,0xFB,0x0F
#define CUSTOM_ATT_ENCRYPTKEY_UUID 0x1A,0xA8,0xAB,0x55,0x4F,0x97,0xF9,0xA6,0x24,0x47,0x4B,0xAF,0x1B,0xB4,0x0F,0xEB
#define CUSTOM_ATT_DEVICEMODE_UUID  0x74,0xDA,0xE1,0xC5,0x97,0xD5,0xE1,0x81,0x73,0x45,0x2E,0xD3,0x00,0xA8,0x46,0x95
#define CUSTOM_ATT_DATAFORMAT_UUID  0x74,0xDA,0xE1,0xC5,0x97,0xD5,0xE1,0x81,0x73,0x45,0x2E,0xD3,0x01,0xA8,0x46,0x95
#define CUSTOM_ATT_BTHOMEDATA_UUID 0xDB,0x5D,0x6A,0xF6,0xD5,0x70,0x1B,0xBE,0x21,0x4D,0xAC,0x98,0xDF,0x46,0x22,0xD5

#define CUSTOM_ATT_FACTORYRESET_UUID 0xDB,0x4B,0xA2,0x86,0x36,0xEB,0x1C,0x80,0xDB,0x49,0x87,0x2B,0x0F,0xE4,0xA7,0xB0
static const u8 att_CustomServiceUUID16[16] = WRAPPING_BRACES(CUSTOM_SERVICE_UUID);
static const u8 att_CustomAttPincodeUUID16[16] = WRAPPING_BRACES(CUSTOM_ATT_PINCODE_UUID);
static const u8 att_CustomAttEncryptKeyUUID16[16] = WRAPPING_BRACES(CUSTOM_ATT_ENCRYPTKEY_UUID);
#if (BLE_ATT_CUSTOMCONFIG)
static const u16 att_CustomAttPowerLevelUUID = CHARACTERISTIC_UUID_POWER_LEVEL;
static const u8 att_CustomAttDeviceModeUUID16[16] = WRAPPING_BRACES(CUSTOM_ATT_DEVICEMODE_UUID);
static const u8 att_CustomAttDataFormatUUID16[16] = WRAPPING_BRACES(CUSTOM_ATT_DATAFORMAT_UUID);
#endif
static const u8 att_CustomAttBTHomeDataUUID16[16] = WRAPPING_BRACES(CUSTOM_ATT_BTHOMEDATA_UUID);
static const u8 att_CustomAttFactoryResetUUID16[16] = WRAPPING_BRACES(CUSTOM_ATT_FACTORYRESET_UUID);

_attribute_data_retention_ static u8 att_customPincode_val[4] = {0,0,0,0};
_attribute_data_retention_ static u8 att_customEncryptKey_val[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#if (BLE_ATT_CUSTOMCONFIG)
_attribute_data_retention_ static u8 att_customPowerLevel_val[1] = {3};
_attribute_data_retention_ static u8 att_customDeviceMode_val[1] = {0};
_attribute_data_retention_ static u8 att_customDataFormat_val[1] = {0};
#endif
_attribute_data_retention_ static u8 att_customBTHomeData_val[20];
_attribute_data_retention_ static u8 att_customBTHomeData_ccc[2] = {0,0};
_attribute_data_retention_ static u8 att_customFactoryReset_val[1] = {0};

static const u8 att_customPincode_desc[]={'P','i','n','c','o','d','e'};
static const u8 att_customEncryptKey_desc[]={'E','n','c','r','y','p','t','i','o','n',' ','K','e','y'};
#if (BLE_ATT_CUSTOMCONFIG)
static const u8 att_customDeviceMode_desc[]={'D','e','v','i','c','e',' ','M','o','d','e'};
static const u8 att_customDataFormat_desc[]={'D','a','t','a',' ','F','o','r','m','a','t'};
#endif
static const u8 att_customBTHomeData_desc[]={'B','T','H','o','m','e',' ','D','a','t','a'};
static const u8 att_customFactoryReset_desc[]={'F','a','c','t','o','r','y',' ','R','e','s','e','t'};

static const u8 att_customPincode_def[19] = {
	CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP | CHAR_PROP_WRITE,
	U16_LO(CustomConfig_Pincode_DP_H), U16_HI(CustomConfig_Pincode_DP_H),
	CUSTOM_ATT_PINCODE_UUID
};

static const u8 att_customEncryptKey_def[19] = {
	CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP | CHAR_PROP_WRITE,
	U16_LO(CustomConfig_EncryptKey_DP_H), U16_HI(CustomConfig_EncryptKey_DP_H),
	CUSTOM_ATT_ENCRYPTKEY_UUID
};

#if (BLE_ATT_CUSTOMCONFIG)
static const u8 att_customPowerLevel_def[5] = {
	CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP | CHAR_PROP_WRITE,
	U16_LO(CustomConfig_PowerLevel_DP_H), U16_HI(CustomConfig_PowerLevel_DP_H),
	U16_LO(CHARACTERISTIC_UUID_POWER_LEVEL), U16_HI(CHARACTERISTIC_UUID_POWER_LEVEL)
};

static const u8 att_customDeviceMode_def[19] = {
	CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP | CHAR_PROP_WRITE,
	U16_LO(CustomConfig_DeviceMode_DP_H), U16_HI(CustomConfig_DeviceMode_DP_H),
	CUSTOM_ATT_DEVICEMODE_UUID
};

static const u8 att_customDataFormat_def[19] = {
	CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP | CHAR_PROP_WRITE,
	U16_LO(CustomConfig_DataFormat_DP_H), U16_HI(CustomConfig_DataFormat_DP_H),
	CUSTOM_ATT_DATAFORMAT_UUID
};
#endif

static const u8 att_customBTHomeData_def[19] = {
	CHAR_PROP_READ | CHAR_PROP_NOTIFY,
	U16_LO(CustomConfig_BTHomeData_DP_H), U16_HI(CustomConfig_BTHomeData_DP_H),
	CUSTOM_ATT_BTHOMEDATA_UUID
};

static const u8 att_customAttFactoryReset_def[19] = {
	CHAR_PROP_WRITE_WITHOUT_RSP | CHAR_PROP_WRITE,
	U16_LO(CustomConfig_FactoryReset_DP_H), U16_HI(CustomConfig_FactoryReset_DP_H),
	CUSTOM_ATT_FACTORYRESET_UUID
};

static int customConfigWriteCB(void *p)
{
	rf_packet_att_data_t *req = (rf_packet_att_data_t*) p;
    // DEBUGFMT(APP_ATT_LOG_EN, "[ATT] WriteCB att=%u handle=%u l2cap=%u data=%u", req->att, req->handle, req->l2cap, req->dat[0]);
	if (req->l2cap < 4)   return 1;
	u16 att = req->handle, len = req->l2cap - 3; u8 *data=req->dat;
	if (att == CustomConfig_Pincode_DP_H)
	{
		if (len != 4)   return 1;
		u32 pin_old=get_u32(att_customPincode_val), pin_new=get_u32(data);
	    DEBUGFMT(APP_ATT_LOG_EN, "[ATT] Write Pincode %u (%u)", pin_new, pin_old);
	    userActionCB(p); // reset connection timeout
		if (pin_new == pin_old)   return 0;
	    set_u32(att_customPincode_val, pin_new);
	    app_config_set_pincode(pin_new); // update config
	    if ((pin_new!=0 && pin_old==0) || (pin_new==0 && pin_old!=0))
	    {	// delete bonding info -> new pairing required
	    	app_ble_async_command(APP_BLE_CMD_DELETEBOND); // exec after disconnect
	    }
	    return 1; // all done
	}
	if (att == CustomConfig_EncryptKey_DP_H)
	{
		if (len != 16)   return 1;
		#if (BLE_ATT_CRYPTKEY_CHANGE_ENABLE)
	    DEBUGHEXBUF(APP_ATT_LOG_EN, "[ATT] Write EncryptKey: %s", data, 16);
	    memcpy(att_customEncryptKey_val, data, 16);
	    app_config_set_key(data);
		#else
	    DEBUGSTR(APP_ATT_LOG_EN, "[ATT] Write EncryptKey: disabled by config");
		#endif
	    return 1; // all done
	}
	#if (BLE_ATT_CUSTOMCONFIG)
	if (att == CustomConfig_PowerLevel_DP_H)
	{
		signed char powerlevel_old=(signed char)att_customPowerLevel_val[0];
		signed char powerlevel_new=(signed char)req->dat[0];
	    DEBUGFMT(APP_ATT_LOG_EN, "[ATT] Write PowerLevel %d (%d)", powerlevel_new, powerlevel_old);
	    userActionCB(p); // reset connection timeout
	    if (powerlevel_new == powerlevel_old)   return 0;
	    att_customPowerLevel_val[0]=(u8)powerlevel_new;
	    app_config_set_power_level(powerlevel_new); // update config
	    app_ble_set_powerlevel(powerlevel_new); // update RF driver
	    return 1;
	}
	if (att == CustomConfig_DeviceMode_DP_H)
	{
		u8 mode_old=att_customDeviceMode_val[0], mode_new=req->dat[0];
	    DEBUGFMT(APP_ATT_LOG_EN, "[ATT] Write DeviceMode %u (%u)", mode_new, mode_old);
	    userActionCB(p); // reset connection timeout
	    if (mode_new == mode_old)   return 0;
	    att_customDeviceMode_val[0]=mode_new;
	    app_config_set_mode(mode_new); // update config
//TODO check if restart is needed
	    app_ble_device_disconnect_restart(); // restart on disconnect
	    return 1;
	}
	if (att == CustomConfig_DataFormat_DP_H)
	{
		u8 fmt_old=att_customDataFormat_val[0], fmt_new=req->dat[0];
	    DEBUGFMT(APP_ATT_LOG_EN, "[ATT] Write DataFormat %u (%u)", fmt_new, fmt_old);
	    userActionCB(p); // reset connection timeout
	    if (fmt_new == fmt_old)   return 0;
	    att_customDataFormat_val[0]=fmt_new;
	    app_config_set_dataformat(fmt_new); // update config
	    app_ble_set_sensor_data_changed();
	    app_ble_att_set_bthome_data(0, 0);
	    return 1;
	}
	#endif
	if (att == CustomConfig_FactoryReset_DP_H)
	{
		u8 val_new=req->dat[0];
	    DEBUGFMT(APP_ATT_LOG_EN, "[ATT] Write FactoryReset %u", val_new);
	    userActionCB(p); // reset connection timeout
	    att_customFactoryReset_val[0]=val_new;
	    // some values for testing/debug
		#if (APP_DEBUG_ENABLE)
	    if (val_new==0x80) // deepsleep for ever
			cpu_sleep_wakeup(DEEPSLEEP_MODE, 0, 0);
	    if (val_new==0x81) // stall
			while (1);
		#endif
	    return 1;
	    // notes:
	    //  - exec on disconnect
	    //  - seems that some DIY devices use this UUID,
	    //    to avoid conflicts with third party apps we use own values
	    //  - value 0x02: soft restart device
	    //    value 0x03: to run a factory reset
	}
	return 0;
}

void app_ble_att_setup_config(void)
{
    memset(att_customPincode_val, 0, sizeof(att_customPincode_val));
    memset(att_customEncryptKey_val, 0, sizeof(att_customEncryptKey_val));
	u8 security_level=app_ble_get_security_level();
	if (security_level==Unauthenticated_Pairing_with_Encryption ||
		security_level==Authenticated_Pairing_with_Encryption)
	{
		u32 pin=app_config_get_pincode(); set_u32(att_customPincode_val, pin);
	    DEBUGFMT(APP_ATT_LOG_EN, "[ATT] Setup Pincode %u", pin);
		#if (BLE_ATT_CUSTOMCONFIG)
		u8 level=app_config_get_power_level(); att_customPowerLevel_val[0]=level;
	    DEBUGFMT(APP_ATT_LOG_EN, "[ATT] Setup PowerLevel %u", level);
		u8 mode=app_config_get_mode(); att_customDeviceMode_val[0]=mode;
	    DEBUGFMT(APP_ATT_LOG_EN, "[ATT] Setup DeviceMode %u", mode);
		u8 datafmt=app_config_get_dataformat(); att_customDataFormat_val[0]=datafmt;
	    DEBUGFMT(APP_ATT_LOG_EN, "[ATT] Setup DataFormat %u", datafmt);
		#endif
	}
	if (security_level==Authenticated_Pairing_with_Encryption)
	{
	    app_config_get_key(att_customEncryptKey_val);
	    DEBUGHEXBUF(APP_ATT_LOG_EN, "[ATT] Setup EncryptKey %s", att_customEncryptKey_val, 16);
	}
	memset(att_customBTHomeData_val, 0xFF, sizeof(att_customBTHomeData_val));
}

u8 app_ble_att_get_factoryreset(u8 newval)
{
	u8 val=att_customFactoryReset_val[0];
	if (newval != 0xFF)   att_customFactoryReset_val[0]=newval;
	return val;
}


// OTA
#if (BLE_OTA_SERVER_ENABLE)
// TELINK_OTA_UUID_SERVICE 00010203-0405-0607-0809-0a0b0c0d1912
// TELINK_SPP_DATA_OTA     00010203-0405-0607-0809-0a0b0c0d2b12
static const  u8 att_otaServiceUUID16[16] = WRAPPING_BRACES(TELINK_OTA_UUID_SERVICE);
static const  u8 att_otaDataUUID16[16] = WRAPPING_BRACES(TELINK_SPP_DATA_OTA);

_attribute_data_retention_	static u8 att_otaData_val[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
_attribute_data_retention_	static u8 att_otaData_ccc[2] = {0,0};

static const u8 att_otaData_desc[]= {'O', 'T', 'A'};

static const u8 att_otaData_def[19] = {
	CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP | CHAR_PROP_NOTIFY | CHAR_PROP_WRITE,
	U16_LO(OTA_CMD_OUT_DP_H), U16_HI(OTA_CMD_OUT_DP_H),
	TELINK_SPP_DATA_OTA,
};
#endif

// GATT Profile Definition
_attribute_data_retention_ static attribute_t att_Attributes[] =
{
	{ATT_END_H - 1, 0,0,0,0,0,0,0},	// total count of attributes
	// 0x0001 - 0x0007  GAP 0x1800
	{7,ATT_PERMISSIONS_READ,2,2,(u8*)(&att_primaryServiceUUID),(u8*)(&att_gapServiceGenericAccessUUID),0,0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_devNameChar_def),(u8*)(&att_characterUUID),(u8*)(att_devNameChar_def),0,0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_devName_val),(u8*)(&att_devNameUUID), (u8*)(att_devName_val),0,0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_devAppearanceCharVal_def),(u8*)(&att_characterUUID),(u8*)(att_devAppearanceCharVal_def),0,0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_devAppearance_val),(u8*)(&att_devAppearanceUUID),(u8*)(&att_devAppearance_val),0,0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_periConnParamChar_def),(u8*)(&att_characterUUID),(u8*)(att_periConnParamChar_def),0,0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_periConnParameters_val),(u8*)(&att_periConnParamUUID),(u8*)(&att_periConnParameters_val),0,0},
	// 0x0008 - 0x000B GATT 0x1801
	{4,ATT_PERMISSIONS_READ,2,2,(u8*)(&att_primaryServiceUUID),(u8*)(&att_gattServiceUUID),0,0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_serviceChangeChar_def),(u8*)(&att_characterUUID),(u8*)(att_serviceChangeChar_def),0,0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_serviceChange_val),(u8*)(&att_serviceChangeUUID),(u8*)(&att_serviceChange_val),0,0},
	{0,ATT_PERMISSIONS_RDWR,2,sizeof(att_serviceChange_ccc),(u8*)(&att_clientCharacterCfgUUID),(u8*)(att_serviceChange_ccc),0,0},
	// 0x000C - 0x0018 Device Information Service
	{13,ATT_PERMISSIONS_READ,2,2,(u8*)(&att_primaryServiceUUID),(u8*)(&att_devInfoServiceUUID), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_ModChar_def),(u8*)(&att_characterUUID),(u8*)(att_ModChar_def), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_ModelStr_val),(u8*)(&att_devInfoModelUUID),(u8*)(att_ModelStr_val), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_SerialChar_def),(u8*)(&att_characterUUID),(u8*)(att_SerialChar_def), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_SerialStr_val),(u8*)(&att_devInfoSerialUUID),(u8*)(att_SerialStr_val), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_FirmChar_def),(u8*)(&att_characterUUID),(u8*)(att_FirmChar_def), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_FirmStr_val),(u8*)(&att_devInfoFirmwareRevUUID),(u8*)(att_FirmStr_val), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_HardChar_def),(u8*)(&att_characterUUID),(u8*)(att_HardChar_def), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_HardStr_val),(u8*)(&att_devInfoHardwareRevUUID),(u8*)(att_HardStr_val), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_SoftChar_def),(u8*)(&att_characterUUID),(u8*)(att_SoftChar_def), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_SoftStr_val),(u8*)(&att_devInfoSoftwareRevUUID),(u8*)(att_SoftStr_val), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_ManChar_def),(u8*)(&att_characterUUID),(u8*)(att_ManChar_def), 0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_ManStr_val),(u8*)(&att_devInfoManufacturerUUID),(u8*)(att_ManStr_val), 0},
	// 0x0019 - 0x001C Battery Service 0x180F
	{4,ATT_PERMISSIONS_READ,2,2,(u8*)(&att_primaryServiceUUID),(u8*)(&att_batServiceUUID),0,0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_batCharVal_def),(u8*)(&att_characterUUID),(u8*)(att_batCharVal_def),0,0}, // prop
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_bat_val),(u8*)(&att_batCharUUID),(u8*)(att_bat_val),0,0}, // value
	{0,ATT_PERMISSIONS_RDWR,2,sizeof(att_bat_ccc),(u8*)(&att_clientCharacterCfgUUID),(u8*)(att_bat_ccc),0,0}, // value ccc
    // 0x001D - 0x0032 Custom Configuration Service
	{22,ATT_PERMISSIONS_READ,2,16,(u8*)(&att_primaryServiceUUID),(u8*)(att_CustomServiceUUID16),0,0},
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_customPincode_def),(u8*)(&att_characterUUID),(u8*)(att_customPincode_def),0,0}, // prop
	{0,ATT_PERMISSIONS_ENCRYPT_RDWR,16,sizeof(att_customPincode_val),(u8*)(att_CustomAttPincodeUUID16),(u8*)(att_customPincode_val),&customConfigWriteCB,0}, // value
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_customPincode_desc),(u8*)(&att_userdesc_UUID),(u8*)(att_customPincode_desc),0,0}, // desc
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_customEncryptKey_def),(u8*)(&att_characterUUID),(u8*)(att_customEncryptKey_def),0,0}, // prop
	{0,ATT_PERMISSIONS_SECURE_CONN_RDWR,16,sizeof(att_customEncryptKey_val),(u8*)(att_CustomAttEncryptKeyUUID16),(u8*)(att_customEncryptKey_val),&customConfigWriteCB,0}, // value
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_customEncryptKey_desc),(u8*)(&att_userdesc_UUID),(u8*)(att_customEncryptKey_desc),0,0}, // desc
	#if (BLE_ATT_CUSTOMCONFIG)
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_customPowerLevel_def),(u8*)(&att_characterUUID),(u8*)(att_customPowerLevel_def),0,0}, // prop
	{0,ATT_PERMISSIONS_ENCRYPT_RDWR,2,sizeof(att_customPowerLevel_val),(u8*)(&att_CustomAttPowerLevelUUID),(u8*)(att_customPowerLevel_val),&customConfigWriteCB,0}, // value
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_customDeviceMode_def),(u8*)(&att_characterUUID),(u8*)(att_customDeviceMode_def),0,0}, // prop
	{0,ATT_PERMISSIONS_ENCRYPT_RDWR,16,sizeof(att_customDeviceMode_val),(u8*)(&att_CustomAttDeviceModeUUID16),(u8*)(att_customDeviceMode_val),&customConfigWriteCB,0}, // value
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_customDeviceMode_desc),(u8*)(&att_userdesc_UUID),(u8*)(att_customDeviceMode_desc),0,0}, // desc
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_customDataFormat_def),(u8*)(&att_characterUUID),(u8*)(att_customDataFormat_def),0,0}, // prop
	{0,ATT_PERMISSIONS_ENCRYPT_RDWR,16,sizeof(att_customDataFormat_val),(u8*)(&att_CustomAttDataFormatUUID16),(u8*)(att_customDataFormat_val),&customConfigWriteCB,0}, // value
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_customDataFormat_desc),(u8*)(&att_userdesc_UUID),(u8*)(att_customDataFormat_desc),0,0}, // desc
	#endif
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_customBTHomeData_def),(u8*)(&att_characterUUID),(u8*)(att_customBTHomeData_def),0,0}, // prop
	{0,ATT_PERMISSIONS_ENCRYPT_READ,16,sizeof(att_customBTHomeData_val),(u8*)(att_CustomAttBTHomeDataUUID16),(u8*)(att_customBTHomeData_val),0,0}, // value (initial size 0)
	{0,ATT_PERMISSIONS_RDWR,2,sizeof(att_customBTHomeData_ccc),(u8*)(&att_clientCharacterCfgUUID),(u8*)(att_customBTHomeData_ccc),0,0}, // value ccc
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_customBTHomeData_desc),(u8*)(&att_userdesc_UUID),(u8*)(att_customBTHomeData_desc),0,0}, // desc
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_customAttFactoryReset_def),(u8*)(&att_characterUUID),(u8*)(att_customAttFactoryReset_def),0,0}, // prop
	{0,ATT_PERMISSIONS_SECURE_CONN_WRITE,16,sizeof(att_customFactoryReset_val),(u8*)(att_CustomAttFactoryResetUUID16),(u8*)(att_customFactoryReset_val),customConfigWriteCB,0}, // value
	{0,ATT_PERMISSIONS_READ,2,sizeof(att_customFactoryReset_desc),(u8*)(&att_userdesc_UUID),(u8*)(att_customFactoryReset_desc),0,0}, // desc
	// 0x0033 - 0x0037 TELink OTA Service
	#if (BLE_OTA_SERVER_ENABLE)
	{5,ATT_PERMISSIONS_READ,2,16,(u8*)(&att_primaryServiceUUID),(u8*)(att_otaServiceUUID16),0,0},
	{0,ATT_PERMISSIONS_READ,2, sizeof(att_otaData_def),(u8*)(&att_characterUUID),(u8*)(att_otaData_def),0,0}, // prop
	{0,ATT_PERMISSIONS_ENCRYPT_RDWR,16,sizeof(att_otaData_val),(u8*)(att_otaDataUUID16),(att_otaData_val),&otaWrite,0}, // value
	{0,ATT_PERMISSIONS_RDWR,2,sizeof(att_otaData_ccc),(u8*)(&att_clientCharacterCfgUUID),(u8*)(att_otaData_ccc),0,0}, // value ccc
	{0,ATT_PERMISSIONS_READ,2,sizeof (att_otaData_desc),(u8*)(&att_userdesc_UUID),(u8*)(att_otaData_desc),0,0}, // desc
	#endif
};

// Init attribute table
void app_ble_att_init(void)
{
	app_ble_att_setup_serial();
	app_ble_att_setup_config();
	att_Attributes[CustomConfig_BTHomeData_DP_H].attrLen = 0; // variable length data
	bls_att_setAttributeTable((u8 *)att_Attributes);
}

void app_ble_att_set_battery_data(u8 val)
{
	att_bat_val[0]=val;
	if (val_in_ccc(att_bat_ccc))
		bls_att_pushNotifyData(BATT_LEVEL_INPUT_DP_H, att_bat_val, sizeof(att_bat_val));
}

void app_ble_att_set_bthome_data(const u8 *data, u8 len)
{
	if (len > sizeof(att_customBTHomeData_val))   return;
	att_Attributes[CustomConfig_BTHomeData_DP_H].attrLen = len;
	if (!data || len==0)   return;
	memset(att_customBTHomeData_val, 0xFF, sizeof(att_customBTHomeData_val));
	memcpy(att_customBTHomeData_val, data, len);
	if (val_in_ccc(att_customBTHomeData_ccc))
		bls_att_pushNotifyData(CustomConfig_BTHomeData_DP_H, att_customBTHomeData_val, len);
}

void app_ble_att_set_xiaomi_data(const u8 *data, u8 len)
{
	att_Attributes[CustomConfig_BTHomeData_DP_H].attrLen = 0;
}


#endif // #if (APP_BLE_ATT)  // component enabled

