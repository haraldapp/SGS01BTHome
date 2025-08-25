/********************************************************************************************************
 * @file    app_mcu_serial.c
 *
 * @brief   Third party MCU serial protocol
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
#include "app_config.h"

//
// notes:
//  - in current version we use polling the UART fifo
//    since the protocol is only running at 9600 baud
//

#if (APP_MCU_SERIAL)  // component enabled
#include "app.h"

#include "string.h"
#include "timer.h"
#include "uart.h"
#include "lib/include/pm.h"
#include "stack/ble/ble.h"

#ifndef UART_TX_PIN
#error "UART_TX_PIN not defined in app_config.h"
#endif
#ifndef MODULE_WAKEUP_PIN
#error "MODULE_WAKEUP_PIN not defined in app_config.h"
#endif

#ifndef MODULE_WAKEUP_ALIVE_TIME
#define MODULE_WAKEUP_ALIVE_TIME	200000	// 200 ms (stay alive time after wake up from MCU)
#endif
#ifndef MCU_TX_WAKEUP_DELAY
#define MCU_TX_WAKEUP_DELAY		10000	// 10 ms
#endif
#ifndef MCU_TX_RESPONSE_DELAY
#define MCU_TX_RESPONSE_DELAY	200		// 0.2 ms
#endif
// #define MCU_RX_BYTE_TIMEOUT     2000	// 2 ms (9600 baud)
#ifndef MCU_TXRX_PACKET_TIMEOUT
#define MCU_TXRX_PACKET_TIMEOUT	160000	// 160 ms
#endif

#ifndef APP_SERIAL_LOG_EN
#define APP_SERIAL_LOG_EN 0
#endif
#ifndef APP_SERIAL_DEBUG_EN
#define APP_SERIAL_DEBUG_EN 0
#endif

//
// protocol packet
//
#define MCU_PACKET_HDRLEN	6
#define MCU_PACKET_MAXDATA	48
#define MCU_PACKET_CRCLEN	1

typedef struct _attribute_packed_ _mcu_packet_t
{
	u8 header1; // 0x55
	u8 header2; // 0xAA
	u8 version; // 0x00
	u8 command;
	u8 datalen_h;
	u8 datalen_l;
	u8 data[MCU_PACKET_MAXDATA];
} mcu_packet_t;

// packet type
enum {
	PTYPE_NONE=0,
	PTYPE_CMD,
 	PTYPE_RESP,
};

// processing state
enum {
 	PSTATE_NONE=0,
 	PSTATE_WAKEUP,
 	PSTATE_SENDDELAY,
	PSTATE_DATA,
	PSTATE_DONE,
};

enum {
	PERROR_NONE=0,
	PERROR_TIMEOUT,
	PERROR_FORMAT,
	PERROR_SIZE,
	PERROR_CRC
};

#if (APP_SERIAL_LOG_EN)
static const char *perror_txt[]={"", "timeout", "format", "size", "crc"};
#endif

// buffer state
enum {
	BSTATE_IDLE=0,
	BSTATE_READY,
	BSTATE_PROCESS,
	BSTATE_DONE,
	BSTATE_ERROR,
};
// packet buffer
typedef struct _attribute_packed_
{
	u8 bstate; // buffer state BSTATE_xxx
	u8 ptype; // packet type PTYPE_xxx
	u8 pstate; // processing state PSTATE_xxx
	u8 perror; // PERROR_xxx
	u16 dataofs;
	u32 clocktime;
	u16 datalen;
	u8 data[MCU_PACKET_HDRLEN+MCU_PACKET_MAXDATA];
} mcu_buf_t;

static inline mcu_packet_t *buf_data_packet(mcu_buf_t *buf) { return (mcu_packet_t *)buf->data; }

#if (APP_SERIAL_LOG_EN)
static void MCU_DEBUG_PACKET(const char *info, u8 tx, const u8 *data, u16 datalen);
#else
#define MCU_DEBUG_PACKET(...) ((void)0)
#endif

// send/receive buffers
#define TXBUF_CNT 3
static _attribute_data_retention_ u8 mcu_tx_buf_in = 0;
static _attribute_data_retention_ u8 mcu_tx_buf_out = 0;
static _attribute_data_retention_ mcu_buf_t mcu_tx_buf[TXBUF_CNT];
static _attribute_data_retention_ mcu_buf_t mcu_rx_buf;

//
// packet CRC calculation
//
_attribute_optimize_size_ static u16 packet_datalen(const mcu_packet_t *packet)
{
	u16 l=packet->datalen_h; l<<=8; l|=packet->datalen_l;
	return l;
}

_attribute_optimize_size_ static u8 calc_packet_crc(const u8 *p, u16 len)
{   // CRC = sum( header + data )
	u8 crc=0;
	while (len > 0)  { crc+=(*p); p++; len--; }
	return crc;
}

_attribute_optimize_size_ static void add_packet_crc(mcu_packet_t *packet)
{
	u16 datalen=packet_datalen(packet);
	u8 crc=calc_packet_crc((u8 *)packet, MCU_PACKET_HDRLEN+datalen);
	packet->data[datalen]=crc;
}

_attribute_optimize_size_ static u8 check_packet_crc(mcu_packet_t *packet)
{
	u16 datalen=packet_datalen(packet);
	u8 crc=calc_packet_crc((u8 *)packet, MCU_PACKET_HDRLEN+datalen);
	if (packet->data[datalen] != crc)   return 0;
	return 1;
}

//
// target/hardware mapping
//
static u8 mcu_uart_initialized = 0;
static u8 mcu_pad_wakeup = 0;
static u32 mcu_pad_wakeup_time = 0;

static void mcu_uart_init(void)
{   // init normal and DeepRetn
	uart_gpio_set(UART_TX_PIN, UART_RX_PIN);
	uart_reset(); // reset all UART registers
	uart_ndma_clear_tx_index(); uart_ndma_clear_rx_index();
	uart_init_baudrate(UART_BAUDRATE, CLOCK_SYS_CLOCK_HZ, PARITY_NONE, STOP_BIT_ONE);
	uart_irq_enable(0, 0); // disable
	mcu_uart_initialized = 1;
}

static u8 inline get_tx_isbusy(void)
{
	return uart_tx_is_busy();
}
static u8 inline get_tx_fifo_cnt(void)
{
	return (reg_uart_buf_cnt >> 4);
}
static u8 inline get_rx_fifo_cnt(void)
{
	return (reg_uart_buf_cnt & 0x0F);
}
static void inline push_tx_fifo(u8 b)
{
	uart_ndma_send_byte(b);
}
static u8 inline pop_rx_fifo(void)
{
	return uart_ndma_read_byte();
}

void mcu_wakeup_init(void)
{
	// MCU wakeup (pull down resistor 10K on board or MCU pulldown)
	gpio_set_func(MCU_WAKEUP_PIN, AS_GPIO);
	gpio_setup_up_down_resistor(MCU_WAKEUP_PIN, PM_PIN_PULLDOWN_100K);
	gpio_set_output_en(MCU_WAKEUP_PIN, 0);
	gpio_set_input_en(MCU_WAKEUP_PIN, 0);
	gpio_set_data_strength(MCU_WAKEUP_PIN, 0);
	// Module wakeup
	gpio_set_func(MODULE_WAKEUP_PIN, AS_GPIO);
	gpio_setup_up_down_resistor(MODULE_WAKEUP_PIN, PM_PIN_PULLDOWN_100K);
	gpio_set_output_en(MODULE_WAKEUP_PIN, 0);
	gpio_set_input_en(MODULE_WAKEUP_PIN, 1);
	cpu_set_gpio_wakeup(MODULE_WAKEUP_PIN, Level_High, 1);
}

_attribute_ram_code_ void mcu_wakeup_init_deepRetn(void)
{
	uart_ndma_clear_tx_index(); // must
	uart_ndma_clear_rx_index();
	gpio_set_input_en(MODULE_WAKEUP_PIN, 1);
	cpu_set_gpio_wakeup(MODULE_WAKEUP_PIN, Level_High, 1);
}

static void inline mcu_wakeup_start(void)
{   // pin high
	gpio_set_output_en(MCU_WAKEUP_PIN, 1);
	gpio_write(MCU_WAKEUP_PIN, 1);
}

static void inline mcu_wakeup_end(void)
{	// pin low
	gpio_set_output_en(MCU_WAKEUP_PIN, 1);
	gpio_write(MCU_WAKEUP_PIN, 0);
}

_attribute_ram_code_ u8 module_wakeup_status(void)
{
	return gpio_read(MODULE_WAKEUP_PIN);
}

//
// send / receive packets
//
enum
{
	RXTX_EVT_RECV=0, RXTX_EVT_SEND=1,
	RXTX_EVT_ERR_TIMEOUT=-1, RXTX_EVT_ERR_FORMAT=-2, RXTX_EVT_ERR_SIZE=-3, RXTX_EVT_ERR_CRC=-4,
};

static u8 rxtx_notify(int evt, u8 stat, const mcu_packet_t *pkt);

static void mcu_init_serial(u8 resetbuf)
{	// init normal and DeepRetn
	mcu_uart_init();
	mcu_tx_buf_in = 0; mcu_tx_buf_out = 0;
	mcu_rx_buf.bstate = BSTATE_IDLE;
	if (resetbuf)
	{
		memset( &mcu_tx_buf[0], 0, TXBUF_CNT*sizeof(mcu_buf_t) );
		memset( &mcu_rx_buf, 0, sizeof(mcu_buf_t) );
	}
}

_attribute_optimize_size_ int mcu_send(u8 ptype, u8 cmd, u8 datalen, const u8* data)
{
	mcu_buf_t *buf=&mcu_tx_buf[mcu_tx_buf_in];
	if (buf->bstate!=BSTATE_IDLE)
    {
        DEBUGSTR(APP_SERIAL_LOG_EN, "[MCU] Send buffer busy");
		return -2; // no free buffer
	}
	mcu_tx_buf_in++; if (mcu_tx_buf_in>=TXBUF_CNT)   mcu_tx_buf_in=0;
	if (datalen>MCU_PACKET_MAXDATA-1)
	{
        DEBUGSTR(APP_SERIAL_LOG_EN, "[MCU] CMD Packet length error");
		return -3; // too large
	}
	// build packet
	mcu_packet_t *pkt=buf_data_packet(buf);
	pkt->header1 = 0x55; pkt->header2 = 0xAA;
	pkt->version = 0x00; pkt->command = cmd;
	pkt->datalen_h = 0x00; pkt->datalen_l = datalen;
    if (datalen>0 && data)   memcpy(pkt->data, data, datalen);
	add_packet_crc(pkt);
	buf->datalen=MCU_PACKET_HDRLEN+datalen+1; buf->dataofs=0;
    // DEBUGHEXBUF(APP_SERIAL_LOG_EN, "[MCU] > %s", buf->data, buf->datalen);
	MCU_DEBUG_PACKET("[MCU]", 1, buf->data, buf->datalen);
    // buffer state
	buf->bstate=BSTATE_READY; buf->ptype=ptype; buf->pstate=PSTATE_NONE;
	buf->clocktime=clock_time();
    return 1;
}

int mcu_send_ack(u8 cmd)
{
	return mcu_send(PTYPE_RESP, cmd, 0, 0);
}

int mcu_send_ack_status(u8 cmd, u8 status)
{
	return mcu_send(PTYPE_RESP, cmd, 1, &status);
}

_attribute_optimize_size_ static u8 mcu_handle_send(void)
{
	mcu_buf_t *buf=&mcu_tx_buf[mcu_tx_buf_out];
	if (buf->bstate == BSTATE_IDLE)
		return 0; // nothing to send
	// init send state
	if (buf->bstate == BSTATE_READY)
	{
		buf->bstate = BSTATE_PROCESS;
		buf->pstate = PSTATE_NONE;
	}
	// handle send states
	if (buf->bstate == BSTATE_PROCESS)
	{
		if (buf->pstate == PSTATE_NONE)
		{	// init
			buf->dataofs=0; buf->clocktime=clock_time();
			buf->pstate=PSTATE_WAKEUP;
		}
		if (buf->pstate == PSTATE_WAKEUP)
		{	// wake up MCU
			if (buf->ptype == PTYPE_CMD)
				mcu_wakeup_start();
			buf->pstate=PSTATE_SENDDELAY;
		}
		if (buf->pstate == PSTATE_SENDDELAY)
		{   // send delay
			u32 delay=0; // send delay
			if (buf->ptype == PTYPE_CMD)   delay=MCU_TX_WAKEUP_DELAY;
			if (buf->ptype == PTYPE_RESP)   delay=MCU_TX_RESPONSE_DELAY;
			if (delay && !clock_time_exceed(buf->clocktime,delay))    return 1; // busy
			buf->pstate=PSTATE_DATA; // send delay processed
		}
		while (buf->pstate == PSTATE_DATA)
		{
			if (buf->dataofs >= buf->datalen) { buf->pstate++; break; } // all data done
			if (get_tx_fifo_cnt()>7)    return 1;  // TX FIFO busy
			push_tx_fifo( buf->data[buf->dataofs] );
			buf->dataofs++;
		}
		if (buf->pstate == PSTATE_DONE)
		{
			if (get_tx_fifo_cnt()>0)    return 1;  // wait for fifo send
			if (get_tx_isbusy()>0)    return 1;  // wait for all data send
			if (buf->ptype == PTYPE_CMD)
				mcu_wakeup_end();
			rxtx_notify(RXTX_EVT_SEND, 0, buf_data_packet(buf));
			buf->bstate=BSTATE_IDLE;
			mcu_tx_buf_out++; if (mcu_tx_buf_out>=TXBUF_CNT)   mcu_tx_buf_out=0; // next rx buffer
		}
	}
	return 0; // all done
}

static inline u8 mcu_tx_busy(void)
{
	return (mcu_tx_buf[mcu_tx_buf_out].bstate == BSTATE_IDLE) ? 0 : 1;
}

_attribute_optimize_size_ static u8 mcu_handle_receive(u8 irq)
{
	mcu_buf_t *buf=&mcu_rx_buf; // only one rx buffer
	// Check for MCU notifications in idle state
	if (buf->bstate == BSTATE_IDLE)
    {
		if (get_rx_fifo_cnt()==0)    return 0;  // nothing to receive
		// start receive packet
		buf->datalen = 0; buf->dataofs = 0;
		buf->clocktime = clock_time();
		buf->pstate = PSTATE_DATA; buf->perror = PERROR_NONE;
		buf->bstate = BSTATE_PROCESS;
    }
	while (buf->bstate == BSTATE_PROCESS && buf->pstate == PSTATE_DATA)
	{
		if (get_rx_fifo_cnt() == 0)
		{
			if (!clock_time_exceed(buf->clocktime,MCU_TXRX_PACKET_TIMEOUT))    return 1; // wait for next byte
			buf->bstate = BSTATE_ERROR; buf->perror = PERROR_TIMEOUT;
			break;
		}
		u8 b=pop_rx_fifo();
		if (buf->dataofs < sizeof(buf->data))
			buf->data[buf->dataofs++]=b;
		buf->datalen++; buf->clocktime=clock_time();
		// pre check packet data
		mcu_packet_t *pkt=buf_data_packet(buf);
		if ((buf->datalen==1 && pkt->header1!=0x55) ||
			(buf->datalen==2 && pkt->header2!=0xAA))
		{
			buf->datalen = 0; buf->dataofs = 0; // restart rx
		}
		if (buf->datalen > MCU_PACKET_HDRLEN)
		{
			u16 pktdatalen=packet_datalen(pkt);
			if (pktdatalen > 500)
			{
				buf->bstate = BSTATE_ERROR; buf->perror = PERROR_FORMAT;
			}
			else if (buf->datalen==MCU_PACKET_HDRLEN+pktdatalen+1)
				buf->pstate = PSTATE_DONE; // header+data+crc done
		}
	}
	if (buf->bstate == BSTATE_PROCESS && buf->pstate == PSTATE_DONE && !irq)
	{
		u8 ret=0; mcu_packet_t *pkt=buf_data_packet(buf);
		if (buf->datalen > MCU_PACKET_HDRLEN+MCU_PACKET_MAXDATA)
		{
			buf->bstate = BSTATE_ERROR; buf->perror = PERROR_SIZE;
		}
		else if (!check_packet_crc(pkt))
		{
			buf->bstate = BSTATE_ERROR; buf->perror = PERROR_CRC;
		}
	    else
	    {
	        // DEBUGHEXBUF(APP_SERIAL_LOG_EN, "[MCU] < %s", buf->data, buf->datalen);
	    	MCU_DEBUG_PACKET("[MCU]", 0, buf->data, buf->datalen);
	    	ret=rxtx_notify(RXTX_EVT_RECV, 0, pkt);
			buf->bstate = BSTATE_IDLE; buf->pstate = PSTATE_NONE;
	    }
		return ret;
	}
	if (buf->bstate == BSTATE_ERROR && !irq)
	{
		u8 perror=buf->perror;
		#if (APP_SERIAL_LOG_EN)
		DEBUGFMT(APP_SERIAL_LOG_EN, "[MCU] Receive error: %s", perror_txt[perror] );
		#endif
		buf->bstate = BSTATE_IDLE;
		return rxtx_notify(RXTX_EVT_RECV, perror, 0);
	}
	return 0;
}

static inline u8 mcu_rx_busy(void)
{
	return (mcu_rx_buf.bstate == BSTATE_IDLE) ? 0 : 1;
}


//
// Third party MCU protocol
//

enum {
	// to MCU
	CMD_DetectHeartbeat = 0x00, // datalen=0, response ack status (0=first after restart, 1=running)
	CMD_GetMCUInformation = 0x01, // datalen=0, response ack data (ProduktID,Version,...)
	CMD_RequestWorkingMode = 0x02, // datalen=0, response ack
	CMD_SendModuleStatus = 0x03, // datalen=1, response ack status (0=success)
	CMD_SendCommands = 0x06, // datalen=x DP list, response ack status (0=success)
	CMD_QueryStatus = 0x08, // datalen=0, response ReportStatus=0x07
	CMD_NotifyFactoryReset = 0xA1, // datalen=0, response none
	CMD_QueryMCUVersion = 0xE8, // datalen=0, response ack data (version info)
	// from MCU
	CMD_ResetModule = 0x04, // datalen=0, response ack
	CMD_ResetModuleNew = 0x05, // datalen=0, response ack
	CMD_ReportStatus = 0x07, // datalen=x DP list
	CMD_UnbindModule = 0x09, // datalen=0, response ack status
	CMD_QueryConnectionStatus = 0x0A, // datalen=0, response SendModuleStatus
	CMD_QueryRSSI = 0x0E, // datalen=0, response ack data (RSSI info)
	CMD_QueryModuleVersion = 0xA0, // datalen=0, response ack data (version info)
	CMD_ReportData = 0xE0, // DP list, response ack status (0=success)
	CMD_GetCurrentTime = 0xE1, // datalen=1 time type, response ack data (time data)
	CMD_ConfigSystemTimer = 0xE4, // datalen=1 1=enable, response ack status (0=success)
	CMD_EnableLowPower = 0xE5, // datalen=1 1=enable, response ack status (0=success)
	CMD_ReportMCUVersion = 0xE9, // datalen=6 version info, response ack status (0=success)
	// internal
	CMD_None = 0xFF
};

enum {
	DATA_Ack_Status_Success=0
};

enum {
	DATA_ModuleStatus_Idle=0, // not bound+not connected
	DATA_ModuleStatus_Bound=1, // bound+not connected
	DATA_ModuleStatus_Connected=2, // bound+connected
};

typedef struct _attribute_packed_ _mcu_data_version_t
{
	u8 soft_ver[3];
	u8 hard_ver[3];
} mcu_data_version_t;

typedef struct _attribute_packed_ _mcu_data_time1_t
{
	u8 result; // = 0 success
	u8 format; // = 1
	char time_string[13]; // UNIX time string ms
	u8 time_zone0;
	u8 time_zone1;
} mcu_data_time1_t;

#if (APP_SERIAL_LOG_EN)
static void MCU_DEBUG_PACKET(const char *info, u8 tx, const u8 *data, u16 datalen)
{
	u8 u; const mcu_packet_t *pkt=(const mcu_packet_t *)data;
	DEBUGOUTSTR(info); DEBUGOUT(' ');
	DEBUGOUT(tx ? '>' : '<' ); DEBUGOUT(' ');
	for (u=0; u<datalen; u++)   DEBUGOUTHEX(data[u]);
	if (datalen<MCU_PACKET_HDRLEN) { DEBUGOUT('\n'); return; }
	enum { PARAM_None=0, PARAM_Ack, PARAM_Status, PARAM_Enable, PARAM_Running, PARAM_ModStat, PARAM_PID, PARAM_Version, PARAM_Time, PARAM_DPData };
	static const struct { u8 cmd; const char *txt; u8 notify; u8 txparam; u8 rxparam; } _cmd2txt[]={
		{CMD_DetectHeartbeat, "DetectHeartbeat", 0, PARAM_None, PARAM_Running},
		{CMD_GetMCUInformation, "GetMCUInformation", 0, PARAM_None, PARAM_PID},
		{CMD_RequestWorkingMode, "RequestWorkingMode", 0, PARAM_None, PARAM_Ack},
		{CMD_SendModuleStatus, "SendModuleStatus", 0, PARAM_ModStat, PARAM_None},
		{CMD_SendCommands, "SendCommands", 0},
		{CMD_QueryStatus, "QueryStatus", 0},
		{CMD_NotifyFactoryReset, "NotifyFactoryReset", 0},
		{CMD_QueryMCUVersion, "QueryMCUVersion", 0, PARAM_None, PARAM_Version},
		{CMD_ResetModule, "ResetModule", 1, PARAM_Ack, PARAM_None},
		{CMD_ResetModuleNew, "ResetModuleNew", 1, PARAM_Ack, PARAM_None},
		{CMD_ReportStatus, "ReportStatus", 1, PARAM_DPData, PARAM_None},
		{CMD_UnbindModule, "UnbindModule", 1, PARAM_Ack, PARAM_None},
		{CMD_QueryConnectionStatus, "QueryConnectionStatus", 1},
		{CMD_QueryRSSI, "QueryRSSI", 1},
		{CMD_QueryModuleVersion, "QueryModuleVersion", 1, PARAM_Version, PARAM_None},
		{CMD_ReportData, "ReportData", 1, PARAM_Ack, PARAM_DPData},
		{CMD_GetCurrentTime, "GetCurrentTime", 1, PARAM_Time, PARAM_None},
		{CMD_ConfigSystemTimer, "ConfigSystemTimer", 1},
		{CMD_EnableLowPower, "EnableLowPower", 1, PARAM_Ack, PARAM_Enable},
		{CMD_ReportMCUVersion, "ReportMCUVersion", 1},
		{CMD_None}
	};
	u8 cmd=pkt->command;
	for (u=0; _cmd2txt[u].cmd!=CMD_None; u++)
	{
		if ( _cmd2txt[u].cmd!=cmd)   continue;
		DEBUGOUT(' '); DEBUGOUTSTR(_cmd2txt[u].txt);
		u16 pkt_data_len=packet_datalen(pkt); u8 ofs, param_type=PARAM_None;
		if (tx)  param_type=_cmd2txt[u].txparam;
		else     param_type=_cmd2txt[u].rxparam;
		if (param_type!=PARAM_None)
			DEBUGOUT(' ');
		if (param_type==PARAM_Ack)
			DEBUGOUTSTR("ack");
		if (param_type==PARAM_Status && pkt_data_len>=1)
			DEBUGOUTSTR(pkt->data[0] ? "error" : "success");
		if (param_type==PARAM_Enable && pkt_data_len>=1)
			DEBUGOUTSTR(pkt->data[0] ? "enable" : "disable");
		if (param_type==PARAM_Running && pkt_data_len>=1)
			DEBUGOUTSTR(pkt->data[0] ? "running" : "reset");
		if (param_type==PARAM_ModStat && pkt_data_len>=1) {
			static const char *modstat[]={"idle","bound","bound+connected"};
			if (pkt->data[0]<3)	DEBUGOUTSTR(modstat[pkt->data[0]]);
			else				DEBUGOUTSTR("?");
		}
		if (param_type==PARAM_PID && pkt_data_len>=8) {
			DEBUGOUTSTR("PID: ");
			for(ofs=0; ofs<8; ofs++)   DEBUGOUT(pkt->data[ofs]);
		}
		if (param_type==PARAM_Version && pkt_data_len>=6) {
			DEBUGOUTSTR("Soft: ");
			for(ofs=0; ofs<3; ofs++) { DEBUGOUT('0'+pkt->data[ofs]); if (ofs<2) DEBUGOUT('.'); }
			DEBUGOUTSTR(" Hard: ");
			for(ofs=3; ofs<6; ofs++) { DEBUGOUT('0'+pkt->data[ofs]); if (ofs<5) DEBUGOUT('.'); }
		}
		break;
	}
	DEBUGOUT('\n');
}
#endif

//
// send / receive
//

typedef struct _attribute_packed_ _mcu_cmd_seq_t
{
	u8 cmd;
	u8 cmddatalen;
	const void* cmddata;
	u8 resp;
	u8 respdatalen;
	void* respdata;
	u8 appnotify;
} mcu_cmd_seq_t;

static const u8 data_module_status_idle = DATA_ModuleStatus_Idle;
// static const u8 data_module_status_bound = DATA_ModuleStatus_Bound;
static const u8 data_module_status_connected = DATA_ModuleStatus_Connected;

static _attribute_data_retention_ char mcu_pid[8]={0,0,0,0,0,0,0,0}; // product id
static _attribute_data_retention_ struct _mcu_data_version_t mcu_ver={ {0,0,0}, {0,0,0} }; // MCU version
static const struct _mcu_data_version_t moduleVersionInfo={ {1,0,0}, {1,0,0} };
static const char moduleRSSIInfo[]= {"\"ret\":true,\"rssi\":\"-55\""};

static const struct _mcu_cmd_seq_t mcu_init_cmd_seq[]={
	{CMD_DetectHeartbeat, 0, 0, CMD_DetectHeartbeat, 0, 0},
	{CMD_GetMCUInformation, 0, 0, CMD_GetMCUInformation, sizeof(mcu_pid), mcu_pid},
	{CMD_QueryMCUVersion, 0, 0, CMD_QueryMCUVersion, sizeof(mcu_ver), &mcu_ver},
	{CMD_QueryStatus, 0, 0, CMD_ReportStatus, 0, 0},
	{CMD_SendModuleStatus, 1, &data_module_status_idle, CMD_None, 0, 0},
	{CMD_None}
};

static const struct _mcu_cmd_seq_t mcu_start_measure_cmd_seq[]={
	{CMD_DetectHeartbeat, 0, 0, CMD_DetectHeartbeat, 0, 0},
	{CMD_RequestWorkingMode, 0, 0, CMD_RequestWorkingMode, 0, 0},
	{CMD_QueryStatus, 0, 0, CMD_ReportStatus, 0, 0},
	{CMD_SendModuleStatus, 1, &data_module_status_connected, CMD_None, 0, 0},
	{CMD_None}
};

static const struct _mcu_cmd_seq_t mcu_start_connect_cmd_seq[]={
	{CMD_DetectHeartbeat, 0, 0, CMD_DetectHeartbeat, 0, 0},
	{CMD_SendModuleStatus, 1, &data_module_status_idle, CMD_None, 0, 0},
	{CMD_None}
};

static const struct _mcu_cmd_seq_t mcu_update_connect_cmd_seq[]={
	{CMD_DetectHeartbeat, 0, 0, CMD_DetectHeartbeat, 0, 0},
	{CMD_SendModuleStatus, 1, &data_module_status_connected, CMD_None, 0, 0},
	{CMD_DetectHeartbeat, 0, 0, CMD_DetectHeartbeat, 0, 0},
	{CMD_SendModuleStatus, 1, &data_module_status_idle, CMD_None, 0, 0},
	{CMD_None}
};

static const struct _mcu_cmd_seq_t mcu_check_stat_cmd_seq[]={
	{CMD_DetectHeartbeat, 0, 0, CMD_DetectHeartbeat, 0, 0},
	{CMD_None}
};

static _attribute_data_retention_ const struct _mcu_cmd_seq_t *current_cmd_seq = 0;
enum { CMD_SEQ_STAT_none=0, CMD_SEQ_STAT_init, CMD_SEQ_STAT_send, CMD_SEQ_STAT_sendingdata, CMD_SEQ_STAT_waitresp, CMD_SEQ_STAT_done };
static _attribute_data_retention_ u8 current_cmd_seq_stat = CMD_SEQ_STAT_none;
static _attribute_data_retention_ u8 current_cmd_seq_retry = 0;
static _attribute_data_retention_ u32 current_cmd_seq_time = 0;

static u8 mcu_cmd_seq_init(const struct _mcu_cmd_seq_t *seq)
{
	if (current_cmd_seq != 0)   return 1;
    DEBUGSTR(APP_SERIAL_LOG_EN, "[MCU] CmdSeq init");
	current_cmd_seq=seq;
	current_cmd_seq_stat=CMD_SEQ_STAT_init;
	return 1;
}

static inline u8 mcu_cmd_seq_active(void)
{
	return current_cmd_seq?1:0;
}

static u8 mcu_cmd_seq_error(void)
{
	current_cmd_seq = 0;
	current_cmd_seq_stat = CMD_SEQ_STAT_none;
	return 0; // not busy
}

_attribute_optimize_size_ static u8 mcu_cmd_seq_loop(void)
{
	if (current_cmd_seq == 0)   return 0;
	if (current_cmd_seq_stat==CMD_SEQ_STAT_init)
	{
		current_cmd_seq_stat++;
		current_cmd_seq_retry = 0; // init retry
	}
	if (current_cmd_seq_stat==CMD_SEQ_STAT_send)
	{
		mcu_send(PTYPE_CMD, current_cmd_seq->cmd, current_cmd_seq->cmddatalen, current_cmd_seq->cmddata);
		current_cmd_seq_stat++;
		current_cmd_seq_time = clock_time();
	}
	if (current_cmd_seq_stat==CMD_SEQ_STAT_sendingdata)
	{
		if (!clock_time_exceed(current_cmd_seq_time,MCU_TXRX_PACKET_TIMEOUT))    return 1; // busy
		current_cmd_seq_retry++; if (current_cmd_seq_retry>2)   return mcu_cmd_seq_error();
        DEBUGSTR(APP_SERIAL_LOG_EN, "[MCU] CmdSeq retry (transmit timeout)");
        current_cmd_seq_stat=CMD_SEQ_STAT_send;
	}
	if (current_cmd_seq_stat==CMD_SEQ_STAT_waitresp)
	{
		if (current_cmd_seq->resp==CMD_None)  { current_cmd_seq_stat++; return 1; } // no response expected
		if (!clock_time_exceed(current_cmd_seq_time,MCU_TXRX_PACKET_TIMEOUT))    return 1; // busy
		current_cmd_seq_retry++; if (current_cmd_seq_retry>2)   return mcu_cmd_seq_error();
        DEBUGFMT(APP_SERIAL_LOG_EN, "[MCU] CmdSeq retry %u (response timeout)", current_cmd_seq_retry);
        current_cmd_seq_stat=CMD_SEQ_STAT_send;
	}
	if (current_cmd_seq_stat==CMD_SEQ_STAT_done)
	{
		current_cmd_seq++; // next command
		if (current_cmd_seq->cmd==CMD_None)
		{
	        DEBUGSTR(APP_SERIAL_LOG_EN, "[MCU] CmdSeq done");
			current_cmd_seq=0; return 0;
		}
		current_cmd_seq_stat=CMD_SEQ_STAT_init;
	}
	return 1;

}

_attribute_optimize_size_ static u8 mcu_cmd_seq_tx_notify(const mcu_packet_t *pkt)
{
	if (current_cmd_seq == 0)   return 0;
	if (current_cmd_seq_stat==CMD_SEQ_STAT_sendingdata && pkt->command==current_cmd_seq->cmd)
	{
		current_cmd_seq_stat++;	current_cmd_seq_time = clock_time();
	}
	return 1;
}

_attribute_optimize_size_ static u8 mcu_cmd_seq_rx_notify(const mcu_packet_t *pkt)
{
	// command sequence
	if (current_cmd_seq && current_cmd_seq_stat==CMD_SEQ_STAT_waitresp && pkt->command==current_cmd_seq->resp)
	{
		if (current_cmd_seq->respdatalen>0 && current_cmd_seq->respdata!=0)
		{	// save data
			u16 len=packet_datalen(pkt);
			if (len > current_cmd_seq->respdatalen)   len=current_cmd_seq->respdatalen;
			memcpy( current_cmd_seq->respdata, pkt->data, len);
		}
		current_cmd_seq_stat++;
	}
	return (current_cmd_seq?1:0); // busy
}

//
// RX/TX notifications
//

_attribute_optimize_size_ static u8 tx_notify(const mcu_packet_t *pkt)
{
	if (!pkt)   return 0;
	u8 ret=mcu_cmd_seq_tx_notify(pkt);
	return ret;
}

_attribute_optimize_size_ static u8 rx_notify(const mcu_packet_t *pkt)
{
	enum { RESP_none=0, RESP_ack, RESP_ack_status, RESP_data };
	if (!pkt)   return 0;
	// app data notify
	u8 notify=0;
	if (pkt->command==CMD_GetMCUInformation)	notify=APP_NOTIFY_PRODUCTID;
	else if (pkt->command==CMD_ReportData)		notify=APP_NOTIFY_DPDATA;
	else if (pkt->command==CMD_ReportStatus)	notify=APP_NOTIFY_DPDATA;
	else if (pkt->command==CMD_ResetModule)		notify=APP_NOTIFY_FACTORYRESET;
	else if (pkt->command==CMD_ResetModuleNew)	notify=APP_NOTIFY_FACTORYRESET;
	if (notify)   app_notify(notify, pkt->data, packet_datalen(pkt));
	// command response from MCU
	u8 cmdresp=RESP_none;
	switch (pkt->command)
	{
		case CMD_DetectHeartbeat: // 0x00, datalen=0, response ack status (0=first after restart, 1=running)
			cmdresp=RESP_ack_status; break;
		case CMD_GetMCUInformation: // 0x01, datalen=0, response ack data (ProduktID,Version,...)
			cmdresp=RESP_data; break;
		case CMD_RequestWorkingMode: // 0x02 datalen=0, response ack
			cmdresp=RESP_ack; break;
		case CMD_SendCommands: // 0x06, datalen=x DP list, response ack status (0=success)
			cmdresp=RESP_ack_status; break;
		case CMD_ReportStatus: // 0x07 DP list (response to CMD_QueryStatus)
			cmdresp=RESP_data; break;
		case CMD_QueryMCUVersion: // 0xE8 datalen=0, reponse ack data (version info)
			cmdresp=RESP_data; break;
	}
	if (cmdresp != RESP_none)
	{
		u8 ret=mcu_cmd_seq_rx_notify(pkt); // handle command sequence response
		return ret;
	}
	// command from MCU
	u8 resp=RESP_none, respcmd=pkt->command, respack=DATA_Ack_Status_Success;
	switch (pkt->command)
	{
		case CMD_ResetModule: // 0x04
		case CMD_ResetModuleNew: // 0x05
			resp=RESP_ack; break;
		case CMD_UnbindModule: // 0x09
			resp=RESP_ack_status; break;
		case CMD_QueryConnectionStatus: // 0x0A response SendModuleStatus
			mcu_send(PTYPE_RESP, CMD_SendModuleStatus, 1, &data_module_status_connected);
			resp=RESP_data;	break;
		case CMD_QueryRSSI: // 0x0E RSSI info
			mcu_send(PTYPE_RESP, CMD_QueryRSSI, sizeof(moduleRSSIInfo), (const u8 *)moduleRSSIInfo);
			resp=RESP_data;	break;
		case CMD_QueryModuleVersion: // 0xA0 version info
			mcu_send(PTYPE_RESP, CMD_QueryModuleVersion, sizeof(moduleVersionInfo), (const u8 *)&moduleVersionInfo);
			resp=RESP_data; break;
		case CMD_ReportData: // 0xE0 DP list
			resp=RESP_ack_status; break;
		case CMD_GetCurrentTime: // 0xE1, // datalen=1 time type, reponse ack data (time data)
		{
			if (packet_datalen(pkt) != 1)   break;
			// u8 time_format=(pkt->data[0] & 0x0F);
			// u8 time_source=(pkt->data[0] >> 4) & 0x03;
			static const char *utime="1749986458000"; // send some time
			mcu_data_time1_t t; t.result=DATA_Ack_Status_Success; t.format=1;
			memcpy(t.time_string, utime, 13);
			t.time_zone0=0; t.time_zone1=0xc8;
			mcu_send(PTYPE_RESP, CMD_GetCurrentTime, sizeof(t), (const u8 *)&t);
			resp=RESP_data; break;
		} break;
		case CMD_ConfigSystemTimer: // 0xE4, // datalen=1 1=enable
			resp=RESP_ack_status; break;
		case CMD_EnableLowPower: // 0xE5, // datalen=1 1=enable
			resp=RESP_ack_status; break;
		case CMD_ReportMCUVersion: // 0xE9, // datalen=6 version info
			resp=RESP_ack_status; break;
	}
	if (resp != RESP_none)
	{
		// send generic ACK response
		if (resp == RESP_ack)		 { mcu_send_ack(respcmd); }
		if (resp == RESP_ack_status) { mcu_send_ack_status(respcmd, respack); }
		return 1; // 1=busy
	}
	return 0;
}

_attribute_optimize_size_ static u8 rxtx_notify(int evt, u8 stat, const mcu_packet_t *pkt)
{
	if (evt==RXTX_EVT_RECV && stat==0 && pkt)
		return rx_notify(pkt);
	if (evt==RXTX_EVT_SEND && stat==0 && pkt)
		return tx_notify(pkt);
	return 0;
}

//
// app interface
//

static u32 mcu_init_cmd_start=0;

void app_serial_init_normal(void)
{
	mcu_uart_initialized = 0;
	mcu_pad_wakeup = 0;
	mcu_init_serial(1);
	mcu_init_cmd_start = clock_time() | 1; // start init command sequence delayed
	mcu_pad_wakeup_time=0;
}

_attribute_optimize_size_ void app_serial_init_deepRetn(void)
{
	mcu_uart_initialized = 0;
	mcu_pad_wakeup = pm_is_deepPadWakeup();
	mcu_init_cmd_start = 0;
	mcu_pad_wakeup_time=0;
	if (mcu_pad_wakeup)
	{
		mcu_pad_wakeup_time=clock_time()|1;
		DEBUGSTR(APP_SERIAL_DEBUG_EN, "[MCU] Module Pad Wakeup");
	}
}

static _attribute_data_retention_ const struct _mcu_cmd_seq_t *next_cmd_seq = 0;
static _attribute_data_retention_ u32 mcu_cmd_seq_start_clock;
static _attribute_data_retention_ u32 mcu_cmd_seq_start_delay;

_attribute_optimize_size_ u8 app_serial_loop(void)
{
	if (mcu_pad_wakeup && !mcu_uart_initialized)
	{
		mcu_init_serial(1);
		//TODO: may check the result of CMDDetectHeartbeat and initialize MCU if it did reset
		if (!next_cmd_seq)
			app_serial_cmd_seq_start(MCU_CMD_SEQ_CHECKSTAT, 0);
	}
	if (!mcu_cmd_seq_active() && next_cmd_seq && clock_time_exceed(mcu_cmd_seq_start_clock,mcu_cmd_seq_start_delay))
	{
		mcu_cmd_seq_init(next_cmd_seq);
		next_cmd_seq = 0;
	}
    #ifdef MODULE_WAKEUP_ALIVE_TIME
	if (mcu_pad_wakeup_time && clock_time_exceed(mcu_pad_wakeup_time,MODULE_WAKEUP_ALIVE_TIME))
	{
		DEBUGSTR(APP_SERIAL_DEBUG_EN, "[MCU] PAD Wakeup time done");
		mcu_pad_wakeup_time=0;
	}
	#endif
	u8 busy=0;
	if (next_cmd_seq)   busy=1;
	if (mcu_pad_wakeup_time)   busy=1;
	busy |= mcu_handle_send();
	busy |= mcu_handle_receive(0);
	busy |= mcu_cmd_seq_loop();
	busy |= module_wakeup_status();
	// debug status
    #if (APP_SERIAL_DEBUG_EN)
	static _attribute_data_retention_ u8 wakeup_last=0xFF;
	u8 wakeup=module_wakeup_status();
	if (wakeup!=wakeup_last) {
		wakeup_last=wakeup;	DEBUGFMT(APP_SERIAL_DEBUG_EN, "[MCU] Module Wakeup %u", wakeup);
	}
	static _attribute_data_retention_ u8 busy_last=0xFF;
	if (busy!=busy_last) {
		busy_last=busy;	DEBUGFMT(APP_SERIAL_DEBUG_EN, "[MCU] Serial Stat %s", (busy?"busy":"idle"));
	}
	#endif
	//TODO: may check if we get data at least xxx minutes - query data
	return busy ? APP_PM_DISABLE_SLEEP : APP_PM_DEFAULT;
}

u8 app_serial_rxtx_busy(void)
{
	return mcu_rx_busy() | mcu_tx_busy();
}

static const mcu_cmd_seq_t *cmd_seq_def[]={
	0, mcu_init_cmd_seq, mcu_start_measure_cmd_seq,
	mcu_start_connect_cmd_seq, mcu_update_connect_cmd_seq,
	mcu_check_stat_cmd_seq};
#if (APP_SERIAL_DEBUG_EN)
static const char *cmd_seq_dbg[] = {"<none>", "init", "measure", "connect", "update", "checkstat"};
#endif

_attribute_optimize_size_ void app_serial_cmd_seq_start(u8 cmd_seq, u32 delay)
{
	if (cmd_seq>=sizeof(cmd_seq_def)/sizeof(cmd_seq_def[0]))   return;
	DEBUGFMT(APP_SERIAL_DEBUG_EN, "[MCU] Serial Start CmdSeq %u=%s delay %u", cmd_seq, cmd_seq_dbg[cmd_seq], delay);
	next_cmd_seq=cmd_seq_def[cmd_seq];
	mcu_cmd_seq_start_clock=clock_time();
	mcu_cmd_seq_start_delay=delay;
	if (next_cmd_seq && !mcu_uart_initialized)		mcu_init_serial(1);
}

_attribute_optimize_size_ u8 app_serial_cmd_seq_stat(void)
{
	u8 stat=0;
	if (mcu_cmd_seq_active())	stat|=1;
	if (next_cmd_seq != 0)		stat|=2;
	return stat;
}

#endif // #if (APP_MCU_SERIAL)
