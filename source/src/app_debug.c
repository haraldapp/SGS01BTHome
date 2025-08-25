/********************************************************************************************************
 * @file    app_debug.c
 *
 * @brief   Debug helpers (using Telink SDK)
 *
 * @author  haraldapp
 * @date    06,2024
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

#if (APP_DEBUG_ENABLE)
#define UART_PRINT_DEBUG_ENABLE 1
#define PRINT_BAUD_RATE 1000000
#endif

#ifdef VENDOR_COMMON_TLKAPI_DEBUG_H_
#error "tlkapi_debug.h included before?"
#endif

// we use files from the BLE SDK vendor section "inline"
#include "application/print/putchar.c"
#include "application/print/u_printf.c"
#include "vendor/common/tlkapi_debug.h"
#include "vendor/common/tlkapi_debug.c"
#include "stack/ble/debug/debug.h"

// debug init
void app_debug_init(void)
{
	// gpio_set_func(DEBUG_SWS_PIN, AS_SWIRE);
    #if (APP_DEBUG_ENABLE)
	gpio_set_func(DEBUG_INFO_TX_PIN, AS_GPIO);
	gpio_write(DEBUG_INFO_TX_PIN, 1);
	gpio_set_output_en(DEBUG_INFO_TX_PIN, 1);
	tlkapi_debug_init();
    blc_debug_enableStackLog(STK_LOG_DISABLE);
    #endif
}

// debug helpers
#if (APP_DEBUG_ENABLE)
void DEBUGOUTHEX(u8 u)
{
	u8 h=(u >> 4);
    if (h>=10)  DEBUGOUT(h-10+'A'); else DEBUGOUT(h+'0');
	h=(u & 0x0F);
    if (h>=10)  DEBUGOUT(h-10+'A'); else DEBUGOUT(h+'0');
}

void DEBUGOUTSTR(const char *txt)
{
	while (*txt)  { DEBUGOUT(*txt); txt++; }
}

void DEBUGOUTINT(int val, int digits)
{
	char buf[20]; u8 ofs=sizeof(buf);
	if (digits<1) { digits=1; }
	if (val<0) { DEBUGOUT('-'); val=-val; }
	while (1)
	{
		if (digits<=0 && val==0)   break;
		buf[--ofs]=(char)((val%10)+'0');
		val/=10; digits--;
	}
	while (ofs<sizeof(buf))   DEBUGOUT(buf[ofs++]);
}

#endif








