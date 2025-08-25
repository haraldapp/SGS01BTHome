/********************************************************************************************************
 * @file    main.c
 *
 * @brief   Main entry point
 *
 * @author  haraldapp
 * @date    12,2023
 *
 * @par     Copyright (c) 2023, haraldapp, https://github.com/haraldapp
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
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "app.h"

#ifndef MODULE_WATCHDOG_ENABLE
#define MODULE_WATCHDOG_ENABLE 0
#endif

// The main entry points have to be in ram code.
// To keep ram code small, init and loop are
// implemented in app.c by app_init_xx() and app_main_loop()

// IRQ handler
_attribute_ram_code_ void irq_handler(void)
{
	// SDK IRQ handler
	irq_blt_sdk_handler();
}

// Main function
_attribute_ram_code_ int main(void)  // must: ramcode
{
    // use internal 32k oscillator
	blc_pm_select_internal_32k_crystal();
    // CPU init
	#if (MCU_CORE_TYPE == MCU_CORE_825x)
		cpu_wakeup_init();
	#elif (MCU_CORE_TYPE == MCU_CORE_827x)
		cpu_wakeup_init(LDO_MODE, EXTERNAL_XTAL_24M);
	#endif
    // get startup type
	int deepRetentionWakeUp = pm_is_MCU_deepRetentionWakeup();
    // init SDK ble driver
	rf_drv_ble_init();
	// init gpio (analog resistance setup will keep in deepSleep mode)
	gpio_init(deepRetentionWakeUp!=0);
    // init system clock
	clock_init(SYS_CLK_TYPE);
    // init app
	if(deepRetentionWakeUp)
		app_init_deepRetn();
	else
		app_init_normal();
	// watch dog
	#if (MODULE_WATCHDOG_ENABLE)
	reg_tmr_ctrl = MASK_VAL(
		FLD_TMR_WD_CAPT, (MODULE_WATCHDOG_ENABLE ? (WATCHDOG_INIT_TIMEOUT * CLOCK_SYS_CLOCK_1MS >> 18):0),
		FLD_TMR_WD_EN, (MODULE_WATCHDOG_ENABLE?1:0));
	#endif
	// enable IRQ
    irq_enable();
	while (1) {
		#if (MODULE_WATCHDOG_ENABLE)
		wd_clear(); // clear watch dog
		#endif
		app_main_loop();
	}
}

