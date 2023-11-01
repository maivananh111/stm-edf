
#include "system/startup.h"

#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "math.h"



rcc_config_t rcc = {
#if CONFIG_CLK_SOURCE_HSI
	.sysclk_source = HSI_CRYSTAL,
#elif CONFIG_CLK_SOURCE_HSE
	.sysclk_source = HSE_CRYSTAL,
#else
	.sysclk_source = PLL_CLOCK,
#endif


#if CONFIG_CLK_PLL_SOURCE_HSI
	.pll_source = PLL_SOURCE_HSI,
#else
	.pll_source = PLL_SOURCE_HSE,
#endif

	.hclk_freq_mhz = CONFIG_CLK_HCLKF,
	.apb1_freq_mhz = CONFIG_CLK_PCLK1F,
	.apb2_freq_mhz = CONFIG_CLK_PCLK2F,
	.sdio_usb_freq_mhz = CONFIG_CLK_SDIOF_USBF,
};

#if CONFIG_MEM_SUPPORT_SDRAM
sdram_config_t sdram_conf = {};
#endif /* CONFIG_USE_SDRAM && ENABLE_FMC */


#if CONFIG_USE_LOG_MONITOR
static const char* TAG = (const char *)"STARTUP";
#endif /* CONFIG_USE_LOG_MONITOR */

static void app_main_task(void *);


#if defined(USE_HAL_DRIVER)
int edf_main_application(void){
#else
int main(void){
#endif /* USE_HAL_DRIVER */
	system_init();
	rcc_init(&rcc);

	dev_get_cpu_info(&cpu_info);
#if CONFIG_WATCHDOG_EN
	iwdg_config_t iwdg_conf = iwdg_cal_param(CONFIG_WATCHDOG_WAIT_RESET_TIME);
	iwdg_init(&iwdg_conf);
	iwdg_disable_in_debugmode();
#endif /* CONFIG_WATCHDOG_EN */

#if CONFIG_MEM_SUPPORT_SDRAM
	fmc_sdram_init(&sdram_conf);
#endif /* CONFIG_MEM_SUPPORT_SDRAM */

#if	CONFIG_PERIPH_TRNG_EN
	trng_init();
#endif /* CONFIG_PERIPH_RNG_EN */

#if CONFIG_USE_LOG_MONITOR
	logstream_init();
	LOG_INFO(TAG, "SDK version   	 : %s",     CONFIG_EDF_VERSION);
	LOG_INFO(TAG, "Device name   	 : %s",     cpu_info.device_name);
	LOG_INFO(TAG, "Device CPU    	 : %s",     CONFIG_CPU_CORE);
	LOG_INFO(TAG, "Device ID     	 : 0x%04x", cpu_info.device_id);
	LOG_INFO(TAG, "Revision ID   	 : 0x%04x", cpu_info.revision_id);
	LOG_INFO(TAG, "Unique ID     	 : 0x%08x%08x%08x", cpu_info.unique_id[2], cpu_info.unique_id[1], cpu_info.unique_id[0]);
	LOG_INFO(TAG, "Flash capacity	 : %dKbytes",  cpu_info.flashsize_Kb);
	LOG_INFO(TAG, "Ram capacity  	 : %dKbytes",  CONFIG_MEM_IRAM_CAPACITY);
	LOG_INFO(TAG, "AHB frequency 	 : %luHz",     rcc_get_bus_frequency(BUS_HCLK));
	LOG_INFO(TAG, "APB1 frequency	 : %luHz",     rcc_get_bus_frequency(BUS_APB1));
	LOG_INFO(TAG, "APB2 frequency    : %luHz",     rcc_get_bus_frequency(BUS_APB2));
	LOG_INFO(TAG, "SDIO-USB frequency: %luHz", rcc_get_bus_frequency(BUS_SDIO_USB));

#endif
	BaseType_t app_start_status = xTaskCreate(app_main_task, "app_main_task",
			kilobytes_to_words(CONFIG_RTOS_APP_MAIN_TASK_STACK_SIZE), NULL, CONFIG_RTOS_APP_MAIN_TASK_PRIORITY, NULL);
	if(app_start_status != pdTRUE) {
#if CONFIG_USE_LOG_MONITOR
		LOG_ERROR(TAG, "Error when start main application at %s -> %s Line: %d", __FILE__, __FUNCTION__, __LINE__);
#endif /* CONFIG_USE_LOG_MONITOR */
		return 0;
	}
#if CONFIG_USE_LOG_MONITOR
	LOG_INFO(TAG, "Starting scheduler on CPU.");
#endif /* CONFIG_USE_LOG_MONITOR */
	vTaskStartScheduler();

	return (int)app_start_status;
}


static void app_main_task(void *param){
#if CONFIG_USE_LOG_MONITOR
	LOG_INFO(TAG, "Calling app_main().");
#endif /* CONFIG_USE_LOG_MONITOR */
	extern void app_main(void);
	app_main();
#if CONFIG_USE_LOG_MONITOR
	LOG_INFO(TAG, "Returned from app_main().");
#endif /* CONFIG_USE_LOG_MONITOR */
	vTaskDelete(NULL);
}













