/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_config.h"
#include "app_console_cmd.h"
#include "app_cpld_spi.h"
#include "app_dac8571.h"
#include "app_gps_uart.h"
#include "app_gps_rmc.h"
#include "app_gpsdo.h"
#include "app_gpsdo10.h"
#include "app_pps.h"
#include "app_timebase.h"
#include "app_tmp117.h"
#include "app_uart_debug.h"
#include "app_vct_adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */
/* 0xFF is a debugger-only "not yet sampled" sentinel; the explicit cast keeps
 * armcc from flagging an enumerated type mixed with another type. */
volatile HAL_StatusTypeDef g_rcc_osc_status = (HAL_StatusTypeDef)0xFF;
volatile HAL_StatusTypeDef g_rcc_clk_status = (HAL_StatusTypeDef)0xFF;

volatile uint32_t g_rcc_cr_after_osc = 0;
volatile uint32_t g_rcc_cfgr_after_osc = 0;
volatile uint32_t g_rcc_cr_after_clk = 0;
volatile uint32_t g_rcc_cfgr_after_clk = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  HAL_StatusTypeDef app_status;
  uint32_t sig;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  app_clk_measure_init();
#if APP_UART1_BAUD_SWEEP_ENABLE
  /* Must run before app_log_init(): the log ring takes over the same port. */
  app_uart_baud_sweep();
#endif
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  app_log_init();
  app_log_write("\r\nF103_VCTCXO boot\r\n");
  app_log_printf("CLK HSE=%lu SYSCLK=%lu HCLK=%lu PCLK1=%lu PCLK2=%lu HSE_ON=1\r\n",
                 (unsigned long)APP_HSE_VALUE_HZ,
                 (unsigned long)HAL_RCC_GetSysClockFreq(),
                 (unsigned long)HAL_RCC_GetHCLKFreq(),
                 (unsigned long)HAL_RCC_GetPCLK1Freq(),
                 (unsigned long)HAL_RCC_GetPCLK2Freq());
  app_log_printf("SPI1 MODE: CPOL=%lu CPHA=%s %s PRESCALER=%lu\r\n",
                 (hspi1.Init.CLKPolarity == SPI_POLARITY_LOW) ? 0UL : 1UL,
                 (hspi1.Init.CLKPhase == SPI_PHASE_1EDGE) ? "1EDGE" : "2EDGE",
                 (hspi1.Init.FirstBit == SPI_FIRSTBIT_MSB) ? "MSB" : "LSB",
                 (hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_2) ? 2UL :
                 (hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_4) ? 4UL :
                 (hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_8) ? 8UL :
                 (hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_16) ? 16UL :
                 (hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_32) ? 32UL :
                 (hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_64) ? 64UL :
                 (hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_128) ? 128UL :
                 (hspi1.Init.BaudRatePrescaler == SPI_BAUDRATEPRESCALER_256) ? 256UL : 0UL);

  app_log_write("MODULE_PORT: PA0=PPS, ADC1/I2C2/TMP117 disabled\r\n");
  app_log_printf("UART1_BAUD=%lu (PA9/PA10). CLK_CHECK: PPS_MS and SEQ10_MS in GPSDO_RESULT must read ~1000 and ~10000; if both are off by the same factor then HSE is not APP_HSE_VALUE_HZ and the true baud is scaled by that factor.\r\n",
                 (unsigned long)APP_UART1_BAUD);

  app_dac8571_init();
  app_log_write("DAC8571 VREF=3.3600\r\n");
  app_log_printf("DAC8571 ADDR7=0x%02lX HAL_ADDR=0x%02lX INIT_CODE=%lu BROADCAST_TEST_ADDR7=0x%02lX\r\n",
                 (unsigned long)DAC8571_ADDR_7BIT,
                 (unsigned long)(DAC8571_ADDR_7BIT << 1),
                 (unsigned long)DAC_CODE_INIT,
                 (unsigned long)DAC8571_BROADCAST_WRITE_TEST_ADDR_7BIT);
#if APP_DAC_WRITE_INIT_ON_BOOT
  app_status = app_dac8571_set_initial_near_lock();
  app_log_printf("DAC_INIT_WRITE status=%lu code=%lu\r\n",
                 (unsigned long)app_status,
                 (unsigned long)app_dac8571_get_last_code());
  if (app_status == HAL_OK) {
    app_log_dac_adjust_event("INIT", 0U, app_dac8571_get_last_code(),
                             0UL, app_log_dac_code_to_uv(app_dac8571_get_last_code()), "INIT_WRITE");
  } else {
    app_log_dac_adjust_fail_event("INIT", 0U, DAC_CODE_INIT, "INIT_WRITE", app_status);
  }
#else
  app_log_write("DAC_INIT_WRITE disabled; no boot DAC write\r\n");
#endif

  app_cpld_spi_init();
  app_log_printf("CPLD_SIG_DUMMY=0x%08lX\r\n",
                 (unsigned long)app_cpld_get_dummy_signature());
  app_status = app_cpld_read_signature(&sig);
  if (app_status == HAL_OK) {
    app_log_printf("CPLD_SIG=0x%08lX EXPECT=0x%08lX\r\n",
                   (unsigned long)sig,
                   (unsigned long)CPLD_SIGNATURE_VALUE);
  } else {
    app_log_printf("CPLD_SIG_READ_FAIL status=%lu\r\n", (unsigned long)app_status);
  }

#if GPSDO_SPI_RAW_DIAG_ENABLE
  /* One-time SPI_RAW diagnosis 鈥?reads 12 addresses with full TX/RX hex dump */
  {
    static const uint8_t diag_addrs[] = {
        0x00U, 0x01U, 0x02U, 0x05U, 0x12U, 0x14U,
        0x16U, 0x17U, 0x7AU, 0x7BU, 0x7CU, 0x7DU
    };
    uint8_t diag_i;
    for (diag_i = 0U; diag_i < sizeof(diag_addrs); diag_i++) {
        app_cpld_spi_diag_raw_read(diag_addrs[diag_i]);
    }
  }
#endif

  app_pps_init();

  app_rmc_init();
  app_status = app_gps_uart_init();
  app_log_printf("GPS_UART_DMA status=%lu baud=%lu\r\n",
                 (unsigned long)app_status,
                 (unsigned long)APP_GPS_UART_BAUD);
  app_log_printf("GPS_UART=%lu (PA3=RX from GPS TX, PA2=TX to GPS RX)\r\n",
                 (unsigned long)APP_GPS_UART_BAUD);
  app_log_printf("GPS_WIRING:NMEA=GPS-TX->PA3,PPS=GPS-TIMEPULSE->PA0 AND CPLD-U1-pin49,GND common\r\n");

  app_gpsdo_init();
  app_gpsdo10_init();
  app_console_init();
  app_gpsdo_poll_1s();

  /* One-time GPS/PPS/RMC status diag */
  {
    const app_rtc_state_t *rtc = app_rtc_get_state();
    app_pps_snapshot_t pps;
    uint32_t ms = HAL_GetTick();
    uint32_t pps_age_ms;
    uint8_t pps_ok;
    uint8_t gps_ok;
    app_pps_get_snapshot(&pps);
    pps_age_ms = ms - pps.last_tick_ms;
    pps_ok = (pps_age_ms <= 2000UL) ? 1U : 0U;
    gps_ok = ((rtc->rtc_valid != 0U) && (rtc->rtc_synced != 0U) && (rtc->rmc_valid != 0U)) ? 1U : 0U;
    app_log_printf("GPS_DIAG: gps_valid=%lu pps_valid=%lu rtc_valid=%lu rtc_synced=%lu rmc_valid=%lu pps_age_ms=%lu rmc_age_s=%lu\r\n",
                   (unsigned long)gps_ok,
                   (unsigned long)pps_ok,
                   (unsigned long)rtc->rtc_valid,
                   (unsigned long)rtc->rtc_synced,
                   (unsigned long)rtc->rmc_valid,
                   (unsigned long)pps_age_ms,
                   (unsigned long)rtc->sec_since_sync);
  }

  app_status = app_timebase_init();
  app_log_printf("TIM3 timebase status=%lu irq_hz=%lu irq_ms=%lu tick20_div=%lu\r\n",
                 (unsigned long)app_status,
                 (unsigned long)APP_TIM3_IRQ_HZ,
                 (unsigned long)APP_TIM3_IRQ_MS,
                 (unsigned long)APP_TICK20_TIM3_DIV);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    app_console_poll();

    /* Re-run the baud sweep periodically while APP_UART1_BAUD_SWEEP_REPEAT_ENABLE
     * is set, so the diagnostic cannot be missed by not resetting the board. */
    app_uart_baud_sweep_tick();

    /* Re-run the PA3/PA0 electrical probe while the GPS is still silent, so a
     * terminal that connects after boot still captures the verdict.  It stops
     * by itself once either rx_bytes or the PPS count becomes non-zero. */
    app_gps_probe_tick();

    if (app_timebase_take_20ms_flag() != 0U) {
      app_gps_uart_poll();
      app_rmc_process_pending();
    }

    if (app_timebase_take_1s_flag() != 0U) {
      app_gps_uart_poll();
      app_rmc_process_pending();
      app_gpsdo_poll_1s();
      app_gpsdo10_poll_1s();
      app_rtc_tick_second();
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
