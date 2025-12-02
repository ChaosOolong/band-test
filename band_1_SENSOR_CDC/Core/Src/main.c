/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc.h"
#include "i2c.h"
#include "ipcc.h"
#include "rf.h"
#include "rtc.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <bno055.h>
#include "usbd_cdc_if.h"
#include <stdint.h>
#include "stm32wbxx_hal.h"
#include "W25Q.h"
//#include "fat16_image.h"
//#include "stm32_seq.h"
extern volatile bool cpu2_ready;
extern USBD_HandleTypeDef hUsbDeviceFS;
extern volatile uint8_t g_cdc_ready;
extern uint32_t HAL_GetTick(void);




extern const uint8_t tiny_volume[];
extern const uint32_t tiny_volume_size;
//volatile uint8_t g_cdc_ready = 0;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


///SENSOR TEST

struct bno055_t bno;

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	return HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg_addr,
                             I2C_MEMADD_SIZE_8BIT, reg_data, cnt, 100) == HAL_OK ? 0 : -1;
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    return HAL_I2C_Mem_Read(&hi2c1, dev_addr <<1 , reg_addr,
                            I2C_MEMADD_SIZE_8BIT, reg_data, cnt, 100) == HAL_OK ? 0 : -1;
}

void BNO055_delay_msec(u32 msec)
{
    HAL_Delay(msec);
}
typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
    float yaw, roll, pitch;
    uint32_t t_ms;
} SensorFrame;

SensorFrame read_sensor(void)
{
    SensorFrame f = {0};

    struct bno055_euler_t eul;
    struct bno055_accel_t acc;
    struct bno055_gyro_t gyr;

    bno055_read_euler_hrp(&eul);
    bno055_read_accel_xyz(&acc);
    bno055_read_gyro_xyz(&gyr);

    f.t_ms = HAL_GetTick();

    // unit conversion (根据 Bosch 手册)
    f.yaw   = (float)eul.h / 16.0f;   // raw 单位 1/16°
    f.roll  = (float)eul.r / 16.0f;
    f.pitch = (float)eul.p / 16.0f;

    f.ax = (float)acc.x / 100.0f;     // mg → m/s²
    f.ay = (float)acc.y / 100.0f;
    f.az = (float)acc.z / 100.0f;

    f.gx = (float)gyr.x / 16.0f;      // raw 单位 1/16 dps
    f.gy = (float)gyr.y / 16.0f;
    f.gz = (float)gyr.z / 16.0f;

    return f;
}


extern uint32_t HAL_GetTick(void);



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

static int usb_send_buf(const uint8_t* buf, uint16_t len) {
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || !g_cdc_ready) {
        return -1; // USB not configured or CDC not ready
    }

    uint32_t t0 = HAL_GetTick();
    while (CDC_Transmit_FS((uint8_t*)buf, len) == USBD_BUSY) {
			    //MX_APPE_Process();                   
        if (HAL_GetTick() - t0 > 20) { // Timeout after 20ms
            return -2; // Timeout error
        }
    }
    return 0;
}



static int usb_print(const char* s) {
    return usb_send_buf((const uint8_t*)s, (uint16_t)strlen(s));
}

static int usb_printf(const char *fmt, ...) {
    static char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) return n;
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;  
    return usb_send_buf((uint8_t*)buf, (uint16_t)n);
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void write_data_to_flash(uint32_t addr, const uint8_t *data, uint32_t size) {
  if ((addr % 4096) == 0) {
    W25Q01_SectorErase4K(addr);  // 每 4KB 擦除一次
  }
  W25Q01_PageProgram(addr, data, size);  // 写入数据
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	

	//crystal test
/*	bool hse_ok = __HAL_RCC_GET_FLAG(RCC_FLAG_HSERDY);
	bool lse_ok = __HAL_RCC_GET_FLAG(RCC_FLAG_LSERDY);
	if (hse_ok) {
			usb_print("hse ok\r\n");
	}
	if (lse_ok) {
			usb_print("lse ok\r\n");
	}
	*/
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */


  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */




  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_RTC_Init();

  MX_RF_Init();
	
	
  MX_APPE_Config();
	LL_HSEM_1StepLock( HSEM, 5 );
  MX_APPE_Init();
	
	MX_USB_Device_Init();
	
	HAL_Delay(7000);  // 给 BNO055 上电时间
	usb_print("Init BNO055...1\r\n");
	HAL_Delay(7000);  // 给 BNO055 上电时间
	usb_print("Init BNO055...2\r\n");
// 初始化 I2C 接口
	bno.bus_read = BNO055_I2C_bus_read;
	bno.bus_write = BNO055_I2C_bus_write;
	bno.delay_msec = BNO055_delay_msec;
	bno.dev_addr = BNO055_I2C_ADDR1;// 0x29 or 1

	s8 rslt = bno055_init(&bno);
	usb_printf("init result = %d\r\n", rslt);

	rslt = bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
	usb_printf("set mode = %d\r\n", rslt);
	HAL_Delay(500);
	
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */

	usb_printf("ptr_read=%p write=%p delay=%p\n",
           bno.bus_read, bno.bus_write, bno.delay_msec);
uint8_t id = 0;

HAL_I2C_Mem_Read(&hi2c1, 0x28<<1, 0x00, 1, &id, 1, 100);
usb_printf("ID_28 = 0x%02X\r\n", id);

HAL_I2C_Mem_Read(&hi2c1, 0x29<<1, 0x00, 1, &id, 1, 100);
usb_printf("ID_29 = 0x%02X\r\n", id);

	HAL_Delay(5000);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */
  //  MX_APPE_Process();

    /* USER CODE BEGIN 3 */
		
		//SENSOR TEST
    SensorFrame f = read_sensor();

	//	usb_printf("%.2f\n",ADC1_IN15);
		
    usb_printf("%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
        f.t_ms, f.ax, f.ay, f.az, f.gx, f.gy, f.gz, f.yaw, f.pitch);
        HAL_Delay(200);  // Delay for 50Hz 
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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the SYSCFG APB clock
  */
  __HAL_RCC_CRS_CLK_ENABLE();

  /** Configures CRS
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_LSE;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,32768);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_LSE;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
