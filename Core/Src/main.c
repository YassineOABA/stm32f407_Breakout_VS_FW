/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "bmi160.h"
#include "bmi160_wrappers.h"
#include "bme280.h"
#include "bme280_wrappers.h"
#include "w25q128jv.h"
#include "lfs.h"
#include "lfs_w25q128.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct bmi160_sensor_data_real
{
  //Acc
  float ax ;
  float ay ;
  float az ;
  //Gyro
  float gx ;
  float gy ;
  float gz ;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LOG_INTERVAL  UINT16_C(25)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim11;

/* Definitions for usbTask */
osThreadId_t usbTaskHandle;
const osThreadAttr_t usbTask_attributes = {
  .name = "usbTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for bmi160Task */
osThreadId_t bmi160TaskHandle;
const osThreadAttr_t bmi160Task_attributes = {
  .name = "bmi160Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for bme280Task */
osThreadId_t bme280TaskHandle;
const osThreadAttr_t bme280Task_attributes = {
  .name = "bme280Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for dataLoggerTask */
osThreadId_t dataLoggerTaskHandle;
const osThreadAttr_t dataLoggerTask_attributes = {
  .name = "dataLoggerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for monitorTask */
osThreadId_t monitorTaskHandle;
const osThreadAttr_t monitorTask_attributes = {
  .name = "monitorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
/*! Timer for freertos task profiling */
volatile unsigned long ulHighFrequencyTimerTicks = 0;

uint8_t u8LedActivity = 0;

struct bmi160_dev bmi160;
struct bmi160_sensor_data_real accel_gyro;

struct bme280_dev bme280;
struct bme280_settings settings;
struct bme280_data comp_data;

struct w25q128jv_dev w25q128jv;

// variables used by the filesystem
extern const struct lfs_config cfg;
lfs_t lfs;

uint8_t closeFilesRequest = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM8_Init(void);
static void MX_RTC_Init(void);
void StartUsbTask(void *argument);
void StartBmi160Task(void *argument);
void StartBme280Task(void *argument);
void StartLedTask(void *argument);
void StartDataLoggerTask(void *argument);
void StartMonitorTaskTask(void *argument);

/* USER CODE BEGIN PFP */
static uint8_t bme280_get_data(uint32_t period, struct bme280_dev *dev);
static uint8_t bmi160_get_data(uint8_t select_sensor, struct bmi160_sensor_data_real *accel_gyro, const struct bmi160_dev *dev);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int _write(int file, char *ptr, int len)
{
  CDC_Transmit_FS((uint8_t *)ptr, len);  // Send data via USB CDC
  return len;
}

/*!
 *  @brief This internal API is used to get compensated humidity data.
 */
static uint8_t bme280_get_data(uint32_t period, struct bme280_dev *dev)
{
    uint8_t rslt = BME280_E_NULL_PTR;
    uint8_t status_reg;


    rslt = bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
    if (status_reg & BME280_STATUS_MEAS_DONE)
    {
        /* Measurement time delay given to read sample */
        dev->delay_us(period);

        /* Read compensated data */
        rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);

#ifndef BME280_DOUBLE_ENABLE
        comp_data.humidity = comp_data.humidity / 1000;
        comp_data.temperature = comp_data.temperature / 100;
#endif
    }

    return rslt;
}

static uint8_t bmi160_get_data(uint8_t select_sensor, struct bmi160_sensor_data_real *accel_gyro, const struct bmi160_dev *dev)
{
  uint8_t rslt = BMI160_E_NULL_PTR;
  struct bmi160_sensor_data accel = {0};
  struct bmi160_sensor_data gyro = {0};

  rslt = bmi160_get_sensor_data(select_sensor, &accel, &gyro, dev);
  if(rslt == BMI160_OK)
  {
    accel_gyro->ax = (float)(accel.x * 9.8) / 16384;
    accel_gyro->ay = (float)(accel.y * 9.8) / 16384;
    accel_gyro->az = (float)(accel.z * 9.8) / 16384;
    accel_gyro->gx = (float)(gyro.x * 2000)/0x8000;
    accel_gyro->gy = (float)(gyro.y * 2000)/0x8000;
    accel_gyro->gz = (float)(gyro.z * 2000)/0x8000;
  }
  
  return rslt;
}

void HAL_Delay_us(uint32_t us)
{
    uint32_t tickstart = __HAL_TIM_GET_COUNTER(&htim8);  // Get current counter value
    uint32_t wait = us;
   
    while((__HAL_TIM_GET_COUNTER(&htim8) - tickstart) < wait)
    {
    }
}

static void lfs_static_test(void) {
  int err;
  uint8_t write_buffer[256];
  uint8_t read_buffer[256];

  // Step 1: Mount the filesystem
  err = lfs_mount(&lfs, &cfg);
  if (err) {
      // Step 2: Reformat if mounting fails
      printf("Filesystem mount failed, formatting...\n");
      lfs_format(&lfs, &cfg);
      err = lfs_mount(&lfs, &cfg);
      if (err) {
          printf("Failed to mount filesystem after formatting\n");
          return;
      }
  }
  printf("Filesystem mounted successfully\n");

  // Step 3: Prepare test data
  for (int i = 0; i < 256; i++) {
      write_buffer[i] = i;  // Filling with 0x00, 0x01, ..., 0xFF
  }

  // Step 4: Open the file for writing
  lfs_file_t file;
  err = lfs_file_open(&lfs, &file, "test", LFS_O_RDWR | LFS_O_CREAT);
  if (err < 0) {
      printf("Failed to open file for writing\n");
      return;
  }

  // Step 5: Write 256 bytes to the file
  err = lfs_file_write(&lfs, &file, write_buffer, 256);
  if (err < 0) {
      printf("Failed to write to file\n");
      lfs_file_close(&lfs, &file);
      return;
  }

  // Step 6: Close the file
  err = lfs_file_close(&lfs, &file);
  if (err < 0) {
      printf("Failed to close file\n");
      return;
  }

  printf("Data written successfully\n");

  // Step 7: Open the file for reading
  err = lfs_file_open(&lfs, &file, "test", LFS_O_RDONLY);
  if (err < 0) {
      printf("Failed to open file for reading\n");
      return;
  }

  // Step 8: Read 256 bytes from the file
  memset(read_buffer, 0, sizeof(read_buffer)); // Clear buffer
  err = lfs_file_read(&lfs, &file, read_buffer, 256);
  if (err < 0) {
      printf("Failed to read from file\n");
      lfs_file_close(&lfs, &file);
      return;
  }

  // Step 9: Compare read data with written data
  if (memcmp(write_buffer, read_buffer, 256) == 0) {
      printf("Data verification successful: Read matches written data\n");
  } else {
      printf("Data mismatch: Read data is different from written data!\n");
  }

  // Step 10: Close the file
  err = lfs_file_close(&lfs, &file);
  if (err < 0) {
      printf("Failed to close file after reading\n");
      return;
  }

  // Step 11: Delete (remove) the file
  err = lfs_remove(&lfs, "test");
  if (err < 0) {
      printf("Failed to delete file\n");
      return;
  }

  printf("File deleted successfully\n");

  // Step 12: Try to open the deleted file (should fail)
  err = lfs_file_open(&lfs, &file, "test", LFS_O_RDONLY);
  if (err < 0) {
      printf("File correctly does not exist after deletion\n");
  } else {
      printf("Error: File still exists after deletion!\n");
      lfs_file_close(&lfs, &file);
  }

  // Step 13: Unmount the filesystem
  err = lfs_unmount(&lfs);
  if (err < 0) {
      printf("Failed to unmount filesystem\n");
      return;
  }

  printf("Filesystem unmounted successfully\n");
}

void enter_standby_mode(void)
{
  // Stop all tasks (except Idle task)
  vTaskSuspendAll();

  // Disable Peripherals
  HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);
  HAL_SPI_DeInit(&hspi1);
  HAL_SPI_DeInit(&hspi3);
  HAL_I2C_DeInit(&hi2c1);

  // Clear any wakeup flag
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, (60 *5), RTC_WAKEUPCLOCK_CK_SPRE_16BITS);

  // Enter Standby mode
  HAL_PWR_EnterSTANDBYMode();
}


/**
  * @brief  Wakeup Timer callback.
  * @param  hrtc pointer to a RTC_HandleTypeDef structure that contains
  *                the configuration information for RTC.
  * @retval None
  */
 void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
 {
   /* Prevent unused argument(s) compilation warning */
   UNUSED(hrtc);

   __HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc, RTC_FLAG_WUTF);  // Clear RTC wakeup flag
    HAL_RTCEx_DeactivateWakeUpTimer(hrtc);  // Stop wakeup timer (optional)
   printf("System woke up from Standby or Stop mode.\n");
 }
 
 void read_and_send_file(const char *filename)
 {
  int err;
  char buffer[128];  // Buffer to hold the data read from the file
  int bytes_read;
  lfs_file_t file;

  // Mount the filesystem (if not already mounted)
  err = lfs_mount(&lfs, &cfg);
  if (err < 0) {
      printf("Error mounting filesystem\n");
      fflush(stdout);
      return;
  }

  // Open the file in read-only mode
  err = lfs_file_open(&lfs, &file, filename, LFS_O_RDONLY);
  if (err < 0) {
      printf("Error opening file %s\n", filename);
      fflush(stdout);
      return;
  }

  // Read the file contents
  while ((bytes_read = lfs_file_read(&lfs, &file, buffer, sizeof(buffer))) > 0) {
      // Print data efficiently
      fwrite(buffer, 1, bytes_read, stdout);
      fflush(stdout);  // Ensure immediate output
  }

  // Check if there was an error while reading
  if (bytes_read < 0) {
      printf("Error reading file\n");
      fflush(stdout);
  }

  // Close the file
  lfs_file_close(&lfs, &file);
  lfs_unmount(&lfs);
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI3_Init();
  MX_TIM11_Init();
  MX_TIM8_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();
  HAL_Delay(1000); // Wait 1 second before printing
  printf("System Started...\n");
  W25Q128JV_interface_Init(&w25q128jv, &hspi3, FLASH_NSS_GPIO_Port, FLASH_NSS_Pin, FLASH_WP_GPIO_Port, FLASH_WP_Pin, FLASH_HLD_GPIO_Port, FLASH_HLD_Pin);
  W25Q128JV_Init(&w25q128jv);
  W25Q128JV_ReadJedecID(&w25q128jv);
  W25Q128JV_ReadUniqueID(&w25q128jv);

  // Read the file and send its contents to the terminal
  printf("Downloading file bmi160_data\n");
  read_and_send_file("bmi160_data");
  printf("Finished Downloading file bmi160_data\n");
  // Read the file and send its contents to the terminal
  printf("Downloading file bmi280_data\n");
  read_and_send_file("bme280_data");
  printf("Finished Downloading file bmi280_data\n");
  //lfs_format(&lfs, &cfg);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of usbTask */
  usbTaskHandle = osThreadNew(StartUsbTask, NULL, &usbTask_attributes);

  /* creation of bmi160Task */
  bmi160TaskHandle = osThreadNew(StartBmi160Task, NULL, &bmi160Task_attributes);

  /* creation of bme280Task */
  bme280TaskHandle = osThreadNew(StartBme280Task, NULL, &bme280Task_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

  /* creation of dataLoggerTask */
  dataLoggerTaskHandle = osThreadNew(StartDataLoggerTask, NULL, &dataLoggerTask_attributes);

  /* creation of monitorTask */
  monitorTaskHandle = osThreadNew(StartMonitorTaskTask, NULL, &monitorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 600, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
  HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */
  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */
  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */
  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 83;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */
  HAL_TIM_Base_Start(&htim8);
  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 167;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 99;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FLASH_WP_Pin|FLASH_HLD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BMI160_NSS_Pin|FLASH_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FLASH_WP_Pin FLASH_HLD_Pin */
  GPIO_InitStruct.Pin = FLASH_WP_Pin|FLASH_HLD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BMI160_NSS_Pin FLASH_NSS_Pin */
  GPIO_InitStruct.Pin = BMI160_NSS_Pin|FLASH_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : BMI160_INT1_Pin BMI160_INT2_Pin */
  GPIO_InitStruct.Pin = BMI160_INT1_Pin|BMI160_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DEBUG_LED_Pin */
  GPIO_InitStruct.Pin = DEBUG_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DEBUG_LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartUsbTask */
/**
  * @brief  Function implementing the usbTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartUsbTask */
void StartUsbTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    //printf("Temp: %.2fdC, Pressure: %.2fhPa, Humidity: %.2f%%\n",
    //        (float)comp_data.temperature, (float)comp_data.pressure / 100.0, (float)comp_data.humidity);
    //printf("Accel (g): X=%.2f Y=%.2f Z=%.2f | Gyro (d/s): X=%.2f Y=%.2f Z=%.2f\n",
    //       accel_gyro.ax, accel_gyro.ay, accel_gyro.az, accel_gyro.gx, accel_gyro.gy, accel_gyro.gz);
    osDelay(500);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBmi160Task */
/**
* @brief Function implementing the bmi160Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBmi160Task */
void StartBmi160Task(void *argument)
{
  /* USER CODE BEGIN StartBmi160Task */
  uint8_t rslt = 0;

  UNUSED(argument);
  rslt =  bmi160_interface_init(&bmi160, &hspi1, BMI160_NSS_GPIO_Port, BMI160_NSS_Pin);
  rslt += bmi160_init(&bmi160);
  
  bmi160.accel_cfg.bw     = BMI160_ACCEL_BW_NORMAL_AVG4;
  bmi160.accel_cfg.odr    = BMI160_ACCEL_ODR_100HZ;
  bmi160.accel_cfg.range  = BMI160_ACCEL_RANGE_2G;
  bmi160.accel_cfg.power  = BMI160_ACCEL_NORMAL_MODE;

  bmi160.gyro_cfg.bw      = BMI160_GYRO_BW_NORMAL_MODE;
  bmi160.gyro_cfg.odr     = BMI160_GYRO_ODR_100HZ;
  bmi160.gyro_cfg.range   = BMI160_GYRO_RANGE_2000_DPS;
  bmi160.gyro_cfg.power   = BMI160_GYRO_NORMAL_MODE;

  rslt += bmi160_set_sens_conf(&bmi160);
  rslt += bmi160_get_sens_conf(&bmi160);
  rslt += bmi160_get_power_mode(&bmi160);
  if (rslt)
  {
    printf("BMI160 initialization failed with error code %i\n", rslt);
  }
  else
  {
    printf("BMI160 initialization succeeded\n");
  }
  
  
  /* Infinite loop */
  for(;;)
  {
    if(rslt == BMI160_OK)
    {
      bmi160_get_data(BMI160_BOTH_ACCEL_AND_GYRO|BMI160_TIME_SEL, &accel_gyro, &bmi160);
    }
    else
    {
      Error_Handler();
    }

    osDelay(1000);
  }
  /* USER CODE END StartBmi160Task */
}

/* USER CODE BEGIN Header_StartBme280Task */
/**
* @brief Function implementing the bme280Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBme280Task */
void StartBme280Task(void *argument)
{
  /* USER CODE BEGIN StartBme280Task */
  uint8_t rslt = 0;
  uint32_t period;

  UNUSED(argument);
  bme280.dma_reading_trig = 0;
  bme280.dma_reading_req = 0;
  rslt = bme280_interface_init(&bme280, &hi2c1);
  rslt +=bme280_init(&bme280);
  rslt += bme280_get_sensor_settings(&settings, &bme280);
  /* Configuring the over-sampling rate, filter coefficient and standby time */
  /* Overwrite the desired settings */
  settings.filter = BME280_FILTER_COEFF_8;

  /* Over-sampling rate for humidity, temperature and pressure */
  settings.osr_h = BME280_OVERSAMPLING_8X;
  settings.osr_p = BME280_OVERSAMPLING_8X;
  settings.osr_t = BME280_OVERSAMPLING_8X;
  
  /* Setting the standby time */
  settings.standby_time = BME280_STANDBY_TIME_500_MS;

  rslt += bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, &bme280);
  
  /* Always set the power mode after setting the configuration */
  rslt += bme280_set_sensor_mode(BME280_POWERMODE_NORMAL, &bme280);
  rslt += bme280_cal_meas_delay(&period, &settings);
  if (rslt)
  {
    printf("BME280 initialization failed with error code %i\n", rslt);
  }
  else
  {
    printf("BME280 initialization succeeded\n");
  }
  
  /* Infinite loop */
  for(;;)
  {
    if(rslt == BMI160_OK)
    {
      bme280_get_data(period, &bme280);
    }
    else
    {
      Error_Handler();
    }

    osDelay(1000);
  }
  /* USER CODE END StartBme280Task */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_SET);
    osDelay(100);
    HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, GPIO_PIN_RESET);
    
    osDelay(4900);
  }
  /* USER CODE END StartLedTask */
}

/* USER CODE BEGIN Header_StartDataLoggerTask */
/**
* @brief Function implementing the dataLoggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDataLoggerTask */
void StartDataLoggerTask(void *argument)
{
  /* USER CODE BEGIN StartDataLoggerTask */
  UNUSED(argument);
  lfs_file_t bme280_file;
  lfs_file_t bmi160_file;
  lfs_soff_t file_size;
  uint16_t log_count = 0;
  int err = 0;

  //Mount the filesystem
  err = lfs_mount(&lfs, &cfg);

  if (err) 
  {
    // Reformat if mounting fails
    printf("Filesystem mount failed, formatting...\n");
    lfs_format(&lfs, &cfg);
    err = lfs_mount(&lfs, &cfg);
    if (err)
    {
      printf("Failed to mount filesystem after formatting\n");
      Error_Handler();
    }
    else
    {
      printf("Filesystem mounted successfully\n");
    }
  }
  else
  {
    printf("Filesystem mounted successfully\n");
  }
 
  //Open the files for writing
  err = lfs_file_open(&lfs, &bme280_file, "bme280_data", LFS_O_RDWR | LFS_O_CREAT);
  if (err < 0)
  {
    printf("Failed to open bme280 file for writing\n");
    Error_Handler();
  }
  else
  {
    printf("Succeeded to open bme280 file for writing\n");
  }
  fflush(stdout); 
  // Get the file size using lfs_file_size()
  file_size = lfs_file_size(&lfs, &bme280_file);
  printf("bme280 file size:: %ld\n", file_size);
  err = lfs_file_open(&lfs, &bmi160_file, "bmi160_data", LFS_O_RDWR | LFS_O_CREAT);
  if (err < 0)
  {
    printf("Failed to open bmi160 file for writing\n");
    Error_Handler();
  }
  else
  {
    printf("Succeeded to open bmi160 file for writing\n");
  }
  // Get the file size using lfs_file_size()
  file_size = lfs_file_size(&lfs, &bmi160_file);
  printf("bmi160 file size: %ld\n", file_size);
  /* Infinite loop */
  for(;;)
  {
    if (closeFilesRequest == 0)
    {
      // Ensure the file pointer is at the end
      lfs_file_seek(&lfs, &bme280_file, 0, LFS_SEEK_END);
      lfs_file_seek(&lfs, &bmi160_file, 0, LFS_SEEK_END);

      // Write data
      lfs_file_write(&lfs, &bme280_file, (const void *)&comp_data, sizeof(comp_data));
      lfs_file_write(&lfs, &bmi160_file, (const void *)&accel_gyro, sizeof(accel_gyro));

      // Sync the file every LOG_INTERVAL
      if (log_count++ >= LOG_INTERVAL)
      {
        log_count = 0;
        err = 0;//lfs_file_sync(&lfs, &bme280_file);
        if (err < 0)
        {
          printf("Failed to sync bme280 file\n");
          Error_Handler();
        }
        else
        {
          printf("Succeeded to sync bme280 file\n");
          osDelay(5);
        }

        err = 0;//lfs_file_sync(&lfs, &bmi160_file);
        if (err < 0)
        {
          printf("Failed to sync bmi160 file\n");
          Error_Handler();
        }
        else
        {
          printf("Succeeded to sync bmi160 file\n");
          osDelay(5);
        }
      }
    }
    else if (closeFilesRequest == 1)
    {
      closeFilesRequest = 2;
      printf("Closing bme280 file \n");
      err = lfs_file_close(&lfs, &bme280_file);
      if (err < 0) 
      {
        printf("Failed to close bme280 file\n");
      }
      else
      {
        printf("Succeded to close bme280 file\n");
      }
      printf("Closing bmi160 file \n");
      err = lfs_file_close(&lfs, &bmi160_file);
      if (err < 0) 
      {
        printf("Failed to close bmi160 file\n");
      }
      else
      {
        printf("Succeded to close bmi160 file\n");
      }
    }
    else if (closeFilesRequest == 2)
    {
      closeFilesRequest = 3;
      printf("Unmounting file system\n");
        // Step 13: Unmount the filesystem
      err = lfs_unmount(&lfs);
      if (err < 0) 
      {
        printf("Failed to unmount filesystem\n");
      }
      else
      {
        printf("Filesystem unmounted successfully\n");
      }
    }
    else
    {
      /* code */
    }
    
    osDelay(1000);
  }
  /* USER CODE END StartDataLoggerTask */
}

/* USER CODE BEGIN Header_StartMonitorTaskTask */
/**
* @brief Function implementing the monitorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMonitorTaskTask */
void StartMonitorTaskTask(void *argument)
{
  /* USER CODE BEGIN StartMonitorTaskTask */
  uint32_t runnigTime = 0;
  /* Infinite loop */
  for(;;)
  {
    // Check if system is running for 30 seconds
    if(runnigTime == 30)
    {
      if(0 == closeFilesRequest)
      {
        printf("Request data logger to close open files...\n");
        closeFilesRequest = 1;
      }
      else if (3 == closeFilesRequest)
      {
        printf("System going into standby mode...\n");
        enter_standby_mode();
      }
      else
      {

      }
      
    }
    else
    {
      runnigTime++;
    }
    osDelay(1000);
  }
  /* USER CODE END StartMonitorTaskTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM11)
  {
    ulHighFrequencyTimerTicks++;
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  while (1)
  {

  }
  
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
