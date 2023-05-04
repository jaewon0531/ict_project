/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "fatfs.h"
#include "fdcan.h"
#include "i2c.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "icm20948.h"
#include "Adafruit_ADXL375.h"
#include "cQueue.h"
#include "gps.h"
#include "math.h"

//#include "DS1302.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define _DEBUG_
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//extern int16_t accel_data[3];
//extern int16_t gyro_data[3];

volatile uint8_t userKey1Flag = 0;
volatile uint8_t dataReadyFlag = 0;
volatile int dataReadyCount = 0;
volatile int magFlag = 0;

extern uint8_t gps_rx_data;
extern uint8_t gps_rx_buffer[GPSBUFSIZE];
extern uint8_t gps_rx_index;
extern uint8_t gps_cplt_flag;
extern GPS_t GPS;

extern flag = 0;


axises my_gyro;
axises my_accel;
axises my_mag;
int16_t mag_data[3];
volatile IMUDATA imuData ; //= {0,0,0,0,0,0,0,0,0,0,0};
IMUDATA* pIMUDATA;
volatile uint8_t startFlag = 0;
volatile uint8_t spi4Flag = 0;
volatile uint8_t imuCalibrationFlag = 0;

uint8_t save_Count = 0;
volatile QueueType* cQ;
uint32_t pre_time = 0;

volatile char bt_rx_data = 0;
volatile char bt_rx_buffer[20] = {0,};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
//With GCC, small printf (option LD Linker->Libraries->Small printf set to 'Yes') calls __io_putchar()
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

int makeNewFileName();

int detectAccident(IMUDATA * data);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart1){
    GPS_UART_CallBack();
  }
  if(huart == &huart5){

  }
}

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_4)
  {
    userKey1Flag = 1;
    HAL_UART_Transmit(&huart4, "in\r\n", 4, 1000);
  }
  if (GPIO_Pin == GPIO_PIN_15)
  {
    imuCalibrationFlag = 1;
    if (startFlag)
    {
      spi4Flag = 1;
      dataReadyFlag = 1;
    }
  }
}

int isSDcard(){
  return HAL_GPIO_ReadPin(SD_CD_GPIO_Port, SD_CD_Pin);
}


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
  char uart_buffer[100];
  cQ = createCQueue ();
  pIMUDATA = &imuData;
  IMUDATA* pImuForDetect;
  int imuDataSize = sizeof(imuData);
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SDMMC1_SD_Init();
  MX_UART4_Init();
  MX_SPI2_Init();
  MX_TIM7_Init();
  MX_FATFS_Init();
  MX_FDCAN1_Init();
  MX_USART1_UART_Init();
  MX_SPI4_Init();
  MX_UART5_Init();
  MX_I2C4_Init();
  /* USER CODE BEGIN 2 */

  printf ("start main(0)\r\n");

  HAL_Delay (10);
  icm20948_init ();
  printf ("icm20948 ok(0)\r\n");

  HAL_Delay (10);

  ak09916_init ();
  printf ("ak09916 ok(2)\r\n");
  HAL_Delay(1000);

  ADXL375_begin ();
  printf ("adxl375 ok(3)\r\n");

  HAL_GPIO_WritePin (LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
  GPS_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  char fileName[12];
  int i = 0;
  uint32_t bw, br;
  int sd_check_flag = 0;

  int fileNameIndex = 0;

  do
  {
    fileNameIndex = makeNewFileName ();
  }
  while (fileNameIndex < 0);

  sprintf (fileName, "data%04d.bin", fileNameIndex);
  HAL_UART_Transmit (&huart4, (uint8_t*) fileName, 12, 0xFFFF);

#ifdef _DEBUG_
  printf("file name index : %d\r\n", fileNameIndex);
#endif

  HAL_GPIO_WritePin (LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);

  while (1)
  {
    if (startFlag)
    {
      if (dataReadyFlag)
      {
        icm20948_accel_gyro_read (pIMUDATA);

        if( gps_cplt_flag ){
          gps_cplt_flag = 0;
          pIMUDATA->latitude = GPS.dec_latitude;
          pIMUDATA->longitude = GPS.dec_longitude;
        }

        if (magFlag >= 2)
        {
          magFlag = 0;
          ak09916_mag_read (pIMUDATA);
        }
        ADXL375_GETXYZ (pIMUDATA);
        pIMUDATA->time = HAL_GetTick ();
        int resultDA = detectAccident(pIMUDATA);
        uint8_t tx_bt = resultDA/10;
        if (resultDA > 20)
        {
          HAL_UART_Transmit(&huart5, &tx_bt, 1, 1000);
        }
        enCQueue (cQ, imuData);
        magFlag++;
        dataReadyFlag = 0;
      }

      int bufferSize = sizeOfCQueue (cQ);
      for (int i = 0; i < bufferSize; i++)
      {
        IMUDATA *writeData = deCQueue (cQ);
        f_write (&SDFile, (uint8_t*) writeData, imuDataSize, &bw);
        //HAL_UART_Transmit(&huart4, "s\r\n", 3, HAL_MAX_DELAY);
      }
    }

    if (userKey1Flag == 1)
    {
      if (startFlag == 1)
      {
        printf ("save end\r\n");
        f_close (&SDFile);
        fileNameIndex = makeNewFileName();
        sprintf (fileName, "data%04d.bin", fileNameIndex);
      }
      else
      {

        retSD = f_open (&SDFile, fileName, FA_OPEN_APPEND | FA_WRITE);

        if (retSD == FR_OK)
        {
          printf ("save start\r\n");
        }
        else
        {
          HAL_UART_Transmit (&huart4, fileName, 12, 0xFFFF);
          printf ("  Failed %d \r\n", retSD);
        }
      }

      if (startFlag)
      {
        HAL_GPIO_WritePin (LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
      }
      else
      {
        HAL_GPIO_WritePin (LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
      }
      startFlag = !startFlag;
      HAL_Delay (500);

      userKey1Flag = 0;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDMMC|RCC_PERIPHCLK_FDCAN;
  PeriphClkInitStruct.PLL2.PLL2M = 5;
  PeriphClkInitStruct.PLL2.PLL2N = 192;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 10;
  PeriphClkInitStruct.PLL2.PLL2R = 10;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit (&huart4, (uint8_t*) &ch, 1, 0xFFFF);

  return ch;
}


int makeNewFileName(){
  static int fileIndex = 0;
  static char fileName[12] = {0,};

  if ((retSD = f_mount (&SDFatFS, SDPath, 0)) == FR_OK)
  {
    printf ("1. f_mount OK %d \r\n", retSD);
  }
  else
  {
    printf ("1. f_mount OK failed %d\r\n", retSD);
  }
  do
  {
    fileIndex++;
    sprintf (fileName, "data%04d.bin", fileIndex);
    retSD = f_open (&SDFile, fileName, FA_OPEN_EXISTING | FA_READ);
#ifdef _DEBUG_
    //HAL_UART_Transmit (&huart4, fileName, 20, HAL_MAX_DELAY);
    printf("data%04d,%d\r\n", fileIndex, retSD);
#endif

    HAL_Delay (10);
    if (retSD == FR_OK)
      f_close (&SDFile);
    else if ( retSD == FR_NO_FILE )
      break;
    else if ( retSD == FR_NOT_READY ){
      f_close (&SDFile);
      return -1;
    }
    else if ( retSD == FR_INT_ERR ){
      f_close (&SDFile);
      return -2;
    }
  }while (retSD == FR_OK);

  f_close (&SDFile);

  return fileIndex;
}

int detectAccident(IMUDATA * data){
  float acc_x = data->accel_x/2048.0;
  float acc_y = data->accel_y/2048.0;
  float acc_z = (data->accel_z-2048)/2048.0;
  float accel_result = acc_x*acc_x + acc_y*acc_y + acc_z*acc_z;
  if( accel_result >= 4 && accel_result < 196 ){
    return (int)(sqrt(accel_result)*10);

  }
  else if( accel_result >= 196){
    acc_x = data->accel200g_x/164.0;
    acc_y = data->accel200g_y/164.0;
    acc_z = (data->accel200g_z)/164.0;
    accel_result = acc_x*acc_x + acc_y*acc_y + acc_z*acc_z;
    return (int)(sqrt(accel_result)*10);
  }else{
    return 0;
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq ();
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
