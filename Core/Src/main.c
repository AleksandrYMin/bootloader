/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef  void (*pFunction)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FALSE 0
#define TRUE 1
#define VECT_TAB_OFFSET  0x10000  /* 0x00 */
pFunction Jump_To_Application;
uint32_t JumpAddress;
/* Global variables. */
static uint8_t xmodem_packet_number = 1u;         /**< Packet number counter. */
static uint32_t xmodem_actual_flash_address = 0u; /**< Address where we have to write. */
static uint8_t x_first_packet_received = FALSE;   /**< First packet or not. */

/* Local functions. */
//https://github.com/ferenc-nemeth/stm32-bootloader
/* Xmodem (128 bytes) packet format
 * Byte  0:       Header
 * Byte  1:       Packet number
 * Byte  2:       Packet number complemen
 * Bytes 3-130:   Data
 * Bytes 131-132: CRC
 */

/* Xmodem (1024 bytes) packet format
 * Byte  0:         Header
 * Byte  1:         Packet number
 * Byte  2:         Packet number complement
 * Bytes 3-1026:    Data
 * Bytes 1027-1028: CRC
 */

/* Maximum allowed errors (user defined). */
#define X_MAX_ERRORS ((uint8_t)3u)

/* Sizes of the packets. */
#define X_PACKET_NUMBER_SIZE  ((uint16_t)2u)
#define X_PACKET_128_SIZE     ((uint16_t)128u)
#define X_PACKET_1024_SIZE    ((uint16_t)1024u)
#define X_PACKET_CRC_SIZE     ((uint16_t)2u)

/* Indexes inside packets. */
#define X_PACKET_NUMBER_INDEX             ((uint16_t)0u)
#define X_PACKET_NUMBER_COMPLEMENT_INDEX  ((uint16_t)1u)
#define X_PACKET_CRC_HIGH_INDEX           ((uint16_t)0u)
#define X_PACKET_CRC_LOW_INDEX            ((uint16_t)1u)


/* Bytes defined by the protocol. */
#define X_SOH ((uint8_t)0x01u)  /**< Start Of Header (128 bytes). */
#define X_STX ((uint8_t)0x02u)  /**< Start Of Header (1024 bytes). */
#define X_EOT ((uint8_t)0x04u)  /**< End Of Transmission. */
#define X_ACK ((uint8_t)0x06u)  /**< Acknowledge. */
#define X_NAK ((uint8_t)0x15u)  /**< Not Acknowledge. */
#define X_CAN ((uint8_t)0x18u)  /**< Cancel. */
#define X_C   ((uint8_t)0x43u)  /**< ASCII "C" to notify the host we want to use CRC16. */

#define FLASH_OK ((uint16_t)1u)

typedef enum {
	UART_OK,
	UART_ERROR
} uart_status;

/* Status report for the functions. */
typedef enum {
  X_OK            = 0x00u, /**< The action was successful. */
  X_ERROR_CRC     = 0x01u, /**< CRC calculation error. */
  X_ERROR_NUMBER  = 0x02u, /**< Packet number mismatch error. */
  X_ERROR_UART    = 0x04u, /**< UART communication error. */
  X_ERROR_FLASH   = 0x08u, /**< Flash related error. */
  X_ERROR         = 0xFFu  /**< Generic error. */
} xmodem_status;
static uint16_t xmodem_calc_crc(uint8_t *data, uint16_t length);
static xmodem_status xmodem_handle_packet(uint8_t size);
static xmodem_status xmodem_error_handler(uint8_t *error_number, uint8_t max_error_number);
int xmodem_receive(void);
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
const uint16_t crctab16[] = {
	0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
	0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
	0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
	0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
	0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
	0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
	0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
	0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
	0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
	0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
	0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
	0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
	0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
	0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
	0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
	0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
	0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
	0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
	0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
	0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
	0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
	0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
	0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
	0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
	0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
	0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
	0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
	0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
	0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
	0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
	0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
	0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78
};

uint16_t crc16(size_t start, size_t stop) {
	uint16_t res = 0xffff;

	for (size_t address = start; address<stop;address++) {
		uint8_t b = *(uint8_t*)address;
		res = ((res >> 8) & 0xff) ^ crctab16[(res ^ b) & 0xff];
	}

	return (~res) & 0xffff;
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	int i=0;
	for(i=0 ; i<len ; i++)
		ITM_SendChar((*ptr++));
	return len;
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
  MX_GPIO_Init();
  //If button is not pressed bootmode_pin has logic high
  if (HAL_GPIO_ReadPin(bootmode_GPIO_Port, bootmode_Pin)){
	  //Check if there is new frameware? uploaded via web interface
	  if (((*(__IO uint32_t*)FLASH_USER_START_ADDR) & 0x2FFE0000 ) == 0x20020000){
		  HAL_GPIO_TogglePin(VD1_GPIO_Port, VD1_Pin|VD2_Pin);

		  //Calculate checksum
		  uint16_t checksum = *(uint16_t*)(ADDR_FLASH_SECTOR_11+(128*1024)-2);
		  int size = *(int*)(ADDR_FLASH_SECTOR_11+128*1024-10);

		  uint16_t savedcrc = crc16(FLASH_USER_START_ADDR,FLASH_USER_START_ADDR+size);

		  if(savedcrc==checksum){//CRC OK, update application
			  clearFlash(CONFIGURATION_START_ADDR,CONFIGURATION_START_ADDR);
			  clearFlash(APP_FLASH_FIRST_PAGE_ADDRESS, FLASH_USER_START_ADDR-1);
			  CopyData(APP_FLASH_FIRST_PAGE_ADDRESS, FLASH_USER_START_ADDR, (ADDR_FLASH_SECTOR_11+GetSectorSize(ADDR_FLASH_SECTOR_11)-1));
		  }

		  clearFlash(FLASH_USER_START_ADDR, ADDR_FLASH_SECTOR_11);
		  HAL_NVIC_SystemReset();
	  }

	  /* Check if valid stack address (RAM address) then jump to user application */
	  if ((((*(__IO uint32_t*)APP_FLASH_FIRST_PAGE_ADDRESS) & 0x2FFE0000 ) == 0x20020000))
	  {
		  /* Jump to user application */
		  JumpAddress = *(__IO uint32_t*) (APP_FLASH_FIRST_PAGE_ADDRESS + 4);
		  __ISB();
		  __DSB();
		  //reset RCC settings
		  SysTick->CTRL &= ~(SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);
		  Jump_To_Application = (pFunction) JumpAddress;
		  /* Initialize user application's Stack Pointer */
		  __set_MSP(*(__IO uint32_t*) APP_FLASH_FIRST_PAGE_ADDRESS);
		  Jump_To_Application();
	  }
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  //Button was pressed, go to XMODEM update mode
  //https://github.com/ferenc-nemeth/stm32-bootloader

  uint8_t error_number = 0u;

  x_first_packet_received = FALSE;
  xmodem_packet_number = 1u;
  xmodem_actual_flash_address = APP_FLASH_FIRST_PAGE_ADDRESS;

  unsigned char header;

  while (1)
  {
	  /* USER CODE END WHILE */

	  /* USER CODE BEGIN 3 */
	  //XMODEM
	  if(!x_first_packet_received){
		  HAL_Delay(500);
		  uart_transmit_ch('C');
	  }

	  if (CDC_ReadRxBuffer_FS(&header, 1) == USB_CDC_RX_BUFFER_OK){
		  xmodem_status packet_status = X_ERROR;
		  switch(header)
		  {
		  case X_SOH:
		  case X_STX:
			  while(CDC_GetRxBufferBytesAvailable_FS()<132);
			  packet_status = xmodem_handle_packet(header);
			  if (X_OK == packet_status)
			  {
				  x_first_packet_received = TRUE;
				  uart_transmit_ch(X_ACK);
			  }
			  /* If the error was flash related, then immediately set the error counter to max (graceful abort). */
			  else if (X_ERROR_FLASH == packet_status)
			  {
				  error_number = X_MAX_ERRORS;
				  xmodem_error_handler(&error_number, X_MAX_ERRORS);
			  }
			  /* Error while processing the packet, either send a NAK or do graceful abort. */
			  else
			  {
				  xmodem_error_handler(&error_number, X_MAX_ERRORS);
			  }
			  break;
			  /* End of Transmission. */
		  case X_EOT:
			  /* ACK, feedback to user (as a text), then jump to user application. */
			  (void)uart_transmit_ch(X_ACK);
			  clearFlash(CONFIGURATION_START_ADDR,CONFIGURATION_START_ADDR);
			  (void)uart_transmit_str((uint8_t*)"\n\rFirmware updated!\n\r");
			  break;
			  /* Abort from host. */
		  case X_CAN:
			  //status = X_ERROR;
			  break;
		  default:
			  /* Wrong header. */
				xmodem_error_handler(&error_number, X_MAX_ERRORS);
			  break;
		  }
	  }
	  header = 0x00;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 60000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_Base_Start_IT(&htim1); // Start the uptime timer
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VD2_GPIO_Port, VD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VD1_GPIO_Port, VD1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : VD2_Pin VD1_Pin */
  GPIO_InitStruct.Pin = VD2_Pin|VD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : bootmode_Pin */
  GPIO_InitStruct.Pin = bootmode_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(bootmode_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief   Calculates the CRC-16 for the input package.
 * @param   *data:  Array of the data which we want to calculate.
 * @param   length: Size of the data, either 128 or 1024 bytes.
 * @return  status: The calculated CRC.
 */
static uint16_t xmodem_calc_crc(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0u;
    while (length)
    {
        length--;
        crc = crc ^ ((uint16_t)*data++ << 8u);
        for (uint8_t i = 0u; i < 8u; i++)
        {
            if (crc & 0x8000u)
            {
                crc = (crc << 1u) ^ 0x1021u;
            }
            else
            {
                crc = crc << 1u;
            }
        }
    }
    return crc;
}

/**
 * @brief   This function handles the data packet we get from the xmodem protocol.
 * @param   header: SOH or STX.
 * @return  status: Report about the packet.
 */
static xmodem_status xmodem_handle_packet(uint8_t header)
{
  xmodem_status status = X_OK;
  uint16_t size = 0u;

  /* 2 bytes for packet number, 1024 for data, 2 for CRC*/
  uint8_t received_packet_number[X_PACKET_NUMBER_SIZE];
  uint8_t received_packet_data[X_PACKET_1024_SIZE];
  uint8_t received_packet_crc[X_PACKET_CRC_SIZE];

  /* Get the size of the data. */
  if (X_SOH == header)
  {
    size = X_PACKET_128_SIZE;
  }
  else if (X_STX == header)
  {
    size = X_PACKET_1024_SIZE;
  }
  else
  {
    /* Wrong header type. This shoudn't be possible... */
    status |= X_ERROR;
  }

  uart_status comm_status = UART_OK;
  /* Get the packet number, data and CRC from UART. */

  comm_status |= CDC_ReadRxBuffer_FS(&received_packet_number[0u], X_PACKET_NUMBER_SIZE);
  comm_status |= CDC_ReadRxBuffer_FS(&received_packet_data[0u], size);
  comm_status |= CDC_ReadRxBuffer_FS(&received_packet_crc[0u], X_PACKET_CRC_SIZE);

  CDC_FlushRxBuffer_FS();
  /* Merge the two bytes of CRC. */
  uint16_t crc_received = ((uint16_t)received_packet_crc[X_PACKET_CRC_HIGH_INDEX] << 8u) | ((uint16_t)received_packet_crc[X_PACKET_CRC_LOW_INDEX]);
  /* We calculate it too. */
  uint16_t crc_calculated = xmodem_calc_crc(&received_packet_data[0u], size);

  /* Communication error. */
  if (UART_OK != comm_status)
  {
    status |= X_ERROR_UART;
  }

  /* If it is the first packet, then erase the memory. */
  if ((X_OK == status) && (FALSE == x_first_packet_received))
  {
    if (FLASH_OK == clearFlash(APP_FLASH_FIRST_PAGE_ADDRESS, FLASH_USER_START_ADDR-1))
    {
      x_first_packet_received = TRUE;
    }
    else
    {
      status |= X_ERROR_FLASH;
    }
  }

  /* Error handling and flashing. */
  if (X_OK == status)
  {
    if (xmodem_packet_number != received_packet_number[0u])
    {
      /* Packet number counter mismatch. */
      status |= X_ERROR_NUMBER;
    }
    if (255u != (received_packet_number[X_PACKET_NUMBER_INDEX] + received_packet_number[X_PACKET_NUMBER_COMPLEMENT_INDEX]))
    {
      /* The sum of the packet number and packet number complement aren't 255. */
      /* The sum always has to be 255. */
      status |= X_ERROR_NUMBER;
    }
    if (crc_calculated != crc_received)
    {
      /* The calculated and received CRC are different. */
      status |= X_ERROR_CRC;
    }
  }

    /* Do the actual flashing (if there weren't any errors). */
    if ((X_OK == status) && (FLASH_OK != WriteDeviceAddressOffset((char*)&received_packet_data[0u], size, xmodem_actual_flash_address-APP_FLASH_FIRST_PAGE_ADDRESS)))
    {
      /* Flashing error. */
      status |= X_ERROR_FLASH;
    }

  /* Raise the packet number and the address counters (if there weren't any errors). */
  if (X_OK == status)
  {
    xmodem_packet_number++;
    xmodem_actual_flash_address += size;
  }

  return status;
}

/**
 * @brief   Handles the xmodem error.
 *          Raises the error counter, then if the number of the errors reached critical, do a graceful abort, otherwise send a NAK.
 * @param   *error_number:    Number of current errors (passed as a pointer).
 * @param   max_error_number: Maximal allowed number of errors.
 * @return  status: X_ERROR in case of too many errors, X_OK otherwise.
 */
static xmodem_status xmodem_error_handler(uint8_t *error_number, uint8_t max_error_number)
{
  xmodem_status status = X_OK;
  /* Raise the error counter. */
  (*error_number)++;
  /* If the counter reached the max value, then abort. */
  if ((*error_number) >= max_error_number)
  {
    /* Graceful abort. */
    (void)uart_transmit_ch(X_CAN);
    (void)uart_transmit_ch(X_CAN);
    status = X_ERROR;
    clearFlash(FLASH_USER_START_ADDR, FLASH_USER_END_ADDR);
  }
  /* Otherwise send a NAK for a repeat. */
  else
  {
    (void)uart_transmit_ch(X_NAK);
    status = X_OK;
  }
  return status;
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
  __disable_irq();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
