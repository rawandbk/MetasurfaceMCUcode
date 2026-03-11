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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

// SPI receive buffer (one 16-bit frame at a time)
//static volatile uint16_t rx_word[3] = {0, 0, 0};

// Buffer to hold 3x 16-bit frames (48 channels total)
//static volatile uint16_t rx_buf[3] = {0, 0, 0};
//static volatile uint8_t  rx_idx = 0;            /* which frame we are filling: 0,1,2 */
//static volatile uint8_t  rx_group_ready = 0;    /* set to 1 when we have all 3 frames */


// Indicators for "code received"
//static volatile uint16_t rx_last_word = 0;
//static volatile uint32_t rx_count     = 0;  increments on each SPI frame */
//static volatile uint32_t rx_tick_ms   = 0;  time of last RX for LED pulse window */

#define SYNC_WORD   0xA5A5u
#define SEQ_PREFIX  0x5A00u   // top byte = 0x5A

static volatile uint16_t rx_w = 0;      // one 16-bit word at a time
static volatile uint16_t rx_buf[3] = {0,0,0};
static volatile uint8_t  rx_group_ready = 0;

static volatile uint8_t  rx_seq_last = 0;
static volatile uint32_t rx_packets_ok = 0;
static volatile uint32_t rx_packets_bad = 0;
static volatile uint32_t rx_resyncs = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
static void ApplyPattern48(uint16_t f0, uint16_t f1, uint16_t f2);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef enum {
    RX_WAIT_SYNC = 0,
    RX_WAIT_SEQ,
    RX_WAIT_F0,
    RX_WAIT_F1,
    RX_WAIT_F2
} rx_state_t;

static volatile rx_state_t rx_state = RX_WAIT_SYNC;
static volatile uint8_t rx_seq_cur = 0;
static volatile uint16_t f0_tmp = 0, f1_tmp = 0, f2_tmp = 0;


static void RX_ResetToSync(void)
{
    rx_state = RX_WAIT_SYNC;
}


static inline GPIO_PinState bit_to_state(uint16_t w, uint8_t bit)
{
    return (w & (1U << bit)) ? GPIO_PIN_SET : GPIO_PIN_RESET;

}

static void SPI_Rearm_1word(void)
{
    if (HAL_SPI_Receive_IT(&hspi1, (uint8_t*)&rx_w, 1) != HAL_OK)
    {
        rx_packets_bad++;   // counts re-arm failures
    }
}


/* Apply 3 frames in order:
 *  frame0 ; CH1-CH16 (bits 0..15)
 *  frame1 ;CH17-CH32 (bits 0..15)
 *  frame2 ; CH33-CH48 (bits 0..15)
 */
static void ApplyPattern48(uint16_t pattern1, uint16_t pattern2, uint16_t pattern3)
{
    /* FRAME 1 (pattern0): CH1-CH16 */

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,  bit_to_state(pattern1, 0));//CH1
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,  bit_to_state(pattern1, 1));//CH2
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4,  bit_to_state(pattern1, 2));//CH3
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, bit_to_state(pattern1, 3));//CH4
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, bit_to_state(pattern1, 4));//CH5
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, bit_to_state(pattern1, 5));//CH6
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3,  bit_to_state(pattern1, 6));//CH7
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,  bit_to_state(pattern1, 7));//CH8
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, bit_to_state(pattern1, 8));//CH9
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, bit_to_state(pattern1, 9));//CH10
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,  bit_to_state(pattern1, 10));//CH11
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, bit_to_state(pattern1, 11));//CH12
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, bit_to_state(pattern1, 12));//CH13
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6,  bit_to_state(pattern1, 13));//CH14
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5,  bit_to_state(pattern1, 14));//CH15
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,  bit_to_state(pattern1, 15));//CH16

    /*  FRAME 2 (pattern1): CH17-CH32 */

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, bit_to_state(pattern2, 0));//CH17
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, bit_to_state(pattern2, 1));//CH18
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, bit_to_state(pattern2, 2));//CH19
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,  bit_to_state(pattern2, 3));//CH20
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,  bit_to_state(pattern2, 4));//CH21
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,  bit_to_state(pattern2, 5));//CH22
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, bit_to_state(pattern2, 6));//CH23
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,  bit_to_state(pattern2, 7));//CH24
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,  bit_to_state(pattern2, 8));//CH25
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  bit_to_state(pattern2, 9));//CH26
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1,  bit_to_state(pattern2, 10));//CH27
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2,  bit_to_state(pattern2, 11));//CH28
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,  bit_to_state(pattern2, 12));//CH29
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, bit_to_state(pattern2, 13));//CH30
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, bit_to_state(pattern2, 14));//CH31
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,  bit_to_state(pattern2, 15));//CH32


    /*  FRAME 3 (pattern2): CH33-CH48 */

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8,  bit_to_state(pattern3, 0));//CH33
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,  bit_to_state(pattern3, 1));//CH34
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, bit_to_state(pattern3, 2));//CH35
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, bit_to_state(pattern3, 3));//CH36
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,  bit_to_state(pattern3, 4));//CH37
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  bit_to_state(pattern3, 5));//CH38
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, bit_to_state(pattern3, 6));//CH39
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, bit_to_state(pattern3, 7));//CH40
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,  bit_to_state(pattern3, 8));//CH41
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8,  bit_to_state(pattern3, 9));//CH42
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,  bit_to_state(pattern3, 10));//CH43
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,  bit_to_state(pattern3, 11));//CH44
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,  bit_to_state(pattern3, 12));//CH45
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,  bit_to_state(pattern3, 13));//CH46
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, bit_to_state(pattern3, 14));//CH47
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7,  bit_to_state(pattern3, 15));//CH48

    //HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, (pattern & (1U << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* SPI Rx complete interrupt callback: runs after one 16-bit frame arrives */
/* SPI Rx complete interrupt callback: runs after one 16-bit frame arrives */
/*void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        // Copy the full 3-word group into a stable buffer
        rx_buf[0] = rx_word[0];
        rx_buf[1] = rx_word[1];
        rx_buf[2] = rx_word[2];

        rx_group_ready = 1;

        // Re-arm for the next 3-word group
        HAL_SPI_Receive_IT(&hspi1, (uint8_t*)rx_word, 3);
    }
}*/
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;

    uint16_t w = rx_w;

    switch (rx_state)
    {
        case RX_WAIT_SYNC:
            if (w == SYNC_WORD) {
                rx_state = RX_WAIT_SEQ;
            }
            break;

        case RX_WAIT_SEQ:
            if ((w & 0xFF00u) == SEQ_PREFIX) {
                rx_seq_cur = (uint8_t)(w & 0x00FFu);
                rx_state = RX_WAIT_F0;
            } else if (w == SYNC_WORD) {
                rx_resyncs++;
                rx_state = RX_WAIT_SEQ;
            } else {
                rx_packets_bad++;
                RX_ResetToSync();
            }
            break;

        case RX_WAIT_F0:
            if (w == SYNC_WORD) {
                rx_resyncs++;
                rx_state = RX_WAIT_SEQ;   // we already have sync, now wait for seq
                break;
            }
            f0_tmp = w;
            rx_state = RX_WAIT_F1;
            break;

        case RX_WAIT_F1:
            if (w == SYNC_WORD) {
                rx_resyncs++;
                rx_state = RX_WAIT_SEQ;
                break;
            }
            f1_tmp = w;
            rx_state = RX_WAIT_F2;
            break;

        case RX_WAIT_F2:
            if (w == SYNC_WORD) {
                rx_resyncs++;
                rx_state = RX_WAIT_SEQ;
                break;
            }
            f2_tmp = w;

            // publish
            rx_buf[0] = f0_tmp;
            rx_buf[1] = f1_tmp;
            rx_buf[2] = f2_tmp;
            rx_group_ready = 1;

            rx_seq_last = rx_seq_cur;
            rx_packets_ok++;

            RX_ResetToSync();
            break;

        default:
            RX_ResetToSync();
            break;
    }

    SPI_Rearm_1word();
}


//*  recover from SPI error */
//void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
/*{
    if (hspi->Instance == SPI1)
    {
        rx_group_ready = 0;

        // Clear OVR if it happened (STM32F4: read DR then SR)
        volatile uint32_t tmp;
        tmp = hspi->Instance->DR;
        tmp = hspi->Instance->SR;
        (void)tmp;

        // Re-arm for next 3-word group
        HAL_SPI_Receive_IT(&hspi1, (uint8_t*)rx_word, 3);
    }
}
*/
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance != SPI1) return;

    rx_packets_bad++;
    RX_ResetToSync();

    // Clear OVR if needed
    volatile uint32_t tmp;
    tmp = hspi->Instance->DR;
    tmp = hspi->Instance->SR;
    (void)tmp;

    SPI_Rearm_1word();
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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  /* Boot banner so you know firmware is running */
  //const char hello[] = "Boot OK: SPI1 slave listening (16-bit, mode 0)\r\n";
  //HAL_UART_Transmit(&huart3, (uint8_t*)hello, sizeof(hello)-1, 100);

  /* Start with outputs low and arm SPI interrupt for one 16-bit word */
  ApplyPattern48(0, 0, 0);
  //HAL_SPI_Receive_IT(&hspi1, (uint8_t *)rx_word, 3);
  SPI_Rearm_1word();

  /* Local shadow used to detect new RX events without disabling IRQs */
  uint32_t rx_count_seen = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* If a full 3-frame group arrived, apply it once */
	  if (rx_group_ready)
	  {
	      __disable_irq();
	      uint16_t f0 = rx_buf[0];
	      uint16_t f1 = rx_buf[1];
	      uint16_t f2 = rx_buf[2];
	      rx_group_ready = 0;
	      __enable_irq();

	      ApplyPattern48(f0, f1, f2);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE9
                           PE10 PE11 PE12 PE13
                           PE14 PE15 PE0 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 PB3 PB4 PB5
                           PB6 PB7 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PF14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11
                           PD12 PD13 PD14 PD15
                           PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_SOF_Pin USB_ID_Pin USB_DM_Pin USB_DP_Pin */
  GPIO_InitStruct.Pin = USB_SOF_Pin|USB_ID_Pin|USB_DM_Pin|USB_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_VBUS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
