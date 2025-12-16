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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Traffic_Logic.h"
#include "test.h"
#include "queue.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MASK_VERTICAL   0x0A // TL2 | TL4
#define MASK_HORIZONTAL 0x05 // TL1 | TL3
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart2;

/* Definitions for TrafficLogicTas */
osThreadId_t TrafficLogicTasHandle;
const osThreadAttr_t TrafficLogicTas_attributes = {
  .name = "TrafficLogicTas",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for InputTask */
osThreadId_t InputTaskHandle;
const osThreadAttr_t InputTask_attributes = {
  .name = "InputTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for inputQueue */
osMessageQueueId_t inputQueueHandle;
const osMessageQueueAttr_t inputQueue_attributes = {
  .name = "inputQueue"
};
/* Definitions for stateMutex */
osMutexId_t stateMutexHandle;
const osMutexAttr_t stateMutex_attributes = {
  .name = "stateMutex"
};
/* USER CODE BEGIN PV */
TrafficConfig_t sysConfig = {
    .toggleFreqVal = 500,      // R1.2: Blink frequency (ms)
    .pedestrianDelay = 4000,   // R1.3: Delay before stopping cars (ms)
    .walkingDelay = 3000,      // R1.4: Walk duration (ms)
    .orangeDelay = 1000,        // R1.6: Yellow light duration (ms)
	.greenDelay = 5000,        // R2.4: Green light duration when no cars at any traffic light
	.redDelayMax = 10000       // R2.6: Max time a car at a red light waits for a busy lane to become red and itself become green
};

// Global Traffic State (for debugging or other tasks)
TrafficState_t globalState = STATE_INIT;

// Global Car Presence State (for achieving level-detection logic within edge-detection logic)
volatile uint8_t globalCarState = 0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
void StartTrafficLogicTask(void *argument);
void StartInputTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Shift register driver
void updateTrafficLights(uint8_t byteU3, uint8_t byteU2, uint8_t byteU1) {

    // Set Clock low
    HAL_GPIO_WritePin(u595_STCP_GPIO_Port, u595_STCP_Pin, GPIO_PIN_RESET);

    // Set bytes to send, and send them via SPI channel
    uint8_t dataBuffer[3] = {byteU3, byteU2, byteU1};
    HAL_SPI_Transmit(&hspi3, dataBuffer, 3, 100);

    // Set Clock high
    HAL_GPIO_WritePin(u595_STCP_GPIO_Port, u595_STCP_Pin, GPIO_PIN_SET);
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
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of stateMutex */
  stateMutexHandle = osMutexNew(&stateMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of inputQueue */
  inputQueueHandle = osMessageQueueNew (10, sizeof(uint8_t), &inputQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TrafficLogicTas */
  TrafficLogicTasHandle = osThreadNew(StartTrafficLogicTask, NULL, &TrafficLogicTas_attributes);

  /* creation of InputTask */
  InputTaskHandle = osThreadNew(StartInputTask, NULL, &InputTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|u595_Reset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(u595_STCP_GPIO_Port, u595_STCP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(u595_Enable_GPIO_Port, u595_Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TL1_Car_Pin */
  GPIO_InitStruct.Pin = TL1_Car_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TL1_Car_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : u595_STCP_Pin */
  GPIO_InitStruct.Pin = u595_STCP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(u595_STCP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TL2_Car_Pin TL3_Car_Pin PL2_Switch_Pin */
  GPIO_InitStruct.Pin = TL2_Car_Pin|TL3_Car_Pin|PL2_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : u595_Enable_Pin */
  GPIO_InitStruct.Pin = u595_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(u595_Enable_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : u595_Reset_Pin */
  GPIO_InitStruct.Pin = u595_Reset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(u595_Reset_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TL4_Car_Pin PL1_Switch_Pin */
  GPIO_InitStruct.Pin = TL4_Car_Pin|PL1_Switch_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTrafficLogicTask */
/**
  * @brief  Function implementing the TrafficLogicTas thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTrafficLogicTask */
void StartTrafficLogicTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	// Version 10 code: Fix R3.5
	// 1. Hardware Initialization
	HAL_GPIO_WritePin(u595_Reset_GPIO_Port, u595_Reset_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(u595_Enable_GPIO_Port, u595_Enable_Pin, GPIO_PIN_RESET);

	InputEvent_t event;
	osStatus_t status;

	// Request Flags
	uint8_t reqPed1 = 0;
	uint8_t reqPed2 = 0;
	uint8_t reqHorz = 0;
	uint8_t reqVert = 0;

	// Timestamps for button presses (R1.3)
	uint32_t ped1RequestTick = 0;
	uint32_t ped2RequestTick = 0;

	// Current Light Output buffer
	uint8_t u3 = 0, u2 = 0, u1 = 0;

	// Helper to apply lights immediately
	void applyLights() {
		updateTrafficLights(u3, u2, u1);
	}

	// State initialisation
	globalState = STATE_INIT;

	// Elapsed time variables
	uint32_t elapsed = 0;
	uint32_t lastWakeTime;

	// Blinking Timer (R1.2)
	uint32_t lastToggleTick = 0;

	// Helper Booleans for readability
	uint8_t iAmBusy, theyAreWaiting;

	// Contention variables (R2.6)
	uint32_t contentionStartTick = 0;
	uint8_t contentionActive = 0;

	for(;;)
	{
		switch (globalState)
		{
		case STATE_INIT:
			// R2.8: Init Vertical Green, Horizontal Red
			globalState = STATE_VERTICAL_GREEN;
			break;

		// ================= VERTICAL FLOW (TL1 & TL3) =================
		case STATE_VERTICAL_PREPARE:
			u3 = TL3_RED | TL4_RED | TL4_YELLOW;
			u2 = TL2_RED | TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_VERTICAL_GREEN;
			break;

		case STATE_VERTICAL_GREEN: // (TL2 & TL4 are Green)
			reqVert = 0;

			// Lights: Vertical Green, Horizontal Red
			u3 = TL3_RED | TL4_GREEN;
			u2 = TL2_GREEN | PL2_RED;
			u1 = TL1_RED | PL1_RED;

			// If requests persist from previous states, set timestamp now if not already set
			if (reqPed1 && ped1RequestTick == 0) ped1RequestTick = osKernelGetTickCount();
			if (reqPed2 && ped2RequestTick == 0) ped2RequestTick = osKernelGetTickCount();

			// Initial Indicator State
			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			elapsed = 0;
			lastWakeTime = osKernelGetTickCount();
			lastToggleTick = osKernelGetTickCount(); // Reset toggle timer

			contentionStartTick = 0;
			contentionActive = 0;

			while (1)
			{
				// 1. Calculate Wait Time for Next Toggle
				uint32_t currentTick = osKernelGetTickCount();
				uint32_t timeSinceToggle = currentTick - lastToggleTick;
				uint32_t waitTime = 0;

				if (timeSinceToggle < sysConfig.toggleFreqVal) {
					waitTime = sysConfig.toggleFreqVal - timeSinceToggle;
				} else {
					waitTime = 0; // Toggle immediately
				}

				// 2. Wait for Input or Blink Timeout
				status = osMessageQueueGet(inputQueueHandle, &event, NULL, waitTime);

				currentTick = osKernelGetTickCount(); // Update after wait
				elapsed += (currentTick - lastWakeTime);
				lastWakeTime = currentTick;

				if (status == osOK) {
					if (event == EVENT_PED1_BTN_PRESS){
						if(reqPed1 == 0) ped1RequestTick = currentTick;
						reqPed1 = 1;
					}
					if (event == EVENT_PED2_BTN_PRESS){
						if(reqPed2 == 0) ped2RequestTick = currentTick;
						reqPed2 = 1;
					}
				}

				// 3. Handle Blinking (R1.2)
				// Check if 500ms has passed since last toggle
				if ((currentTick - lastToggleTick) >= sysConfig.toggleFreqVal) {
					if (reqPed1) u1 ^= PL1_BLUE;
					if (reqPed2) u2 ^= PL2_BLUE;
					applyLights();
					lastToggleTick = currentTick;
				}

				// 4. Refresh Car Status
				iAmBusy = (globalCarState & MASK_VERTICAL);
				theyAreWaiting = (globalCarState & MASK_HORIZONTAL);

				// 5. Update Contention Timer (R2.6)
				if (theyAreWaiting && !contentionActive) {
					contentionActive = 1;
					contentionStartTick = currentTick;
				}
				else if (!theyAreWaiting) {
					contentionActive = 0;
				}
				uint32_t timeSinceArrival = (contentionActive) ? (currentTick - contentionStartTick) : 0;

				// 6. Decision Logic
				int stayGreen = 0;

				// R1.3: Pedestrian Priority with Delay
				if (reqPed1 && (currentTick - ped1RequestTick >= sysConfig.pedestrianDelay)) {
					stayGreen = 0; // Time is up for Ped 1
				}
				else if (reqPed2 && (currentTick - ped2RequestTick >= sysConfig.pedestrianDelay)) {
					stayGreen = 0; // Time is up for Ped 2
				}
				else if (reqPed1 || reqPed2) {
					stayGreen = 1; // Someone is waiting, but time not up yet
				}
				// Car Logic
				else {
					if (!theyAreWaiting) {
						if (iAmBusy || elapsed < sysConfig.greenDelay) stayGreen = 1;
					}
					else {
						if (iAmBusy && timeSinceArrival < sysConfig.redDelayMax) stayGreen = 1;
					}
				}

				if (!stayGreen) break;
			}

			// If Pedestrian 1 is the reason we broke the loop, we can transition
			// DIRECTLY to STATE_PED1_WALK because Vertical Cars stay Green there.
			if (reqPed1) {
				globalState = STATE_PED1_WALK; // Seamless transition (Green -> Green)
			}
			else {
				// Otherwise (Ped2 or just switching to Horizontal), we must stop.
				if (!reqPed2) reqHorz = 1;
				globalState = STATE_VERTICAL_STOPPING;
			}
			break;

		case STATE_VERTICAL_STOPPING:
			u3 = TL3_RED | TL4_YELLOW;
			u2 = TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();
			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		// ================= SAFETY BUFFER =================
		case STATE_ALL_RED:
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();
			osDelay(1000);

			// DECISION LOGIC: First Come, First Served
			// 1. Check if BOTH are waiting
			if (reqPed1 && reqPed2) {
				// Compare timestamps (Handle potential overflow if necessary, but simple comparison usually works for short uptimes)
				if (ped1RequestTick <= ped2RequestTick) {
					globalState = STATE_PED1_WALK; // Ped 1 was first (or same time)
				} else {
					globalState = STATE_PED2_WALK; // Ped 2 was first
				}
			}
			// 2. Check if ONLY Ped 1 is waiting
			else if (reqPed1) {
				globalState = STATE_PED1_WALK;
			}
			// 3. Check if ONLY Ped 2 is waiting
			else if (reqPed2) {
				globalState = STATE_PED2_WALK;
			}
			// 4. Default to Traffic flows
			else if (reqHorz) {
				globalState = STATE_HORIZONTAL_PREPARE;
			}
			else {
				globalState = STATE_VERTICAL_PREPARE;
			}
			break;

		// ================= PEDESTRIAN STATES =================
		case STATE_PED1_WALK:
			// Check if we are coming from Red (TL4 is NOT Green).
			// If u3 & TL4_GREEN is 0, it means we were in ALL_RED (or similar), so we need Yellow first.
			if ((u3 & TL4_GREEN) == 0) {
				// Prepare Phase: Vertical Cars get Red + Yellow
				u3 = TL3_RED | TL4_RED | TL4_YELLOW;
				u2 = TL2_RED | TL2_YELLOW | PL2_RED;
				u1 = TL1_RED | PL1_RED;
				applyLights();
				osDelay(sysConfig.orangeDelay); // Wait for Orange duration
			}

			// PL1 Green, Cars Red
			u3 = TL3_RED | TL4_GREEN;     // TL4 Green (Vertical Car)
			u2 = TL2_GREEN | PL2_RED;     // TL2 Green (Vertical Car)
			u1 = TL1_RED | PL1_GREEN;     // TL1 Red, PL1 Green (Horizontal Ped)

			// Ensure Ped 2 Blue is ON if waiting (Simple check)
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			reqPed1 = 0;
			ped1RequestTick = 0;

			// Use Loop for Walk Delay to allow Ped 2 to register/blink
			uint32_t walkStart = osKernelGetTickCount();
			lastToggleTick = walkStart; // Reset blink timer for Ped 2

			while((osKernelGetTickCount() - walkStart) < sysConfig.walkingDelay)
			{
				uint32_t curTick = osKernelGetTickCount();
				uint32_t timeSinceToggle = curTick - lastToggleTick;

				// Calculate Wait Time
				uint32_t waitTime = 0;
				if (timeSinceToggle < sysConfig.toggleFreqVal) waitTime = sysConfig.toggleFreqVal - timeSinceToggle;

				// Clamp waitTime so we don't overshoot walkingDelay
				uint32_t timeToWalkEnd = sysConfig.walkingDelay - (curTick - walkStart);
				if (waitTime > timeToWalkEnd) waitTime = timeToWalkEnd;

				status = osMessageQueueGet(inputQueueHandle, &event, NULL, waitTime);

				curTick = osKernelGetTickCount();

				// Handle Ped 2 Input
				if (status == osOK && event == EVENT_PED2_BTN_PRESS) {
					if(reqPed2 == 0) ped2RequestTick = curTick;
					reqPed2 = 1;
				}

				// Toggle Ped 2 Blinker
				if ((curTick - lastToggleTick) >= sysConfig.toggleFreqVal) {
					if (reqPed2) {
						u2 ^= PL2_BLUE;
						applyLights();
					}
					lastToggleTick = curTick;
				}
			}

			// Check if we actually need to stop
			uint8_t horzWaiting = (globalCarState & MASK_HORIZONTAL);

			if (reqPed2 || horzWaiting) {
				// If we are stopping for Horizontal Cars, we MUST tell ALL_RED to switch.
				if (horzWaiting) reqHorz = 1;

				globalState = STATE_VERTICAL_STOPPING;
			}
			else {
				// No one is waiting. Seamlessly stay Vertical Green.
				globalState = STATE_VERTICAL_GREEN;
			}
			break;

		case STATE_PED2_WALK:
			// Check if we are coming from Red (TL3 is NOT Green).
			if ((u3 & TL3_GREEN) == 0) {
				// Prepare Phase: Horizontal Cars get Red + Yellow
				u3 = TL3_RED | TL3_YELLOW | TL4_RED;
				u2 = TL2_RED | PL2_RED;
				u1 = TL1_RED | TL1_YELLOW | PL1_RED;
				applyLights();
				osDelay(sysConfig.orangeDelay); // Wait for Orange duration
			}

			// PL2 Green, Cars Red
			u3 = TL3_GREEN | TL4_RED;     // TL3 Green (Horizontal Car)
			u2 = TL2_RED | PL2_GREEN;     // TL2 Red, PL2 Green (Vertical Ped)
			u1 = TL1_GREEN | PL1_RED;     // TL1 Green (Horizontal Car)

			// Ensure Ped 1 Blue is ON if waiting
			if (reqPed1) u1 |= PL1_BLUE;
			applyLights();

			reqPed2 = 0;
			ped2RequestTick = 0;

			// Loop for Walk Delay
			walkStart = osKernelGetTickCount();
			lastToggleTick = walkStart;

			while((osKernelGetTickCount() - walkStart) < sysConfig.walkingDelay)
			{
				uint32_t curTick = osKernelGetTickCount();
				uint32_t timeSinceToggle = curTick - lastToggleTick;

				uint32_t waitTime = 0;
				if (timeSinceToggle < sysConfig.toggleFreqVal) waitTime = sysConfig.toggleFreqVal - timeSinceToggle;

				uint32_t timeToWalkEnd = sysConfig.walkingDelay - (curTick - walkStart);
				if (waitTime > timeToWalkEnd) waitTime = timeToWalkEnd;

				status = osMessageQueueGet(inputQueueHandle, &event, NULL, waitTime);

				curTick = osKernelGetTickCount();

				if (status == osOK && event == EVENT_PED1_BTN_PRESS) {
					if(reqPed1 == 0) ped1RequestTick = curTick;
					reqPed1 = 1;
				}

				// Toggle Ped 1 Blinker
				if ((curTick - lastToggleTick) >= sysConfig.toggleFreqVal) {
					if (reqPed1) {
						u1 ^= PL1_BLUE;
						applyLights();
					}
					lastToggleTick = curTick;
				}
			}

			// Check if we actually need to stop
			uint8_t vertWaiting = (globalCarState & MASK_VERTICAL);

			if (reqPed1 || vertWaiting) {
				// If we are stopping for Vertical Cars, we MUST tell ALL_RED to switch.
				if (vertWaiting) reqVert = 1;

				globalState = STATE_HORIZONTAL_STOPPING;
			}
			else {
				// No one is waiting. Seamlessly stay Horizontal Green.
				globalState = STATE_HORIZONTAL_GREEN;
			}
			break;

		// ================= HORIZONTAL FLOW (TL2 & TL4) =================
		case STATE_HORIZONTAL_PREPARE:
			u3 = TL3_RED | TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_HORIZONTAL_GREEN;
			break;

		case STATE_HORIZONTAL_GREEN: // (TL1 & TL3 are Green)
			reqHorz = 0;

			// Lights: Horizontal Green, Vertical Red
			u3 = TL3_GREEN | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_GREEN | PL1_RED;

			if (reqPed1 && ped1RequestTick == 0) ped1RequestTick = osKernelGetTickCount();
			if (reqPed2 && ped2RequestTick == 0) ped2RequestTick = osKernelGetTickCount();

			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			elapsed = 0;
			lastWakeTime = osKernelGetTickCount();
			lastToggleTick = osKernelGetTickCount(); // Reset toggle timer

			contentionStartTick = 0;
			contentionActive = 0;

			while (1)
			{
				uint32_t currentTick = osKernelGetTickCount();
				uint32_t timeSinceToggle = currentTick - lastToggleTick;
				uint32_t waitTime = 0;

				if (timeSinceToggle < sysConfig.toggleFreqVal) {
					waitTime = sysConfig.toggleFreqVal - timeSinceToggle;
				} else {
					waitTime = 0;
				}

				status = osMessageQueueGet(inputQueueHandle, &event, NULL, waitTime);

				currentTick = osKernelGetTickCount();
				elapsed += (currentTick - lastWakeTime);
				lastWakeTime = currentTick;

				if (status == osOK) {
					if (event == EVENT_PED1_BTN_PRESS) {
						if(reqPed1 == 0) ped1RequestTick = currentTick;
						reqPed1 = 1;
					}
					if (event == EVENT_PED2_BTN_PRESS) {
						if(reqPed2 == 0) ped2RequestTick = currentTick;
						reqPed2 = 1;
					}
				}

				// Blinking Logic
				if ((currentTick - lastToggleTick) >= sysConfig.toggleFreqVal) {
					if (reqPed1) u1 ^= PL1_BLUE;
					if (reqPed2) u2 ^= PL2_BLUE;
					applyLights();
					lastToggleTick = currentTick;
				}

				// Car Logic
				iAmBusy = (globalCarState & MASK_HORIZONTAL);
				theyAreWaiting = (globalCarState & MASK_VERTICAL);

				// Update Contention Timer
				if (theyAreWaiting && !contentionActive) {
					contentionActive = 1;
					contentionStartTick = currentTick;
				}
				else if (!theyAreWaiting) {
					contentionActive = 0;
				}
				uint32_t timeSinceArrival = (contentionActive) ? (currentTick - contentionStartTick) : 0;

				int stayGreen = 0;

				// Pedestrian Logic (R1.3)
				if (reqPed1 && (currentTick - ped1RequestTick >= sysConfig.pedestrianDelay)) {
					stayGreen = 0;
				}
				else if (reqPed2 && (currentTick - ped2RequestTick >= sysConfig.pedestrianDelay)) {
					stayGreen = 0;
				}
				else if (reqPed1 || reqPed2) {
					stayGreen = 1;
				}
				else {
					if (!theyAreWaiting) {
						if (iAmBusy || elapsed < sysConfig.greenDelay) stayGreen = 1;
					}
					else {
						if (iAmBusy && timeSinceArrival < sysConfig.redDelayMax) stayGreen = 1;
					}
				}

				if (!stayGreen) break;
			}

			// If Pedestrian 2 is the reason we broke the loop, transition
			// DIRECTLY to STATE_PED2_WALK because Horizontal Cars stay Green there.
			if (reqPed2) {
				globalState = STATE_PED2_WALK; // Seamless transition (Green -> Green)
			}
			else {
				// Otherwise (Ped1 or just switching to Vertical), we must stop.
				if (!reqPed1) reqVert = 1;
				globalState = STATE_HORIZONTAL_STOPPING;
			}
			break;

		case STATE_HORIZONTAL_STOPPING:
			u3 = TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		default:
			globalState = STATE_INIT;
			break;
		}
	}

	/*// Version 9 code: 1.3 ToggleFreq parallel button press fix
	// 1. Hardware Initialization
	HAL_GPIO_WritePin(u595_Reset_GPIO_Port, u595_Reset_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(u595_Enable_GPIO_Port, u595_Enable_Pin, GPIO_PIN_RESET);

	InputEvent_t event;
	osStatus_t status;

	// Request Flags
	uint8_t reqPed1 = 0;
	uint8_t reqPed2 = 0;
	uint8_t reqHorz = 0;
	uint8_t reqVert = 0;

	// Timestamps for button presses (R1.3)
	uint32_t ped1RequestTick = 0;
	uint32_t ped2RequestTick = 0;

	// Current Light Output buffer
	uint8_t u3 = 0, u2 = 0, u1 = 0;

	// Helper to apply lights immediately
	void applyLights() {
		updateTrafficLights(u3, u2, u1);
	}

	// State initialisation
	globalState = STATE_INIT;

	// Elapsed time variables
	uint32_t elapsed = 0;
	uint32_t lastWakeTime;

	// Blinking Timer (R1.2)
	uint32_t lastToggleTick = 0;

	// Helper Booleans for readability
	uint8_t iAmBusy, theyAreWaiting;

	// Contention variables (R2.6)
	uint32_t contentionStartTick = 0;
	uint8_t contentionActive = 0;

	for(;;)
	{
		switch (globalState)
		{
		case STATE_INIT:
			// R2.8: Init Vertical Green, Horizontal Red
			globalState = STATE_VERTICAL_GREEN;
			break;

		// ================= VERTICAL FLOW (TL1 & TL3) =================
		case STATE_VERTICAL_PREPARE:
			u3 = TL3_RED | TL4_RED | TL4_YELLOW;
			u2 = TL2_RED | TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_VERTICAL_GREEN;
			break;

		case STATE_VERTICAL_GREEN: // (TL2 & TL4 are Green)
			reqVert = 0;

			// Lights: Vertical Green, Horizontal Red
			u3 = TL3_RED | TL4_GREEN;
			u2 = TL2_GREEN | PL2_RED;
			u1 = TL1_RED | PL1_RED;

			// If requests persist from previous states, set timestamp now if not already set
			if (reqPed1 && ped1RequestTick == 0) ped1RequestTick = osKernelGetTickCount();
			if (reqPed2 && ped2RequestTick == 0) ped2RequestTick = osKernelGetTickCount();

			// Initial Indicator State
			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			elapsed = 0;
			lastWakeTime = osKernelGetTickCount();
			lastToggleTick = osKernelGetTickCount(); // Reset toggle timer

			contentionStartTick = 0;
			contentionActive = 0;

			while (1)
			{
				// 1. Calculate Wait Time for Next Toggle
				uint32_t currentTick = osKernelGetTickCount();
				uint32_t timeSinceToggle = currentTick - lastToggleTick;
				uint32_t waitTime = 0;

				if (timeSinceToggle < sysConfig.toggleFreqVal) {
					waitTime = sysConfig.toggleFreqVal - timeSinceToggle;
				} else {
					waitTime = 0; // Toggle immediately
				}

				// 2. Wait for Input or Blink Timeout
				status = osMessageQueueGet(inputQueueHandle, &event, NULL, waitTime);

				currentTick = osKernelGetTickCount(); // Update after wait
				elapsed += (currentTick - lastWakeTime);
				lastWakeTime = currentTick;

				if (status == osOK) {
					if (event == EVENT_PED1_BTN_PRESS){
						if(reqPed1 == 0) ped1RequestTick = currentTick;
						reqPed1 = 1;
					}
					if (event == EVENT_PED2_BTN_PRESS){
						if(reqPed2 == 0) ped2RequestTick = currentTick;
						reqPed2 = 1;
					}
				}

				// 3. Handle Blinking (R1.2)
				// Check if 500ms has passed since last toggle
				if ((currentTick - lastToggleTick) >= sysConfig.toggleFreqVal) {
					if (reqPed1) u1 ^= PL1_BLUE;
					if (reqPed2) u2 ^= PL2_BLUE;
					applyLights();
					lastToggleTick = currentTick;
				}

				// 4. Refresh Car Status
				iAmBusy = (globalCarState & MASK_VERTICAL);
				theyAreWaiting = (globalCarState & MASK_HORIZONTAL);

				// 5. Update Contention Timer (R2.6)
				if (theyAreWaiting && !contentionActive) {
				    contentionActive = 1;
				    contentionStartTick = currentTick;
				}
				else if (!theyAreWaiting) {
				    contentionActive = 0;
				}
				uint32_t timeSinceArrival = (contentionActive) ? (currentTick - contentionStartTick) : 0;

				// 6. Decision Logic
				int stayGreen = 0;

				// R1.3: Pedestrian Priority with Delay
				if (reqPed1 && (currentTick - ped1RequestTick >= sysConfig.pedestrianDelay)) {
				    stayGreen = 0; // Time is up for Ped 1
				}
				else if (reqPed2 && (currentTick - ped2RequestTick >= sysConfig.pedestrianDelay)) {
				    stayGreen = 0; // Time is up for Ped 2
				}
				else if (reqPed1 || reqPed2) {
				    stayGreen = 1; // Someone is waiting, but time not up yet
				}
				// Car Logic
				else {
					if (!theyAreWaiting) {
						if (iAmBusy || elapsed < sysConfig.greenDelay) stayGreen = 1;
					}
					else {
						if (iAmBusy && timeSinceArrival < sysConfig.redDelayMax) stayGreen = 1;
					}
				}

				if (!stayGreen) break;
			}

			if (!reqPed1 && !reqPed2) reqHorz = 1;

			globalState = STATE_VERTICAL_STOPPING;
			break;

		case STATE_VERTICAL_STOPPING:
			u3 = TL3_RED | TL4_YELLOW;
			u2 = TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();
			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		// ================= SAFETY BUFFER =================
		case STATE_ALL_RED:
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();
			osDelay(1000);

			// DECISION LOGIC: First Come, First Served
			// 1. Check if BOTH are waiting
			if (reqPed1 && reqPed2) {
				// Compare timestamps (Handle potential overflow if necessary, but simple comparison usually works for short uptimes)
				if (ped1RequestTick <= ped2RequestTick) {
					globalState = STATE_PED1_WALK; // Ped 1 was first (or same time)
				} else {
					globalState = STATE_PED2_WALK; // Ped 2 was first
				}
			}
			// 2. Check if ONLY Ped 1 is waiting
			else if (reqPed1) {
				globalState = STATE_PED1_WALK;
			}
			// 3. Check if ONLY Ped 2 is waiting
			else if (reqPed2) {
				globalState = STATE_PED2_WALK;
			}
			// 4. Default to Traffic flows
			else if (reqHorz) {
				globalState = STATE_HORIZONTAL_PREPARE;
			}
			else {
				globalState = STATE_VERTICAL_PREPARE;
			}
			break;

		// ================= PEDESTRIAN STATES =================
		case STATE_PED1_WALK:
			// PL1 Green, Cars Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_GREEN;

			// Ensure Ped 2 Blue is ON if waiting (Simple check)
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			reqPed1 = 0;
			ped1RequestTick = 0;

			// Use Loop for Walk Delay to allow Ped 2 to register/blink
			uint32_t walkStart = osKernelGetTickCount();
			lastToggleTick = walkStart; // Reset blink timer for Ped 2

			while((osKernelGetTickCount() - walkStart) < sysConfig.walkingDelay)
			{
				uint32_t curTick = osKernelGetTickCount();
				uint32_t timeSinceToggle = curTick - lastToggleTick;

				// Calculate Wait Time
				uint32_t waitTime = 0;
				if (timeSinceToggle < sysConfig.toggleFreqVal) waitTime = sysConfig.toggleFreqVal - timeSinceToggle;

				// Clamp waitTime so we don't overshoot walkingDelay
				uint32_t timeToWalkEnd = sysConfig.walkingDelay - (curTick - walkStart);
				if (waitTime > timeToWalkEnd) waitTime = timeToWalkEnd;

				status = osMessageQueueGet(inputQueueHandle, &event, NULL, waitTime);

				curTick = osKernelGetTickCount();

				// Handle Ped 2 Input
				if (status == osOK && event == EVENT_PED2_BTN_PRESS) {
					if(reqPed2 == 0) ped2RequestTick = curTick;
					reqPed2 = 1;
				}

				// Toggle Ped 2 Blinker
				if ((curTick - lastToggleTick) >= sysConfig.toggleFreqVal) {
					if (reqPed2) {
						u2 ^= PL2_BLUE;
						applyLights();
					}
					lastToggleTick = curTick;
				}
			}

			globalState = STATE_ALL_RED;
			break;

		case STATE_PED2_WALK:
			// PL2 Green, Cars Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_GREEN;
			u1 = TL1_RED | PL1_RED;

			// Ensure Ped 1 Blue is ON if waiting
			if (reqPed1) u1 |= PL1_BLUE;
			applyLights();

			reqPed2 = 0;
			ped2RequestTick = 0;

			// Loop for Walk Delay
			walkStart = osKernelGetTickCount();
			lastToggleTick = walkStart;

			while((osKernelGetTickCount() - walkStart) < sysConfig.walkingDelay)
			{
				uint32_t curTick = osKernelGetTickCount();
				uint32_t timeSinceToggle = curTick - lastToggleTick;

				uint32_t waitTime = 0;
				if (timeSinceToggle < sysConfig.toggleFreqVal) waitTime = sysConfig.toggleFreqVal - timeSinceToggle;

				uint32_t timeToWalkEnd = sysConfig.walkingDelay - (curTick - walkStart);
				if (waitTime > timeToWalkEnd) waitTime = timeToWalkEnd;

				status = osMessageQueueGet(inputQueueHandle, &event, NULL, waitTime);

				curTick = osKernelGetTickCount();

				if (status == osOK && event == EVENT_PED1_BTN_PRESS) {
					if(reqPed1 == 0) ped1RequestTick = curTick;
					reqPed1 = 1;
				}

				// Toggle Ped 1 Blinker
				if ((curTick - lastToggleTick) >= sysConfig.toggleFreqVal) {
					if (reqPed1) {
						u1 ^= PL1_BLUE;
						applyLights();
					}
					lastToggleTick = curTick;
				}
			}

			globalState = STATE_ALL_RED;
			break;

		// ================= HORIZONTAL FLOW (TL2 & TL4) =================
		case STATE_HORIZONTAL_PREPARE:
			u3 = TL3_RED | TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_HORIZONTAL_GREEN;
			break;

		case STATE_HORIZONTAL_GREEN: // (TL1 & TL3 are Green)
			reqHorz = 0;

			// Lights: Horizontal Green, Vertical Red
			u3 = TL3_GREEN | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_GREEN | PL1_RED;

			if (reqPed1 && ped1RequestTick == 0) ped1RequestTick = osKernelGetTickCount();
			if (reqPed2 && ped2RequestTick == 0) ped2RequestTick = osKernelGetTickCount();

			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			elapsed = 0;
			lastWakeTime = osKernelGetTickCount();
			lastToggleTick = osKernelGetTickCount(); // Reset toggle timer

			contentionStartTick = 0;
			contentionActive = 0;

			while (1)
			{
				uint32_t currentTick = osKernelGetTickCount();
				uint32_t timeSinceToggle = currentTick - lastToggleTick;
				uint32_t waitTime = 0;

				if (timeSinceToggle < sysConfig.toggleFreqVal) {
					waitTime = sysConfig.toggleFreqVal - timeSinceToggle;
				} else {
					waitTime = 0;
				}

				status = osMessageQueueGet(inputQueueHandle, &event, NULL, waitTime);

				currentTick = osKernelGetTickCount();
				elapsed += (currentTick - lastWakeTime);
				lastWakeTime = currentTick;

				if (status == osOK) {
					if (event == EVENT_PED1_BTN_PRESS) {
						if(reqPed1 == 0) ped1RequestTick = currentTick;
						reqPed1 = 1;
					}
					if (event == EVENT_PED2_BTN_PRESS) {
						if(reqPed2 == 0) ped2RequestTick = currentTick;
						reqPed2 = 1;
					}
				}

				// Blinking Logic
				if ((currentTick - lastToggleTick) >= sysConfig.toggleFreqVal) {
					if (reqPed1) u1 ^= PL1_BLUE;
					if (reqPed2) u2 ^= PL2_BLUE;
					applyLights();
					lastToggleTick = currentTick;
				}

				// Car Logic
				iAmBusy = (globalCarState & MASK_HORIZONTAL);
				theyAreWaiting = (globalCarState & MASK_VERTICAL);

				// Update Contention Timer
				if (theyAreWaiting && !contentionActive) {
				    contentionActive = 1;
				    contentionStartTick = currentTick;
				}
				else if (!theyAreWaiting) {
				    contentionActive = 0;
				}
				uint32_t timeSinceArrival = (contentionActive) ? (currentTick - contentionStartTick) : 0;

				int stayGreen = 0;

				// Pedestrian Logic (R1.3)
				if (reqPed1 && (currentTick - ped1RequestTick >= sysConfig.pedestrianDelay)) {
					stayGreen = 0;
				}
				else if (reqPed2 && (currentTick - ped2RequestTick >= sysConfig.pedestrianDelay)) {
					stayGreen = 0;
				}
				else if (reqPed1 || reqPed2) {
					stayGreen = 1;
				}
				else {
					if (!theyAreWaiting) {
						if (iAmBusy || elapsed < sysConfig.greenDelay) stayGreen = 1;
					}
					else {
						if (iAmBusy && timeSinceArrival < sysConfig.redDelayMax) stayGreen = 1;
					}
				}

				if (!stayGreen) break;
			}

			if (!reqPed1 && !reqPed2) reqVert = 1;

			globalState = STATE_HORIZONTAL_STOPPING;
			break;

		case STATE_HORIZONTAL_STOPPING:
			u3 = TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		default:
			globalState = STATE_INIT;
			break;
		}
	}*/

	/*// Version 8 code: 1.3 ToggleFreq fix
	//1. Initialization
	HAL_GPIO_WritePin(u595_Reset_GPIO_Port, u595_Reset_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(u595_Enable_GPIO_Port, u595_Enable_Pin, GPIO_PIN_RESET);

	InputEvent_t event;
	osStatus_t status;

	// Request Flags
	uint8_t reqPed1 = 0;
	uint8_t reqPed2 = 0;
	uint8_t reqHorz = 0;
	uint8_t reqVert = 0;

	// Current Light Output buffer
	uint8_t u3 = 0, u2 = 0, u1 = 0;

	// Helper to apply lights immediately
	void applyLights() {
		updateTrafficLights(u3, u2, u1);
	}

	// State initialisation
	globalState = STATE_INIT;

	// Elapsed time variable
	uint32_t elapsed = 0;
	uint32_t lastWakeTime;

	// Helper Booleans for readability
	uint8_t iAmBusy, theyAreWaiting;

	// More variables
	uint32_t contentionStartTick = 0;
	uint8_t contentionActive = 0;    // Flag to track if we are currently counting

	// Variable to track when the button was pressed
	uint32_t pedRequestTick = 0;

	for(;;)
	{
		switch (globalState)
		{
		case STATE_INIT:
			// R2.8: Init Vertical Green, Horizontal Red
			globalState = STATE_VERTICAL_PREPARE;
			break;

		// VERTICAL FLOW (TL1 & TL3)
		case STATE_VERTICAL_PREPARE: // Red -> Orange (Get Ready)
			// Cars Red+Yellow (or just Yellow), Peds Red
			u3 = TL3_RED | TL4_RED | TL4_YELLOW;
			u2 = TL2_RED | TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay); // R2.3
			globalState = STATE_VERTICAL_GREEN;
			break;

		case STATE_VERTICAL_GREEN: // (TL2 & TL4 are Green)
			// Reset request flag
			reqVert = 0;

			// Reset Ped Timer
			pedRequestTick = 0;

			// Set Lights: Vertical Green, Horizontal Red
			u3 = TL3_RED | TL4_GREEN;
			u2 = TL2_GREEN | PL2_RED;
			u1 = TL1_RED | PL1_RED;

			// Check for initial pedestrian requests (from previous state)
			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;

			// If peds were already waiting, start the timer now
			if (reqPed1 || reqPed2) {
				pedRequestTick = osKernelGetTickCount();
			}

			// Update traffic lights
			applyLights();

			elapsed = 0;
			lastWakeTime = osKernelGetTickCount(); // Get Start Time

			// redDelayMax helper variables, 'contention' = 'car arrives at red light'
			contentionStartTick = 0;
			contentionActive = 0;

			while (1) // We use 'break' to control exit
			{
				// 1. Update Inputs

				status = osMessageQueueGet(inputQueueHandle, &event, NULL, sysConfig.toggleFreqVal);

				uint32_t currentTick = osKernelGetTickCount();
				elapsed += (currentTick - lastWakeTime);
				lastWakeTime = currentTick;

				if (status == osOK) {
					if (event == EVENT_PED1_BTN_PRESS){
						if(reqPed1 == 0) pedRequestTick = currentTick; // Capture time of press
						reqPed1 = 1;
					}
					if (event == EVENT_PED2_BTN_PRESS){
						if(reqPed2 == 0) pedRequestTick = currentTick; // Capture time of press
						reqPed2 = 1;
					}
				}

				// 2. Refresh Car Status (Volatile Read)
				// Vertical is "I Am Busy", Horizontal is "They Are Waiting"
				iAmBusy = (globalCarState & MASK_VERTICAL);
				theyAreWaiting = (globalCarState & MASK_HORIZONTAL);

				// 3. Update Contention Timer (R2.6 Logic)
				if (theyAreWaiting && !contentionActive) {
					// Car just arrived! Snapshot the time.
					contentionActive = 1;
					contentionStartTick = currentTick;
				}
				else if (!theyAreWaiting) {
					contentionActive = 0;
				}

				// Calculate waiting time
				uint32_t timeSinceArrival = (contentionActive) ? (currentTick - contentionStartTick) : 0;

				// 4. Decision Logic
				int stayGreen = 0;

				// PRIORITIZE PEDESTRIANS (R1.3 Implementation)
				if (reqPed1 || reqPed2) {
					// Calculate how long since the button was pressed
					uint32_t timeSincePedPress = currentTick - pedRequestTick;

					// If we haven't waited long enough, STAY GREEN.
					// This ensures we wait pedestrianDelay while still toggling blue lights.
					if (timeSincePedPress < sysConfig.pedestrianDelay) {
						stayGreen = 1;
					} else {
						stayGreen = 0; // Time is up, switch immediately
					}
				}
				// If no pedestrians, use Car Logic
				else {
					if (!theyAreWaiting) {
						// No contention: Stay if Busy OR if Min Time not passed
						if (iAmBusy || elapsed < sysConfig.greenDelay) stayGreen = 1;
					}
					else {
						// Contention: Stay if Busy AND Time Limit not reached
						if (iAmBusy && timeSinceArrival < sysConfig.redDelayMax) stayGreen = 1;
					}
				}

				// 5. Check Exit Triggers
				if (!stayGreen) break;

				// 6. Update Lights (Blink Indicators)
				if (reqPed1) u1 ^= PL1_BLUE;
				if (reqPed2) u2 ^= PL2_BLUE;
				applyLights();
			}

			if (!reqPed1 && !reqPed2) {
				reqHorz = 1;
			}

			// Prepare for state change
			globalState = STATE_VERTICAL_STOPPING;
			break;

		case STATE_VERTICAL_STOPPING: // Green -> Orange
			u3 = TL3_RED | TL4_YELLOW;
			u2 = TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			// Keep Blue lights off or on? Usually off during change.
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		// SAFETY BUFFER
		case STATE_ALL_RED: // Everyone Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(1000); // Safety pause

			// Decide Logic: Who goes next?
			// Priority: Pedestrians -> Horizontal -> Vertical (Default)
			if (reqPed1) {
				globalState = STATE_PED1_WALK;
			} else if (reqPed2) {
				globalState = STATE_PED2_WALK;
			} else if (reqHorz) {
				globalState = STATE_HORIZONTAL_PREPARE;
			} else {
				// Default back to Vertical if nothing else
				globalState = STATE_VERTICAL_PREPARE;
			}
			break;

		// PEDESTRIAN STATES
		case STATE_PED1_WALK:
			// R1.4: PL1 Green, Cars Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_GREEN; // PL1 Green!
			applyLights();

			reqPed1 = 0; // Clear Request
			osDelay(sysConfig.walkingDelay);

			// Blinking Green for Pedestrians? (Optional, usually good practice)
			// For now, straight to Red
			globalState = STATE_ALL_RED;
			break;

		case STATE_PED2_WALK:
			// PL2 Green, Cars Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_GREEN; // PL2 Green!
			u1 = TL1_RED | PL1_RED;
			applyLights();

			reqPed2 = 0; // Clear Request
			osDelay(sysConfig.walkingDelay);

			globalState = STATE_ALL_RED;
			break;

		// HORIZONTAL FLOW (TL2 & TL4)
		case STATE_HORIZONTAL_PREPARE:
			// TL2/TL4 Red+Yellow
			u3 = TL3_RED | TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_HORIZONTAL_GREEN;
			break;

		case STATE_HORIZONTAL_GREEN: // (TL1 & TL3 are Green)
			// Reset request flag
			reqHorz = 0;

			// Reset Ped Timer
			pedRequestTick = 0;

			// Set Lights: Horizontal Green, Vertical Red
			u3 = TL3_GREEN | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_GREEN | PL1_RED;

			// Update pedestrian light state
			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;

			// Start pedestrian timer
			if (reqPed1 || reqPed2) {
				pedRequestTick = osKernelGetTickCount();
			}

			// Update traffic shield lights
			applyLights();

			elapsed = 0;
			lastWakeTime = osKernelGetTickCount(); // Get Start Time

			// Helper Booleans for readability
			uint8_t iAmBusy, theyAreWaiting;

			// Check redDelayMax comparison
			uint32_t contentionStartTick = 0;
			uint8_t contentionActive = 0;

			while (1) // We use 'break' to control exit
			{
				// 1. Update Inputs
				status = osMessageQueueGet(inputQueueHandle, &event, NULL, sysConfig.toggleFreqVal);

				uint32_t currentTick = osKernelGetTickCount();
				elapsed += (currentTick - lastWakeTime);
				lastWakeTime = currentTick;

				if (status == osOK) {
					if (event == EVENT_PED1_BTN_PRESS) {
						if(reqPed1 == 0) pedRequestTick = currentTick;
						reqPed1 = 1;
					}
					if (event == EVENT_PED2_BTN_PRESS) {
						if(reqPed2 == 0) pedRequestTick = currentTick;
						reqPed2 = 1;
					}
				}

				// 2. Refresh Car Status (Volatile Read)
				// Horizonal is "I Am Busy", Vertical is "They Are Waiting"
				iAmBusy = (globalCarState & MASK_HORIZONTAL);
				theyAreWaiting = (globalCarState & MASK_VERTICAL);

				if (theyAreWaiting && !contentionActive) {
					// Vertical car JUST arrived! Snapshot the time.
					contentionActive = 1;
					contentionStartTick = currentTick;
				}
				else if (!theyAreWaiting) {
					contentionActive = 0;
				}

				// Calculate how long Vertical has been waiting
				uint32_t timeSinceArrival = (contentionActive) ? (currentTick - contentionStartTick) : 0;

				// 3. Decision Logic
				int stayGreen = 0;

				if (reqPed1 || reqPed2) {
					uint32_t timeSincePedPress = currentTick - pedRequestTick;
					if (timeSincePedPress < sysConfig.pedestrianDelay) {
						stayGreen = 1;
					} else {
						stayGreen = 0;
					}
				}
				else {
					if (!theyAreWaiting) {
						if (iAmBusy || elapsed < sysConfig.greenDelay) stayGreen = 1;
					}
					else {
						if (iAmBusy && timeSinceArrival < sysConfig.redDelayMax) stayGreen = 1;
					}
				}

				// 4. Exit Checks
				if (!stayGreen) break;          // Traffic Rules force switch

				// 5. Update Lights (Blink Indicators)
				if (reqPed1) u1 ^= PL1_BLUE;
				if (reqPed2) u2 ^= PL2_BLUE;
				applyLights();
			}

			if (!reqPed1 && !reqPed2) {
					reqVert = 1;
			}

			// 6. Prepare state change
			globalState = STATE_HORIZONTAL_STOPPING;
			break;

		case STATE_HORIZONTAL_STOPPING:
			u3 = TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		default:
			globalState = STATE_INIT;
			break;
		}
	}*/

	/*// Version 7 code: New 2.6 RedDelayMax interpretation
	//1. Initialization
	HAL_GPIO_WritePin(u595_Reset_GPIO_Port, u595_Reset_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(u595_Enable_GPIO_Port, u595_Enable_Pin, GPIO_PIN_RESET);

	InputEvent_t event;
	osStatus_t status;

	// Request Flags
	uint8_t reqPed1 = 0;
	uint8_t reqPed2 = 0;
	uint8_t reqHorz = 0;
	uint8_t reqVert = 0;

	// Current Light Output buffer
	uint8_t u3 = 0, u2 = 0, u1 = 0;

	// Helper to apply lights immediately
	void applyLights() {
		updateTrafficLights(u3, u2, u1);
	}

	// Start in Init
	globalState = STATE_INIT;

	// Elapsed time variable
	uint32_t elapsed = 0;
	uint32_t lastWakeTime;

	// Helper Booleans for readability
	uint8_t iAmBusy, theyAreWaiting;

	// More variables
	uint32_t contentionStartTick = 0;
	uint8_t contentionActive = 0;    // Flag to track if we are currently counting

	for(;;)
	{
		switch (globalState)
		{
		case STATE_INIT:
			// R2.8: Init Vertical Green, Horizontal Red
			globalState = STATE_VERTICAL_PREPARE;
			break;

		// VERTICAL FLOW (TL1 & TL3)
		case STATE_VERTICAL_PREPARE: // Red -> Orange (Get Ready)
			// Cars Red+Yellow (or just Yellow), Peds Red
			u3 = TL3_RED | TL4_RED | TL4_YELLOW;
			u2 = TL2_RED | TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay); // R2.3
			globalState = STATE_VERTICAL_GREEN;
			break;

		case STATE_VERTICAL_GREEN: // (TL2 & TL4 are Green)
			// Reset request flag
			reqVert = 0;

			// Set Lights: Vertical Green, Horizontal Red
			u3 = TL3_RED | TL4_GREEN;
			u2 = TL2_GREEN | PL2_RED;
			u1 = TL1_RED | PL1_RED;

			// Check for initial pedestrian requests (from previous state)
			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			elapsed = 0;
			lastWakeTime = osKernelGetTickCount(); // Get Start Time

			contentionStartTick = 0;
			contentionActive = 0;

			while (1) // We use 'break' to control exit
			{
				// 1. Update Inputs

				status = osMessageQueueGet(inputQueueHandle, &event, NULL, sysConfig.toggleFreqVal);

				uint32_t currentTick = osKernelGetTickCount();
				elapsed += (currentTick - lastWakeTime);
				lastWakeTime = currentTick;

				if (status == osOK) {
					if (event == EVENT_PED1_BTN_PRESS) reqPed1 = 1;
					if (event == EVENT_PED2_BTN_PRESS) reqPed2 = 1;
				}

				// 2. Refresh Car Status (Volatile Read)
				// Vertical is "I Am Busy", Horizontal is "They Are Waiting"
				iAmBusy = (globalCarState & MASK_VERTICAL);
				theyAreWaiting = (globalCarState & MASK_HORIZONTAL);

				// 3. Update Contention Timer (R2.6 Logic)
				if (theyAreWaiting && !contentionActive) {
					// Car just arrived! Snapshot the time.
					contentionActive = 1;
					contentionStartTick = currentTick;
				}
				else if (!theyAreWaiting) {
					contentionActive = 0;
				}

				// Calculate waiting time
				uint32_t timeSinceArrival = (contentionActive) ? (currentTick - contentionStartTick) : 0;

				// 4. Decision Logic
				int stayGreen = 0;

				if (!theyAreWaiting) {
					// No contention: Stay if Busy OR if Min Time not passed
					if (iAmBusy || elapsed < sysConfig.greenDelay) stayGreen = 1;
				}
				else {
					// Contention: Stay if Busy AND Time Limit not reached
					if (iAmBusy && timeSinceArrival < sysConfig.redDelayMax) stayGreen = 1;
				}

				// 5. Check Exit Triggers
				if (!stayGreen) break;
				if (reqPed1 || reqPed2) break;

				// 6. Update Lights (Blink Indicators)
				if (reqPed1) u1 ^= PL1_BLUE;
				if (reqPed2) u2 ^= PL2_BLUE;
				applyLights();
			}

			if (!reqPed1 && !reqPed2) {
				reqHorz = 1;
			}

			// Prepare for state change
			if (reqPed1 || reqPed2) osDelay(sysConfig.pedestrianDelay); // (R1.3)
			globalState = STATE_VERTICAL_STOPPING;
			break;

		case STATE_VERTICAL_STOPPING: // Green -> Orange
			u3 = TL3_RED | TL4_YELLOW;
			u2 = TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			// Keep Blue lights off or on? Usually off during change.
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		// SAFETY BUFFER
		case STATE_ALL_RED: // Everyone Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(1000); // Safety pause

			// Decide Logic: Who goes next?
			// Priority: Pedestrians -> Horizontal -> Vertical (Default)
			if (reqPed1) {
				globalState = STATE_PED1_WALK;
			} else if (reqPed2) {
				globalState = STATE_PED2_WALK;
			} else if (reqHorz) {
				globalState = STATE_HORIZONTAL_PREPARE;
			} else {
				// Default back to Vertical if nothing else
				globalState = STATE_VERTICAL_PREPARE;
			}
			break;

		// PEDESTRIAN STATES
		case STATE_PED1_WALK:
			// R1.4: PL1 Green, Cars Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_GREEN; // PL1 Green!
			applyLights();

			reqPed1 = 0; // Clear Request
			osDelay(sysConfig.walkingDelay);

			// Blinking Green for Pedestrians? (Optional, usually good practice)
			// For now, straight to Red
			globalState = STATE_ALL_RED;
			break;

		case STATE_PED2_WALK:
			// PL2 Green, Cars Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_GREEN; // PL2 Green!
			u1 = TL1_RED | PL1_RED;
			applyLights();

			reqPed2 = 0; // Clear Request
			osDelay(sysConfig.walkingDelay);

			globalState = STATE_ALL_RED;
			break;

		// HORIZONTAL FLOW (TL2 & TL4)
		case STATE_HORIZONTAL_PREPARE:
			// TL2/TL4 Red+Yellow
			u3 = TL3_RED | TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_HORIZONTAL_GREEN;
			break;

		case STATE_HORIZONTAL_GREEN: // (TL1 & TL3 are Green)
			// Reset request flag
			reqHorz = 0;

			// Set Lights: Horizontal Green, Vertical Red
			u3 = TL3_GREEN | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_GREEN | PL1_RED;

			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			elapsed = 0;
			lastWakeTime = osKernelGetTickCount(); // Get Start Time

			// Helper Booleans for readability
			uint8_t iAmBusy, theyAreWaiting;

			// Check redDelayMax comparison
			uint32_t contentionStartTick = 0;
			uint8_t contentionActive = 0;

			while (1) // We use 'break' to control exit
			{
				// 1. Update Inputs
				status = osMessageQueueGet(inputQueueHandle, &event, NULL, sysConfig.toggleFreqVal);

				uint32_t currentTick = osKernelGetTickCount();
				elapsed += (currentTick - lastWakeTime);
				lastWakeTime = currentTick;

				if (status == osOK) {
					if (event == EVENT_PED1_BTN_PRESS) reqPed1 = 1;
					if (event == EVENT_PED2_BTN_PRESS) reqPed2 = 1;
				}

				// 2. Refresh Car Status (Volatile Read)
				// Horizonal is "I Am Busy", Vertical is "They Are Waiting"
				iAmBusy = (globalCarState & MASK_HORIZONTAL);
				theyAreWaiting = (globalCarState & MASK_VERTICAL);

				if (theyAreWaiting && !contentionActive) {
					// Vertical car JUST arrived! Snapshot the time.
					contentionActive = 1;
					contentionStartTick = currentTick;
				}
				else if (!theyAreWaiting) {
					contentionActive = 0;
				}

				// Calculate how long Vertical has been waiting
				uint32_t timeSinceArrival = (contentionActive) ? (currentTick - contentionStartTick) : 0;

				// E. Decision Logic
				int stayGreen = 0;

				if (!theyAreWaiting) {
					// NO CONTENTION:
					// Stay if I (Horizontal) am busy (R2.5) OR Min Time not passed (R2.4)
					if (iAmBusy || elapsed < sysConfig.greenDelay) stayGreen = 1;
				}
				else {
					// CONTENTION:
					// Someone (Vertical) is waiting.
					// Stay ONLY if I am busy AND their wait time < Max (R2.6)
					if (iAmBusy && timeSinceArrival < sysConfig.redDelayMax) stayGreen = 1;
				}

				// F. Exit Checks
				if (!stayGreen) break;          // Traffic Rules force switch
				if (reqPed1 || reqPed2) break;  // Pedestrians override everything

				// 5. Update Lights (Blink Indicators)
				if (reqPed1) u1 ^= PL1_BLUE;
				if (reqPed2) u2 ^= PL2_BLUE;
				applyLights();
			}

			if (!reqPed1 && !reqPed2) {
					reqVert = 1;
			}

			if (reqPed1 || reqPed2) osDelay(sysConfig.pedestrianDelay);
			globalState = STATE_HORIZONTAL_STOPPING;
			break;

		case STATE_HORIZONTAL_STOPPING:
			u3 = TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		default:
			globalState = STATE_INIT;
			break;
		}
	}*/

	/*// Version 6 code: Fixing 2.6: Redmaxdelay implementation
	//1. Initialization
	HAL_GPIO_WritePin(u595_Reset_GPIO_Port, u595_Reset_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(u595_Enable_GPIO_Port, u595_Enable_Pin, GPIO_PIN_RESET);

	InputEvent_t event;
	osStatus_t status;

	// Request Flags
	uint8_t reqPed1 = 0;
	uint8_t reqPed2 = 0;
	uint8_t reqHorz = 0;
	uint8_t reqVert = 0;

	// Current Light Output buffer
	uint8_t u3 = 0, u2 = 0, u1 = 0;

	// Helper to apply lights immediately
	void applyLights() {
		updateTrafficLights(u3, u2, u1);
	}

	// Start in Init
	globalState = STATE_INIT;

	// Elapsed time variable
	uint32_t elapsed = 0;
	uint32_t lastWakeTime;

	// Helper Booleans for readability
	uint8_t iAmBusy, theyAreWaiting;

	for(;;)
	{
		switch (globalState)
		{
		case STATE_INIT:
			// R2.8: Init Vertical Green, Horizontal Red
			globalState = STATE_VERTICAL_PREPARE;
			break;

		// VERTICAL FLOW (TL1 & TL3)
		case STATE_VERTICAL_PREPARE: // Red -> Orange (Get Ready)
			// Cars Red+Yellow (or just Yellow), Peds Red
			u3 = TL3_RED | TL4_RED | TL4_YELLOW;
			u2 = TL2_RED | TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay); // R2.3
			globalState = STATE_VERTICAL_GREEN;
			break;

		case STATE_VERTICAL_GREEN: // (TL2 & TL4 are Green)
			// Reset request flag
			reqVert = 0;

			// Set Lights: Vertical Green, Horizontal Red
			u3 = TL3_RED | TL4_GREEN;
			u2 = TL2_GREEN | PL2_RED;
			u1 = TL1_RED | PL1_RED;

			// Check for initial pedestrian requests (from previous state)
			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			elapsed = 0;
			lastWakeTime = osKernelGetTickCount(); // Get Start Time

			while (1) // We use 'break' to control exit
			{
				// 1. Update Inputs
				status = osMessageQueueGet(inputQueueHandle, &event, NULL, sysConfig.toggleFreqVal);

				uint32_t currentTick = osKernelGetTickCount();
				elapsed += (currentTick - lastWakeTime);
				lastWakeTime = currentTick;

				if (status == osOK) {
					if (event == EVENT_PED1_BTN_PRESS) reqPed1 = 1;
					if (event == EVENT_PED2_BTN_PRESS) reqPed2 = 1;
				}

				// 2. Refresh Car Status (Volatile Read)
				// Vertical is "I Am Busy", Horizontal is "They Are Waiting"
				iAmBusy = (globalCarState & MASK_VERTICAL);
				theyAreWaiting = (globalCarState & MASK_HORIZONTAL);

				// 3. Evaluate Conditions to STAY GREEN
				int canStayGreen = 0;

				if (!theyAreWaiting && iAmBusy) {
					// R2.5: I am busy, nobody else waiting -> Stay Forever
					canStayGreen = 1;
				}
				else if (!theyAreWaiting && !iAmBusy) {
					// R2.4: Everyone empty -> Stay until greenDelay (Min Cycle)
					if (elapsed < sysConfig.greenDelay) canStayGreen = 1;
				}
				else if (theyAreWaiting && iAmBusy) {
					// R2.6: CONTENTION -> Stay only until redDelayMax
					if (elapsed < sysConfig.redDelayMax) canStayGreen = 1;
				}
				// else if (theyAreWaiting && !iAmBusy) -> R2.7: Switch Immediately (canStayGreen = 0)

				// 4. Check Exit Triggers
				if (!canStayGreen) break;         // Traffic Rules say switch
				if (reqPed1 || reqPed2) break;    // Pedestrians override everything

				// 5. Update Lights (Blink Indicators)
				if (reqPed1) u1 ^= PL1_BLUE;
				if (reqPed2) u2 ^= PL2_BLUE;
				applyLights();
			}

			if (!reqPed1 && !reqPed2) {
				reqHorz = 1;
			}

			// Prepare for state change
			if (reqPed1 || reqPed2) osDelay(sysConfig.pedestrianDelay); // (R1.3)
			globalState = STATE_VERTICAL_STOPPING;
			break;

		case STATE_VERTICAL_STOPPING: // Green -> Orange
			u3 = TL3_RED | TL4_YELLOW;
			u2 = TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			// Keep Blue lights off or on? Usually off during change.
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		// SAFETY BUFFER
		case STATE_ALL_RED: // Everyone Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(1000); // Safety pause

			// Decide Logic: Who goes next?
			// Priority: Pedestrians -> Horizontal -> Vertical (Default)
			if (reqPed1) {
				globalState = STATE_PED1_WALK;
			} else if (reqPed2) {
				globalState = STATE_PED2_WALK;
			} else if (reqHorz) {
				globalState = STATE_HORIZONTAL_PREPARE;
			} else {
				// Default back to Vertical if nothing else
				globalState = STATE_VERTICAL_PREPARE;
			}
			break;

		// PEDESTRIAN STATES
		case STATE_PED1_WALK:
			// R1.4: PL1 Green, Cars Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_GREEN; // PL1 Green!
			applyLights();

			reqPed1 = 0; // Clear Request
			osDelay(sysConfig.walkingDelay);

			// Blinking Green for Pedestrians? (Optional, usually good practice)
			// For now, straight to Red
			globalState = STATE_ALL_RED;
			break;

		case STATE_PED2_WALK:
			// PL2 Green, Cars Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_GREEN; // PL2 Green!
			u1 = TL1_RED | PL1_RED;
			applyLights();

			reqPed2 = 0; // Clear Request
			osDelay(sysConfig.walkingDelay);

			globalState = STATE_ALL_RED;
			break;

		// HORIZONTAL FLOW (TL2 & TL4)
		case STATE_HORIZONTAL_PREPARE:
			// TL2/TL4 Red+Yellow
			u3 = TL3_RED | TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_HORIZONTAL_GREEN;
			break;

		case STATE_HORIZONTAL_GREEN: // (TL1 & TL3 are Green)
			// Reset request flag
			reqHorz = 0;

			// Set Lights: Horizontal Green, Vertical Red
			u3 = TL3_GREEN | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_GREEN | PL1_RED;

			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			elapsed = 0;
			lastWakeTime = osKernelGetTickCount(); // Get Start Time

			// Helper Booleans for readability
			uint8_t iAmBusy, theyAreWaiting;

			while (1) // We use 'break' to control exit
			{
				// 1. Update Inputs
				status = osMessageQueueGet(inputQueueHandle, &event, NULL, sysConfig.toggleFreqVal);

				uint32_t currentTick = osKernelGetTickCount();
				elapsed += (currentTick - lastWakeTime);
				lastWakeTime = currentTick;

				if (status == osOK) {
					if (event == EVENT_PED1_BTN_PRESS) reqPed1 = 1;
					if (event == EVENT_PED2_BTN_PRESS) reqPed2 = 1;
				}

				// 2. Refresh Car Status (Volatile Read)
				// Horizonal is "I Am Busy", Vertical is "They Are Waiting"
				iAmBusy = (globalCarState & MASK_HORIZONTAL);
				theyAreWaiting = (globalCarState & MASK_VERTICAL);

				// 3. Evaluate Conditions to STAY GREEN
				int canStayGreen = 0;

				if (!theyAreWaiting && iAmBusy) {
					// R2.5: I am busy, nobody else waiting -> Stay Forever
					canStayGreen = 1;
				}
				else if (!theyAreWaiting && !iAmBusy) {
					// R2.4: Everyone empty -> Stay until greenDelay (Min Cycle)
					if (elapsed < sysConfig.greenDelay) canStayGreen = 1;
				}
				else if (theyAreWaiting && iAmBusy) {
					// R2.6: CONTENTION -> Stay only until redDelayMax
					if (elapsed < sysConfig.redDelayMax) canStayGreen = 1;
				}
				// else if (theyAreWaiting && !iAmBusy) -> R2.7: Switch Immediately (canStayGreen = 0)

				// 4. Check Exit Triggers
				if (!canStayGreen) break;         // Traffic Rules say switch
				if (reqPed1 || reqPed2) break;    // Pedestrians override everything

				// 5. Update Lights (Blink Indicators)
				if (reqPed1) u1 ^= PL1_BLUE;
				if (reqPed2) u2 ^= PL2_BLUE;
				applyLights();
			}

			if (!reqPed1 && !reqPed2) {
					reqVert = 1;
			}

			if (reqPed1 || reqPed2) osDelay(sysConfig.pedestrianDelay);
			globalState = STATE_HORIZONTAL_STOPPING;
			break;

		case STATE_HORIZONTAL_STOPPING:
			u3 = TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		default:
			globalState = STATE_INIT;
			break;
		}
	}*/

	/*// Version 5 code: Volatile int car level states: New attempt 2.4 and 2.5
	//1. Initialization
	HAL_GPIO_WritePin(u595_Reset_GPIO_Port, u595_Reset_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(u595_Enable_GPIO_Port, u595_Enable_Pin, GPIO_PIN_RESET);

	InputEvent_t event;
	osStatus_t status;

	// Request Flags
	uint8_t reqPed1 = 0;
	uint8_t reqPed2 = 0;
	uint8_t reqHorz = 0;
	uint8_t reqVert = 0;

	// Current Light Output buffer
	uint8_t u3 = 0, u2 = 0, u1 = 0;

	// Helper to apply lights immediately
	void applyLights() {
		updateTrafficLights(u3, u2, u1);
	}

	// Start in Init
	globalState = STATE_INIT;

	// Elapsed time variable
	uint32_t elapsed = 0;
	uint32_t lastWakeTime;

	for(;;)
	{
		switch (globalState)
		{
		case STATE_INIT:
			// R2.8: Init Vertical Green, Horizontal Red
			globalState = STATE_VERTICAL_PREPARE;
			break;

		// VERTICAL FLOW (TL1 & TL3)
		case STATE_VERTICAL_PREPARE: // Red -> Orange (Get Ready)
			// Cars Red+Yellow (or just Yellow), Peds Red
			u3 = TL3_RED | TL4_RED | TL4_YELLOW;
			u2 = TL2_RED | TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay); // R2.3
			globalState = STATE_VERTICAL_GREEN;
			break;

		case STATE_VERTICAL_GREEN: // (TL2 & TL4 are Green)
				// Reset request flag
				reqVert = 0;

		        // Set Lights: Vertical Green, Horizontal Red
		        u3 = TL3_RED | TL4_GREEN;
		        u2 = TL2_GREEN | PL2_RED;
		        u1 = TL1_RED | PL1_RED;

		        // Check for initial pedestrian requests (from previous state)
		        if (reqPed1) u1 |= PL1_BLUE;
		        if (reqPed2) u2 |= PL2_BLUE;
		        applyLights();

		        elapsed = 0;
		        lastWakeTime = osKernelGetTickCount(); // Get Start Time

		        // --- THE R2.4 & R2.5 LOGIC LOOP ---
		        // We stay in this loop (Green) IF:
		        // A) Time hasn't expired (elapsed < sysConfig.greenDelay)
		        //    OR
		        // B) (R2.5) We have Vertical cars AND Horizontal is empty
		        while (elapsed < sysConfig.greenDelay ||
		              ((globalCarState & MASK_VERTICAL) && !(globalCarState & MASK_HORIZONTAL)))
		        {
		            // Wait for event (Edge) or timeout (Tick)
		            status = osMessageQueueGet(inputQueueHandle, &event, NULL, sysConfig.toggleFreqVal);

		            // Calculate REAL time elapsed
		            uint32_t currentTick = osKernelGetTickCount();
		            elapsed += (currentTick - lastWakeTime); // Only add actual time passed
		            lastWakeTime = currentTick;              // Update for next loop

		            if (status == osOK) {
		                // Handle Buttons
		                if (event == EVENT_PED1_BTN_PRESS) reqPed1 = 1;
		                if (event == EVENT_PED2_BTN_PRESS) reqPed2 = 1;

		                // (R2.7) IMMEDIATE YIELD:
		                // If Horizontal has cars (0x05) AND Vertical is empty (0x0A is 0)
		                // We must break immediately, ignoring the timer.
		                if ((globalCarState & MASK_HORIZONTAL) && !(globalCarState & MASK_VERTICAL)) {
		                	reqHorz = 1;
		                    break;
		                }
		            }

		            // Toggle Blue Indicators (R1.2)
		            if (reqPed1) u1 ^= PL1_BLUE;
		            if (reqPed2) u2 ^= PL2_BLUE;
		            applyLights();

		            // Stop Condition: If pedestrians are waiting and min time passed, cycle.
		            if ((reqPed1 || reqPed2) && elapsed >= sysConfig.greenDelay) {
		                break;
		            }
		        }

		        if (!reqPed1 && !reqPed2) {
					reqHorz = 1;
				}

		        // Prepare for state change
		        if (reqPed1 || reqPed2) osDelay(sysConfig.pedestrianDelay); // (R1.3)
		        globalState = STATE_VERTICAL_STOPPING;
		        break;

		case STATE_VERTICAL_STOPPING: // Green -> Orange
			u3 = TL3_RED | TL4_YELLOW;
			u2 = TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			// Keep Blue lights off or on? Usually off during change.
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		// SAFETY BUFFER
		case STATE_ALL_RED: // Everyone Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(1000); // Safety pause

			// Decide Logic: Who goes next?
			// Priority: Pedestrians -> Horizontal -> Vertical (Default)
			if (reqPed1) {
				globalState = STATE_PED1_WALK;
			} else if (reqPed2) {
				globalState = STATE_PED2_WALK;
			} else if (reqHorz) {
				globalState = STATE_HORIZONTAL_PREPARE;
			} else {
				// Default back to Vertical if nothing else
				globalState = STATE_VERTICAL_PREPARE;
			}
			break;

		// PEDESTRIAN STATES
		case STATE_PED1_WALK:
			// R1.4: PL1 Green, Cars Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_GREEN; // PL1 Green!
			applyLights();

			reqPed1 = 0; // Clear Request
			osDelay(sysConfig.walkingDelay);

			// Blinking Green for Pedestrians? (Optional, usually good practice)
			// For now, straight to Red
			globalState = STATE_ALL_RED;
			break;

		case STATE_PED2_WALK:
			// PL2 Green, Cars Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_GREEN; // PL2 Green!
			u1 = TL1_RED | PL1_RED;
			applyLights();

			reqPed2 = 0; // Clear Request
			osDelay(sysConfig.walkingDelay);

			globalState = STATE_ALL_RED;
			break;

		// HORIZONTAL FLOW (TL2 & TL4)
		case STATE_HORIZONTAL_PREPARE:
			// TL2/TL4 Red+Yellow
			u3 = TL3_RED | TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_HORIZONTAL_GREEN;
			break;

		case STATE_HORIZONTAL_GREEN: // (TL1 & TL3 are Green)
				// Reset request flag
				reqHorz = 0;

		        // Set Lights: Horizontal Green, Vertical Red
		        u3 = TL3_GREEN | TL4_RED;
		        u2 = TL2_RED | PL2_RED;
		        u1 = TL1_GREEN | PL1_RED;

		        if (reqPed1) u1 |= PL1_BLUE;
		        if (reqPed2) u2 |= PL2_BLUE;
		        applyLights();

		        elapsed = 0;
		        lastWakeTime = osKernelGetTickCount(); // Get Start Time

		        // --- R2.4 & R2.5 LOGIC (Swapped Masks) ---
		        // Stay Green IF: Time left OR (Horizontal Busy AND Vertical Empty)
		        while (elapsed < sysConfig.greenDelay ||
		              ((globalCarState & MASK_HORIZONTAL) && !(globalCarState & MASK_VERTICAL)))
		        {
		            status = osMessageQueueGet(inputQueueHandle, &event, NULL, sysConfig.toggleFreqVal);

		            // Calculate REAL time elapsed
					uint32_t currentTick = osKernelGetTickCount();
					elapsed += (currentTick - lastWakeTime); // Only add actual time passed
					lastWakeTime = currentTick;              // Update for next loop

		            if (status == osOK) {
		                if (event == EVENT_PED1_BTN_PRESS) reqPed1 = 1;
		                if (event == EVENT_PED2_BTN_PRESS) reqPed2 = 1;

		                // (R2.7) Immediate Yield to Vertical
		                if ((globalCarState & MASK_VERTICAL) && !(globalCarState & MASK_HORIZONTAL)) {
		                	reqVert = 1;
		                    break;
		                }
		            }

		            if (reqPed1) u1 ^= PL1_BLUE;
		            if (reqPed2) u2 ^= PL2_BLUE;
		            applyLights();

		            if ((reqPed1 || reqPed2) && elapsed >= sysConfig.greenDelay) {
		                break;
		            }
		        }

		        if (!reqPed1 && !reqPed2) {
		                reqVert = 1;
				}

		        if (reqPed1 || reqPed2) osDelay(sysConfig.pedestrianDelay);
		        globalState = STATE_HORIZONTAL_STOPPING;
		        break;

		case STATE_HORIZONTAL_STOPPING:
			u3 = TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		default:
			globalState = STATE_INIT;
			break;
		}
	}*/

	/*// Version 4 code: Failed implementation of 2.4 and 2.5
	// 1. Hardware Initialization
	HAL_GPIO_WritePin(u595_Reset_GPIO_Port, u595_Reset_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(u595_Enable_GPIO_Port, u595_Enable_Pin, GPIO_PIN_RESET);

	InputEvent_t event;
	osStatus_t status;

	// Request Flags
	uint8_t reqPed1 = 0;
	uint8_t reqPed2 = 0;
	// SWAPPED DEFINITIONS:
	// reqVert tracks TL2 & TL4 (Physical "Vertical")
	// reqHorz tracks TL1 & TL3 (Physical "Horizontal")
	uint8_t reqVert = 0;
	uint8_t reqHorz = 0;

	// Light Output buffer
	uint8_t u3 = 0, u2 = 0, u1 = 0;

	// Loop Variables
	uint32_t lastToggleTick = 0;
	uint32_t elapsed = 0;

	void applyLights() {
		updateTrafficLights(u3, u2, u1);
	}

	globalState = STATE_INIT;

	for(;;)
	{
		switch (globalState)
		{
		case STATE_INIT:
			// R2.8: Init Vertical Green immediately (Skip Prepare)
			globalState = STATE_VERTICAL_GREEN;
			break;

		// ================= VERTICAL FLOW (TL2 & TL4) [Primary] =================
		case STATE_VERTICAL_PREPARE:
			// TL2/TL4 Red+Yellow (Prepare), TL1/TL3 Red
			u3 = TL3_RED | TL4_RED | TL4_YELLOW;
			u2 = TL2_RED | TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_VERTICAL_GREEN;
			break;

		case STATE_VERTICAL_GREEN:
			// TL2/TL4 Green, TL1/TL3 Red
			u3 = TL3_RED | TL4_GREEN;
			u2 = TL2_GREEN | PL2_RED;
			u1 = TL1_RED | PL1_RED;

			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			// We are now Green. Clear the request that got us here.
			reqVert = 0;

			elapsed = 0;
			lastToggleTick = osKernelGetTickCount();
			const uint32_t currentGreenTimeV = sysConfig.greenDelay;

			// Wait while (Time is not up) OR (We have cars AND they don't)
			while (elapsed < currentGreenTimeV || (reqVert && !reqHorz && !reqPed1 && !reqPed2)) {

				 uint32_t currentTick = osKernelGetTickCount();
				 uint32_t timeSinceToggle = currentTick - lastToggleTick;
				 uint32_t waitTime = (timeSinceToggle < sysConfig.toggleFreqVal) ?
									 (sysConfig.toggleFreqVal - timeSinceToggle) : 0;

				 status = osMessageQueueGet(inputQueueHandle, &event, NULL, waitTime);

				 if (status == osOK) {
					 if (event == EVENT_PED1_BTN_PRESS) reqPed1 = 1;
					 if (event == EVENT_PED2_BTN_PRESS) reqPed2 = 1;

					 // R2.5 FIX: Listen to BOTH directions
					 // 1. Conflicting traffic (Horizontal)
					 if (event == EVENT_CAR_DETECTED_TL1 || event == EVENT_CAR_DETECTED_TL3) reqHorz = 1;
					 // 2. Current traffic (Vertical) - Keeps light green!
					 if (event == EVENT_CAR_DETECTED_TL2 || event == EVENT_CAR_DETECTED_TL4) reqVert = 1;
				 }
				 else if (status == osErrorTimeout) {
					 if (reqPed1) u1 ^= PL1_BLUE;
					 if (reqPed2) u2 ^= PL2_BLUE;
					 applyLights();

					 lastToggleTick = osKernelGetTickCount();
					 elapsed += sysConfig.toggleFreqVal;
				 }

				 // EXIT 1: Min time passed AND someone else is waiting
				 if (elapsed >= currentGreenTimeV && (reqHorz || reqPed1 || reqPed2)) {
					 break;
				 }
				 // EXIT 2 (R2.4): Min time passed AND NO ONE is waiting ANYWHERE -> Switch
				 if (elapsed >= currentGreenTimeV && !reqVert && !reqHorz && !reqPed1 && !reqPed2) {
					 reqHorz = 1; // Force switch to Horizontal
					 break;
				 }
			}

			if (reqPed1 || reqPed2) osDelay(sysConfig.pedestrianDelay);
			globalState = STATE_VERTICAL_STOPPING;
			break;

		case STATE_VERTICAL_STOPPING:
			// TL2/TL4 Orange
			u3 = TL3_RED | TL4_YELLOW;
			u2 = TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		// ================= SAFETY BUFFER =================
		case STATE_ALL_RED:
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(1000);

			if (reqPed1) {
				globalState = STATE_PED1_WALK;
			} else if (reqPed2) {
				globalState = STATE_PED2_WALK;
			} else if (reqHorz) {
				globalState = STATE_HORIZONTAL_PREPARE;
			} else {
				// Default back to Vertical (TL2/TL4)
				globalState = STATE_VERTICAL_PREPARE;
			}
			break;

		// ================= HORIZONTAL FLOW (TL1 & TL3) [Secondary] =================
		case STATE_HORIZONTAL_PREPARE:
			// TL1/TL3 Red+Yellow (Prepare), TL2/TL4 Red
			u3 = TL3_RED | TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_HORIZONTAL_GREEN;
			break;

		case STATE_HORIZONTAL_GREEN:
			// TL1/TL3 Green, TL2/TL4 Red
			u3 = TL3_GREEN | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_GREEN | PL1_RED;

			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			reqHorz = 0; // Clear own request

			elapsed = 0;
			lastToggleTick = osKernelGetTickCount();
			const uint32_t currentGreenTimeH = sysConfig.greenDelay;

			while (elapsed < currentGreenTimeH || (reqHorz && !reqVert && !reqPed1 && !reqPed2)) {

				uint32_t currentTick = osKernelGetTickCount();
				uint32_t timeSinceToggle = currentTick - lastToggleTick;
				uint32_t waitTime = (timeSinceToggle < sysConfig.toggleFreqVal) ?
									(sysConfig.toggleFreqVal - timeSinceToggle) : 0;

				status = osMessageQueueGet(inputQueueHandle, &event, NULL, waitTime);

				if (status == osOK) {
					if (event == EVENT_PED1_BTN_PRESS) reqPed1 = 1;
					if (event == EVENT_PED2_BTN_PRESS) reqPed2 = 1;

					// R2.5 FIX: Listen to BOTH directions
					// 1. Conflicting traffic (Vertical)
					if (event == EVENT_CAR_DETECTED_TL2 || event == EVENT_CAR_DETECTED_TL4) reqVert = 1;
					// 2. Current traffic (Horizontal) - Keeps light green!
					if (event == EVENT_CAR_DETECTED_TL1 || event == EVENT_CAR_DETECTED_TL3) reqHorz = 1;
				}
				else if (status == osErrorTimeout) {
					if (reqPed1) u1 ^= PL1_BLUE;
					if (reqPed2) u2 ^= PL2_BLUE;
					applyLights();

					lastToggleTick = osKernelGetTickCount();
					elapsed += sysConfig.toggleFreqVal;
				}

				if (elapsed >= currentGreenTimeH && (reqVert || reqPed1 || reqPed2)) {
					break;
				}
				if (elapsed >= currentGreenTimeH && !reqHorz && !reqVert && !reqPed1 && !reqPed2) {
					reqVert = 1; // Force switch to Vertical
					break;
				}
			}

			if (reqPed1 || reqPed2) osDelay(sysConfig.pedestrianDelay);
			globalState = STATE_HORIZONTAL_STOPPING;
			break;

		case STATE_HORIZONTAL_STOPPING:
			// TL1/TL3 Orange
			u3 = TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		// ================= PEDESTRIAN STATES =================
		case STATE_PED1_WALK:
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_GREEN;
			applyLights();
			reqPed1 = 0;
			osDelay(sysConfig.walkingDelay);
			globalState = STATE_ALL_RED;
			break;

		case STATE_PED2_WALK:
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_GREEN;
			u1 = TL1_RED | PL1_RED;
			applyLights();
			reqPed2 = 0;
			osDelay(sysConfig.walkingDelay);
			globalState = STATE_ALL_RED;
			break;

		default:
			globalState = STATE_INIT;
			break;
		}
	}*/

	/*// Version 3 Code
	// 1. Initialization
	HAL_GPIO_WritePin(u595_Reset_GPIO_Port, u595_Reset_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(u595_Enable_GPIO_Port, u595_Enable_Pin, GPIO_PIN_RESET);

	InputEvent_t event;
	osStatus_t status;

	// Request Flags
	uint8_t reqPed1 = 0;
	uint8_t reqPed2 = 0;
	uint8_t reqHorz = 0;
	uint8_t reqVert = 0;

	// Current Light Output buffer
	uint8_t u3 = 0, u2 = 0, u1 = 0;

	// Helper to apply lights immediately
	void applyLights() {
		updateTrafficLights(u3, u2, u1);
	}

	// Start in Init
	globalState = STATE_INIT;

	for(;;)
	{
		switch (globalState)
		{
		case STATE_INIT:
			// R2.8: Init Vertical Green, Horizontal Red
			globalState = STATE_VERTICAL_PREPARE;
			break;

		// VERTICAL FLOW (TL1 & TL3)
		case STATE_VERTICAL_PREPARE: // Red -> Orange (Get Ready)
			// Cars Red+Yellow (or just Yellow), Peds Red
			u3 = TL3_RED | TL4_RED | TL4_YELLOW;
			u2 = TL2_RED | TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay); // R2.3
			globalState = STATE_VERTICAL_GREEN;
			break;

		case STATE_VERTICAL_GREEN: // Green
			// Cars Green, Peds Red
			u3 = TL3_RED | TL4_GREEN;
			u2 = TL2_GREEN | PL2_RED;
			u1 = TL1_RED | PL1_RED;

			// Task 1: If Ped requested, toggle Blue Light
			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			// Wait for events or timeout
			// We wait in small chunks to handle "Indicator Toggling" (R1.2)
			uint32_t elapsed = 0;
			const uint32_t minGreenTime = 3000;

			while (elapsed < minGreenTime || (!reqHorz && !reqPed1 && !reqPed2)) {
				// Wait for event with timeout
				status = osMessageQueueGet(inputQueueHandle, &event, NULL, sysConfig.toggleFreqVal);

				if (status == osOK) {
					// Handle Input
					if (event == EVENT_PED1_BTN_PRESS) reqPed1 = 1;
					if (event == EVENT_PED2_BTN_PRESS) reqPed2 = 1;
					if (event == EVENT_CAR_DETECTED_TL1 || event == EVENT_CAR_DETECTED_TL3) reqHorz = 1;
					// Note: We are already Vertical, so ignore Vert car inputs or use to extend time
				}

				// R1.2: Toggle Indicator if Ped waiting
				if (reqPed1) { u1 ^= PL1_BLUE; }
				if (reqPed2) { u2 ^= PL2_BLUE; }
				applyLights();

				elapsed += sysConfig.toggleFreqVal;

				// R2.7: If no active cars in our lane (Vertical), but request elsewhere, switch immediately
				// (Simplified: We just check if we passed minGreenTime)
				if (elapsed >= minGreenTime && (reqHorz || reqPed1 || reqPed2)) {
					break; // Exit loop to change state
				}
			}

			// R1.3: Wait pedestrianDelay before turning red if it was a ped request
			if (reqPed1 || reqPed2) {
				osDelay(sysConfig.pedestrianDelay);
			}

			globalState = STATE_VERTICAL_STOPPING;
			break;

		case STATE_VERTICAL_STOPPING: // Green -> Orange
			u3 = TL3_RED | TL4_YELLOW;
			u2 = TL2_YELLOW | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			// Keep Blue lights off or on? Usually off during change.
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		// SAFETY BUFFER
		case STATE_ALL_RED: // Everyone Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_RED;
			applyLights();

			osDelay(1000); // Safety pause

			// Decide Logic: Who goes next?
			// Priority: Pedestrians -> Horizontal -> Vertical (Default)
			if (reqPed1) {
				globalState = STATE_PED1_WALK;
			} else if (reqPed2) {
				globalState = STATE_PED2_WALK;
			} else if (reqHorz) {
				globalState = STATE_HORIZONTAL_PREPARE;
			} else {
				// Default back to Vertical if nothing else
				globalState = STATE_VERTICAL_PREPARE;
			}
			break;

		// PEDESTRIAN STATES
		case STATE_PED1_WALK:
			// R1.4: PL1 Green, Cars Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | PL1_GREEN; // PL1 Green!
			applyLights();

			reqPed1 = 0; // Clear Request
			osDelay(sysConfig.walkingDelay);

			// Blinking Green for Pedestrians? (Optional, usually good practice)
			// For now, straight to Red
			globalState = STATE_ALL_RED;
			break;

		case STATE_PED2_WALK:
			// PL2 Green, Cars Red
			u3 = TL3_RED | TL4_RED;
			u2 = TL2_RED | PL2_GREEN; // PL2 Green!
			u1 = TL1_RED | PL1_RED;
			applyLights();

			reqPed2 = 0; // Clear Request
			osDelay(sysConfig.walkingDelay);

			globalState = STATE_ALL_RED;
			break;

		// HORIZONTAL FLOW (TL2 & TL4)
		case STATE_HORIZONTAL_PREPARE:
			// TL2/TL4 Red+Yellow
			u3 = TL3_RED | TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_RED | TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_HORIZONTAL_GREEN;
			break;

		case STATE_HORIZONTAL_GREEN:
			// TL2/TL4 Green
			u3 = TL3_GREEN | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_GREEN | PL1_RED;

			// Indicator Toggle logic (same as Vertical)
			if (reqPed1) u1 |= PL1_BLUE;
			if (reqPed2) u2 |= PL2_BLUE;
			applyLights();

			reqHorz = 0; // Clear own request
			elapsed = 0;

			while (elapsed < minGreenTime || (!reqVert && !reqPed1 && !reqPed2)) {
				 status = osMessageQueueGet(inputQueueHandle, &event, NULL, sysConfig.toggleFreqVal);

				 if (status == osOK) {
					 if (event == EVENT_PED1_BTN_PRESS) reqPed1 = 1;
					 if (event == EVENT_PED2_BTN_PRESS) reqPed2 = 1;
					 if (event == EVENT_CAR_DETECTED_TL2 || event == EVENT_CAR_DETECTED_TL4) reqVert = 1;
				 }

				 if (reqPed1) u1 ^= PL1_BLUE;
				 if (reqPed2) u2 ^= PL2_BLUE;
				 applyLights();

				 elapsed += sysConfig.toggleFreqVal;

				 if (elapsed >= minGreenTime && (reqVert || reqPed1 || reqPed2)) {
					 break;
				 }
			}

			if (reqPed1 || reqPed2) osDelay(sysConfig.pedestrianDelay);

			globalState = STATE_HORIZONTAL_STOPPING;
			break;

		case STATE_HORIZONTAL_STOPPING:
			u3 = TL3_YELLOW | TL4_RED;
			u2 = TL2_RED | PL2_RED;
			u1 = TL1_YELLOW | PL1_RED;
			applyLights();

			osDelay(sysConfig.orangeDelay);
			globalState = STATE_ALL_RED;
			break;

		default:
			globalState = STATE_INIT;
			break;
		}
	}*/

  /* Version 2 Code
	// 1. Initialization

		HAL_GPIO_WritePin(u595_Reset_GPIO_Port, u595_Reset_Pin, GPIO_PIN_SET);
	    HAL_GPIO_WritePin(u595_Enable_GPIO_Port, u595_Enable_Pin, GPIO_PIN_RESET);

	    InputEvent_t event;
	    osStatus_t status;

	    // Request Flags
	    uint8_t reqPed1 = 0;
	    uint8_t reqPed2 = 0;
	    uint8_t reqHorz = 0;
	    uint8_t reqVert = 0;

	    // Current Light Output buffer
	    uint8_t u3 = 0, u2 = 0, u1 = 0;

	    // Helper to apply lights immediately
	    void applyLights() {
	        updateTrafficLights(u3, u2, u1);
	    }

	    // Start in Init
	    globalState = STATE_INIT;

	    for(;;)
	    {
	        switch (globalState)
	        {
	        case STATE_INIT:
	            // R2.8: Init Vertical Green, Horizontal Red
	            globalState = STATE_VERTICAL_PREPARE;
	            break;

	        // VERTICAL FLOW (TL1 & TL3)
	        case STATE_VERTICAL_PREPARE: // Red -> Orange (Get Ready)
	            // Cars Red+Yellow (or just Yellow), Peds Red
	            u3 = TL3_RED | TL3_YELLOW | TL4_RED;
	            u2 = TL2_RED | PL2_RED;
	            u1 = TL1_RED | TL1_YELLOW | PL1_RED;
	            applyLights();

	            osDelay(sysConfig.orangeDelay); // R2.3
	            globalState = STATE_VERTICAL_GREEN;
	            break;

	        case STATE_VERTICAL_GREEN: // Green
	            // Cars Green, Peds Red
	            u3 = TL3_GREEN | TL4_RED;
	            u2 = TL2_RED | PL2_RED;
	            u1 = TL1_GREEN | PL1_RED;

	            // Task 1: If Ped requested, toggle Blue Light
	            if (reqPed1) u1 |= PL1_BLUE;
	            if (reqPed2) u2 |= PL2_BLUE;
	            applyLights();

	            // Wait for events or timeout
	            // We wait in small chunks to handle "Indicator Toggling" (R1.2)
	            uint32_t elapsed = 0;
	            const uint32_t minGreenTime = 3000;

	            while (elapsed < minGreenTime || (!reqHorz && !reqPed1 && !reqPed2)) {
	                // Wait for event with timeout
	                status = osMessageQueueGet(inputQueueHandle, &event, NULL, sysConfig.toggleFreqVal);

	                if (status == osOK) {
	                    // Handle Input
	                    if (event == EVENT_PED1_BTN_PRESS) reqPed1 = 1;
	                    if (event == EVENT_PED2_BTN_PRESS) reqPed2 = 1;
	                    if (event == EVENT_CAR_DETECTED_TL2 || event == EVENT_CAR_DETECTED_TL4) reqHorz = 1;
	                    // Note: We are already Vertical, so ignore Vert car inputs or use to extend time
	                }

	                // R1.2: Toggle Indicator if Ped waiting
	                if (reqPed1) { u1 ^= PL1_BLUE; }
	                if (reqPed2) { u2 ^= PL2_BLUE; }
	                applyLights();

	                elapsed += sysConfig.toggleFreqVal;

	                // R2.7: If no active cars in our lane (Vertical), but request elsewhere, switch immediately
	                // (Simplified: We just check if we passed minGreenTime)
	                if (elapsed >= minGreenTime && (reqHorz || reqPed1 || reqPed2)) {
	                    break; // Exit loop to change state
	                }
	            }

	            // R1.3: Wait pedestrianDelay before turning red if it was a ped request
	            if (reqPed1 || reqPed2) {
	                osDelay(sysConfig.pedestrianDelay);
	            }

	            globalState = STATE_VERTICAL_STOPPING;
	            break;

	        case STATE_VERTICAL_STOPPING: // Green -> Orange
	            u3 = TL3_YELLOW | TL4_RED;
	            u2 = TL2_RED | PL2_RED;
	            u1 = TL1_YELLOW | PL1_RED;
	            // Keep Blue lights off or on? Usually off during change.
	            applyLights();

	            osDelay(sysConfig.orangeDelay);
	            globalState = STATE_ALL_RED;
	            break;

	        // SAFETY BUFFER
	        case STATE_ALL_RED: // Everyone Red
	            u3 = TL3_RED | TL4_RED;
	            u2 = TL2_RED | PL2_RED;
	            u1 = TL1_RED | PL1_RED;
	            applyLights();

	            osDelay(1000); // Safety pause

	            // Decide Logic: Who goes next?
	            // Priority: Pedestrians -> Horizontal -> Vertical (Default)
	            if (reqPed1) {
	                globalState = STATE_PED1_WALK;
	            } else if (reqPed2) {
	                globalState = STATE_PED2_WALK;
	            } else if (reqHorz) {
	                globalState = STATE_HORIZONTAL_PREPARE;
	            } else {
	                // Default back to Vertical if nothing else
	                globalState = STATE_VERTICAL_PREPARE;
	            }
	            break;

	        // PEDESTRIAN STATES
	        case STATE_PED1_WALK:
	            // R1.4: PL1 Green, Cars Red
	            u3 = TL3_RED | TL4_RED;
	            u2 = TL2_RED | PL2_RED;
	            u1 = TL1_RED | PL1_GREEN; // PL1 Green!
	            applyLights();

	            reqPed1 = 0; // Clear Request
	            osDelay(sysConfig.walkingDelay);

	            // Blinking Green for Pedestrians? (Optional, usually good practice)
	            // For now, straight to Red
	            globalState = STATE_ALL_RED;
	            break;

	        case STATE_PED2_WALK:
	            // PL2 Green, Cars Red
	            u3 = TL3_RED | TL4_RED;
	            u2 = TL2_RED | PL2_GREEN; // PL2 Green!
	            u1 = TL1_RED | PL1_RED;
	            applyLights();

	            reqPed2 = 0; // Clear Request
	            osDelay(sysConfig.walkingDelay);

	            globalState = STATE_ALL_RED;
	            break;

	        // HORIZONTAL FLOW (TL2 & TL4)
	        case STATE_HORIZONTAL_PREPARE:
	            // TL2/TL4 Red+Yellow
	            u3 = TL3_RED | TL4_RED | TL4_YELLOW;
	            u2 = TL2_RED | TL2_YELLOW | PL2_RED;
	            u1 = TL1_RED | PL1_RED;
	            applyLights();

	            osDelay(sysConfig.orangeDelay);
	            globalState = STATE_HORIZONTAL_GREEN;
	            break;

	        case STATE_HORIZONTAL_GREEN:
	            // TL2/TL4 Green
	            u3 = TL3_RED | TL4_GREEN;
	            u2 = TL2_GREEN | PL2_RED;
	            u1 = TL1_RED | PL1_RED;

	            // Indicator Toggle logic (same as Vertical)
	            if (reqPed1) u1 |= PL1_BLUE;
	            if (reqPed2) u2 |= PL2_BLUE;
	            applyLights();

	            reqHorz = 0; // Clear own request
	            elapsed = 0;

	            while (elapsed < minGreenTime || (!reqVert && !reqPed1 && !reqPed2)) {
	                 status = osMessageQueueGet(inputQueueHandle, &event, NULL, sysConfig.toggleFreqVal);

	                 if (status == osOK) {
	                     if (event == EVENT_PED1_BTN_PRESS) reqPed1 = 1;
	                     if (event == EVENT_PED2_BTN_PRESS) reqPed2 = 1;
	                     if (event == EVENT_CAR_DETECTED_TL1 || event == EVENT_CAR_DETECTED_TL3) reqVert = 1;
	                 }

	                 if (reqPed1) u1 ^= PL1_BLUE;
	                 if (reqPed2) u2 ^= PL2_BLUE;
	                 applyLights();

	                 elapsed += sysConfig.toggleFreqVal;

	                 if (elapsed >= minGreenTime && (reqVert || reqPed1 || reqPed2)) {
	                     break;
	                 }
	            }

	            if (reqPed1 || reqPed2) osDelay(sysConfig.pedestrianDelay);

	            globalState = STATE_HORIZONTAL_STOPPING;
	            break;

	        case STATE_HORIZONTAL_STOPPING:
	            u3 = TL3_RED | TL4_YELLOW;
	            u2 = TL2_YELLOW | PL2_RED;
	            u1 = TL1_RED | PL1_RED;
	            applyLights();

	            osDelay(sysConfig.orangeDelay);
	            globalState = STATE_ALL_RED;
	            break;

	        default:
	            globalState = STATE_INIT;
	            break;
	        }
	    }
	*/

  /* Version 1 Code
    // Safety: Ensure Reset is High and Output Enable is Low
	InputEvent_t receivedValue; //uint8_t receivedValue;
	// Safety: Ensure Reset is High and Output Enable is Low
	HAL_GPIO_WritePin(u595_Reset_GPIO_Port, u595_Reset_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(u595_Enable_GPIO_Port, u595_Enable_Pin, GPIO_PIN_RESET);

	// Traffic Light Initialisation
	updateTrafficLights(0x24,0x0C,0x0C);
	osDelay(1000);

  // Infinite loop
  for(;;)
  {
	if (osMessageQueueGet(inputQueueHandle, &receivedValue, NULL, osWaitForever) == osOK){
		if (receivedValue == 1){
			for(int i = 0; i < 5; i++){
				togglePed1();
			}
		}
		if (receivedValue == 2){
			for(int i = 0; i < 5; i++){
				togglePed2();
			}
		}
	}
	osDelay(10);
  }*/
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartInputTask */
/**
* @brief Function implementing the InputTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInputTask */
void StartInputTask(void *argument)
{
  /* USER CODE BEGIN StartInputTask */

	// Code Version 3: Level-detection car state
	// 1. Setup Polling Frequency (50ms is good for debouncing)
	  const TickType_t xPeriod = pdMS_TO_TICKS(50);
	  TickType_t xLastWakeTime = xTaskGetTickCount();

	  // 2. Initialize "Last State" memory for Edge Detection
	  // We assume Pull-Up resistors (Default HIGH, Active LOW)
	  GPIO_PinState lastStatePed1 = GPIO_PIN_SET;
	  GPIO_PinState lastStatePed2 = GPIO_PIN_SET;
	  GPIO_PinState lastStateCar1 = GPIO_PIN_SET; // PC4
	  GPIO_PinState lastStateCar2 = GPIO_PIN_SET; // PB13
	  GPIO_PinState lastStateCar3 = GPIO_PIN_SET; // PB14
	  GPIO_PinState lastStateCar4 = GPIO_PIN_SET; // PA10

	  InputEvent_t eventToSend;

	  for(;;)
	  {
		eventToSend = EVENT_NO_EVENT;
		uint8_t currentSensorMask = 0; // Temp variable for globalCarState

		// PEDESTRIAN INPUTS
		// Pedestrian Button 1
		GPIO_PinState currStatePed1 = HAL_GPIO_ReadPin(PL1_Switch_GPIO_Port, PL1_Switch_Pin);
		if (currStatePed1 == GPIO_PIN_RESET && lastStatePed1 == GPIO_PIN_SET) {
			eventToSend = EVENT_PED1_BTN_PRESS;
			osMessageQueuePut(inputQueueHandle, &eventToSend, 0, 0);
		}
		lastStatePed1 = currStatePed1;

		// Pedestrian Button 2
		GPIO_PinState currStatePed2 = HAL_GPIO_ReadPin(PL2_Switch_GPIO_Port, PL2_Switch_Pin);
		if (currStatePed2 == GPIO_PIN_RESET && lastStatePed2 == GPIO_PIN_SET) {
			eventToSend = EVENT_PED2_BTN_PRESS;
			osMessageQueuePut(inputQueueHandle, &eventToSend, 0, 0);
		}
		lastStatePed2 = currStatePed2;

		// CAR INPUTS
		// Car 1 (Vertical, TL1)
		GPIO_PinState car1 = HAL_GPIO_ReadPin(TL1_Car_GPIO_Port, TL1_Car_Pin);
		if (car1 == GPIO_PIN_RESET) {
			currentSensorMask |= 0x01; // Update Mask (Bit 0)
			if (lastStateCar1 == GPIO_PIN_SET) eventToSend = EVENT_CAR_DETECTED_TL1;
		}
		lastStateCar1 = car1;

		// Car 2 (Horizontal, TL2)
		GPIO_PinState car2 = HAL_GPIO_ReadPin(TL2_Car_GPIO_Port, TL2_Car_Pin);
		if (car2 == GPIO_PIN_RESET) {
			currentSensorMask |= 0x02; // Update Mask (Bit 1)
			if (lastStateCar2 == GPIO_PIN_SET) eventToSend = EVENT_CAR_DETECTED_TL2;
		}
		lastStateCar2 = car2;

		// Car 3 (Vertical, TL3)
		GPIO_PinState car3 = HAL_GPIO_ReadPin(TL3_Car_GPIO_Port, TL3_Car_Pin);
		if (car3 == GPIO_PIN_RESET) {
			currentSensorMask |= 0x04; // Update Mask (Bit 2)
			if (lastStateCar3 == GPIO_PIN_SET) eventToSend = EVENT_CAR_DETECTED_TL3;
		}
		lastStateCar3 = car3;

		// Car 4 (Horizontal, TL4)
		GPIO_PinState car4 = HAL_GPIO_ReadPin(TL4_Car_GPIO_Port, TL4_Car_Pin);
		if (car4 == GPIO_PIN_RESET) {
			currentSensorMask |= 0x08; // Update Mask (Bit 3)
			if (lastStateCar4 == GPIO_PIN_SET) eventToSend = EVENT_CAR_DETECTED_TL4;
		}
		lastStateCar4 = car4;

		// --- 3. FINAL UPDATES ---

		// Update the Global Volatile State for Logic Task to read
		globalCarState = currentSensorMask;

		// If a car JUST arrived (Edge), send event to wake up Logic Task immediately
		if (eventToSend != EVENT_NO_EVENT &&
			eventToSend != EVENT_PED1_BTN_PRESS &&
			eventToSend != EVENT_PED2_BTN_PRESS) {
			osMessageQueuePut(inputQueueHandle, &eventToSend, 0, 0);
		}

		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	  }

  /*// Code version 2: Struct events
  // 1. Setup Polling Frequency (50ms is good for debouncing)
  const TickType_t xPeriod = pdMS_TO_TICKS(50);
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // 2. Initialize "Last State" memory for Edge Detection
  // We assume Pull-Up resistors (Default HIGH, Active LOW)
  GPIO_PinState lastStatePed1 = GPIO_PIN_SET;
  GPIO_PinState lastStatePed2 = GPIO_PIN_SET;

  GPIO_PinState lastStateCar1 = GPIO_PIN_SET; // PC4
  GPIO_PinState lastStateCar2 = GPIO_PIN_SET; // PB13
  GPIO_PinState lastStateCar3 = GPIO_PIN_SET; // PB14
  GPIO_PinState lastStateCar4 = GPIO_PIN_SET; // PA10

  InputEvent_t eventToSend;

  for(;;)
  {
	  eventToSend = EVENT_NO_EVENT;

	  // Check Pedestrian Button 1 (PA15)
	  GPIO_PinState currStatePed1 = HAL_GPIO_ReadPin(PL1_Switch_GPIO_Port, PL1_Switch_Pin);
	  // Check for Falling Edge (High -> Low)
	  if (currStatePed1 == GPIO_PIN_RESET && lastStatePed1 == GPIO_PIN_SET) {
		  eventToSend = EVENT_PED1_BTN_PRESS;
		  osMessageQueuePut(inputQueueHandle, &eventToSend, 0, 0);
	  }
	  lastStatePed1 = currStatePed1;

	  // Check Pedestrian Button 2 (PB7)
	  GPIO_PinState currStatePed2 = HAL_GPIO_ReadPin(PL2_Switch_GPIO_Port, PL2_Switch_Pin);
	  if (currStatePed2 == GPIO_PIN_RESET && lastStatePed2 == GPIO_PIN_SET) {
		  eventToSend = EVENT_PED2_BTN_PRESS;
		  osMessageQueuePut(inputQueueHandle, &eventToSend, 0, 0);
	  }
	  lastStatePed2 = currStatePed2;

	  // Check Car Sensor 1 (PC4)
	  GPIO_PinState currStateCar1 = HAL_GPIO_ReadPin(TL1_Car_GPIO_Port, TL1_Car_Pin);
	  if (currStateCar1 == GPIO_PIN_RESET && lastStateCar1 == GPIO_PIN_SET) {
		  eventToSend = EVENT_CAR_DETECTED_TL1;
		  osMessageQueuePut(inputQueueHandle, &eventToSend, 0, 0);
	  }
	  lastStateCar1 = currStateCar1;

	  // Check Car Sensor 2 (PB13)
	  GPIO_PinState currStateCar2 = HAL_GPIO_ReadPin(TL2_Car_GPIO_Port, TL2_Car_Pin);
	  if (currStateCar2 == GPIO_PIN_RESET && lastStateCar2 == GPIO_PIN_SET) {
		  eventToSend = EVENT_CAR_DETECTED_TL2;
		  osMessageQueuePut(inputQueueHandle, &eventToSend, 0, 0);
	  }
	  lastStateCar2 = currStateCar2;

	  // Check Car Sensor 3 (PB14)
	  GPIO_PinState currStateCar3 = HAL_GPIO_ReadPin(TL3_Car_GPIO_Port, TL3_Car_Pin);
	  if (currStateCar3 == GPIO_PIN_RESET && lastStateCar3 == GPIO_PIN_SET) {
		  eventToSend = EVENT_CAR_DETECTED_TL3;
		  osMessageQueuePut(inputQueueHandle, &eventToSend, 0, 0);
	  }
	  lastStateCar3 = currStateCar3;

	  // Check Car Sensor 4 (PA10)
	  GPIO_PinState currStateCar4 = HAL_GPIO_ReadPin(TL4_Car_GPIO_Port, TL4_Car_Pin);
	  if (currStateCar4 == GPIO_PIN_RESET && lastStateCar4 == GPIO_PIN_SET) {
		  eventToSend = EVENT_CAR_DETECTED_TL4;
		  osMessageQueuePut(inputQueueHandle, &eventToSend, 0, 0);
	  }
	  lastStateCar4 = currStateCar4;

	  // Wait until next poll cycle
	  vTaskDelayUntil(&xLastWakeTime, xPeriod);
  }*/

	/* Code version 1: Non-struct event
	 * TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS(20); // 20ms Period
	xLastWakeTime = xTaskGetTickCount();          // Initialize once


	GPIO_PinState lastState = GPIO_PIN_SET;
	uint8_t buttonPressed = 1;

	for(;;)
	{
		GPIO_PinState currentState = HAL_GPIO_ReadPin(PL2_Switch_GPIO_Port, PL2_Switch_Pin);
		// GPIO_PinState currentState = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin); // Nucleo Blue Button test
		if (currentState == GPIO_PIN_RESET && lastState == GPIO_PIN_SET){ // "Rising" edge for Active Low
			//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			osMessageQueuePut(inputQueueHandle, &buttonPressed, 0, 0);
		}
		lastState = currentState;

		vTaskDelayUntil(&xLastWakeTime, xPeriod);
	} */
  /* USER CODE END StartInputTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
