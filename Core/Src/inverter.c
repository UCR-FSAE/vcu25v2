/**
  ******************************************************************************
  * @file    Inverter.c
  * @brief   Vehicle Control Unit implementation
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <inverter.h>
#include <string.h>



/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define ADC_MAX_VALUE     4095    /* 12-bit ADC max value */
#define TORQUE_MAX_VALUE  32767   /* 16-bit max torque value */
#define ADC_THRESHOLD     100     /* Noise threshold for ADC */
#define BRAKE_THRESHOLD   100     /* Threshold to detect brake press */
#define NUM_POINTS 6

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t InverterActive = 0;


static uint8_t prevState = 0;
uint8_t currentState;

float pedal_table[NUM_POINTS] = {0.15f, 0.35f, 0.5f, 0.65f, 0.75f, 0.95f};
uint16_t torque_table[NUM_POINTS] = {0, 100, 200, 300, 350, 400};


/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern volatile float global_accel_position;

/* Private function prototypes -----------------------------------------------*/
static void Inverter_ProcessAnalogInputs(void);
static void Inverter_TransmitCANMessage(uint16_t torque, uint8_t direction, uint8_t inverterEnable);
/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize the Vehicle Control Unit
  * @retval None
  */
void Inverter_Init(void)
{
  /* Set initial state */
  InverterActive = 0;

  /* Send initial disable message to ensure inverter is off */
  Inverter_DisableInverter();
  HAL_Delay(10);
  Inverter_ClearInverterFaults();
  HAL_Delay(10);
  Inverter_EnableInverter();
  HAL_Delay(10);
}

/**
  * @brief  Process Inverter main functionality (to be called periodically)
  * @retval None
  */
void Inverter_Process(void) {
	// put checks like plausibility here
	Inverter_ProcessAnalogInputs();
}

/**
  * @brief  Process analog inputs to derive control commands
  * @retval None
  */
static void Inverter_ProcessAnalogInputs(void)
{
	/* Only process if Inverter is active */
	// LD1 is active if the Inverter is active.
	if (!InverterActive) {
		HAL_GPIO_WritePin(GPIOB, LD1_Pin, RESET);
		return;
	}
	else { HAL_GPIO_WritePin(GPIOB, LD1_Pin, SET); }


	if (global_accel_position >= 0.8) {
		currentState = 1;
	}
	else if (global_accel_position <= 0.3){
		currentState = 0;
	}
	else {
		currentState = prevState;
	}


	if (currentState != prevState) {
		if (currentState == 1) {
			Inverter_TransmitCANMessage(400, Inverter_DIRECTION_FORWARD, Inverter_INVERTER_ENABLE);
		}
		else {
			Inverter_TransmitCANMessage(0, Inverter_DIRECTION_FORWARD, Inverter_INVERTER_DISABLE);
		}
		prevState = currentState;
	}
}

/**
  * @brief  Transmit CAN message to inverter
  * @param  torque: Torque command value (0-32767)
  * @param  direction: Direction command (0=forward, 1=reverse)
  * @param  inverterEnable: Inverter enable state (0=disable, 1=enable)
  * @retval None
  */
static void Inverter_TransmitCANMessage(uint16_t torque, uint8_t direction, uint8_t inverterEnable)
{
  CAN_TxHeaderTypeDef txHeader;
  uint8_t txData[8];
  uint32_t txMailbox;
  HAL_StatusTypeDef status;

  /* Configure transmission */
  txHeader.StdId = Inverter_INVERTER_COMMAND_ID;
  txHeader.ExtId = 0;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 8;
  txHeader.TransmitGlobalTime = DISABLE;

  /* Pack torque command (little-endian) */
  txData[0] = (uint8_t)(torque & 0xFF);
  txData[1] = (uint8_t)((torque >> 8) & 0xFF);

  /* Speed command (0 for torque control mode) */
  txData[2] = 0;
  txData[3] = 0;

  /* Direction and inverter control */
  txData[4] = direction;
  txData[5] = inverterEnable;

  /* Torque limits (using default) */
  txData[6] = 0;
  txData[7] = 0;

  /* Check if mailboxes are available */
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
	// toggles LD2 if there are no available mailboxes
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, RESET);
  }
  else {
	HAL_GPIO_WritePin(GPIOB, LD2_Pin, SET);

	/* Send CAN message */
	status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

	if (status != HAL_OK) {
		HAL_GPIO_WritePin(GPIOB, LD3_Pin, RESET);
	    if (HAL_CAN_AbortTxRequest(&hcan1, txMailbox) != HAL_OK) {
	      Error_Handler();
	    }
	}
	else { HAL_GPIO_WritePin(GPIOB, LD3_Pin, SET); }
  }
}

/**
  * @brief  Enable inverter
  * @retval None
  */
void Inverter_EnableInverter(void)
{
  /* Set system to active */
  InverterActive = 1;
  /* Send enable message with zero torque command */
  Inverter_TransmitCANMessage(0, Inverter_DIRECTION_FORWARD, Inverter_INVERTER_ENABLE);
}

/**
  * @brief  Disable inverter
  * @retval None
  */
void Inverter_DisableInverter(void)
{
  /* Set system to inactive */
  InverterActive = 0;
  /* Send disable message with zero torque command */
  Inverter_TransmitCANMessage(0, Inverter_DIRECTION_FORWARD, Inverter_INVERTER_DISABLE);
}


void Inverter_ClearInverterFaults(void)
{
	  CAN_TxHeaderTypeDef txHeader;
	  uint8_t txData[8];
	  uint32_t txMailbox;
	  HAL_StatusTypeDef status;

	  /* Configure transmission */
	  txHeader.StdId = Inverter_INVERTER_CLEAR_ID;
	  txHeader.ExtId = 0;
	  txHeader.IDE = CAN_ID_STD;
	  txHeader.RTR = CAN_RTR_DATA;
	  txHeader.DLC = 8;
	  txHeader.TransmitGlobalTime = DISABLE;

	  txData[0] = (uint8_t) 0x14; // command id for clearing fault
	  txData[1] = 0;

	  txData[2] = 1; // command to clear faults
	  txData[3] = 0;

	  txData[4] = 0;
	  txData[5] = 0;

	  txData[6] = 0;
	  txData[7] = 0;

	  /* Check if mailboxes are available */
	  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
	    // No free mailboxes - abort oldest message and try again
	    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0);
	    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX1);
	    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX2);
	  }

	  /* Send CAN message */
	  status = HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);

	  if (status != HAL_OK) {
	    // Failure, try to abort and retry once
	    if (HAL_CAN_AbortTxRequest(&hcan1, txMailbox) != HAL_OK) {
	      Error_Handler();
	    }
	  }
}
