/**
  ******************************************************************************
  * @file    Inverter.h
  * @brief   Vehicle Control Unit header file
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __Inverter_H
#define __Inverter_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Inverter_Init(void);
void Inverter_Process(void);
void Inverter_EnableInverter(void);
void Inverter_DisableInverter(void);
void Inverter_ClearInverterFaults(void);

/* Private defines -----------------------------------------------------------*/
/* CAN message IDs */
#define Inverter_INVERTER_COMMAND_ID 0x0C0
#define Inverter_INVERTER_CLEAR_ID 0x0C1

/* Control bytes */
#define Inverter_DIRECTION_FORWARD 0x00
#define Inverter_DIRECTION_REVERSE 0x01
#define Inverter_INVERTER_ENABLE  0x01
#define Inverter_INVERTER_DISABLE 0x00

#ifdef __cplusplus
}
#endif

#endif /* __Inverter_H */
