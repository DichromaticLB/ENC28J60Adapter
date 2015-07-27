/*********************************************
*    User configuration for the library.		 *
*   Everything that should be modified for 	 *
*    your particular system should be done   *
*                in this file								 *
*																						 *
**********************************************/


#ifndef _USER_DEFINED_ENC28J60MAR
#define _USER_DEFINED_ENC28J60MAR
/* This is the setup I used for the STDiscovery F3 in case you want to keep it
#include "SPI_eth_conf.h"
#include "stm32f3xx_hal.h"
#include "stm32f3_discovery.h"
#include "MediaAccess.h"

#define RXBUFSIZE 1524
#define TXBUFSIZE 1524


#define BIGENDIAN
	

#define NUMBER_OF_INTERFACES 2


extern SPI_HandleTypeDef InterfaceSPI[];
extern GPIO_TypeDef * InterfaceGPIO[];
extern uint16_t InterfacePin[NUMBER_OF_INTERFACES];



#define _ENC28J60_EnableEth()  HAL_GPIO_WritePin(InterfaceGPIO[ethInterface],InterfacePin[ethInterface],GPIO_PIN_RESET)
#define _ENC28J60_DisableEth() HAL_GPIO_WritePin(InterfaceGPIO[ethInterface],InterfacePin[ethInterface],GPIO_PIN_SET)

#define _ENC28J60_SPITransfer(bytes,dest) HAL_SPI_Transmit(&InterfaceSPI[ethInterface],dest,(uint16_t)bytes,500)
#define _ENC28J60_SPIReceive(bytes,source)  HAL_SPI_Receive(&InterfaceSPI[ethInterface],source,(uint16_t)bytes,500)

#define setSPI3 ethInterface=0
#define setSPI1 ethInterface=1
*/
#endif
