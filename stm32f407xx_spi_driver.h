/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: Jun 12, 2024
 *      Author: Karan Patel
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_



#include "stm32f407xx.h"


typedef struct {

	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

// Handle structure for SPIX peripheral

typedef struct {
	SPI_RegDef_t  *pSPIx;
	SPI_Config_t   SPIConfig;
	uint8_t       *pTxbuffer;
	uint8_t       *pRxbuffer;
	uint32_t       TxLen;
	uint32_t       RxLen;
	uint8_t       TxState;
	uint8_t       RxState;

}SPI_Handle_t;

//SPI Device Mode
#define SPI_DEVICE_MODE_MASTER   1
#define SPI_DEVICE_MODE_SLAVE    0


//SPI bus configuration

#define SPI_BUS_CONFIG_FD                 1
#define SPI_BUS_CONFIG_HD                 2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY     3

//spi CLK SPEED

#define SPI_CLK_SPEED_DIV2         0
#define SPI_CLK_SPEED_DIV4         1
#define SPI_CLK_SPEED_DIV8         2
#define SPI_CLK_SPEED_DIV16        3
#define SPI_CLK_SPEED_DIV32        4
#define SPI_CLK_SPEED_DIV64        5
#define SPI_CLK_SPEED_DIV128       6
#define SPI_CLK_SPEED_DIV256       7

// SPI DFF(data format)

#define SPI_DFF_8BITS   0
#define SPI_DFF_16BITS  1

//CPOL
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW  0

//CPHA

#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW  0


//SPI SSM
#define SPI_SSM_EN   1
#define SPI_SSM_DI   0



//SPI related status flags defination.

#define SPI_RXNE_FLAG      (1<<SPI_SR_RXNE)
#define SPI_TXE_FLAG       (1<<SPI_SR_TXE)
#define SPI_CHSIDE_FLAG    (1<<SPI_SR_CHSIDE)
#define SPI_UDR_FLAG       (1<<SPI_SR_UDR)
#define SPI_CRCERR_FLAG    (1<<SPI_SR_CRCERR)
#define SPI_MODF_FLAG      (1<<SPI_SR_MODF)
#define SPI_OVR_FLAG       (1<<SPI_SR_OVR)
#define SPI_BSY_FLAG       (1<<SPI_SR_BSY)
#define SPI_FRE_FLAG       (1<<SPI_SR_FRE)



//Possible SPI application states.

#define SPI_READY      0
#define SPI_BUSY_IN_RX 1
#define SPI_BUSY_IN_TX 2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   1
#define SPI_EVENT_RX_CMPLT   2
#define SPI_EVENT_OVR_ERR    3
#define SPI_EVENT_CRC_ERR    4
//API'S supported by this driver .
//Peripheral clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);

//Init and Deinit
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

// Data send and recieve.
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint8_t Len);

///IRQ configuration and ISR handling.
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t flagname);
// other peripheral control API's
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
