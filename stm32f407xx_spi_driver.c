/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: Jun 12, 2024
 *      Author: Karan Patel
 */









#include "stm32f407xx_spi_driver.h"


static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

//Peripheral clock setup

/*  @fn                - GPIO_PeriClockControl
*
*   @brief             -This function enables or disables peripheral clock for given GPIO port.
*
*   @param[in]         - base address of GPIO peripheral.
*   @param[in]         -ENABLE or DISABLE macros
*   @param[in]         -
*
*   @return            -none
*
*   @Note              -none
*/
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi){


	if(EnorDi==ENABLE){
		 if(pSPIx==SPI1){
			 SPI1_PCLK_EN();
		 }
		 else if(pSPIx==SPI2){
			 SPI2_PCLK_EN();
		 }
		 else if(pSPIx==SPI3){
			 SPI3_PCLK_EN();
		 }

	}
	else {
		 if(pSPIx==SPI1){
			SPI1_PCLK_DI();
		}
		 else if(pSPIx==SPI2){
			SPI2_PCLK_DI();
		 }
		 else if(pSPIx==SPI3){
			SPI3_PCLK_DI();
			}
	}
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}
//Peripheral clock setup

/*  @fn                - GPIO_PeriClockControl
*
*   @brief             -This function enables or disables peripheral clock for given GPIO port.
*
*   @param[in]         - base address of GPIO peripheral.
*   @param[in]         -ENABLE or DISABLE macros
*   @param[in]         -
*
*   @return            -none
*
*   @Note              -none
*/


void SPI_Init(SPI_Handle_t *pSPIHandle){
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
    //First let us configure the SPI_CR1 register
	uint32_t temp=0;
	//1) Configure the device mode
     temp|=pSPIHandle->SPIConfig.SPI_DeviceMode<<SPI_CR1_MSTR;
     //2) Configure the bus Config.
     if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_FD){
    	 //bidi mode should be cleared.
    	 temp&=~(1<<SPI_CR1_BIDIMODE);
     }
     else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_HD){
    	 //bidi mode should be set
    	 temp|=(1<<SPI_CR1_BIDIMODE);
     }
     else if (pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BUS_CONFIG_SIMPLEX_RXONLY){
    	 //bidi mode should be cleared
    	 temp&=~(1<<SPI_CR1_BIDIMODE);
    	 //rxonly set
    	 temp|=(1<<SPI_CR1_RXONLY);
     }
    //3)configure the spi serial clock (baud rate)
     temp|=pSPIHandle->SPIConfig.SPI_SclkSpeed<<SPI_CR1_BR;

     //4)	configure the DFF
     temp|=pSPIHandle->SPIConfig.SPI_DFF<<SPI_CR1_DFF;

     //5)configure the CPOL
     temp|=pSPIHandle->SPIConfig.SPI_CPOL<<SPI_CR1_CPOL;
     //6) configure the CPHA.

     temp|=pSPIHandle->SPIConfig.SPI_CPHA<<SPI_CR1_CPHA;

      //7)Enable or disable ssm.
     temp|=pSPIHandle->SPIConfig.SPI_SSM<<SPI_CR1_SSM;

     pSPIHandle->pSPIx->CR1=temp;
}



//spi deinit


void SPI_DeInit(SPI_RegDef_t *pSPIx){

	  if(pSPIx==SPI1){
		  SPI1_REG_RESET();
	  }
	  else if(pSPIx==SPI2){
		  SPI2_REG_RESET();
	  }
	  else if (pSPIx==SPI3){
		  SPI3_REG_RESET();
	  }
}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t flagname){
	if(pSPIx->SR&flagname){
		return 1;
	}
	else return 0;
}


void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t Len){
    while(Len!=0){
    	//wait unitil TXE is set.
    	while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)==0);
       //2) check the dff bit in CR1
      if((pSPIx->CR1 &(1<<SPI_CR1_DFF))){
    	   //16 BIT DFF
    	  //1)load the data in DR
    	  pSPIx->DR=*((uint16_t*)pTxBuffer);
    	  (uint16_t*)pTxBuffer++;
    	  Len--;
    	  Len--;
      }
      else {
    	  //8bit dff
    	  //1)load the data in DR
    	  pSPIx->DR=(*pTxBuffer);
    	   pTxBuffer++;
    	  Len--;
      }
    }
}



void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer,uint32_t Len){
	 while(Len!=0){
	    	//wait unitil TXE is set.
	    	while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)==0);
	       //2) check the dff bit in CR1
	      if((pSPIx->CR1 &(1<<SPI_CR1_DFF))){
	    	   //16 BIT DFF
	    	  //1)load the data in DR
	    	  *((uint16_t*)pRxBuffer)= pSPIx->DR;
	    	  (uint16_t*)pRxBuffer++;
	    	  Len--;
	    	  Len--;
	      }
	      else {
	    	  //8bit dff
	    	  //1)load the data in DR
	    	  *(pRxBuffer)=pSPIx->DR;
	    	   pRxBuffer++;
	    	  Len--;
	      }
	    }
}




void SPI_PeripheralControl(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	   if(EnorDi==ENABLE){
		   pSPIx->CR1|=(1<<SPI_CR1_SPE);
	   }
	   else {
		   pSPIx->CR1&=~(1<<SPI_CR1_SPE);
	   }
}

void  SPI_SSIConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	   if(EnorDi==ENABLE){
		   pSPIx->CR1|=(1<<SPI_CR1_SSI);
	   }
	   else {
		   pSPIx->CR1&=~(1<<SPI_CR1_SSI);
	   }
}

void  SPI_SSOEConfig(SPI_RegDef_t *pSPIx,uint8_t EnorDi){
	   if(EnorDi==ENABLE){
		   pSPIx->CR2|=(1<<SPI_CR2_SSOE);
	   }
	   else {
		   pSPIx->CR2&=~(1<<SPI_CR2_SSOE);
	   }
}
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER3 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 6 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER3 |= ( 1 << (IRQNumber % 64) );
		}
	}

}





uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t Len){

	uint8_t state=pSPIHandle->TxState;
	if(state!=SPI_BUSY_IN_TX){

   //1) Save the TX buffer address and len information in some global variables.
     pSPIHandle->pTxbuffer=pTxBuffer;
     pSPIHandle->TxLen=Len;
	//2)Mark the SPI state as busy in transmission so that no other code
	//can takeover same SPI peripheral until transmission is over.
     pSPIHandle->TxState=SPI_BUSY_IN_TX;

	//3)Enable the TXEIE control bit to get interrupt whenever TXE falg is set in SR.
     pSPIHandle->pSPIx->CR2|=(1<<SPI_CR2_TXEIE);
	//4)Data transmission will be handled by the ISR code.
}
	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer,uint8_t Len){
	uint8_t state=pSPIHandle->RxState;
	if(state!=SPI_BUSY_IN_RX){

   //1) Save the TX buffer address and len information in some global variables.
     pSPIHandle->pRxbuffer=pRxBuffer;
     pSPIHandle->RxLen=Len;
	//2)Mark the SPI state as busy in transmission so that no other code
	//can takeover same SPI peripheral until transmission is over.
     pSPIHandle->RxState=SPI_BUSY_IN_TX;

	//3)Enable the TXEIE control bit to get interrupt whenever TXE falg is set in SR.
     pSPIHandle->pSPIx->CR2|=(1<<SPI_CR2_TXEIE);
	//4)Data transmission will be handled by the ISR code.
}
	return state;
}


void SPI_IRQHandling(SPI_Handle_t *pHandle){

	//First lets check for TXE
    uint8_t temp1,temp2;
    temp1=pHandle->pSPIx->SR&(1<<SPI_SR_TXE);
    temp2=pHandle->pSPIx->CR2&(1<<SPI_CR2_TXEIE);

    if(temp1&&temp2){
    	spi_txe_interrupt_handle(pHandle);
    }

   temp1=pHandle->pSPIx->SR&(1<<SPI_SR_RXNE);
   temp2=pHandle->pSPIx->CR2&(1<<SPI_CR2_RXNEIE);

   if(temp1&&temp2){
	   spi_rxne_interrupt_handle(pHandle);
   }
   temp1=pHandle->pSPIx->SR&(1<<SPI_SR_OVR);
   temp2=pHandle->pSPIx->CR2&(1<<SPI_CR2_ERRIE);

   if(temp1&&temp2){
	   spi_ovr_err_interrupt_handle(pHandle);
   }

}

void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2&=~(1<<SPI_CR2_TXEIE);
	pSPIHandle->TxLen=0;
    pSPIHandle->pTxbuffer=NULL;
    pSPIHandle->TxState=SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxbuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}
static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle){

	//check the DFF format
	if(pSPIHandle->pSPIx->CR1&(1<<SPI_CR1_DFF)){
		pSPIHandle->pSPIx->DR=*((uint16_t*)pSPIHandle->pTxbuffer);
		(uint16_t*)pSPIHandle->pTxbuffer++;
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
	}
	else {
		pSPIHandle->pSPIx->DR=*(pSPIHandle->pTxbuffer);
		pSPIHandle->TxLen--;
	    pSPIHandle->pTxbuffer++;
	}

	if(pSPIHandle->TxLen==0){
	  SPI_CloseTransmisson(pSPIHandle);
	  SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}
}

static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle){

	//check the DFF format
	if(pSPIHandle->pSPIx->CR1&(1<<SPI_CR1_DFF)){
		*((uint16_t*)pSPIHandle->pRxbuffer)=(uint16_t)(pSPIHandle->pSPIx->DR);
		(uint16_t*)pSPIHandle->pRxbuffer++;
		pSPIHandle->RxLen--;
		pSPIHandle->RxLen--;
	}
	else {
		*(pSPIHandle->pRxbuffer)=pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
	    pSPIHandle->pRxbuffer++;
	}

	if(pSPIHandle->RxLen==0){
	  SPI_CloseReception(pSPIHandle);
	  SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}

static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}





void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}


void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	//1. first lets find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber %4 ;

	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASE_ADDR + iprx ) |=  ( IRQPriority << shift_amount );

}
