/*
 * deca_spi.c
 *
 *  Created on: Aug 18, 2017
 *      Author: vedagupt
 */
#include "deca_spi.h"
#include "deca_types.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_device_api.h"






/*! ------------------------------------------------------------------------------------------------------------------
 * @fn readfromspi()
 *
 * @brief
 * NB: In porting this to a particular microprocessor, the implementer needs to define the two low
 * level abstract functions to write to and read from the SPI the definitions should be in deca_spi.c file.
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 *
 * Note: The body of this function is defined in deca_spi.c and is platform specific
 *
 * input parameters:
 * @param headerLength  - number of bytes header to write
 * @param headerBuffer  - pointer to buffer containing the 'headerLength' bytes of header to write
 * @param readlength    - number of bytes data being read
 * @param readBuffer    - pointer to buffer containing to return the data (NB: size required = headerLength + readlength)
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success (and the position in the buffer at which data begins), or DWT_ERROR for error
 */
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer){
	DW_SPI_START();
	for(uint16 i=0;i<headerLength;++i){
		DW_SPI_SEND(headerBuffer[i]);
	}
	for(uint32 i=0;i<readlength;++i){
		readBuffer[i] = DW_SPI_SEND(0x00);
	}
	DW_SPI_END();
	return DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn writetospi()
 *
 * @brief
 * NB: In porting this to a particular microprocessor, the implementer needs to define the two low
 * level abstract functions to write to and read from the SPI the definitions should be in deca_spi.c file.
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 *
 * Note: The body of this function is defined in deca_spi.c and is platform specific
 *
 * input parameters:
 * @param headerLength  - number of bytes header being written
 * @param headerBuffer  - pointer to buffer containing the 'headerLength' bytes of header to be written
 * @param bodylength    - number of bytes data being written
 * @param bodyBuffer    - pointer to buffer containing the 'bodylength' bytes od data to be written
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer){
	DW_SPI_START();
	for(uint16 i=0;i<headerLength;++i){
		DW_SPI_SEND(headerBuffer[i]);
	}
	for(uint32 i=0;i<bodylength;++i){
		DW_SPI_SEND(bodyBuffer[i]);
	}
	DW_SPI_END();
	return DWT_SUCCESS;
}






void DW_SPI_Init(int mode){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPI_InitTypeDef SPI_InitTypeDefStruct;

	SPI_InitTypeDefStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitTypeDefStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitTypeDefStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitTypeDefStruct.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitTypeDefStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitTypeDefStruct.SPI_NSS = SPI_NSS_Soft;
	if(mode==FAST_SPI)SPI_InitTypeDefStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	else SPI_InitTypeDefStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitTypeDefStruct.SPI_FirstBit = SPI_FirstBit_MSB;

	SPI_Init(SPI1, &SPI_InitTypeDefStruct);


	//Software slave select pin initialization
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOE , ENABLE);

	GPIO_InitTypeDef GPIO_InitTypeDefStruct;

	GPIO_InitTypeDefStruct.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7 | GPIO_Pin_6; //PA5- SCK, PA6- MISO, PA7- MOSI, PE7- SS
	GPIO_InitTypeDefStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitTypeDefStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitTypeDefStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitTypeDefStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitTypeDefStruct);

	GPIO_InitTypeDefStruct.GPIO_Pin = GPIO_Pin_7; //ACM, software slave select
	GPIO_InitTypeDefStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitTypeDefStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitTypeDefStruct.GPIO_PuPd = GPIO_PuPd_UP; //ACM pulling slave select high
	GPIO_InitTypeDefStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOE, &GPIO_InitTypeDefStruct);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);

	GPIO_SetBits(GPIOE, GPIO_Pin_7); //ACM setting slave select high

	SPI_Cmd(SPI1, ENABLE);

}


void DW_SPI_START(){
	GPIO_ResetBits(GPIOE, GPIO_Pin_7); //ACM setting slave select low
}

void DW_SPI_END(){
	GPIO_SetBits(GPIOE, GPIO_Pin_7); //ACM setting slave select low
}

uint8_t DW_SPI_SEND(uint8_t data){
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE));
	SPI_I2S_SendData(SPI1, data);
	while(!SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE));

	return SPI_I2S_ReceiveData(SPI1);
}




