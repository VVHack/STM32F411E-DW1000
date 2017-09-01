/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

/* Pins to use:
 * PA5- SCK
 * PA6- MISO
 * PA7- MOSI
 * PE7- SS
 * PA1- IRQ
*/



#include "stm32f4xx.h"
#include "deca_led.h"
#include <stdbool.h>
#include "deca_spi.h"
#include <stdio.h>
#include <string.h>
#include "deca_device_api.h"
#include "deca_regs.h"

#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255

#define TX_DELAY_MS 1000
#define BLINK_FRAME_SN_IDX 1

/* Buffer to store received frame. */
#define FRAME_LEN_MAX 127

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;


/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 poll_rx_ts;
static uint64 pollack_tx_ts;
static uint64 pollack_rx_ts;
static uint64 range_tx_ts;
static uint64 range_rx_ts;

double range = 0;

#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];
static uint8 tx_data[20];

bool sent = false, received = false, error = false;

/* Hold copy of frame length of frame received (if good) so that it can be examined at a debug breakpoint. */
static uint16 frame_len = 0;

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define ANTD 16500
#define TX_ANT_DLY ANTD
#define RX_ANT_DLY ANTD
//16436, 16384 on Arduino

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 poll_rx_ts;
static uint64 pollack_tx_ts;
static uint64 pollack_rx_ts;
static uint64 range_tx_ts;
static uint64 range_rx_ts;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static uint64 get_ts_inu64(uint8* ts_tab);

//Arduino
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_16M,     /* Pulse repetition frequency. */
	DWT_PLEN_2048,   /* Preamble length. Used in TX only. */
    DWT_PAC8,       /* Preamble acquisition chunk size. Used in RX only. */
    4,               /* TX preamble code. Used in TX only. */
    4,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (2049 + 64 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};


void basicSpiTest();
void setup();
void transmit(uint8 msg[], uint8 size);
void receive();
void checkSysStatus();
void setup();
void transmitPoll();
void transmitRange();
void anchor();
void tag();

#define errorTimeout 9000

int main(void)
{

	setup();

	/* Apply default antenna delay value. See NOTE 1 below. */
	dwt_setrxantennadelay(RX_ANT_DLY);
	dwt_settxantennadelay(TX_ANT_DLY);


	anchor();
	//tag();

//	char helloMsg[] = "Hi Arduino\0";
//	while(1){
//		transmit(helloMsg, sizeof(helloMsg));
//	}


}


void anchor(){
	while (1)
	{

		//TRANSMIT POLL
		tx_data[0] = POLL;
		int rangeInt = range;
		double rangeDec = 100.0 * (range - rangeInt);
		tx_data[1] = rangeInt;
		tx_data[2] = rangeDec;
		transmit(tx_data, sizeof(tx_data));
		if(error){
			error = false;
			dwt_forcetrxoff();
			dwt_rxreset();
			continue;
		}

		poll_tx_ts = get_tx_timestamp_u64();

		//RECEIVE POLL_ACK
		receive();
		if(error || rx_buffer[0]!=POLL_ACK){
			error = false;
			dwt_forcetrxoff();
			dwt_rxreset();
			continue;
		}

		pollack_rx_ts = get_rx_timestamp_u64();

		//TRANSMIT RANGE
		tx_data[0] = RANGE;
		transmit(tx_data, sizeof(tx_data));

		if(error){
			error = false;
			dwt_forcetrxoff();
			dwt_rxreset();
			continue;
		}


		range_tx_ts = get_tx_timestamp_u64();

		//RECEIVE RANGE_REPORT
		receive();
		if(error || rx_buffer[0]!= RANGE_REPORT){
			error = false;
			dwt_forcetrxoff();
			dwt_rxreset();
			continue;
		}
		poll_rx_ts = get_ts_inu64(rx_buffer+1);
		pollack_tx_ts = get_ts_inu64(rx_buffer+6);
		range_rx_ts = get_ts_inu64(rx_buffer+11);
		uint64 round1 = pollack_rx_ts - poll_tx_ts;
		uint64 reply1 = pollack_tx_ts - poll_rx_ts;
		uint64 round2 = range_rx_ts - pollack_tx_ts;
		uint64 reply2 = range_tx_ts - pollack_rx_ts;
		double tof = (double)(round1*round2 - reply1*reply2) / (double)(round1+round2+reply1+reply2);
		//tof = (double)(round1-reply1)/2;
		range = tof * 0.0046917639786159;
		//blinkOrange();
	}
}

void tag(){
	while(1){
		//RECEIVE POLL
		receive();
		if(error || rx_buffer[0]!=POLL){
			error = false;
			dwt_forcetrxoff();
			dwt_rxreset();
			continue;
		}

		dwt_readrxtimestamp(tx_data+1); //getting timestamp data into the transmit buffer

		//TRANSMIT POLL_ACK
		tx_data[0] = POLL_ACK;
		transmit(tx_data, sizeof(tx_data));

		if(error){
			error = false;
			dwt_forcetrxoff();
			dwt_rxreset();
			continue;
		}


		dwt_readtxtimestamp(tx_data+6); //getting timestamp data into the transmit buffer

		//RECEIVE RANGE
		receive();
		if(error || rx_buffer[0]!=RANGE){
			error = false;
			dwt_forcetrxoff();
			dwt_rxreset();
			continue;
		}

		dwt_readrxtimestamp(tx_data+11); //getting timestamp data into the transmit buffer

		//TRANSMIT RANGE_REPORT
		tx_data[0] = RANGE_REPORT;
		transmit(tx_data, sizeof(tx_data));

		if(error){
			error = false;
			dwt_forcetrxoff();
			dwt_rxreset();
			continue;
		}
	}
}


void basicSpiTest(){
	while (1)
	{
		DW_SPI_START();
		DW_SPI_SEND(0x40);
		DW_SPI_SEND(0x02);
		uint8_t arr[] = {0,0,0,0};
		arr[0] = DW_SPI_SEND(0);
		arr[1] = DW_SPI_SEND(0);
		//arr[2] = DW_SPI_SEND(0);
		//arr[3] = DW_SPI_SEND(0);

		DW_SPI_END();



		displayWithLED(arr[0]);
		displayWithLED(arr[1]);
		//displayWithLED(arr[2]);
		//displayWithLED(arr[3]);

		delayMiliseconds(3000);
	}
}

void transmitPoll(){
	tx_data[0] = POLL;
	int rangeInt = range;
	double rangeDec = 100.0 * (range - rangeInt);
	tx_data[1] = rangeInt;
	tx_data[2] = rangeDec;
	transmit(tx_data, sizeof(tx_data));
}

void transmitRange(){
	tx_data[0] = RANGE;
	transmit(tx_data, sizeof(tx_data));
}


//ISR
void EXTI1_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line1)!=RESET){
		checkSysStatus();
		EXTI_ClearITPendingBit(EXTI_Line1);
	}
}

void transmit(uint8 msg[], uint8 size){
	/* Write frame data to DW1000 and prepare transmission. See NOTE 4 below.*/
	dwt_writetxdata(size, msg, 0); /* Zero offset in TX buffer. */
	dwt_writetxfctrl(size, 0, 1); /* Zero offset in TX buffer,  ranging. */

	/* Start transmission. */
	dwt_starttx(DWT_START_TX_IMMEDIATE);
	int count = 0;
	while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
	{
		if(++count>errorTimeout){
			error = true;
			return;
		}
	};
	sent = true;
	dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
	//blinkBlue();
}

void receive(){
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
	int count = 0;
	while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
	{
		if(++count>errorTimeout){
			error = true;
			return;
		}
	};
	if(status_reg & SYS_STATUS_RXFCG){
			received = true;
			//dwt_readrxtimestamp(timeReceived);
			//dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
			frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
			if (frame_len <= FRAME_LEN_MAX)
			{
				dwt_readrxdata(rx_buffer, frame_len, 0);
			}
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
			//blinkOrange();
	}
	if(status_reg & (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO)){
			error = true;
			dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
			//blinkRed();
	}
}

void checkSysStatus(){
	status_reg = dwt_read32bitreg(SYS_STATUS_ID);
	if(status_reg==0xFFFFFFFF){
		return;
	}
	if(status_reg & SYS_STATUS_TXFRS){
		sent = true;
		//dwt_readtxtimestamp(timeSent);
		//dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
		//blinkBlue();
	}
	if(status_reg & SYS_STATUS_RXFCG){
		received = true;
		//dwt_readrxtimestamp(timeReceived);
		//dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
		frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
		if (frame_len <= FRAME_LEN_MAX)
		{
			dwt_readrxdata(rx_buffer, frame_len, 0);
		}
		//blinkOrange();
	}
	if(status_reg & (SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO)){
		error = true;
		//dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
		//blinkRed();
	}
	dwt_write32bitreg(SYS_STATUS_ID, 0xFFFFFFFF);
}


void setup(){
	led_init();
	DW_SPI_Init(SLOW_SPI);

	//basicSpiTest();

	if(dwt_initialise(DWT_LOADUCODE) == DWT_ERROR){
		GPIO_WriteBit(GPIOD, GPIO_Pin_14, Bit_SET); //switch on RED LED
		while(1){}
	}

	DW_SPI_Init(FAST_SPI);

	//basicSpiTest();

	dwt_configure(&config);
	//THERE IS A FUNCTION CALLED dwt_isr


	initializeInterrupts();

	//dwt_setinterrupt(SYS_STATUS_TXFRS, true);
//	dwt_setinterrupt(SYS_STATUS_TXFRS | DWT_INT_RFCG | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_TO, true);
//	dwt_setinterrupt(DWT_INT_RFCG, true);
//	dwt_setinterrupt(SYS_STATUS_ALL_RX_ERR, true);


	dwt_write32bitreg(SYS_STATUS_ID,0xFFFFFFFF);
}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static uint64 get_ts_inu64(uint8* ts_tab){
	uint64 ts = 0;
	for (int i = 4; i >= 0; i--)
	{
		ts <<= 8;
		ts |= ts_tab[i];
	}
	return ts;
}

//This assumes that the clock for GPIO Port A has already been started
void initializeInterrupts(){
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	EXTI_InitTypeDef EXTI_InitSwitch;
	NVIC_InitTypeDef NVIC_InitSwitch;
	//Clock for port A initialized when initializing SPI
	GPIO_InitTypeDef GPIO_LED;

	GPIO_LED.GPIO_Pin = GPIO_Pin_1;
	GPIO_LED.GPIO_Mode = GPIO_Mode_IN;
	GPIO_LED.GPIO_OType = GPIO_OType_PP;
	GPIO_LED.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_LED.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_Init(GPIOA, &GPIO_LED);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource1);

	EXTI_InitSwitch.EXTI_Line = EXTI_Line1;
	EXTI_InitSwitch.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitSwitch.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitSwitch.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitSwitch);

	NVIC_InitSwitch.NVIC_IRQChannel = EXTI1_IRQn;
	NVIC_InitSwitch.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitSwitch.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitSwitch.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitSwitch);
}

