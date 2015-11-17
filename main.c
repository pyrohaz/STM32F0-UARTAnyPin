#include <stm32f0xx_rcc.h>
#include <stm32f0xx_exti.h>
#include <stm32f0xx_syscfg.h>
#include <stm32f0xx_usart.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_misc.h>
#include <stm32f0xx_tim.h>

/*
 * A software UART implementation using EXTI interrupts and a hardware timer
 *
 * Author: Harris Shallcross
 * Year: 17/11/2015
 *
 *
 *Code and example descriptions can be found on my blog at:
 *www.hsel.co.uk
 *
 *The MIT License (MIT)
 *Copyright (c) 2015 Harris Shallcross
 *
 *Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the "Software"), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *
 *The above copyright notice and this permission notice shall be included in all
 *copies or substantial portions of the Software.
 *
 *THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 */

//Uncomment the line below to have output pulses on PA1 at bit sampling periods
//#define BITPERIOD_DEBUG

//Baudrate used
#define USART_BAUDRATE		9600

//Baudrate tolerance, this is the timer counts AFTER the ideal bit period to sample at
#define USART_BRTOL			50

//Amount of bits in a transmission
#define USART_BITSPERTX		9

//Software UART pin defines
//Input pin
#define SUART_RX		GPIO_Pin_0
//Input pin GPIO
#define SUART_GPIO		GPIOA
//Input pin EXTI input
#define SUART_EXTILINE	EXTI_Line0
//Input pin EXTI port source
#define SUART_EXTIPOS	EXTI_PortSourceGPIOA
//Input pin EXTI pin source
#define SUART_EXTIPS	EXTI_PinSource0
//Input pin EXTI interrupt request
#define SUART_EXTIIRQ	EXTI0_1_IRQn

USART_InitTypeDef U;
EXTI_InitTypeDef E;
GPIO_InitTypeDef G;
NVIC_InitTypeDef N;
TIM_TimeBaseInitTypeDef T;
RCC_ClocksTypeDef R;

//Software UART Variables
volatile uint8_t U_Finished = 1;
volatile uint8_t UData = 0, ECnt = 0;
volatile uint32_t PeriodVal, BitPeriod, nBitPeriod = 0;

void EXTI0_1_IRQHandler(void){
	if(EXTI_GetITStatus(EXTI_Line0)){
		EXTI_ClearITPendingBit(EXTI_Line0);

#ifdef BITPERIOD_DEBUG
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
#endif

		//Only receive data if the finished flag is set!
		if(U_Finished){
			//Clear finished flag to indicate data reception
			U_Finished = 0;
			//Upon detection of start bit (initial falling edge)
			//Set current bit period to BitPeriod (nBitPeriod/BitPeriod = current bit)
			nBitPeriod = BitPeriod;

			//Set edge counter to zero and clear USART finished flag
			ECnt = 0;

			//Clear USART data variable
			UData = 0;

			//Disable edge interrupts during reception to avoid interrupts on every falling edge
			E.EXTI_LineCmd = DISABLE;
			EXTI->IMR &= ~EXTI_Line0;

#ifdef BITPERIOD_DEBUG
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);
#endif

			//Set next timer interrupt BitPeriod+USART_BRTOL timer counts ahead from now
			TIM_SetCompare1(TIM15, BitPeriod+USART_BRTOL);

			//Reset timer counter
			TIM_SetCounter(TIM15, 0);

			//Enable capture compare interrupt
			TIM_ITConfig(TIM15, TIM_IT_CC1, ENABLE);
		}
	}
}

void TIM15_IRQHandler(void){
	uint8_t C;
	if(TIM_GetITStatus(TIM15, TIM_IT_CC1)){
		TIM_ClearITPendingBit(TIM15, TIM_IT_CC1);

#ifdef BITPERIOD_DEBUG
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
#endif

		//Set next interrupt moment
		TIM_SetCompare1(TIM15, nBitPeriod+USART_BRTOL);

		//Increment the bit period accumulator
		nBitPeriod += BitPeriod;

		if(ECnt>7){
			//Once all bit periods have passed
			//Set USART reception finished flag
			U_Finished = 1;

			//Disable timer capture compare interrupt
			TIM_ITConfig(TIM15, TIM_IT_CC1, DISABLE);

			//Re-enable falling edge interrupts
			EXTI->IMR |= EXTI_Line0;
		}
		else{
			//Ignore first bit period as that belongs to start bit (always 0)
			if(ECnt>0){
				UData |= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)<<(ECnt-1);
			}

			//Increment current bit counter
			ECnt++;
		}

#ifdef BITPERIOD_DEBUG
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);
#endif
	}
}

//Test function to send USART bytes
void U_Tx(char C){
	USART_SendData(USART2, C);
	while(!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
	while(USART_GetFlagStatus(USART2, USART_FLAG_BUSY));
}

volatile uint32_t MSec = 0;
void SysTick_Handler(void){
	MSec++;
}

void Delay(uint32_t D){
	uint32_t MSS = MSec;
	while((MSec-MSS)<D) __NOP();
}

int main(void)
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_GetClocksFreq(&R);

	//NOTE: This pin is externally connected to PA2 as a loopback. PA2 generates USART TX data.
	//Initialize this pin as external interrupt input
	G.GPIO_Pin = SUART_RX;
	G.GPIO_Mode = GPIO_Mode_IN;
	G.GPIO_OType = GPIO_OType_PP;
	G.GPIO_PuPd = GPIO_PuPd_DOWN;
	G.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(SUART_GPIO, &G);


	//Pin used for monitoring the bit periods
#ifdef BITPERIOD_DEBUG
	G.GPIO_Pin = GPIO_Pin_1;
	G.GPIO_Mode = GPIO_Mode_OUT;
	G.GPIO_OType = GPIO_OType_PP;
	G.GPIO_PuPd = GPIO_PuPd_DOWN;
	G.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &G);
#endif

	//Initialize USART2 GPIO (PA2) as test USART output
	G.GPIO_Pin = GPIO_Pin_2;
	G.GPIO_Mode = GPIO_Mode_AF;
	G.GPIO_OType = GPIO_OType_PP;
	G.GPIO_PuPd = GPIO_PuPd_DOWN;
	G.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &G);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);

	U.USART_BaudRate = USART_BAUDRATE;
	U.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	U.USART_Mode = USART_Mode_Tx;
	U.USART_Parity = USART_Parity_No;
	U.USART_StopBits = USART_StopBits_1;
	U.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &U);
	USART_Cmd(USART2, ENABLE);

	//Initialize Timer15 as USART Input count timer
	T.TIM_ClockDivision = TIM_CKD_DIV1;
	T.TIM_CounterMode = TIM_CounterMode_Up;
	T.TIM_Period = 0xFFFF;
	T.TIM_Prescaler = 0;
	T.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM15, &T);

	TIM_ClearITPendingBit(TIM15, TIM_IT_CC1);
	TIM_ITConfig(TIM15, TIM_IT_CC1, DISABLE);


	//Calculate maximum period value
	PeriodVal = R.PCLK_Frequency*USART_BITSPERTX/USART_BAUDRATE;
	BitPeriod = PeriodVal/USART_BITSPERTX;

	//Initialize external interrupt on PA0
	E.EXTI_Line = SUART_EXTILINE;
	E.EXTI_Mode = EXTI_Mode_Interrupt;
	E.EXTI_Trigger = EXTI_Trigger_Falling;
	E.EXTI_LineCmd = ENABLE;
	EXTI_Init(&E);

	SYSCFG_EXTILineConfig(SUART_EXTIPOS, SUART_EXTIPS);

	N.NVIC_IRQChannel = SUART_EXTIIRQ;
	N.NVIC_IRQChannelPriority = 0;
	N.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&N);

	//Enable timer CC interrupt
	N.NVIC_IRQChannel = TIM15_IRQn;
	N.NVIC_IRQChannelPriority = 1;
	N.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&N);

	TIM_Cmd(TIM15, ENABLE);

	SysTick_Config(SystemCoreClock/1000);

	//Test transmission and reception
	volatile uint16_t DataAr[32] = {0}, C;

	//Initialization delay required otherwise first byte is incorrect!
	//I'm still figuring this one...
	Delay(10);

	for(C = 0; C<32; C++){
		U_Tx(C+10);
		while(!U_Finished);
		DataAr[C] = UData;
		U_Finished = 1;
	}


	while(1)
	{

	}
}
