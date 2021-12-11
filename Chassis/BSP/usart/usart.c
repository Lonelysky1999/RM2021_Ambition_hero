#include "usart.h"
#include "stm32f4xx.h"



void usart2_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); //
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); //

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_5; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //
	GPIO_Init(GPIOD,&GPIO_InitStructure); //

	USART_InitStructure.USART_BaudRate = 115200;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//
	USART_InitStructure.USART_Parity = USART_Parity_No;//
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//
  USART_Init(USART2, &USART_InitStructure); //
	
  USART_Cmd(USART2, ENABLE);  //
	USART_ClearFlag(USART2, USART_FLAG_TC);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//

	//Usart1 NVIC 
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//
	NVIC_Init(&NVIC_InitStructure);	//

}



void usart3_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);//

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //
	GPIO_Init(GPIOD,&GPIO_InitStructure); //

	USART_InitStructure.USART_BaudRate = 115200;//波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//
	USART_InitStructure.USART_Parity = USART_Parity_No;//
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//
  USART_Init(USART3, &USART_InitStructure); //
	
  USART_Cmd(USART3, ENABLE);  //
	USART_ClearFlag(USART3, USART_FLAG_TC);
//	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//



}


