#include "usart.h"
#include "stm32f4xx.h"
#include "stdio.h"

//pritnf
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;

int fputc(int ch, FILE *f)
{ 	
	while((USART6->SR&0X40)==0);
	USART6->DR = (u8) ch;      
	return ch;
}
//

void usart6_dma_init(uint8_t *rx_buf, uint16_t dma_buf_num)
{

        /* -------------- Enable Module Clock Source ----------------------------*/
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG | RCC_AHB1Periph_DMA2, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, DISABLE);

        GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6); 	//PG9  	usart6 RX
        GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6); 	//PG14  usart6 TX
        /* -------------- Configure GPIO ---------------------------------------*/
        {
                GPIO_InitTypeDef GPIO_InitStructure;
                USART_InitTypeDef USART_InitStructure;
                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
                GPIO_Init(GPIOG, &GPIO_InitStructure);

                USART_DeInit(USART6);
                USART_InitStructure.USART_BaudRate = 115200;
                USART_InitStructure.USART_WordLength = USART_WordLength_8b;
                USART_InitStructure.USART_StopBits = USART_StopBits_1;
                USART_InitStructure.USART_Parity = USART_Parity_No;
                USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
                USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
                USART_Init(USART6, &USART_InitStructure);

                USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);

                USART_ClearFlag(USART6, USART_FLAG_IDLE);
                USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);

                USART_Cmd(USART6, ENABLE);
        }

        /* -------------- Configure NVIC ---------------------------------------*/
        {
                NVIC_InitTypeDef NVIC_InitStructure;
                NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = VISUAL_NVIC;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);
        }

        //DMA2 stream1 ch5 rx 
        /* -------------- Configure DMA -----------------------------------------*/
        {
                DMA_InitTypeDef DMA_InitStructure;

                //rx  DMA2 stream1 ch5
                DMA_DeInit(DMA2_Stream1);

                DMA_InitStructure.DMA_Channel            = DMA_Channel_5;                   //DMA??????5
                DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);       //????????????
                DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)rx_buf;                //???????????????
                DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;      //??????????????????
                DMA_InitStructure.DMA_BufferSize         = dma_buf_num;                     //???????????????
                DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;       //?????????????????????
                DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;            //?????????????????????
                DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //??????????????????????????????
                DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;         //?????????????????????????????????
                DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;               //????????????
                DMA_InitStructure.DMA_Priority           = DMA_Priority_Medium;             //???????????????
                DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;            //fifo??????
                DMA_InitStructure.DMA_FIFOThreshold      = DMA_FIFOThreshold_1QuarterFull;  //fifo??????
                DMA_InitStructure.DMA_MemoryBurst        = DMA_MemoryBurst_Single;          //???????????????????????????
                DMA_InitStructure.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;      //????????????????????????
                DMA_Init(DMA2_Stream1, &DMA_InitStructure);
								DMA_ITConfig(DMA2_Stream1,DMA_IT_TC,ENABLE);
                DMA_Cmd(DMA2_Stream1, DISABLE); 
                DMA_Cmd(DMA2_Stream1, ENABLE);
								
        }
}

void uart8_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE); //
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART8,ENABLE);//

	GPIO_PinAFConfig(GPIOE,GPIO_PinSource0,GPIO_AF_UART8); //
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource1,GPIO_AF_UART8); //

  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1; //
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //
	GPIO_Init(GPIOE,&GPIO_InitStructure); //

	USART_InitStructure.USART_BaudRate = 115200;//?????????
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//
	USART_InitStructure.USART_Parity = USART_Parity_No;//
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//
  USART_Init(UART8, &USART_InitStructure); //
	
  USART_Cmd(UART8, ENABLE);  //
	USART_ClearFlag(UART8, USART_FLAG_TC);
	USART_ITConfig(UART8, USART_IT_RXNE, ENABLE);//

	//Usart1 NVIC 
  NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;//
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//
	NVIC_Init(&NVIC_InitStructure);	//


}





