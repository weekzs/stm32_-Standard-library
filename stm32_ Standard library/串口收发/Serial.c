#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
extern int flag;
uint8_t Serial_RxData;
uint8_t Serial_RxFlag;
//void MX_UART2_Init(void)
//{
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//    GPIO_InitTypeDef GPIO_InitStructure;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //RX
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);

//    //Tx
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);

//    USART_InitTypeDef huart2;
//    huart2.USART_BaudRate = 230400;
//    huart2.USART_WordLength = USART_WordLength_8b;
//    huart2.USART_StopBits = USART_StopBits_1;
//    huart2.USART_Parity = USART_Parity_No;
//    huart2.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
//    huart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_Init(USART2, &huart2);
//    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
//    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//    NVIC_InitTypeDef NVIC_InitStructure;
//    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_Init(&NVIC_InitStructure);
//    USART_Cmd(USART2, ENABLE);
//}
void Serial_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //Tx
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitTypeDef huart3;
    huart3.USART_BaudRate = 115200;
    huart3.USART_WordLength = USART_WordLength_8b;
    huart3.USART_StopBits = USART_StopBits_1;
    huart3.USART_Parity = USART_Parity_No;
    huart3.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    huart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART3, &huart3);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    USART_Cmd(USART3, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART3, Byte);
	while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

uint8_t Serial_GetRxData(void)
{
	return Serial_RxData;
}

void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
	//	Serial_RxData=3;
		Serial_RxData = USART_ReceiveData(USART3);
		Serial_RxFlag = 1;
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		//Serial_SendByte(Serial_RxData);
		
	//	flag=3;
	}
}
