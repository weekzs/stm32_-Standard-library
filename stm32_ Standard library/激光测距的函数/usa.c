/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usa.h"
#include "stm32f10x.h"

uint8_t Uart_Receive_buf[4][1];




/* UART4 init function 

T1
*/

void MX_UART4_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    //Tx
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitTypeDef huart4;
    huart4.USART_BaudRate = 230400;
    huart4.USART_WordLength = USART_WordLength_8b;
    huart4.USART_StopBits = USART_StopBits_1;
    huart4.USART_Parity = USART_Parity_No;
    huart4.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    huart4.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART4, &huart4);
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;
    NVIC_Init(&NVIC_InitStructure);
    USART_Cmd(UART4, ENABLE);
}

void MX_UART5_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    //Tx
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitTypeDef huart5;
    huart5.USART_BaudRate = 230400;
    huart5.USART_WordLength = USART_WordLength_8b;
    huart5.USART_StopBits = USART_StopBits_1;
    huart5.USART_Parity = USART_Parity_No;
    huart5.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    huart5.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(UART5, &huart5);
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    USART_Cmd(UART5, ENABLE);
}

void MX_UART2_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;   //RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //Tx
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef huart2;
    huart2.USART_BaudRate = 230400;
    huart2.USART_WordLength = USART_WordLength_8b;
    huart2.USART_StopBits = USART_StopBits_1;
    huart2.USART_Parity = USART_Parity_No;
    huart2.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    huart2.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &huart2);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
    USART_Cmd(USART2, ENABLE);
}


/* USER CODE BEGIN 1 */
LidarPointTypedef Pack_Data[4][12];      /* ???????????????? */
LidarPointTypedef Pack_sum[4];           /* ?????? */
uint16_t receive_cnt[4] = {0,0,0,0};
uint8_t confidence[4];
uint16_t distance[4];
uint16_t noise[4],reftof[4];
uint32_t peak[4],intg[4];

uint8_t state[4] = {0,0,0,0};			//???
uint8_t crc[4] = {0,0,0,0};			//???
uint8_t cnt[4] = {0,0,0,0};			//????12?????
uint8_t PACK_FLAG[4] = {0,0,0,0};       //?????
uint8_t data_len[4]  = {0,0,0,0};       //????
uint32_t timestamp[4] = {0,0,0,0};      //???
uint8_t state_flag[4] = {1,1,1,1};      //?????????
uint8_t state_num[4] = {0,0,0,0};

void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        Uart_Receive_buf[USART2_Index][0] = USART_ReceiveData(USART2);

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }

    uint8_t temp_data;
    uint8_t USART_num=10;   //??2?3?4?5??????0?1?2?3
    USART_num=USART2_Index;



    if(USART_num <= 3)
    {
        temp_data=Uart_Receive_buf[USART_num][0];
        if(state[USART_num]< 4) 																					 /* ????? ?4?????0xAA */
        {
            if(temp_data == HEADER)
                state[USART_num] ++;
            else state[USART_num] = 0;
        }
        else if(state[USART_num]<10 && state[USART_num]>3)
        {
            switch(state[USART_num])
            {
            case 4:
                if(temp_data == device_address)              /* ?????? */
                {
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else
                {
                    state[USART_num] = 0;
                    crc[USART_num] = 0;
                }
                break;
            case 5:
                if(temp_data == PACK_GET_DISTANCE)			/* ???????? */
                {
                    PACK_FLAG[USART_num] = PACK_GET_DISTANCE;
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else if(temp_data == PACK_RESET_SYSTEM) 		 /* ???? */
                {
                    PACK_FLAG[USART_num] = PACK_RESET_SYSTEM;
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else if(temp_data == PACK_STOP)							 /* ?????????? */
                {
                    PACK_FLAG[USART_num] = PACK_STOP;
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else if(temp_data == PACK_ACK)							 /* ????? */
                {
                    PACK_FLAG[USART_num] = PACK_ACK;
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else if(temp_data == PACK_VERSION)					 /* ????????? */
                {
                    PACK_FLAG[USART_num] = PACK_VERSION,
                                           state[USART_num] ++,
                                           crc[USART_num] = crc[USART_num] + temp_data;
                }
                else
                {
                    state[USART_num] = 0;
                    crc[USART_num] = 0;
                }
                break;
            case 6:
                if(temp_data == chunk_offset)          /* ???? */
                {
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else
                {
                    state[USART_num] = 0;
                    crc[USART_num] = 0;
                }
                break;
            case 7:
                if(temp_data == chunk_offset)
                {
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else
                {
                    state[USART_num] = 0,crc[USART_num] = 0;
                }
                break;
            case 8:
                data_len[USART_num] = (u16)temp_data;								 /* ??????? */
                state[USART_num] ++;
                crc[USART_num] = crc[USART_num] + temp_data;
                break;
            case 9:
                data_len[USART_num] = data_len[USART_num] + ((u16)temp_data<<8); 			 /* ??????? */
                state[USART_num] ++;
                crc[USART_num] = crc[USART_num] + temp_data;
                break;
            default:
                break;
            }
        }
        else if(state[USART_num] == 10 ) state_flag[USART_num] = 0;                    /*?switch????state?10,?temp_data???????????,???????*/
        if(PACK_FLAG[USART_num] == PACK_GET_DISTANCE && state_flag[USART_num] == 0)      /* ????????? */
        {
            if(state[USART_num]>9)
            {
                if(state[USART_num]<190)
                {
                    state_num[USART_num] = (state[USART_num]-10)%15;
                    switch(state_num[USART_num])
                    {
                    case 0:
                        Pack_Data[USART_num][cnt[USART_num]].distance = (uint16_t)temp_data ;				 /* ??????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 1:
                        Pack_Data[USART_num][cnt[USART_num]].distance = ((u16)temp_data<<8) + Pack_Data[USART_num][cnt[USART_num]].distance;	 /* ???? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 2:
                        Pack_Data[USART_num][cnt[USART_num]].noise = (u16)temp_data;				 /* ??????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 3:
                        Pack_Data[USART_num][cnt[USART_num]].noise = ((u16)temp_data<<8) + Pack_Data[USART_num][cnt[USART_num]].noise;				 /* ???? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 4:
                        Pack_Data[USART_num][cnt[USART_num]].peak = (u32)temp_data;				 										 /* ????????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 5:
                        Pack_Data[USART_num][cnt[USART_num]].peak = ((u32)temp_data<<8) + Pack_Data[USART_num][cnt[USART_num]].peak;
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 6:
                        Pack_Data[USART_num][cnt[USART_num]].peak = ((u32)temp_data<<16) + Pack_Data[USART_num][cnt[USART_num]].peak;
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 7:
                        Pack_Data[USART_num][cnt[USART_num]].peak = ((u32)temp_data<<24) + Pack_Data[USART_num][cnt[USART_num]].peak;				    /* ?????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 8:
                        Pack_Data[USART_num][cnt[USART_num]].confidence = temp_data;				 /* ??? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 9:
                        Pack_Data[USART_num][cnt[USART_num]].intg = (u32)temp_data;															/* ??????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 10:
                        Pack_Data[USART_num][cnt[USART_num]].intg = ((u32)temp_data<<8) + Pack_Data[USART_num][cnt[USART_num]].intg;
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 11:
                        Pack_Data[USART_num][cnt[USART_num]].intg = ((u32)temp_data<<16) + Pack_Data[USART_num][cnt[USART_num]].intg;
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 12:
                        Pack_Data[USART_num][cnt[USART_num]].intg = ((u32)temp_data<<24) + Pack_Data[USART_num][cnt[USART_num]].intg;				  	 /* ???? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 13:
                        Pack_Data[USART_num][cnt[USART_num]].reftof = (int16_t)temp_data;				 								 /* ???????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 14:
                        Pack_Data[USART_num][cnt[USART_num]].reftof = ((int16_t)temp_data<<8) +Pack_Data[USART_num][cnt[USART_num]].reftof;			/* ????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        cnt[USART_num]++;							 /* ???????? */
                        break;
                    default:
                        break;
                    }
                }
                /* ??? */
                if(state[USART_num] == 190) timestamp[USART_num] = temp_data,state[USART_num]++,crc[USART_num] = crc[USART_num] + temp_data;
                else if(state[USART_num] == 191) timestamp[USART_num] = ((u32)temp_data<<8) + timestamp[USART_num],state[USART_num]++,crc[USART_num] = crc[USART_num] + temp_data;
                else if(state[USART_num] == 192) timestamp[USART_num] = ((u32)temp_data<<16) + timestamp[USART_num],state[USART_num]++,crc[USART_num] = crc[USART_num] + temp_data;
                else if(state[USART_num] == 193) timestamp[USART_num] = ((u32)temp_data<<24) + timestamp[USART_num],state[USART_num]++,crc[USART_num] = crc[USART_num] + temp_data;
                else if(state[USART_num] == 194)
                {
                    if(temp_data == crc[USART_num])   /* ???? *///feng:???????,?????????
                    {
                        data_process(USART_num);  	 /* ??????,????????????? */
                        receive_cnt[USART_num]++;	 	 /* ???????????? */
                    }
                    distance[USART_num] = Pack_Data[USART_num][0].distance;
                    crc[USART_num] = 0;
                    state[USART_num] = 0;
                    state_flag[USART_num] = 1;
                    cnt[USART_num] = 0; 				/* ??*/
                }
            }
        }
        /*switch(USART_num)
        {
        	case USART2_Index:HAL_UART_Receive_IT(&huart2,Uart_Receive_buf[USART_num],sizeof(Uart_Receive_buf[USART_num]));
        	case USART3_Index:HAL_UART_Receive_IT(&huart3,Uart_Receive_buf[USART_num],sizeof(Uart_Receive_buf[USART_num]));
        	case USART4_Index:HAL_UART_Receive_IT(&huart4,Uart_Receive_buf[USART_num],sizeof(Uart_Receive_buf[USART_num]));
        	case USART5_Index:HAL_UART_Receive_IT(&huart5,Uart_Receive_buf[USART_num],sizeof(Uart_Receive_buf[USART_num]));
        	default:break;
        }*/
    }
}
void UART4_IRQHandler(void)
{
    if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
    {
        Uart_Receive_buf[USART4_Index][0] = USART_ReceiveData(UART4);

        USART_ClearITPendingBit(UART4, USART_IT_RXNE);
    }

    uint8_t temp_data;
    uint8_t USART_num=10;   //??2?3?4?5??????0?1?2?3
    USART_num=USART4_Index;



    if(USART_num <= 3)
    {
        temp_data=Uart_Receive_buf[USART_num][0];
        if(state[USART_num]< 4) 																					 /* ????? ?4?????0xAA */
        {
            if(temp_data == HEADER)
                state[USART_num] ++;
            else state[USART_num] = 0;
        }
        else if(state[USART_num]<10 && state[USART_num]>3)
        {
            switch(state[USART_num])
            {
            case 4:
                if(temp_data == device_address)              /* ?????? */
                {
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else
                {
                    state[USART_num] = 0;
                    crc[USART_num] = 0;
                }
                break;
            case 5:
                if(temp_data == PACK_GET_DISTANCE)			/* ???????? */
                {
                    PACK_FLAG[USART_num] = PACK_GET_DISTANCE;
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else if(temp_data == PACK_RESET_SYSTEM) 		 /* ???? */
                {
                    PACK_FLAG[USART_num] = PACK_RESET_SYSTEM;
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else if(temp_data == PACK_STOP)							 /* ?????????? */
                {
                    PACK_FLAG[USART_num] = PACK_STOP;
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else if(temp_data == PACK_ACK)							 /* ????? */
                {
                    PACK_FLAG[USART_num] = PACK_ACK;
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else if(temp_data == PACK_VERSION)					 /* ????????? */
                {
                    PACK_FLAG[USART_num] = PACK_VERSION,
                                           state[USART_num] ++,
                                           crc[USART_num] = crc[USART_num] + temp_data;
                }
                else
                {
                    state[USART_num] = 0;
                    crc[USART_num] = 0;
                }
                break;
            case 6:
                if(temp_data == chunk_offset)          /* ???? */
                {
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else
                {
                    state[USART_num] = 0;
                    crc[USART_num] = 0;
                }
                break;
            case 7:
                if(temp_data == chunk_offset)
                {
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else
                {
                    state[USART_num] = 0,crc[USART_num] = 0;
                }
                break;
            case 8:
                data_len[USART_num] = (u16)temp_data;								 /* ??????? */
                state[USART_num] ++;
                crc[USART_num] = crc[USART_num] + temp_data;
                break;
            case 9:
                data_len[USART_num] = data_len[USART_num] + ((u16)temp_data<<8); 			 /* ??????? */
                state[USART_num] ++;
                crc[USART_num] = crc[USART_num] + temp_data;
                break;
            default:
                break;
            }
        }
        else if(state[USART_num] == 10 ) state_flag[USART_num] = 0;                    /*?switch????state?10,?temp_data???????????,???????*/
        if(PACK_FLAG[USART_num] == PACK_GET_DISTANCE && state_flag[USART_num] == 0)      /* ????????? */
        {
            if(state[USART_num]>9)
            {
                if(state[USART_num]<190)
                {
                    state_num[USART_num] = (state[USART_num]-10)%15;
                    switch(state_num[USART_num])
                    {
                    case 0:
                        Pack_Data[USART_num][cnt[USART_num]].distance = (uint16_t)temp_data ;				 /* ??????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 1:
                        Pack_Data[USART_num][cnt[USART_num]].distance = ((u16)temp_data<<8) + Pack_Data[USART_num][cnt[USART_num]].distance;	 /* ???? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 2:
                        Pack_Data[USART_num][cnt[USART_num]].noise = (u16)temp_data;				 /* ??????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 3:
                        Pack_Data[USART_num][cnt[USART_num]].noise = ((u16)temp_data<<8) + Pack_Data[USART_num][cnt[USART_num]].noise;				 /* ???? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 4:
                        Pack_Data[USART_num][cnt[USART_num]].peak = (u32)temp_data;				 										 /* ????????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 5:
                        Pack_Data[USART_num][cnt[USART_num]].peak = ((u32)temp_data<<8) + Pack_Data[USART_num][cnt[USART_num]].peak;
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 6:
                        Pack_Data[USART_num][cnt[USART_num]].peak = ((u32)temp_data<<16) + Pack_Data[USART_num][cnt[USART_num]].peak;
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 7:
                        Pack_Data[USART_num][cnt[USART_num]].peak = ((u32)temp_data<<24) + Pack_Data[USART_num][cnt[USART_num]].peak;				    /* ?????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 8:
                        Pack_Data[USART_num][cnt[USART_num]].confidence = temp_data;				 /* ??? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 9:
                        Pack_Data[USART_num][cnt[USART_num]].intg = (u32)temp_data;															/* ??????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 10:
                        Pack_Data[USART_num][cnt[USART_num]].intg = ((u32)temp_data<<8) + Pack_Data[USART_num][cnt[USART_num]].intg;
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 11:
                        Pack_Data[USART_num][cnt[USART_num]].intg = ((u32)temp_data<<16) + Pack_Data[USART_num][cnt[USART_num]].intg;
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 12:
                        Pack_Data[USART_num][cnt[USART_num]].intg = ((u32)temp_data<<24) + Pack_Data[USART_num][cnt[USART_num]].intg;				  	 /* ???? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 13:
                        Pack_Data[USART_num][cnt[USART_num]].reftof = (int16_t)temp_data;				 								 /* ???????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 14:
                        Pack_Data[USART_num][cnt[USART_num]].reftof = ((int16_t)temp_data<<8) +Pack_Data[USART_num][cnt[USART_num]].reftof;			/* ????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        cnt[USART_num]++;							 /* ???????? */
                        break;
                    default:
                        break;
                    }
                }
                /* ??? */
                if(state[USART_num] == 190) timestamp[USART_num] = temp_data,state[USART_num]++,crc[USART_num] = crc[USART_num] + temp_data;
                else if(state[USART_num] == 191) timestamp[USART_num] = ((u32)temp_data<<8) + timestamp[USART_num],state[USART_num]++,crc[USART_num] = crc[USART_num] + temp_data;
                else if(state[USART_num] == 192) timestamp[USART_num] = ((u32)temp_data<<16) + timestamp[USART_num],state[USART_num]++,crc[USART_num] = crc[USART_num] + temp_data;
                else if(state[USART_num] == 193) timestamp[USART_num] = ((u32)temp_data<<24) + timestamp[USART_num],state[USART_num]++,crc[USART_num] = crc[USART_num] + temp_data;
                else if(state[USART_num] == 194)
                {
                    if(temp_data == crc[USART_num])   /* ???? *///feng:???????,?????????
                    {
                        data_process(USART_num);  	 /* ??????,????????????? */
                        receive_cnt[USART_num]++;	 	 /* ???????????? */
                    }
                    distance[USART_num] = Pack_Data[USART_num][0].distance;
                    crc[USART_num] = 0;
                    state[USART_num] = 0;
                    state_flag[USART_num] = 1;
                    cnt[USART_num] = 0; 				/* ??*/
                }
            }
        }
        /*switch(USART_num)
        {
        	case USART2_Index:HAL_UART_Receive_IT(&huart2,Uart_Receive_buf[USART_num],sizeof(Uart_Receive_buf[USART_num]));
        	case USART3_Index:HAL_UART_Receive_IT(&huart3,Uart_Receive_buf[USART_num],sizeof(Uart_Receive_buf[USART_num]));
        	case USART4_Index:HAL_UART_Receive_IT(&huart4,Uart_Receive_buf[USART_num],sizeof(Uart_Receive_buf[USART_num]));
        	case USART5_Index:HAL_UART_Receive_IT(&huart5,Uart_Receive_buf[USART_num],sizeof(Uart_Receive_buf[USART_num]));
        	default:break;
        }*/
    }
}

void UART5_IRQHandler(void)
{
    if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
    {
        Uart_Receive_buf[USART5_Index][0] = USART_ReceiveData(UART5);

        USART_ClearITPendingBit(UART5, USART_IT_RXNE);
    }

    uint8_t temp_data;
    uint8_t USART_num=10;   //??2?3?4?5??????0?1?2?3
    USART_num=USART5_Index;



    if(USART_num <= 3)
    {
        temp_data=Uart_Receive_buf[USART_num][0];
        if(state[USART_num]< 4) 																					 /* ????? ?4?????0xAA */
        {
            if(temp_data == HEADER)
                state[USART_num] ++;
            else state[USART_num] = 0;
        }
        else if(state[USART_num]<10 && state[USART_num]>3)
        {
            switch(state[USART_num])
            {
            case 4:
                if(temp_data == device_address)              /* ?????? */
                {
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else
                {
                    state[USART_num] = 0;
                    crc[USART_num] = 0;
                }
                break;
            case 5:
                if(temp_data == PACK_GET_DISTANCE)			/* ???????? */
                {
                    PACK_FLAG[USART_num] = PACK_GET_DISTANCE;
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else if(temp_data == PACK_RESET_SYSTEM) 		 /* ???? */
                {
                    PACK_FLAG[USART_num] = PACK_RESET_SYSTEM;
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else if(temp_data == PACK_STOP)							 /* ?????????? */
                {
                    PACK_FLAG[USART_num] = PACK_STOP;
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else if(temp_data == PACK_ACK)							 /* ????? */
                {
                    PACK_FLAG[USART_num] = PACK_ACK;
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else if(temp_data == PACK_VERSION)					 /* ????????? */
                {
                    PACK_FLAG[USART_num] = PACK_VERSION,
                                           state[USART_num] ++,
                                           crc[USART_num] = crc[USART_num] + temp_data;
                }
                else
                {
                    state[USART_num] = 0;
                    crc[USART_num] = 0;
                }
                break;
            case 6:
                if(temp_data == chunk_offset)          /* ???? */
                {
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else
                {
                    state[USART_num] = 0;
                    crc[USART_num] = 0;
                }
                break;
            case 7:
                if(temp_data == chunk_offset)
                {
                    state[USART_num] ++;
                    crc[USART_num] = crc[USART_num] + temp_data;
                }
                else
                {
                    state[USART_num] = 0,crc[USART_num] = 0;
                }
                break;
            case 8:
                data_len[USART_num] = (u16)temp_data;								 /* ??????? */
                state[USART_num] ++;
                crc[USART_num] = crc[USART_num] + temp_data;
                break;
            case 9:
                data_len[USART_num] = data_len[USART_num] + ((u16)temp_data<<8); 			 /* ??????? */
                state[USART_num] ++;
                crc[USART_num] = crc[USART_num] + temp_data;
                break;
            default:
                break;
            }
        }
        else if(state[USART_num] == 10 ) state_flag[USART_num] = 0;                    /*?switch????state?10,?temp_data???????????,???????*/
        if(PACK_FLAG[USART_num] == PACK_GET_DISTANCE && state_flag[USART_num] == 0)      /* ????????? */
        {
            if(state[USART_num]>9)
            {
                if(state[USART_num]<190)
                {
                    state_num[USART_num] = (state[USART_num]-10)%15;
                    switch(state_num[USART_num])
                    {
                    case 0:
                        Pack_Data[USART_num][cnt[USART_num]].distance = (uint16_t)temp_data ;				 /* ??????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 1:
                        Pack_Data[USART_num][cnt[USART_num]].distance = ((u16)temp_data<<8) + Pack_Data[USART_num][cnt[USART_num]].distance;	 /* ???? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 2:
                        Pack_Data[USART_num][cnt[USART_num]].noise = (u16)temp_data;				 /* ??????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 3:
                        Pack_Data[USART_num][cnt[USART_num]].noise = ((u16)temp_data<<8) + Pack_Data[USART_num][cnt[USART_num]].noise;				 /* ???? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 4:
                        Pack_Data[USART_num][cnt[USART_num]].peak = (u32)temp_data;				 										 /* ????????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 5:
                        Pack_Data[USART_num][cnt[USART_num]].peak = ((u32)temp_data<<8) + Pack_Data[USART_num][cnt[USART_num]].peak;
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 6:
                        Pack_Data[USART_num][cnt[USART_num]].peak = ((u32)temp_data<<16) + Pack_Data[USART_num][cnt[USART_num]].peak;
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 7:
                        Pack_Data[USART_num][cnt[USART_num]].peak = ((u32)temp_data<<24) + Pack_Data[USART_num][cnt[USART_num]].peak;				    /* ?????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 8:
                        Pack_Data[USART_num][cnt[USART_num]].confidence = temp_data;				 /* ??? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 9:
                        Pack_Data[USART_num][cnt[USART_num]].intg = (u32)temp_data;															/* ??????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 10:
                        Pack_Data[USART_num][cnt[USART_num]].intg = ((u32)temp_data<<8) + Pack_Data[USART_num][cnt[USART_num]].intg;
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 11:
                        Pack_Data[USART_num][cnt[USART_num]].intg = ((u32)temp_data<<16) + Pack_Data[USART_num][cnt[USART_num]].intg;
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 12:
                        Pack_Data[USART_num][cnt[USART_num]].intg = ((u32)temp_data<<24) + Pack_Data[USART_num][cnt[USART_num]].intg;				  	 /* ???? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 13:
                        Pack_Data[USART_num][cnt[USART_num]].reftof = (int16_t)temp_data;				 								 /* ???????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        break;
                    case 14:
                        Pack_Data[USART_num][cnt[USART_num]].reftof = ((int16_t)temp_data<<8) +Pack_Data[USART_num][cnt[USART_num]].reftof;			/* ????? */
                        crc[USART_num] = crc[USART_num] + temp_data;
                        state[USART_num]++;
                        cnt[USART_num]++;							 /* ???????? */
                        break;
                    default:
                        break;
                    }
                }
                /* ??? */
                if(state[USART_num] == 190) timestamp[USART_num] = temp_data,state[USART_num]++,crc[USART_num] = crc[USART_num] + temp_data;
                else if(state[USART_num] == 191) timestamp[USART_num] = ((u32)temp_data<<8) + timestamp[USART_num],state[USART_num]++,crc[USART_num] = crc[USART_num] + temp_data;
                else if(state[USART_num] == 192) timestamp[USART_num] = ((u32)temp_data<<16) + timestamp[USART_num],state[USART_num]++,crc[USART_num] = crc[USART_num] + temp_data;
                else if(state[USART_num] == 193) timestamp[USART_num] = ((u32)temp_data<<24) + timestamp[USART_num],state[USART_num]++,crc[USART_num] = crc[USART_num] + temp_data;
                else if(state[USART_num] == 194)
                {
                    if(temp_data == crc[USART_num])   /* ???? *///feng:???????,?????????
                    {
                        data_process(USART_num);  	 /* ??????,????????????? */
                        receive_cnt[USART_num]++;	 	 /* ???????????? */
                    }
                    distance[USART_num] = Pack_Data[USART_num][0].distance;
                    crc[USART_num] = 0;
                    state[USART_num] = 0;
                    state_flag[USART_num] = 1;
                    cnt[USART_num] = 0; 				/* ??*/
                }
            }
        }
        /*switch(USART_num)
        {
        	case USART2_Index:HAL_UART_Receive_IT(&huart2,Uart_Receive_buf[USART_num],sizeof(Uart_Receive_buf[USART_num]));
        	case USART3_Index:HAL_UART_Receive_IT(&huart3,Uart_Receive_buf[USART_num],sizeof(Uart_Receive_buf[USART_num]));
        	case USART4_Index:HAL_UART_Receive_IT(&huart4,Uart_Receive_buf[USART_num],sizeof(Uart_Receive_buf[USART_num]));
        	case USART5_Index:HAL_UART_Receive_IT(&huart5,Uart_Receive_buf[USART_num],sizeof(Uart_Receive_buf[USART_num]));
        	default:break;
        }*/
    }
}
void data_process(uint8_t usart_num)//??????,?????????????
{
    //????
    u8 i;
    u16 count = 0;
    LidarPointTypedef Pack_sum;
    for(i=0; i<12; i++)									 //12?????
    {
        if(Pack_Data[usart_num][i].distance != 0)   //??0??
        {
            count++;
            Pack_sum.distance += Pack_Data[usart_num][i].distance;
            Pack_sum.noise += Pack_Data[usart_num][i].noise;
            Pack_sum.peak += Pack_Data[usart_num][i].peak;
            Pack_sum.confidence += Pack_Data[usart_num][i].confidence;
            Pack_sum.intg += Pack_Data[usart_num][i].intg;
            Pack_sum.reftof += Pack_Data[usart_num][i].reftof;
        }
    }
    if(count!=0)
    {
        distance[usart_num] = Pack_sum.distance/count;
        noise[usart_num] = Pack_sum.noise/count;
        peak[usart_num] = Pack_sum.peak/count;
        confidence[usart_num] = Pack_sum.confidence/count;
        intg[usart_num] = Pack_sum.intg/count;
        reftof[usart_num] = Pack_sum.reftof/count;
        Pack_sum.distance = 0;
        Pack_sum.noise = 0;
        Pack_sum.peak = 0;
        Pack_sum.confidence = 0;
        Pack_sum.intg = 0;
        Pack_sum.reftof = 0;
        count = 0;
    }
}
/* USER CODE END 1 */
int t1(void)
{

    return distance[2];
}
int t2(void)
{

    return distance[3];
}

int t3(void)
{

    return distance[0];
}



