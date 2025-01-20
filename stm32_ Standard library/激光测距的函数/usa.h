/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USA_H__
#define __USA_H__

#ifdef __cplusplus


extern "C" {
#endif


    /* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "stdint.h"
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"

    typedef int32_t  s32;
    typedef int16_t s16;
    typedef int8_t  s8;

    typedef uint32_t  u32;
    typedef uint16_t u16;
    typedef uint8_t  u8;

#define USART_REC_LEN  			200  	//????????? 200
#define EN_USART1_RX 			1		//??(1)/??(0)??1??

#define USART2_Index 0
#define USART3_Index 1
#define USART4_Index 2
#define USART5_Index 3

    extern char  USART_RX_BUF[USART_REC_LEN]; //????,??USART_REC_LEN???.???????
    extern uint16_t USART_RX_STA;         		//??????
    extern uint16_t distance[4];


#define HEADER 0xAA				/* ??? */
#define device_address 0x00     /* ???? */
#define chunk_offset 0x00       /* ?????? */
#define PACK_GET_DISTANCE 0x02 	/* ???????? */
#define PACK_RESET_SYSTEM 0x0D 	/* ???? */
#define PACK_STOP 0x0F 			/* ?????????? */
#define PACK_ACK 0x10           /* ????? */
#define PACK_VERSION 0x14       /* ????????? */

    typedef struct {
        int16_t distance;  						/* ????:???????? mm */
        uint16_t noise;		 					/* ????:??????????????,???????? */
        uint32_t peak;							/* ??????:??????????? */
        uint8_t confidence;						/* ???:??????????????????????? */
        uint32_t intg;     						/* ????:???????????? */
        int16_t reftof;   						/* ?????:?????????????,?????????????????? */
    } LidarPointTypedef;

    struct AckResultData {
        uint8_t ack_cmd_id;						/* ????? id */
        uint8_t result; 						/* 1????,0???? */
    };

    struct LiManuConfig
    {
        uint32_t version; 						/* ????? */
        uint32_t hardware_version; 				/* ????? */
        uint32_t manufacture_date; 				/* ???? */
        uint32_t manufacture_time; 				/* ???? */
        uint32_t id1; 							/* ?? id1 */
        uint32_t id2; 							/* ?? id2 */
        uint32_t id3; 							/* ?? id3 */
        uint8_t sn[8]; 							/* sn */
        uint16_t pitch_angle[4]; 				/* ???? */
        uint16_t blind_area[2]; 				/* ???? */
        uint32_t frequence; 					/* ???? */
    };

    /* USER CODE END Includes */



    /* USER CODE BEGIN Private defines */

    /* USER CODE END Private defines */


    /* USER CODE BEGIN Prototypes */
    void data_process(uint8_t usart_num);/*??????,?????????????*/
    int t1(void);
    int t2(void);
    void MX_UART5_Init(void);
    void MX_UART4_Init(void);
		void MX_UART2_Init(void);
		int t3(void);
    /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

