/*
 * Copyright 2016, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/*

  u8g_dev_uc1701_dogs102.c

  Universal 8bit Graphics Library
  
  Copyright (c) 2011, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  
  
*/

#include "u8g.h"

#define WIDTH 102
#define HEIGHT 64
#define PAGE_HEIGHT 8

static const uint8_t u8g_dev_dogs102_init_seq[] PROGMEM = {
  U8G_ESC_CS(0),             /* disable chip */
  U8G_ESC_ADR(0),           /* instruction mode */
  U8G_ESC_RST(1),           /* do reset low pulse with (1*16)+2 milliseconds */
  U8G_ESC_CS(1),             /* enable chip */
  
  0x0e2,            		/* soft reset */
  0x040,		                /* set display start line to 0 */
  0x0a1,		                /* ADC set to reverse */
  0x0c0,		                /* common output mode */
  0x0a6,		                /* display normal, bit val 0: LCD pixel off. */
  0x0a2,		                /* LCD bias 1/9 */
  0x02f,		                /* all power  control circuits on */
  0x027,		                /* regulator, booster and follower */
  0x081,		                /* set contrast */
  0x00e,		                /* contrast value, EA default: 0x010, previous value for S102: 0x0e */
  0x0fa,		                /* Set Temp compensation */ 
  0x090,		                /* 0.11 deg/c WP Off WC Off*/
  0x0a4,		                /* normal display  */
  0x0af,		                /* display on */
  U8G_ESC_DLY(100),       /* delay 100 ms */
  0x0a5,		                /* display all points, ST7565, UC1610 */
  U8G_ESC_DLY(100),       /* delay 100 ms */
  U8G_ESC_DLY(100),       /* delay 100 ms */
  0x0a4,		                /* normal display */
  U8G_ESC_CS(0),             /* disable chip */
  U8G_ESC_END                /* end of sequence */
};

static const uint8_t u8g_dev_dogs102_data_start[] PROGMEM = {
  U8G_ESC_ADR(0),           /* instruction mode */
  U8G_ESC_CS(1),             /* enable chip */
  0x010,		/* set upper 4 bit of the col adr to 0 */
  0x000,		/* set lower 4 bit of the col adr to 0 */      
  U8G_ESC_END                /* end of sequence */
};

uint8_t u8g_dev_dogs102_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  switch(msg)
  {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_300NS);
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_dogs102_init_seq);
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_NEXT:
      {
        u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
        u8g_WriteEscSeqP(u8g, dev, u8g_dev_dogs102_data_start);    
        u8g_WriteByte(u8g, dev, 0x0b0 | pb->p.page); /* select current page (ST7565R) */
        u8g_SetAddress(u8g, dev, 1);           /* data mode */
        if ( u8g_pb_WriteBuffer(pb, u8g, dev) == 0 )
          return 0;
        u8g_SetChipSelect(u8g, dev, 0);
      }
      break;
    case U8G_DEV_MSG_CONTRAST:
      u8g_SetChipSelect(u8g, dev, 1);
      u8g_SetAddress(u8g, dev, 0);          /* instruction mode */
      u8g_WriteByte(u8g, dev, 0x081);
      u8g_WriteByte(u8g, dev, (*(uint8_t *)arg) >> 2);
      u8g_SetChipSelect(u8g, dev, 0);      
      return 1;
  }
  return u8g_dev_pb8v1_base_fn(u8g, dev, msg, arg);
}

uint8_t u8g_dev_uc1701_dogs102_2x_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  switch(msg)
  {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_300NS);
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_dogs102_init_seq);
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_NEXT:
      {
        u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
	
        u8g_WriteEscSeqP(u8g, dev, u8g_dev_dogs102_data_start);    
        u8g_WriteByte(u8g, dev, 0x0b0 | (2*pb->p.page)); /* select current page (ST7565R) */
        u8g_SetAddress(u8g, dev, 1);           /* data mode */
	u8g_WriteSequence(u8g, dev, pb->width, pb->buf); 
        u8g_SetChipSelect(u8g, dev, 0);
	
        u8g_WriteEscSeqP(u8g, dev, u8g_dev_dogs102_data_start);    
        u8g_WriteByte(u8g, dev, 0x0b0 | (2*pb->p.page+1)); /* select current page (ST7565R) */
        u8g_SetAddress(u8g, dev, 1);           /* data mode */
	u8g_WriteSequence(u8g, dev, pb->width, (uint8_t *)(pb->buf)+pb->width); 
        u8g_SetChipSelect(u8g, dev, 0);
      }
      break;
    case U8G_DEV_MSG_CONTRAST:
      u8g_SetChipSelect(u8g, dev, 1);
      u8g_SetAddress(u8g, dev, 0);          /* instruction mode */
      u8g_WriteByte(u8g, dev, 0x081);
      u8g_WriteByte(u8g, dev, (*(uint8_t *)arg) >> 2);
      u8g_SetChipSelect(u8g, dev, 0);      
      return 1;
  }
  return u8g_dev_pb16v1_base_fn(u8g, dev, msg, arg);
}

U8G_PB_DEV(u8g_dev_uc1701_dogs102_sw_spi , WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_dogs102_fn, U8G_COM_SW_SPI);
U8G_PB_DEV(u8g_dev_uc1701_dogs102_hw_spi , WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_dogs102_fn, U8G_COM_HW_SPI);

uint8_t u8g_dev_uc1701_dogs102_2x_buf[WIDTH*2] U8G_NOCOMMON ; 
u8g_pb_t u8g_dev_uc1701_dogs102_2x_pb = { {16, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_uc1701_dogs102_2x_buf}; 
u8g_dev_t u8g_dev_uc1701_dogs102_2x_sw_spi = { u8g_dev_uc1701_dogs102_2x_fn, &u8g_dev_uc1701_dogs102_2x_pb, U8G_COM_SW_SPI };
u8g_dev_t u8g_dev_uc1701_dogs102_2x_hw_spi = { u8g_dev_uc1701_dogs102_2x_fn, &u8g_dev_uc1701_dogs102_2x_pb, U8G_COM_HW_SPI };

