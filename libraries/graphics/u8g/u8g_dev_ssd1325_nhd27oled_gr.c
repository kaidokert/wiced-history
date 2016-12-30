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

  u8g_dev_ssd1325_nhd27oled_gr.c
  
  2-Bit (gray level) Driver for SSD1325 Controller (OLED Display)
  Tested with NHD-2.7-12864UCY3

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
  
  SSD130x       Monochrom OLED Controller
  SSD131x       Character OLED Controller
  SSD132x       Graylevel OLED Controller
  SSD1331       Color OLED Controller       

*/

#ifdef OBSOLETE_CODE

#include "u8g.h"

#define WIDTH 128
#define HEIGHT 64

/* http://www.newhavendisplay.com/app_notes/OLED_2_7_12864.txt */
static const uint8_t u8g_dev_ssd1325_2bit_nhd_27_12864ucy3_init_seq[] PROGMEM = {
  U8G_ESC_DLY(10),              /* delay 10 ms */
  U8G_ESC_CS(0),                 /* disable chip */
  U8G_ESC_ADR(0),               /* instruction mode */
  U8G_ESC_RST(1),               /* do reset low pulse with (1*16)+2 milliseconds */
  U8G_ESC_CS(1),                /* enable chip */
  0x0ae,                                /* display off, sleep mode */
  0x0b3, 0x091,                    /* set display clock divide ratio/oscillator frequency (set clock as 135 frames/sec) */
  0x0a8, 0x03f,                     /* multiplex ratio: 0x03f * 1/64 duty */
  0x0a2, 0x04c,                     /* display offset, shift mapping ram counter */
  0x0a1, 0x000,                     /* display start line */
  0x0ad, 0x002,                     /* master configuration: disable embedded DC-DC, enable internal VCOMH */
  0x0a0, 0x056,                     /* remap configuration, vertical address increment, enable nibble remap (upper nibble is left) */
  0x086,                                /* full current range (0x084, 0x085, 0x086) */
  0x0b8,                                /* set gray scale table */
      //0x01, 0x011, 0x022, 0x032, 0x043, 0x054, 0x065, 0x076,
      0x01, 0x011, 0x022, 0x032, 0x043, 0x054, 0x077, 0x077,            // 4L mode uses 0, 2, 4, 7
  0x081, 0x070,                    /* contrast, brightness, 0..128, Newhaven: 0x040 */
  0x0b2, 0x051,                    /* frame frequency (row period) */
  0x0b1, 0x055,                    /* phase length */
  0x0bc, 0x010,                    /* pre-charge voltage level */
  0x0b4, 0x002,                    /* set pre-charge compensation level (not documented in the SDD1325 datasheet, but used in the NHD init seq.) */
  0x0b0, 0x028,                    /* enable pre-charge compensation (not documented in the SDD1325 datasheet, but used in the NHD init seq.) */
  0x0be, 0x01c,                     /* VCOMH voltage */
  0x0bf, 0x002|0x00d,           /* VSL voltage level (not documented in the SDD1325 datasheet, but used in the NHD init seq.) */
  0x0a5,                                 /* all pixel on */
  0x0af,                                  /* display on */
  U8G_ESC_DLY(100),             /* delay 100 ms */
  U8G_ESC_DLY(100),             /* delay 100 ms */
  0x0a4,                                 /* normal display mode */
  U8G_ESC_CS(0),             /* disable chip */
  U8G_ESC_END                /* end of sequence */
};

static const uint8_t u8g_dev_ssd1325_2bit_nhd_27_12864ucy3_prepare_page_seq[] PROGMEM = {
  U8G_ESC_ADR(0),               /* instruction mode */
  U8G_ESC_CS(1),                /* enable chip */
  0x015,       /* column address... */
  0x000,       /* start at column 0 */
  0x03f,       /* end at column 63 (which is y == 127), because there are two pixel in one column */
  0x075,       /* row address... */
  U8G_ESC_END                /* end of sequence */
};


static void u8g_dev_ssd1325_2bit_prepare_page(u8g_t *u8g, u8g_dev_t *dev)
{
  uint8_t page = ((u8g_pb_t *)(dev->dev_mem))->p.page;
  
  u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd1325_2bit_nhd_27_12864ucy3_prepare_page_seq);
  
  page <<= 2;
  u8g_WriteByte(u8g, dev, page);       /* start at the selected page */
  page += 3;
  u8g_WriteByte(u8g, dev, page);       /* end within the selected page */  
  
  u8g_SetAddress(u8g, dev, 1);          /* data mode */
}

static void u8g_dev_ssd1325_2bit_2x_prepare_page(u8g_t *u8g, u8g_dev_t *dev, uint8_t is_odd)
{
  uint8_t page = ((u8g_pb_t *)(dev->dev_mem))->p.page;
  
  u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd1325_2bit_nhd_27_12864ucy3_prepare_page_seq);
  
  page <<= 1;
  page += is_odd;
  
  
  page <<= 2;
  u8g_WriteByte(u8g, dev, page);       /* start at the selected page */
  page += 3;
  u8g_WriteByte(u8g, dev, page);       /* end within the selected page */  
  
  u8g_SetAddress(u8g, dev, 1);          /* data mode */
}

/* assumes row autoincrement and activated nibble remap */
static  void u8g_dev_ssd1325_2bit_write_4_pixel(u8g_t *u8g, u8g_dev_t *dev, uint8_t left, uint8_t right)
{
  uint8_t d, tmp, cnt;
  cnt = 4;
  do    
  {
    d = left;
    d &= 3;
    d <<= 4;    
    tmp = right;    
    tmp &= 3;
    d |= tmp;
    d <<= 2;
    u8g_WriteByte(u8g, dev, d);
    left >>= 2;
    right >>= 2;
    cnt--;
  }while ( cnt > 0 );
}

static void u8g_dev_ssd1325_2bit_write_buffer(u8g_t *u8g, u8g_dev_t *dev)
{
  uint8_t cnt, left, right;
  uint8_t *ptr;
  u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
  
  cnt = pb->width;
  cnt >>= 1;
  ptr = pb->buf;
  do
  {
    left = *ptr++;
    right = *ptr++;
    u8g_dev_ssd1325_2bit_write_4_pixel(u8g, dev, left, right);
    cnt--;
  } while( cnt > 0 );
}

static void u8g_dev_ssd1325_2bit_2x_write_buffer(u8g_t *u8g, u8g_dev_t *dev, uint8_t is_odd)
{
  uint8_t cnt, left, right;
  uint8_t *ptr;
  u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);
  
  ptr = pb->buf;
  cnt = pb->width;
  if ( is_odd )
    ptr += cnt;
  cnt >>= 1;
  do
  {
    left = *ptr++;
    right = *ptr++;
    u8g_dev_ssd1325_2bit_write_4_pixel(u8g, dev, left, right);
    cnt--;
  } while( cnt > 0 );
}

static uint8_t u8g_dev_ssd1325_nhd27oled_gr_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  switch(msg)
  {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_300NS);
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd1325_2bit_nhd_27_12864ucy3_init_seq);
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_NEXT:
      {
        u8g_dev_ssd1325_2bit_prepare_page(u8g, dev);
        u8g_dev_ssd1325_2bit_write_buffer(u8g, dev);
        u8g_SetChipSelect(u8g, dev, 0);        
      }
      break;
    case U8G_DEV_MSG_CONTRAST:
      u8g_SetChipSelect(u8g, dev, 1);
      u8g_SetAddress(u8g, dev, 0);          /* instruction mode */
      u8g_WriteByte(u8g, dev, 0x081);
      u8g_WriteByte(u8g, dev, (*(uint8_t *)arg) >> 1);
      u8g_SetChipSelect(u8g, dev, 0);      
      return 1;
  }
  return u8g_dev_pb8v2_base_fn(u8g, dev, msg, arg);
}

static uint8_t u8g_dev_ssd1325_nhd27oled_2x_gr_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  switch(msg)
  {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_300NS);
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_ssd1325_2bit_nhd_27_12864ucy3_init_seq);
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_NEXT:
      {
        u8g_dev_ssd1325_2bit_2x_prepare_page(u8g, dev, 0);
        u8g_dev_ssd1325_2bit_2x_write_buffer(u8g, dev, 0);
        u8g_dev_ssd1325_2bit_2x_prepare_page(u8g, dev, 1);
        u8g_dev_ssd1325_2bit_2x_write_buffer(u8g, dev, 1);
        u8g_SetChipSelect(u8g, dev, 0);        
      }
      break;
    case U8G_DEV_MSG_CONTRAST:
      u8g_SetChipSelect(u8g, dev, 1);
      u8g_SetAddress(u8g, dev, 0);          /* instruction mode */
      u8g_WriteByte(u8g, dev, 0x081);
      u8g_WriteByte(u8g, dev, (*(uint8_t *)arg) >> 1);
      u8g_SetChipSelect(u8g, dev, 0);      
      return 1;
  }
  return u8g_dev_pb16v2_base_fn(u8g, dev, msg, arg);
}

//U8G_PB_DEV(u8g_dev_ssd1325_nhd27oled_gr_sw_spi , WIDTH, HEIGHT, 4, u8g_dev_ssd1325_nhd27oled_gr_fn, U8G_COM_SW_SPI);
//U8G_PB_DEV(u8g_dev_ssd1325_nhd27oled_gr_hw_spi , WIDTH, HEIGHT, 4, u8g_dev_ssd1325_nhd27oled_gr_fn, U8G_COM_HW_SPI);

//uint8_t u8g_dev_ssd1325_nhd27oled_2x_buf[WIDTH*2] U8G_NOCOMMON ; 
//u8g_pb_t u8g_dev_ssd1325_nhd27oled_2x_pb = { {8, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_ssd1325_nhd27oled_2x_buf}; 
//u8g_dev_t u8g_dev_ssd1325_nhd27oled_2x_gr_sw_spi = { u8g_dev_ssd1325_nhd27oled_2x_gr_fn, &u8g_dev_ssd1325_nhd27oled_2x_pb, U8G_COM_SW_SPI };
//u8g_dev_t u8g_dev_ssd1325_nhd27oled_2x_gr_hw_spi = { u8g_dev_ssd1325_nhd27oled_2x_gr_fn, &u8g_dev_ssd1325_nhd27oled_2x_pb, U8G_COM_HW_SPI };


#endif /* OBSOLETE_CODE */
