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

  u8g_dev_t6963_240x64.c
  
  Tested with Varitronix MGLS240128TZ
  
  Universal 8bit Graphics Library
  
  Copyright (c) 2013, olikraus@gmail.com
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
  
        
  Application Notes for the MGLS 240x128
    www.baso.no/content/pdf/T6963C_Application.pdf
  
  Hitachi App Notes:
    https://www.sparkfun.com/datasheets/LCD/Monochrome/AN-029-Toshiba_T6963C.pdf

  Notes:
    The font selection pins should generate the 8x8 font.
    For the MGLS240128TZ only FS1 is available on pin 18.
    FS1 must be low to generate the 8x8 font.
  
  
*/

#include "u8g.h"

#define WIDTH 240
#define HEIGHT 64
#define PAGE_HEIGHT 16


/*
  http://www.mark-products.com/graphics.htm#240x64%20Pixel%20Format
*/

/* text is not used, so settings are not relevant */
static const uint8_t u8g_dev_t6963_240x64_init_seq[] PROGMEM = {
  U8G_ESC_CS(0),             /* disable chip */
  U8G_ESC_ADR(0),           /* data mode */
  U8G_ESC_RST(15),           /* do reset low pulse with (15*16)+2 milliseconds (=maximum delay)*/

  U8G_ESC_CS(1),             /* enable chip */
  U8G_ESC_DLY(50),         /* delay 50 ms */

  U8G_ESC_ADR(0),               /* data mode */
  0x000,                                /* low byte */
  0x000,                                /* height byte */
  U8G_ESC_ADR(1),               /* instruction mode */
  0x021,                                /* set cursor position */
  
  U8G_ESC_ADR(0),               /* data mode */
  0x000,                                /* low byte */
  0x000,                                /* height byte */
  U8G_ESC_ADR(1),               /* instruction mode */
  0x022,                                /* set offset */

  U8G_ESC_ADR(0),               /* data mode */
  0x000,                                /* low byte */
  0x000,                                /* height byte */
  U8G_ESC_ADR(1),               /* instruction mode */
  0x040,				     /* text home */

  U8G_ESC_ADR(0),               /* data mode */
  WIDTH/8,                      /* low byte */
  0x000,                                /* height byte */
  U8G_ESC_ADR(1),               /* instruction mode */
  0x041,				     /* text columns */

  U8G_ESC_ADR(0),               /* data mode */
  0x000,                                /* low byte */
  0x000,                                /* height byte */
  U8G_ESC_ADR(1),               /* instruction mode */
  0x042,				     /* graphics home */

  U8G_ESC_ADR(0),               /* data mode */
  WIDTH/8,                      /* low byte */
  0x000,                                /* height byte */
  U8G_ESC_ADR(1),               /* instruction mode */
  0x043,				     /* graphics columns */
  
  // mode set
  // 0x080: Internal CG, OR Mode
  // 0x081: Internal CG, EXOR Mode
  // 0x083: Internal CG, AND Mode
  // 0x088: External CG, OR Mode
  // 0x089: External CG, EXOR Mode
  // 0x08B: External CG, AND Mode
  U8G_ESC_ADR(1),               /* instruction mode */
  0x080,                                /* mode register: OR Mode, Internal Character Mode */
  
  U8G_ESC_ADR(1),               /* instruction mode */
  // display mode
  // 0x090: Display off
  // 0x094: Graphic off, text on, cursor off, blink off
  // 0x096: Graphic off, text on, cursor on, blink off
  // 0x097: Graphic off, text on, cursor on, blink on
  // 0x098: Graphic on, text off, cursor off, blink off
  // 0x09a: Graphic on, text off, cursor on, blink off
  // ...
  // 0x09c: Graphic on, text on, cursor off, blink off
  // 0x09f: Graphic on, text on, cursor on, blink on
  0x098,                                /* mode register: Display Mode, Graphics on, Text off, Cursor off */
  
  U8G_ESC_ADR(0),               /* data mode */
  0x000,                                /* low byte */
  0x000,                                /* height byte */
  U8G_ESC_ADR(1),               /* instruction mode */
  0x024,                                /* set adr pointer */
  

  U8G_ESC_DLY(100),         /* delay 100 ms */
  
  U8G_ESC_ADR(0),               /* data mode */
  U8G_ESC_CS(0),             /* disable chip */
  U8G_ESC_END                /* end of sequence */
};

uint8_t u8g_dev_t6963_240x64_fn(u8g_t *u8g, u8g_dev_t *dev, uint8_t msg, void *arg)
{
  switch(msg)
  {
    case U8G_DEV_MSG_INIT:
      u8g_InitCom(u8g, dev, U8G_SPI_CLK_CYCLE_NONE);    
      u8g_WriteEscSeqP(u8g, dev, u8g_dev_t6963_240x64_init_seq);
      break;
    case U8G_DEV_MSG_STOP:
      break;
    case U8G_DEV_MSG_PAGE_NEXT:
      {
        uint8_t y, i;
        uint16_t disp_ram_adr;
        uint8_t *ptr;
        u8g_pb_t *pb = (u8g_pb_t *)(dev->dev_mem);

        
	u8g_SetAddress(u8g, dev, 0);           /* data mode */
        u8g_SetChipSelect(u8g, dev, 1);
        y = pb->p.page_y0;
        ptr = pb->buf;
        disp_ram_adr = WIDTH/8;
        disp_ram_adr *= y;
        for( i = 0; i < PAGE_HEIGHT; i ++ )
        {
          u8g_SetAddress(u8g, dev, 0);           /* data mode */
          u8g_WriteByte(u8g, dev, disp_ram_adr&255 );      /* address low byte */
          u8g_WriteByte(u8g, dev, disp_ram_adr>>8 );      /* address hight byte */
          u8g_SetAddress(u8g, dev, 1);           /* cmd mode */
          u8g_WriteByte(u8g, dev, 0x024 );      /* set adr ptr */
	  
          u8g_WriteSequence(u8g, dev, WIDTH/8, ptr);	
	  
          ptr += WIDTH/8;
          disp_ram_adr += WIDTH/8;
        }
	u8g_SetAddress(u8g, dev, 0);           /* data mode */
        u8g_SetChipSelect(u8g, dev, 0);
      }
      break;
  }
  return u8g_dev_pb16h1_base_fn(u8g, dev, msg, arg);
}

// U8G_PB_DEV(u8g_dev_t6963_240x64_8bit, WIDTH, HEIGHT, PAGE_HEIGHT, u8g_dev_t6963_240x64_fn, U8G_COM_T6963);

uint8_t u8g_dev_t6963_240x64_2x_bw_buf[WIDTH/8*PAGE_HEIGHT] U8G_NOCOMMON ; 
u8g_pb_t u8g_dev_t6963_240x64_2x_bw_pb = { {PAGE_HEIGHT, HEIGHT, 0, 0, 0},  WIDTH, u8g_dev_t6963_240x64_2x_bw_buf}; 
u8g_dev_t u8g_dev_t6963_240x64_8bit = { u8g_dev_t6963_240x64_fn, &u8g_dev_t6963_240x64_2x_bw_pb, U8G_COM_T6963 };


