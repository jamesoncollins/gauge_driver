/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.io/license.html
 */

#include "gfx.h"

#if GFX_USE_GDISP

#define GDISP_DRIVER_VMT                        GDISPVMT_SSD1351
#include "gdisp_lld_config.h"
#include "../../../src/gdisp/gdisp_driver.h"

#include "board_SSD1351.h"
#include <string.h>   // for memset

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#ifndef GDISP_SCREEN_HEIGHT
        #define GDISP_SCREEN_HEIGHT             128
#endif
#ifndef GDISP_SCREEN_WIDTH
        #define GDISP_SCREEN_WIDTH              128
#endif
#ifndef GDISP_INITIAL_CONTRAST
#define GDISP_INITIAL_CONTRAST  100
#endif
#ifndef GDISP_INITIAL_BACKLIGHT
#define GDISP_INITIAL_BACKLIGHT 100
#endif
//#ifdef SSD1351_PAGE_PREFIX
//        #define SSD1351_PAGE_WIDTH              (GDISP_SCREEN_WIDTH+1)
//        #define SSD1351_PAGE_OFFSET             1
//#else
//        #define SSD1351_PAGE_WIDTH              (GDISP_SCREEN_WIDTH)
//        #define SSD1351_PAGE_OFFSET             0
//#endif

#define GDISP_FLG_NEEDFLUSH                     (GDISP_FLG_DRIVER<<0)

#include "SSD1351.h"

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

// Some common routines and macros
#define RAM(g)                                                  ((gU16 *)g->priv)
#define write_cmd2(g, cmd1, cmd2)               { write_cmd(g, cmd1); write_cmd(g, cmd2); }
#define write_cmd3(g, cmd1, cmd2, cmd3) { write_cmd(g, cmd1); write_cmd(g, cmd2); write_cmd(g, cmd3); }
#define dummy_read(g)                           { volatile gU16 dummy; dummy = read_data(g); (void) dummy; }
#define write_reg(g, reg, data)         { write_cmd(g, reg); write_data_one(g, data); }

// Some common routines and macros
#define delay(us)                       gfxSleepMicroseconds(us)
#define delayms(ms)                     gfxSleepMilliseconds(ms)

#define xyaddr(x, y)            (((x) + (y)*GDISP_SCREEN_WIDTH))
#define map_color(color) ((color>>8)&0xff) | ((color<<8)&0xff)

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

LLDSPEC gBool gdisp_lld_init (GDisplay *g)
{
  // The private area is the display surface.
  g->priv = gfxAlloc (
      GDISP_SCREEN_HEIGHT * GDISP_SCREEN_WIDTH * sizeof(*RAM(g)));

  // Fill in the prefix command byte on each page line of the display buffer
  // We can do it during initialisation as this byte is never overwritten.
#ifdef SSD1351_PAGE_PREFIX
                {
                        unsigned        i;

                        for(i=0; i < GDISP_SCREEN_HEIGHT/8 * SSD1351_PAGE_WIDTH; i+=SSD1351_PAGE_WIDTH)
                                RAM(g)[i] = SSD1351_PAGE_PREFIX;
                }
        #endif

  // Initialise the board interface
  init_board (g);

  // Hardware reset
  setpin_reset (g, gTrue);
  gfxSleepMilliseconds (20);
  setpin_reset (g, gFalse);
  gfxSleepMilliseconds (20);

  // Get the bus for the following initialisation commands
  acquire_bus (g);

  write_reg(g, 0xFD, 0x12); // unlock OLED driver IC
  write_reg(g, 0xFD, 0xB1); // make commands A1, B1, B3, BB, BE, C1 accesible in unlocked state
  write_cmd (g, 0xAE); // sleep mode ON (display off)
  write_reg(g, 0xB3, 0xF1); // Front clock divider / osc freq - Osc = 0xF; div = 2
  write_reg(g, 0xCA, GDISP_SCREEN_HEIGHT-1); // set MUX ratio
  write_reg(g, 0xA0, 0b01110100);  // Set re-map / color depth
  // [0] : address increment (0: horizontal, 1: vertical, reset 0)
  // [1] : column remap (0: 0..127, 1: 127..0, reset 0)
  // [2] : color remap (0: A->B->C, 1: C->B->A, reset 0)
  // [3] : reserved
  // [4] : column scan direction (0: top->down, 1: bottom->up, reset 0)
  // [5] : odd/even split COM (0: disable, 1: enable, reset 1)
  // [6..7] : color depth (00,01: 65k, 10: 262k, 11: 262k format 2)

  write_cmd (g, 0x15); // Set Column address
  write_data_one (g, 0x00); // start
  write_data_one (g, GDISP_SCREEN_WIDTH - 1); // end

  write_cmd (g, 0x75); // set row address
  write_data_one (g, 0x00); // start
  write_data_one (g, GDISP_SCREEN_HEIGHT - 1); // end

  write_reg(g, 0xA1, 0x00); // set display start line - 0
  write_reg(g, 0xA2, 0x00); // set display offset - 0
  write_reg(g, 0xB5, 0x00); // set GPIO - both HiZ, input disabled
  write_reg(g, 0xAB, 0x01); // enable internal VDD regulator
  write_reg(g, 0xB1, 0x32); // set reset / pre-charge period - phase 2: 3 DCLKs, phase 1: 5 DCLKs
  write_reg(g, 0xBE, 0x05); // set VComH voltage - 0.82*Vcc
  write_reg(g, 0xBB, 0x17); // set pre-charge voltage - 0.6*Vcc
  write_cmd (g, 0xA6); // set display mode: reset to normal display

  write_cmd (g, 0xC1); // set contrast current for A,B,C
  write_data_one (g, 0xC8);
  write_data_one (g, 0x80);
  write_data_one (g, 0xC8);

  write_reg(g, 0xC7, 0x0F); // master contrast current control - no change

  write_cmd (g, 0xB4); // set segment low voltage
  write_data_one (g, 0xA0); // external VSL
  write_data_one (g, 0xB5); // hard value
  write_data_one (g, 0x55); // hard value

  write_reg(g, 0xB6, 0x01); // set second pre-charge period - 1 DCLKs
  write_cmd (g, 0xAF); // sleep mode OFF (display on)
  write_cmd (g, 0x5C); // write to RAM

  // Finish Init
  post_init_board (g);

  // Release the bus
  release_bus (g);

  /* Turn on the back-light */
  set_backlight (g, GDISP_INITIAL_BACKLIGHT);

  /* Initialise the GDISP structure */
  g->g.Width = GDISP_SCREEN_WIDTH;
  g->g.Height = GDISP_SCREEN_HEIGHT;
  g->g.Orientation = gOrientation0;
  g->g.Powermode = gPowerOn;
  g->g.Backlight = GDISP_INITIAL_BACKLIGHT;
  g->g.Contrast = GDISP_INITIAL_CONTRAST;
  return gTrue;

}

#if GDISP_HARDWARE_FLUSH
        LLDSPEC void gdisp_lld_flush(GDisplay *g) {
                gU16 * ram;
                unsigned pages;

                // Don't flush if we don't need it.
                if (!(g->flags & GDISP_FLG_NEEDFLUSH))
                        return;
                ram = RAM(g);
                pages = GDISP_SCREEN_HEIGHT;

                acquire_bus(g);

                int cnt = 0;
                while (pages--) {

                        // fixme: can i avoid this by using some kind of auto-increment mode?
                        write_cmd(g, SSD1351_SET_COLUMN_ADDRESS);
                        write_data_one(g, 0);
                        write_data_one(g, GDISP_SCREEN_WIDTH-1);
                        write_cmd(g, SSD1351_SET_ROW_ADDRESS);
                        write_data_one(g, cnt);
                        write_data_one(g, cnt++);
                        write_cmd(g, SSD1351_WRITE_RAM);

                        write_data(g, (gU8 *)ram, GDISP_SCREEN_WIDTH*sizeof(*RAM(g)));
                        ram += GDISP_SCREEN_WIDTH;
                }
                release_bus(g);

                g->flags &= ~GDISP_FLG_NEEDFLUSH;
        }
#endif

#if GDISP_HARDWARE_FILLS
        LLDSPEC void gdisp_lld_fill_area(GDisplay *g) {
                gCoord          sy, ey;
                gCoord          sx, ex;
                gCoord          col;
                unsigned        spage, zpages;
                gU16 *   base;

                switch(g->g.Orientation) {
                default:
                case gOrientation0:
                        sx = g->p.x;
                        ex = g->p.x + g->p.cx - 1;
                        sy = g->p.y;
                        ey = sy + g->p.cy - 1;
                        break;
                case gOrientation90:
                        sx = g->p.y;
                        ex = g->p.y + g->p.cy - 1;
                        sy = GDISP_SCREEN_HEIGHT - g->p.x - g->p.cx;
                        ey = GDISP_SCREEN_HEIGHT-1 - g->p.x;
                        break;
                case gOrientation180:
                        sx = GDISP_SCREEN_WIDTH - g->p.x - g->p.cx;
                        ex = GDISP_SCREEN_WIDTH-1 - g->p.x;
                        sy = GDISP_SCREEN_HEIGHT - g->p.y - g->p.cy;
                        ey = GDISP_SCREEN_HEIGHT-1 - g->p.y;
                        break;
                case gOrientation270:
                        sx = GDISP_SCREEN_WIDTH - g->p.y - g->p.cy;
                        ex = GDISP_SCREEN_WIDTH-1 - g->p.y;
                        sy = g->p.x;
                        ey = g->p.x + g->p.cx - 1;
                        break;
                }

                spage = sy;
                base = RAM(g) + GDISP_SCREEN_WIDTH * spage;
                zpages = (ey) - spage + 1;

                  while (zpages--) {
                          for (col = sx; col <= ex; col++)
                                  base[col] = map_color(gdispColor2Native(g->p.color));
                          base += GDISP_SCREEN_WIDTH;
                  }


                g->flags |= GDISP_FLG_NEEDFLUSH;
        }
#endif

#if GDISP_HARDWARE_DRAWPIXEL
        LLDSPEC void gdisp_lld_draw_pixel(GDisplay *g) {
                gCoord          x, y;

                switch(g->g.Orientation) {
                default:
                case gOrientation0:
                        x = g->p.x;
                        y = g->p.y;
                        break;
                case gOrientation90:
                        x = g->p.y;
                        y = GDISP_SCREEN_HEIGHT-1 - g->p.x;
                        break;
                case gOrientation180:
                        x = GDISP_SCREEN_WIDTH-1 - g->p.x;
                        y = GDISP_SCREEN_HEIGHT-1 - g->p.y;
                        break;
                case gOrientation270:
                        x = GDISP_SCREEN_WIDTH-1 - g->p.y;
                        y = g->p.x;
                        break;
                }

                RAM(g)[xyaddr(x, y)] = map_color(gdispColor2Native(g->p.color));
                g->flags |= GDISP_FLG_NEEDFLUSH;
        }
#endif

#if GDISP_HARDWARE_PIXELREAD
        LLDSPEC gColor gdisp_lld_get_pixel_color(GDisplay *g) {
                gCoord          x, y;

                switch(g->g.Orientation) {
                default:
                case gOrientation0:
                        x = g->p.x;
                        y = g->p.y;
                        break;
                case gOrientation90:
                        x = g->p.y;
                        y = GDISP_SCREEN_HEIGHT-1 - g->p.x;
                        break;
                case gOrientation180:
                        x = GDISP_SCREEN_WIDTH-1 - g->p.x;
                        y = GDISP_SCREEN_HEIGHT-1 - g->p.y;
                        break;
                case gOrientation270:
                        x = GDISP_SCREEN_WIDTH-1 - g->p.y;
                        y = g->p.x;
                        break;
                }
                return RAM(g)[xyaddr(x, y)];
        }
#endif

#if GDISP_NEED_CONTROL && GDISP_HARDWARE_CONTROL
        LLDSPEC void gdisp_lld_control(GDisplay *g) {
                switch(g->p.x) {
                case GDISP_CONTROL_POWER:
                        if (g->g.Powermode == (gPowermode)g->p.ptr)
                                return;
                        switch((gPowermode)g->p.ptr) {
                        case gPowerOff:
                        case gPowerSleep:
                        case gPowerDeepSleep:
                                acquire_bus(g);
                                write_cmd(g, SSD1351_DISPLAYOFF);
                                release_bus(g);
                                break;
                        case gPowerOn:
                                acquire_bus(g);
                                write_cmd(g, SSD1351_DISPLAYON);
                                release_bus(g);
                                break;
                        default:
                                return;
                        }
                        g->g.Powermode = (gPowermode)g->p.ptr;
                        return;

                case GDISP_CONTROL_ORIENTATION:
                        if (g->g.Orientation == (gOrientation)g->p.ptr)
                                return;
                        switch((gOrientation)g->p.ptr) {
                        /* Rotation is handled by the drawing routines */
                        case gOrientation0:
                        case gOrientation180:
                                g->g.Height = GDISP_SCREEN_HEIGHT;
                                g->g.Width = GDISP_SCREEN_WIDTH;
                                break;
                        case gOrientation90:
                        case gOrientation270:
                                g->g.Height = GDISP_SCREEN_WIDTH;
                                g->g.Width = GDISP_SCREEN_HEIGHT;
                                break;
                        default:
                                return;
                        }
                        g->g.Orientation = (gOrientation)g->p.ptr;
                        return;

                case GDISP_CONTROL_CONTRAST:
            if ((unsigned)g->p.ptr > 100)
                g->p.ptr = (void *)100;
                        acquire_bus(g);
                        write_cmd2(g, SSD1351_SETCONTRAST, (((unsigned)g->p.ptr)<<8)/101);
                        release_bus(g);
            g->g.Contrast = (unsigned)g->p.ptr;
                        return;

                // Our own special controller code to inverse the display
                // 0 = normal, 1 = inverse
                case GDISP_CONTROL_INVERSE:
                        acquire_bus(g);
                        write_cmd(g, g->p.ptr ? SSD1351_INVERTDISPLAY : SSD1351_NORMALDISPLAY);
                        release_bus(g);
                        return;
                }
        }
#endif // GDISP_NEED_CONTROL

#endif // GFX_USE_GDISP

