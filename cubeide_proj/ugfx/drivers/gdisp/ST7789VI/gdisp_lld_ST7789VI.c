/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.io/license.html
 */

#include "gfx.h"

#if GFX_USE_GDISP

#if defined(GDISP_SCREEN_HEIGHT) || defined(GDISP_SCREEN_HEIGHT)
	#if GFX_COMPILER_WARNING_TYPE == GFX_COMPILER_WARNING_DIRECT
		#warning "GDISP: This low level driver does not support setting a screen size. It is being ignored."
	#elif GFX_COMPILER_WARNING_TYPE == GFX_COMPILER_WARNING_MACRO
		COMPILER_WARNING("GDISP: This low level driver does not support setting a screen size. It is being ignored.")
	#endif
	#undef GDISP_SCREEN_WIDTH
	#undef GDISP_SCREEN_HEIGHT
#endif

#define GDISP_DRIVER_VMT			GDISPVMT_ST7789VI
#include "gdisp_lld_config.h"
#include "../../../src/gdisp/gdisp_driver.h"

#include "board_ST7789VI.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define GDISP_FLG_NEEDFLUSH                     (GDISP_FLG_DRIVER<<0)

#ifndef GDISP_SCREEN_HEIGHT
#define GDISP_SCREEN_HEIGHT		320
#endif
#ifndef GDISP_SCREEN_WIDTH
#define GDISP_SCREEN_WIDTH		240
#endif
#ifndef GDISP_INITIAL_CONTRAST
#define GDISP_INITIAL_CONTRAST	50
#endif
#ifndef GDISP_INITIAL_BACKLIGHT
#define GDISP_INITIAL_BACKLIGHT	100
#endif

#include "ST7789VI.h"

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

// Some common routines and macros
#define dummy_read(g)				{ volatile gU16 dummy; dummy = read_data(g); (void) dummy; }
#define write_reg(g, reg, data)		{ write_index(g, reg); write_data_one(g, data); }
#define write_data16(g, data)		{ write_data_one(g, data >> 8); write_data_one(g, (gU8)data); }
#define delay(us)					gfxSleepMicroseconds(us)
#define delayms(ms)					gfxSleepMilliseconds(ms)

// Some common routines and macros
#define RAM(g)                                                  ((gU16 *)g->priv)
#define write_cmd(g,arg1)       write_index(g,arg1)
#define write_cmd2(g, cmd1, cmd2)               { write_cmd(g, cmd1); write_cmd(g, cmd2); }
#define write_cmd3(g, cmd1, cmd2, cmd3) { write_cmd(g, cmd1); write_cmd(g, cmd2); write_cmd(g, cmd3); }
#define xyaddr(x, y)            (((x) + (y)*GDISP_SCREEN_WIDTH))
#define map_color(color) ((color>>8)&0xff) | ((color<<8)&0xff)

static void set_viewport (GDisplay *g)
{
  /*
   * TODO, use appropriate viewport
   * the problem is that the ram-write doesnt support that
   */
  gCoord p_x = 0; //g->p.x;
  gCoord p_y = 0;  //g->p.y;
  gCoord p_cx = GDISP_SCREEN_WIDTH; //g->p.cx;
  gCoord p_cy = GDISP_SCREEN_HEIGHT; //g->p.cy;

  write_index (g, 0x2A);
  write_data_one (g, (p_x >> 8));
  write_data_one (g, (gU8) p_x);
  write_data_one (g, (p_x + p_cx - 1) >> 8);
  write_data_one (g, (gU8) (p_x + p_cx - 1));

  write_index (g, 0x2B);
  write_data_one (g, (p_y >> 8));
  write_data_one (g, (gU8) p_y);
  write_data_one (g, (p_y + p_cy - 1) >> 8);
  write_data_one (g, (gU8) (p_y + p_cy - 1));

}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

LLDSPEC gBool gdisp_lld_init (GDisplay *g)
{

  // The private area is the display surface.
  g->priv = gfxAlloc (
  GDISP_SCREEN_HEIGHT * GDISP_SCREEN_WIDTH * sizeof(*RAM(g)));

  // Initialise the board interface
  init_board (g);

  // Hardware reset
  setpin_reset (g, gTrue);
  gfxSleepMilliseconds (20);
  setpin_reset (g, gFalse);
  gfxSleepMilliseconds (20);

  // Get the bus for the following initialisation commands
  acquire_bus (g);

  write_index (g, 0x28); // Turn off display
  write_index (g, 0x11); // Exit sleep mode
  write_index (g, 0x36);
  write_data_one (g, 0x88); // MADCTL: memory data access control Old: 0x88

  write_index (g, 0x3A);
  write_data_one (g, 0x55); // COLMOD: Interface Pixel format (16-bits per pixel @ 65K colors)

  write_index (g, 0xB2);
  write_data_one (g, 0x0C);
  write_data_one (g, 0x0C);
  write_data_one (g, 0x00);
  write_data_one (g, 0x33);
  write_data_one (g, 0x33); // PORCTRK: Porch setting

  write_index (g, 0xB7);
  write_data_one (g, 0x35); // GCTRL: Gate Control

  write_index (g, 0xBB);
  write_data_one (g, 0x2B); // VCOMS: VCOM setting

  write_index (g, 0xC0);
  write_data_one (g, 0x2C); // LCMCTRL: LCM Control

  write_index (g, 0xC2);
  write_data_one (g, 0x01);
  write_data_one (g, 0xFF); // VDVVRHEN: VDV and VRH Command Enable

  write_index (g, 0xC3);
  write_data_one (g, 0x11); // VRHS: VRH set

  write_index (g, 0xC4);
  write_data_one (g, 0x20); // VDVS: VDV Set

  write_index (g, 0xC6);
  write_data_one (g, 0x0F); // FRCTRL2: Frame Rate control in normal mode

  write_index (g, 0xD0);
  write_data_one (g, 0xA4);
  write_data_one (g, 0xA1); // PWCTRL1: Power Control 1

  write_index (g, 0xE0);
  write_data_one (g, 0xD0);
  write_data_one (g, 0x00);
  write_data_one (g, 0x05);
  write_data_one (g, 0x0E);
  write_data_one (g, 0x15);
  write_data_one (g, 0x0D);
  write_data_one (g, 0x37);
  write_data_one (g, 0x43);
  write_data_one (g, 0x47);
  write_data_one (g, 0x09);
  write_data_one (g, 0x15);
  write_data_one (g, 0x12);
  write_data_one (g, 0x16);
  write_data_one (g, 0x19); // PVGAMCTRL: Positive Voltage Gamma control

  write_index (g, 0xE1);
  write_data_one (g, 0xD0);
  write_data_one (g, 0x00);
  write_data_one (g, 0x05);
  write_data_one (g, 0x0D);
  write_data_one (g, 0x0C);
  write_data_one (g, 0x06);
  write_data_one (g, 0x2D);
  write_data_one (g, 0x44);
  write_data_one (g, 0x40);
  write_data_one (g, 0x0E);
  write_data_one (g, 0x1C);
  write_data_one (g, 0x18);
  write_data_one (g, 0x16);
  write_data_one (g, 0x19); // NVGAMCTRL: Negative Voltage Gamma control

  write_index (g, 0x2A);
  write_data_one (g, 0x00);
  write_data_one (g, 0x00);
  write_data_one (g, 0x00);
  write_data_one (g, 0xEF); // X address set

  write_index (g, 0x2B);
  write_data_one (g, 0x00);
  write_data_one (g, 0x00);
  write_data_one (g, 0x01);
  write_data_one (g, 0x3F); // Y address set

  write_index (g, 0x29); // Display on
  gfxSleepMilliseconds (100);

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

#if GDISP_HARDWARE_STREAM_WRITE
	LLDSPEC	void gdisp_lld_write_start(GDisplay *g) {
		acquire_bus(g);
		set_viewport(g);
		write_index(g, 0x2C);
	}
	LLDSPEC	void gdisp_lld_write_color(GDisplay *g) {
		write_data16(g, gdispColor2Native(g->p.color));
	}
	LLDSPEC	void gdisp_lld_write_stop(GDisplay *g) {
		release_bus(g);
	}
#endif

#if GDISP_HARDWARE_STREAM_READ
	LLDSPEC	void gdisp_lld_read_start(GDisplay *g) {
		acquire_bus(g);
		set_viewport(g);
		write_index(g, 0x2E);
		setreadmode(g);
		dummy_read(g);
	}
	LLDSPEC	gColor gdisp_lld_read_color(GDisplay *g) {
		gU16	data;

		data = read_data(g);
		return gdispNative2Color(data);
	}
	LLDSPEC	void gdisp_lld_read_stop(GDisplay *g) {
		setwritemode(g);
		release_bus(g);
	}
#endif

#if GDISP_HARDWARE_FLUSH
LLDSPEC void gdisp_lld_flush (GDisplay *g)
{
  gU16 *ram;

  // Don't flush if we don't need it.
  if (!(g->flags & GDISP_FLG_NEEDFLUSH))
    return;

  while (bus_busy ());
  ram = RAM(g);

  /*
   * transfer the entire screen at once
   */
  acquire_bus (g);
  set_viewport (g);
  write_index (g, 0x2C);
  write_data (g, (gU8*) ram,
              GDISP_SCREEN_WIDTH * GDISP_SCREEN_HEIGHT * sizeof(*RAM(g)));
  release_bus (g);

  g->flags &= ~GDISP_FLG_NEEDFLUSH;
}
#endif

#if GDISP_HARDWARE_FILLS
LLDSPEC void gdisp_lld_fill_area (GDisplay *g)
{
  gCoord sy, ey;
  gCoord sx, ex;
  gCoord col;
  unsigned spage, zpages;
  gU16 *base;

  switch (g->g.Orientation)
  {
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
      ey = GDISP_SCREEN_HEIGHT - 1 - g->p.x;
      break;
    case gOrientation180:
      sx = GDISP_SCREEN_WIDTH - g->p.x - g->p.cx;
      ex = GDISP_SCREEN_WIDTH - 1 - g->p.x;
      sy = GDISP_SCREEN_HEIGHT - g->p.y - g->p.cy;
      ey = GDISP_SCREEN_HEIGHT - 1 - g->p.y;
      break;
    case gOrientation270:
      sx = GDISP_SCREEN_WIDTH - g->p.y - g->p.cy;
      ex = GDISP_SCREEN_WIDTH - 1 - g->p.y;
      sy = g->p.x;
      ey = g->p.x + g->p.cx - 1;
      break;
  }

  while (bus_busy ());
  spage = sy;
  base = RAM(g) + GDISP_SCREEN_WIDTH * spage;
  zpages = (ey) - spage + 1;

  while (zpages--)
  {
    for (col = sx; col <= ex; col++)
      base[col] = map_color(gdispColor2Native(g->p.color));
    base += GDISP_SCREEN_WIDTH;
  }

  g->flags |= GDISP_FLG_NEEDFLUSH;
}
#endif

#if GDISP_HARDWARE_DRAWPIXEL
LLDSPEC void gdisp_lld_draw_pixel (GDisplay *g)
{
  gCoord x, y;

  switch (g->g.Orientation)
  {
    default:
    case gOrientation0:
      x = g->p.x;
      y = g->p.y;
      break;
    case gOrientation90:
      x = g->p.y;
      y = GDISP_SCREEN_HEIGHT - 1 - g->p.x;
      break;
    case gOrientation180:
      x = GDISP_SCREEN_WIDTH - 1 - g->p.x;
      y = GDISP_SCREEN_HEIGHT - 1 - g->p.y;
      break;
    case gOrientation270:
      x = GDISP_SCREEN_WIDTH - 1 - g->p.y;
      y = g->p.x;
      break;
  }

  while (bus_busy ());
  RAM(g)[xyaddr(x, y)] = map_color(gdispColor2Native(g->p.color));
  g->flags |= GDISP_FLG_NEEDFLUSH;
}
#endif

#if GDISP_HARDWARE_PIXELREAD
LLDSPEC gColor gdisp_lld_get_pixel_color (GDisplay *g)
{
  gCoord x, y;

  switch (g->g.Orientation)
  {
    default:
    case gOrientation0:
      x = g->p.x;
      y = g->p.y;
      break;
    case gOrientation90:
      x = g->p.y;
      y = GDISP_SCREEN_HEIGHT - 1 - g->p.x;
      break;
    case gOrientation180:
      x = GDISP_SCREEN_WIDTH - 1 - g->p.x;
      y = GDISP_SCREEN_HEIGHT - 1 - g->p.y;
      break;
    case gOrientation270:
      x = GDISP_SCREEN_WIDTH - 1 - g->p.y;
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
				write_reg(g, 0x0010, 0x0001);	/* enter sleep mode */
				release_bus(g);
				break;
			case gPowerOn:
				acquire_bus(g);
				write_reg(g, 0x0010, 0x0000);	/* leave sleep mode */
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
			case gOrientation0:
				acquire_bus(g);
				write_reg(g, 0x36, 0x48);	/* X and Y axes non-inverted */
				release_bus(g);
				g->g.Height = GDISP_SCREEN_HEIGHT;
				g->g.Width = GDISP_SCREEN_WIDTH;
				break;
			case gOrientation90:
				acquire_bus(g);
				write_reg(g, 0x36, 0xE8);	/* Invert X and Y axes */
				release_bus(g);
				g->g.Height = GDISP_SCREEN_WIDTH;
				g->g.Width = GDISP_SCREEN_HEIGHT;
				break;
			case gOrientation180:
				acquire_bus(g);
				write_reg(g, 0x36, 0x88);		/* X and Y axes non-inverted */
				release_bus(g);
				g->g.Height = GDISP_SCREEN_HEIGHT;
				g->g.Width = GDISP_SCREEN_WIDTH;
				break;
			case gOrientation270:
				acquire_bus(g);
				write_reg(g, 0x36, 0x28);	/* Invert X and Y axes */
				release_bus(g);
				g->g.Height = GDISP_SCREEN_WIDTH;
				g->g.Width = GDISP_SCREEN_HEIGHT;
				break;
			default:
				return;
			}
			g->g.Orientation = (gOrientation)g->p.ptr;
			return;

        case GDISP_CONTROL_BACKLIGHT:
            if ((unsigned)g->p.ptr > 100)
            	g->p.ptr = (void *)100;
            set_backlight(g, (unsigned)g->p.ptr);
            g->g.Backlight = (unsigned)g->p.ptr;
            return;

		//case GDISP_CONTROL_CONTRAST:
        default:
            return;
		}
	}
#endif

#endif /* GFX_USE_GDISP */
