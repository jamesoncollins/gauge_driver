/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.io/license.html
 */

#include "gfx.h"

#include "main.h"
#include "cpp_main.h"

#if GFX_USE_GDISP

#define GDISP_DRIVER_VMT			GDISPVMT_s6e63d6
#include "gdisp_lld_config.h"
#include "../../../src/gdisp/gdisp_driver.h"

#include "board_s6e63d6.h"
#include <stdlib.h>
#include <string.h>

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define GDISP_FLG_NEEDFLUSH                     (GDISP_FLG_DRIVER<<0)

// trim height from the bottom
// trim width from the left and right
#define TRIM_HEIGHT 65
#define TRIM_WIDTH 0

#ifndef GDISP_SCREEN_HEIGHT
#define GDISP_SCREEN_HEIGHT		(320 - TRIM_HEIGHT) //320
#endif
#ifndef GDISP_SCREEN_WIDTH
#define GDISP_SCREEN_WIDTH		(240 - TRIM_WIDTH) // 240
#endif
#ifndef GDISP_INITIAL_CONTRAST
#define GDISP_INITIAL_CONTRAST	50
#endif
#ifndef GDISP_INITIAL_BACKLIGHT
#define GDISP_INITIAL_BACKLIGHT	100
#endif

// FIXME: something is wrong here, if i try to draw a box of the whole screen the bottom cuts off
#define H_start_address (TRIM_WIDTH>>1) // 0x00
#define H_end_address ((GDISP_SCREEN_WIDTH-1) + H_start_address)
#define V_start_address 0x00
#define V_end_address (GDISP_SCREEN_HEIGHT-1)

#include "s6e63d6.h"


uint16_t ramBuffer[GDISP_SCREEN_HEIGHT * GDISP_SCREEN_WIDTH];
const int ramSize = sizeof(ramBuffer);
uint32_t *ramBufferInt = (uint32_t*)ramBuffer;
const int ramSizeInt = sizeof(ramBuffer) / 4;

/*
 * a flag we use to tell if we've turned the display and regulators on yet.
 *
 * fixme: why do we even do this?  why dont we turn it all on when the board first
 * comes up?  why do we wait for the first flush?
 */
static bool initOnce = false;

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
#define RAM(g)                                                  ((uint16_t*)g->priv)
#define xyaddr(x, y)            (((x) + (y)*GDISP_SCREEN_WIDTH))
#define map_color(color) ((color << 8) | (color >> 8)) // byte swap
//#define map_color(color) (color)



static void set_viewport (GDisplay *g, uint16_t hstart,uint16_t hend, uint16_t vstart, uint16_t vend)
{
//  /*
//   * TODO, use appropriate viewport
//   * the problem is that the ram-write doesnt support that
//   */
//  gCoord p_x = 0; //g->p.x;
//  gCoord p_y = 0;  //g->p.y;
//  gCoord p_cx = GDISP_SCREEN_WIDTH; //g->p.cx;
//  gCoord p_cy = GDISP_SCREEN_HEIGHT; //g->p.cy;

  // we only need to set the viewport once, we do that in init
  // todo: we could do it here to allow for updates of subsections of the screen
//  write_index(g, 0x35);
//  write_data_one(g, vstart);
//  write_index(g, 0x36);
//  write_data_one(g, vend);
//  write_index(g, 0x37);
//  write_data_one(g, ((hstart<<8)|(hend)));

  // this is now just the starting location
  write_index(g, 0x20);
  write_data_one(g, hstart);
  write_index(g, 0x21);
  write_data_one(g, vstart);

}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

LLDSPEC gBool gdisp_lld_init (GDisplay *g)
{

  // The private area is the display surface.
  //g->priv = gfxAlloc (  GDISP_SCREEN_HEIGHT * GDISP_SCREEN_WIDTH * sizeof(*RAM(g)));
  g->priv = ramBuffer;

  // Initialise the board interface
  init_board (g);

  // Hardware reset
  setpin_reset (g, gTrue);
  gfxSleepMilliseconds (20);
  setpin_reset (g, gFalse);
  gfxSleepMilliseconds (20);

  // Get the bus for the following initialisation commands
  acquire_bus (g);

  // https://www.farnell.com/datasheets/320313.pdf

  // standby off
  write_index(g, 0x10);
  write_data_one(g, 0x0000);

  // device code read
  if( read_index(0x0f) != 0x63D6 )
  {
    //exit(-1);
  }

  // 16-bit mode
  // address counter go from top left to bottom right
  // FIXME: this properly flipped the screen left-right but made it so that
  // i cant draw to pixel 0,0.  it maps to the other side of the screen.
  // seems like either this setting was wrong, or set_viewport is still wrong
  write_index(g, 0x03);
  write_data_one(g, 0x0020);


//  // gama
//  write_index(g, 0x70);
//  write_data_one(g, 0x1f00);
//  write_index(g, 0x71);
//  write_data_one(g, 0x2380);
//  write_index(g, 0x72);
//  write_data_one(g, 0x2a80);
//  write_index(g, 0x73);
//  write_data_one(g, 0x1511);
//  write_index(g, 0x74);
//  write_data_one(g, 0x1c11);
//  write_index(g, 0x75);
//  write_data_one(g, 0x1b15);
//  write_index(g, 0x76);
//  write_data_one(g, 0x1a15);
//  write_index(g, 0x77);
//  write_data_one(g, 0x1c18);
//  write_index(g, 0x78);
//  write_data_one(g, 0x2115);
//
//  //write_index(g, 0x02);
//  //write_data_one(g, 0x0002);
//  //write_index(g, 0x03);
//  //write_data_one(g, 0x0030);
//  write_index(g, 0x18);
//  write_data_one(g, 0x0028);
//  write_index(g, 0xF8);
//  write_data_one(g, 0x000F);    // VGH +5V
//  write_index(g, 0xF9);
//  write_data_one(g, 0x000F);    // VGL -5V

  // clear the ENTIRE screen buffer, even if its bigger
  // that the viewport setup in the driver
  set_viewport(g, 0, 239, 0, 319);
  write_index(g, 0x22);
  for(int i=0; i<320*240; i++)
  {
    write_data_one(g, 0);
  }

  // set viewport
  write_index(g, 0x35);
  write_data_one(g, V_start_address);
  write_index(g, 0x36);
  write_data_one(g, V_end_address);
  write_index(g, 0x37);
  write_data_one(g, ((H_start_address<<8)|(H_end_address)));

//  // display on
//  write_index(g, 0x05);
//  write_data_one(g, 0x0001);
//
//  gfxSleepMilliseconds (100);
//
//  // enable the negative rail of the regulator, this should
//  // let the screen actually display something
//  pwr_en(true);

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
  uint16_t *ram;
  while (bus_busy ())
  {
    // if the bus is still busy then we're running very slow
    // dont bother flushing, becuase we know we haven't written
    // anything
    asm("nop");
    //return;
  }
  ram = RAM(g);

  /* fixme:
   * now that we are again filling the screen buffer during init
   * we could move this code back to init as well
   */

  if(!initOnce)
  {
    // display on
    write_index(g, 0x05);
    write_data_one(g, 0x0001);
    gfxSleepMilliseconds (200);
    // enable the negative rail of the regulator, this should
    // let the screen actually display something
    pwr_en(true);

    initOnce = true;

  }

  // Don't flush if we don't need it.
  if (!(g->flags & GDISP_FLG_NEEDFLUSH))
    return;

  /*
   * transfer the entire screen at once
   */
  acquire_bus (g);
  set_viewport (g, H_start_address, H_end_address, V_start_address, V_end_address);
  write_index (g, 0x22);        // start data
  write_data_one_leave_low(g, ram[0], true, true); // sends start byte and leaves CS low
  write_data (g, (uint8_t*)&ram[1],  GDISP_SCREEN_WIDTH * GDISP_SCREEN_HEIGHT * sizeof(*RAM(g)) - 1*sizeof(*RAM(g)));
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
  uint16_t* base;

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

  while (bus_busy ())
  {
    asm("nop");
  }
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

#if GDISP_HARDWARE_CLEARS
LLDSPEC void gdisp_lld_clear (GDisplay *g)
{

  // dont destroy the ram if we're still reading it for the spi transfer
  while (bus_busy ())
  {
    asm("nop");
  }

  uint16_t c = map_color(gdispColor2Native(g->p.color));
  uint32_t c2 = (uint32_t)c | ((uint32_t)c)<<16;
  setClearColor(c2);

  // dont actually clear, it happened automatically during flushing
  if(getAutoClear())
    return;

  //memset( ramBuffer, 0x00, ramSize );
  uint16_t color = map_color(gdispColor2Native(g->p.color));
  uint32_t color2 = color | color<<16;
  for(int i=0; i<ramSizeInt; i++)
    ramBufferInt[i] = color2;
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

  while (bus_busy ())
  {
    asm("nop");
  }
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
  return (RAM(g)[xyaddr(x, y)]);
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
