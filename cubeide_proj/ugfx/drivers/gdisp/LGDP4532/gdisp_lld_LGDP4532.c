/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.io/license.html
 */

#include "gfx.h"

#if GFX_USE_GDISP

/* This controller is only ever used with a 240 x 320 display */
#if defined(GDISP_SCREEN_HEIGHT) || defined(GDISP_SCREEN_HEIGHT)
	#if GFX_COMPILER_WARNING_TYPE == GFX_COMPILER_WARNING_DIRECT
		#warning "GDISP: This low level driver does not support setting a screen size. It is being ignored."
	#elif GFX_COMPILER_WARNING_TYPE == GFX_COMPILER_WARNING_MACRO
		COMPILER_WARNING("GDISP: This low level driver does not support setting a screen size. It is being ignored.")
	#endif
	#undef GDISP_SCREEN_WIDTH
	#undef GDISP_SCREEN_HEIGHT
#endif

#define GDISP_DRIVER_VMT	GDISPVMT_LGDP4532
#include "gdisp_lld_config.h"
#include "../../../src/gdisp/gdisp_driver.h"

#include "board_LGDP4532.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

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

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

// Some common routines and macros
#define dummy_read(g)			{ volatile gU16 dummy; dummy = read_data(g); (void) dummy; }
#define write_reg(g, reg, data)		{ write_index(g, reg); write_data(g, data); }

// Serial write data for fast fill.
#ifndef write_data_repeat
#define write_data_repeat(g, data, count) { int i; for (i = 0; i < count; ++i) write_data (g, data) }
/* TODO: should use DMA mem2mem */
#endif

static void set_cursor(GDisplay *g) {
	switch(g->g.Orientation) {
		default:
		case gOrientation0:
		case gOrientation180:
			write_reg(g, 0x20, g->p.x);
			write_reg(g, 0x21, g->p.y);
			break;

		case gOrientation90:
		case gOrientation270:
			write_reg(g, 0x20, g->p.y);
			write_reg(g, 0x21, g->p.x);
			break;
	}
	write_index(g, 0x22);
}

static void set_viewport(GDisplay* g) {
	switch(g->g.Orientation) {
		default:
		case gOrientation0:
		case gOrientation180:
			write_reg(g, LGDP4532_HORIZONTAL_WINDOW_ADDR1, g->p.x);
			write_reg(g, LGDP4532_HORIZONTAL_WINDOW_ADDR2, g->p.x + g->p.cx - 1);
			write_reg(g, LGDP4532_VERTICAL_WINDOW_ADDR1, g->p.y);
			write_reg(g, LGDP4532_VERTICAL_WINDOW_ADDR2, g->p.y + g->p.cy - 1);
			break;

		case gOrientation90:
		case gOrientation270:
			write_reg(g, LGDP4532_HORIZONTAL_WINDOW_ADDR1, g->p.y);
			write_reg(g, LGDP4532_HORIZONTAL_WINDOW_ADDR2, g->p.y + g->p.cy - 1);
			write_reg(g, LGDP4532_VERTICAL_WINDOW_ADDR1, g->p.x);
			write_reg(g, LGDP4532_VERTICAL_WINDOW_ADDR2, g->p.x + g->p.cx - 1);
			break;
	}
}

LLDSPEC gBool gdisp_lld_init(GDisplay *g) {
	// No private area for this controller
	g->priv = 0;

	// Initialise the board interface
	init_board(g);

	/* Hardware reset */
	setpin_reset(g, gTrue);
	gfxSleepMilliseconds(1);
	setpin_reset(g, gFalse);
	gfxSleepMilliseconds(2);

	acquire_bus(g);
	setwritemode(g);

	write_reg(g, 0x00, 0x0001);
	gfxSleepMilliseconds(10);

	write_reg(g, 0x15, 0x0030);
	write_reg(g, 0x11, 0x0040);
	write_reg(g, 0x10, 0x1628);
	write_reg(g, 0x12, 0x0000);
	write_reg(g, 0x13, 0x104d);
	gfxSleepMilliseconds(10);
	write_reg(g, 0x12, 0x0010);
	gfxSleepMilliseconds(10);
	write_reg(g, 0x10, 0x2620);
	write_reg(g, 0x13, 0x344d);
	gfxSleepMilliseconds(10);

	write_reg(g, 0x01, 0x0100);
	write_reg(g, 0x02, 0x0300);
	write_reg(g, 0x03, 0x1030);
	write_reg(g, 0x08, 0x0604);
	write_reg(g, 0x09, 0x0000);
	write_reg(g, 0x0A, 0x0008);

	write_reg(g, 0x41, 0x0002);
	write_reg(g, 0x60, 0x2700);
	write_reg(g, 0x61, 0x0001);
	write_reg(g, 0x90, 0x0182);
	write_reg(g, 0x93, 0x0001);
	write_reg(g, 0xa3, 0x0010);
	gfxSleepMilliseconds(10);

	write_reg(g, 0x30, 0x0000);
	write_reg(g, 0x31, 0x0502);
	write_reg(g, 0x32, 0x0307);
	write_reg(g, 0x33, 0x0305);
	write_reg(g, 0x34, 0x0004);
	write_reg(g, 0x35, 0x0402);
	write_reg(g, 0x36, 0x0707);
	write_reg(g, 0x37, 0x0503);
	write_reg(g, 0x38, 0x1505);
	write_reg(g, 0x39, 0x1505);
	gfxSleepMilliseconds(10);

	write_reg(g, 0x07, 0x0001);
	gfxSleepMilliseconds(10);
	write_reg(g, 0x07, 0x0021);
	write_reg(g, 0x07, 0x0023);
	gfxSleepMilliseconds(10);
	write_reg(g, 0x07, 0x0033);
	gfxSleepMilliseconds(10);
	write_reg(g, 0x07, 0x0133);

	// Finish Init
	post_init_board(g);

 	// Release the bus
	release_bus(g);

	// Turn on the backlight
	set_backlight(g, GDISP_INITIAL_BACKLIGHT);
	
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
	}
	LLDSPEC	void gdisp_lld_write_color(GDisplay *g) {
		write_data(g, gdispColor2Native(g->p.color));
	}
	LLDSPEC	void gdisp_lld_write_stop(GDisplay *g) {
		release_bus(g);
	}
	LLDSPEC void gdisp_lld_write_pos(GDisplay *g) {
		set_cursor(g);
	}
#endif

#if GDISP_HARDWARE_STREAM_READ
	LLDSPEC	void gdisp_lld_read_start(GDisplay *g) {
		acquire_bus(g);
		set_viewport(g);
		set_cursor(g);
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

#if GDISP_NEED_CONTROL && GDISP_HARDWARE_CONTROL
	LLDSPEC void gdisp_lld_control(GDisplay *g) {
		switch(g->p.x) {
		case GDISP_CONTROL_POWER:
			if (g->g.Powermode == (gPowermode)g->p.ptr)
				return;
			switch((gPowermode)g->p.ptr) {
			case gPowerOff:
				acquire_bus(g);
				write_reg(g, 0x07, 0x0000);
				write_reg(g, 0x10, 0x0000);
				write_reg(g, 0x11, 0x0000);
				write_reg(g, 0x12, 0x0000);
				write_reg(g, 0x13, 0x0000);
				release_bus(g);
				set_backlight(g, 0);
				break;

			case gPowerOn:
				//*************Power On sequence ******************//
				acquire_bus(g);
				write_reg(g, 0x10, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
				write_reg(g, 0x11, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
				write_reg(g, 0x12, 0x0000); /* VREG1OUT voltage */
				write_reg(g, 0x13, 0x0000); /* VDV[4:0] for VCOM amplitude */
				gfxSleepMilliseconds(200);            /* Dis-charge capacitor power voltage */
				write_reg(g, 0x10, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
				write_reg(g, 0x11, 0x0147); /* DC1[2:0], DC0[2:0], VC[2:0] */
				gfxSleepMilliseconds(50);
				write_reg(g, 0x12, 0x013C); /* VREG1OUT voltage */
				gfxSleepMilliseconds(50);
				write_reg(g, 0x13, 0x0E00); /* VDV[4:0] for VCOM amplitude */
				write_reg(g, 0x29, 0x0009); /* VCM[4:0] for VCOMH */
				gfxSleepMilliseconds(50);
				write_reg(g, 0x07, 0x0173); /* 262K color and display ON */
				release_bus(g);
				set_backlight(g, g->g.Backlight);
				break;

			case gPowerSleep:
				acquire_bus(g);
				write_reg(g, 0x07, 0x0000); /* display OFF */
				write_reg(g, 0x10, 0x0000); /* SAP, BT[3:0], APE, AP, DSTB, SLP */
				write_reg(g, 0x11, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
				write_reg(g, 0x12, 0x0000); /* VREG1OUT voltage */
				write_reg(g, 0x13, 0x0000); /* VDV[4:0] for VCOM amplitude */
				gfxSleepMilliseconds(200); /* Dis-charge capacitor power voltage */
				write_reg(g, 0x10, 0x0002); /* SAP, BT[3:0], APE, AP, DSTB, SLP */
				release_bus(g);
				set_backlight(g, g->g.Backlight);
				break;

			case gPowerDeepSleep:
				acquire_bus(g);
				write_reg(g, 0x07, 0x0000); /* display OFF */
				write_reg(g, 0x10, 0x0000); /* SAP, BT[3:0], APE, AP, DSTB, SLP */
				write_reg(g, 0x11, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
				write_reg(g, 0x12, 0x0000); /* VREG1OUT voltage */
				write_reg(g, 0x13, 0x0000); /* VDV[4:0] for VCOM amplitude */
				gfxSleepMilliseconds(200); /* Dis-charge capacitor power voltage */
				write_reg(g, 0x10, 0x0004); /* SAP, BT[3:0], APE, AP, DSTB, SLP */
				release_bus(g);
				set_backlight(g, g->g.Backlight);
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
					write_reg(g, 0x01, 0x0100);
					write_reg(g, 0x03, 0x1030);
					write_reg(g, 0x60, 0x2700);
					release_bus(g);
					g->g.Height = GDISP_SCREEN_HEIGHT;
					g->g.Width = GDISP_SCREEN_WIDTH;
					break;

				case gOrientation90:
					acquire_bus(g);
					write_reg(g, 0x01, 0x0100);
					write_reg(g, 0x03, 0x1038);
					write_reg(g, 0x60, 0xA700);
					release_bus(g);
					g->g.Height = GDISP_SCREEN_WIDTH;
					g->g.Width = GDISP_SCREEN_HEIGHT;
					break;

				case gOrientation180:
					acquire_bus(g);
					write_reg(g, 0x01, 0x0000);
					write_reg(g, 0x03, 0x1030);
					write_reg(g, 0x60, 0xa700);
					release_bus(g);
					g->g.Height = GDISP_SCREEN_HEIGHT;
					g->g.Width = GDISP_SCREEN_WIDTH;
					break;
	
				case gOrientation270:
					acquire_bus(g);
					write_reg(g, 0x01, 0x0000);
					write_reg(g, 0x03, 0x1038);
					write_reg(g, 0x60, 0x2700);
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
		
		default:
			return;
		}
	}
#endif

LLDSPEC void gdisp_lld_draw_pixel(GDisplay *g) {
	set_cursor(g);
	gdisp_lld_write_color (g);

}

#if GDISP_HARDWARE_FILLS
LLDSPEC void gdisp_lld_fill_area(GDisplay *g) {
	LLDCOLOR_TYPE c = gdispColor2Native(g->p.color);

	acquire_bus(g);

	// Set view port if drawing more than 1 line, or write not started
	if (g->p.cy != 1 || !ws) {
		set_viewport(g);
	}

	set_cursor(g);
	write_data_repeat (g,c,g->p.cx*g->p.cy);

	// Restore view port if write started and drawed more than 1 line
	if (g->p.cy != 1 && ws)
	{
		write_reg(g, LGDP4532_HORIZONTAL_WINDOW_ADDR2, svx);
		write_reg(g, LGDP4532_HORIZONTAL_WINDOW_ADDR1, svx + svcx - 1);
		write_reg(g, LGDP4532_VERTICAL_WINDOW_ADDR2, svy);
		write_reg(g, LGDP4532_VERTICAL_WINDOW_ADDR1, svy + svcy - 1);
	}
	release_bus(g);
}
#endif // GDISP_HARDWARE_FILLS

#endif /* GFX_USE_GDISP */
