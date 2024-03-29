/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.io/license.html
 */
 
#ifndef GDISP_LLD_CONFIG_H
#define GDISP_LLD_CONFIG_H

#if GFX_USE_GDISP

#define GDISP_HARDWARE_STREAM_WRITE		GFXON
#define GDISP_HARDWARE_STREAM_READ		GFXON
#define GDISP_HARDWARE_STREAM_POS		GFXON
#define GDISP_HARDWARE_CONTROL			GFXON
#define GDISP_HARDWARE_FILLS			GFXON

#define GDISP_LLD_PIXELFORMAT			GDISP_PIXELFORMAT_RGB565

/* Horizontal Address Start Position */
#define LGDP4532_HORIZONTAL_WINDOW_ADDR1	(0x50u)
/* Horizontal Address End Position */
#define LGDP4532_HORIZONTAL_WINDOW_ADDR2	(0x51u)
/* Vertical Address Start Position */
#define LGDP4532_VERTICAL_WINDOW_ADDR1  	(0x52u)
/* Vertical Address End Position */
#define LGDP4532_VERTICAL_WINDOW_ADDR2  	(0x53u)

#endif	/* GFX_USE_GDISP */
#endif	/* _GDISP_LLD_CONFIG_H */
