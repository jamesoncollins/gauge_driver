/*
 * This file is subject to the terms of the GFX License. If a copy of
 * the license was not distributed with this file, you can obtain one at:
 *
 *              http://ugfx.io/license.html
 */

#ifndef _GDISP_LLD_CONFIG_H
#define _GDISP_LLD_CONFIG_H

#if GFX_USE_GDISP

/*===========================================================================*/
/* Driver hardware support.                                                  */
/*===========================================================================*/

//#define GDISP_HARDWARE_STREAM_WRITE		GFXON
////#define GDISP_HARDWARE_STREAM_READ		GFXON
//#define GDISP_HARDWARE_CONTROL			GFXON
//
//#define GDISP_LLD_PIXELFORMAT			GDISP_PIXELFORMAT_RGB565



#define GDISP_HARDWARE_FLUSH                    GFXON           // This controller requires flushing
#define GDISP_HARDWARE_DRAWPIXEL                GFXON
//#define GDISP_HARDWARE_PIXELREAD                GFXON
//#define GDISP_HARDWARE_CONTROL                  GFXON
#define GDISP_HARDWARE_FILLS                    GFXON
#define GDISP_HARDWARE_CLEARS                   GFXON

#define GDISP_LLD_PIXELFORMAT                   GDISP_PIXELFORMAT_RGB565

//#define GDISP_CONTROL_INVERSE                   (GDISP_CONTROL_LLD+0)

#endif	/* GFX_USE_GDISP */

#endif	/* _GDISP_LLD_CONFIG_H */
