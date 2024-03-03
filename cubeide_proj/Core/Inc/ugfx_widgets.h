/*
 * ugfx_widgets.h
 *
 *  Created on: Mar 2, 2024
 *      Author: user
 */

#ifndef INC_UGFX_WIDGETS_H_
#define INC_UGFX_WIDGETS_H_


#include "gfx.h"

#ifndef COLOR_PRIMARY
#define COLOR_PRIMARY GFX_AMBER
#endif

#ifndef COLOR_SECONDARY
#define COLOR_SECONDARY GFX_RED
#endif

#ifndef COLOR_BACKGROUND
#define COLOR_BACKGROUND GFX_BLACK
#endif

/*
 *
 * TODO: detect GDISP_HARDWARE_FILLS and fill boxes isntead of drawing lines
 */
void drawBarGraph (
    int x, int y,
    int width, int height,
    float max, float min,
    float val,
    bool vert
    );

void drawVertBarGraph (
    int x, int y,
    int width, int height,
    float max, float min,
    float val
    );

void drawHorzBarGraph (
    int x, int y,
    int width, int height,
    float max, float min,
    float val
    );

bool dissolve(int x, int y, int width, int height, int iter);

void drawGimball (int x, int y, int r, int xv, int yv);

#endif /* INC_UGFX_WIDGETS_H_ */
