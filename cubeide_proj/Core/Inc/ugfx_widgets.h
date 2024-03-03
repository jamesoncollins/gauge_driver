/*
 * ugfx_widgets.h
 *
 *  Created on: Mar 2, 2024
 *      Author: user
 */

#ifndef INC_UGFX_WIDGETS_H_
#define INC_UGFX_WIDGETS_H_

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
