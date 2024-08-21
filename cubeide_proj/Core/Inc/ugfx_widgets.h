/*
 * ugfx_widgets.h
 *
 *  Created on: Mar 2, 2024
 *      Author: user
 */

#ifndef INC_UGFX_WIDGETS_H_
#define INC_UGFX_WIDGETS_H_


#include "gfx.h"

void setColors(uint32_t,uint32_t,uint32_t);

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


typedef struct
{
  int r2Max, xMax, yMax;
  uint32_t peakHold_ms = 2000;
  uint32_t peakHold_ms_last = 0;
}
Gimball_t;
void drawGimball ( Gimball_t*, int x, int y, int r, int xv, int yv);


/* a routine for flashing a widget
 * dont use the flasher_fun() function, use the macro and
 * pass the function call you want to execute as the second argument.
 */
typedef struct
{
  int rate_ms;
  int last_ms;
  bool state;
}
flasher_t;
bool flasher_fun(flasher_t*);
#define flasher(flasher_struct, arg) if(flasher_fun(flasher_struct)) arg;


typedef struct
{
  int *data;
  int len = -1;
  int indLatest = -1;
  int indFirst = -1;
  float scalex, scaley;
  bool isInit = false;
  uint32_t color_mode = 0; // 0 means use default, otherwise its a color
  int lineWidth = 2;
  //uint32_t color = GFX_AMBER;
}
LinePlot_t;
void linePlotInit(
    LinePlot_t*linePlot,
    int *data_buffer_ptr, int data_buffer_len,
    int width, int height, int maxVal,
    uint32_t color_mode);
void linePlotPush(LinePlot_t *, int val);
void linePlot(int x, int y, LinePlot_t *);

#endif /* INC_UGFX_WIDGETS_H_ */
