
#include "stm32wbxx_hal.h"

#include "ugfx_widgets.h"
#include "gfx.h"
#include "math.h"

uint32_t COLOR_PRIMARY = GFX_AMBER_YEL;
uint32_t COLOR_SECONDARY = GFX_RED;
uint32_t COLOR_BACKGROUND = GFX_BLACK;

void setColors(uint32_t primary, uint32_t secondary, uint32_t background)
{
  COLOR_PRIMARY = primary;
  COLOR_SECONDARY = secondary;
  COLOR_BACKGROUND = background;
}

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
    )
{
  int b = 1; // boarder gap
  int t = 1; // border thickness
  int g = b + t; // the gap from the outer edge of the box to the edge of the bar
  bool alert = val>=max ? true : false;
  val = val > max ? max : val;
  val = val < min ? min : val;
  float percent = (val - min) / (max - min);

  int barMax = vert ? height - 2 * g : width - 2 * g;
  int barMag = round (barMax * percent);

  for (int i = 0; i < t; i++)
    gdispDrawBox(
        x + i,
        y + i,
        width - i * 2,
        height - i * 2,
        COLOR_PRIMARY);

  if(vert)
  {
    gdispFillArea(
        x + g,
        y + g + (barMax - barMag),
        width - 2 * g,
        barMag,
        alert ? COLOR_SECONDARY : COLOR_PRIMARY );
  }
  else
  {
    gdispFillArea(
        x + g,
        y + g,
        barMag,
        height - 2 * g,
        alert ? COLOR_SECONDARY : COLOR_PRIMARY );
  }
}

void drawVertBarGraph (
    int x, int y,
    int width, int height,
    float max, float min,
    float val
    )
{
  drawBarGraph(x,y,width,height,max,min,val,true);
}

void drawHorzBarGraph (
    int x, int y,
    int width, int height,
    float max, float min,
    float val
    )
{
  drawBarGraph(x,y,width,height,max,min,val,false);
}

//bool dissolve(int x, int y, int width, int height, int iter)
//{
//  for(int i=0; i<width; i+=iter)
//  {
//    for(int j=0; j<height; j+=iter)
//    {
//      gdispFillArea(x+i, y+j, iter, iter, GFX_BLACK);
//    }
//  }
//}


/*
 * xy, coordinates of center
 * r, radius of outtermost ring
 * xy and yv, wher ethe gimbal is
 */
void drawGimball (int x, int y, int r, int xv, int yv)
{
  const int w = 1;
  gdispDrawThickLine( x, y, x+r, y+0, COLOR_PRIMARY, w, false);
  gdispDrawThickLine( x, y, x+0, y+r, COLOR_PRIMARY, w, false);
  gdispDrawThickLine( x, y, x-r, y+0, COLOR_PRIMARY, w, false);
  gdispDrawThickLine( x, y, x-0, y-r, COLOR_PRIMARY, w, false);
  while(r>(w<<2))
  {
    gdispDrawCircle (x, y, r, COLOR_PRIMARY);
    r = r>>1;
  }
  gdispFillCircle (x+xv, y+yv, w<<2, COLOR_SECONDARY);

}

bool flasher_fun(flasher_t *flasher)
{
 int now = HAL_GetTick();
  if(flasher->last_ms+flasher->rate_ms>now)
  {
    flasher->last_ms = now;
    return true;
  }
  return false;

}




void linePlotInit(LinePlot_t*linePlot, int *data_buffer_ptr, int data_buffer_len)
{
  linePlot->data = data_buffer_ptr;
  linePlot->len = data_buffer_len;
  linePlot->indFirst = 0;
  linePlot->indLatest = 0;
  linePlot->isInit = true;
}

void linePlotPush(LinePlot_t *linePlot, int val)
{
  if(!linePlot->isInit)
    return;

  linePlot->indLatest++;
  linePlot->indLatest =  (linePlot->indLatest>=linePlot->len) ? 0 : linePlot->indLatest;
  if(linePlot->indLatest == linePlot->indFirst)
    linePlot->indFirst = (linePlot->indLatest+1) % linePlot->len;
  linePlot->data[linePlot->indLatest] = val;
}

void linePlot(int x, int y, int width, int height, LinePlot_t *linePlot)
{
  if(!linePlot->isInit)
    return;
  int scalex = 3, scaley = 1;
  int ind = (linePlot->indFirst+0) % linePlot->len;
  int from = linePlot->data[ind];
  for(int i=0; i<linePlot->len; i++)
  {
    ind = (ind+1) % linePlot->len;
    int to = linePlot->data[ind];
    gdispDrawThickLine(
        x+i*scalex,     y-from*scaley,
        x+(i+1)*scalex, y-to*scaley,
        COLOR_PRIMARY,
        2,
        false);
    from = to;
  }
}
