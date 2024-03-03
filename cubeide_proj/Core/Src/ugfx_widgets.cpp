
#include "gfx.h"
#include "math.h"

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
        GFX_YELLOW);

  if(vert)
  {
    gdispFillArea(
        x + g,
        y + g + (barMax - barMag),
        width - 2 * g,
        barMag,
        GFX_YELLOW);
  }
  else
  {
    gdispFillArea(
        x + g,
        y + g,
        barMag,
        height - 2 * g,
        GFX_YELLOW);
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

bool dissolve(int x, int y, int width, int height, int iter)
{
  for(int i=0; i<width; i+=iter)
  {
    for(int j=0; j<height; j+=iter)
    {
      gdispFillArea(x+i, y+j, iter, iter, GFX_BLACK);
    }
  }
}


/*
 * xy, coordinates of center
 * r, radius of outtermost ring
 * xy and yv, wher ethe gimbal is
 */
void drawGimball (int x, int y, int r, int xv, int yv)
{
  const int w = 1;
  gdispDrawThickLine( x, y, x+r, y+0, GFX_YELLOW, w, false);
  gdispDrawThickLine( x, y, x+0, y+r, GFX_YELLOW, w, false);
  gdispDrawThickLine( x, y, x-r, y+0, GFX_YELLOW, w, false);
  gdispDrawThickLine( x, y, x-0, y-r, GFX_YELLOW, w, false);
  while(r>(w<<2))
  {
    gdispDrawCircle (x, y, r, GFX_YELLOW);
    r = r>>1;
  }
  gdispFillCircle (x+xv, y+yv, w<<2, GFX_YELLOW);

}


