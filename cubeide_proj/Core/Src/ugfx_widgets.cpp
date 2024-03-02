
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

  int barMax = true ? height - 2 * g : width - 2 * g;
  int barMag = round (barMax * percent);

  for (int i = 0; i < t; i++)
    gdispDrawBox(
        x + i,
        y + i,
        width - i * 2,
        height - i * 2,
        GFX_YELLOW);

  if(true)
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


