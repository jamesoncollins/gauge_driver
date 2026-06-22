#if !defined(__EMSCRIPTEN__)
#error "This platform entrypoint is only for Emscripten builds"
#endif

#include <cstdint>

#include <emscripten.h>

#include "cpp_main.h"

extern "C" uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
  (void)Buf;
  (void)Len;
  return 0;
}

namespace
{
void main_loop_step()
{
  main_cpp_step();
}
}

int main()
{
  main_cpp();
  emscripten_set_main_loop(main_loop_step, 0, 1);
  return 0;
}
