#if !defined(__EMSCRIPTEN__)
#error "This platform entrypoint is only for Emscripten builds"
#endif

#include <cstdint>

#include "cpp_main.h"

extern "C" uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
  (void)Buf;
  (void)Len;
  return 0;
}

int main()
{
  main_cpp();
  return 0;
}