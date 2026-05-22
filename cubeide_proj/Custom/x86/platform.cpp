
#ifndef __x86_64__
#error "THIS PLATFORM IS ONLY FOR X86"
#endif

#include <cstdio>
#include <cstdint>

#include "cpp_main.h"

extern "C" uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len)
{
  if (Buf == nullptr || Len == 0)
    return 1;

  const size_t written = std::fwrite(Buf, 1, Len, stdout);
  std::fflush(stdout);
  return (written == Len) ? 0 : 1;
}

int main()
{
  main_cpp();
  return 0;
}
