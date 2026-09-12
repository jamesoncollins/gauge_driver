
#ifndef __x86_64__
#error "THIS PLATFORM IS ONLY FOR X86"
#endif

#include <cstdio>
#include <cstdint>
#include <thread>

#include "cpp_main.h"
#include "sim_control.hpp"

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
  std::thread([]() {
    std::fprintf(stderr, "sim console ready. Type 'help' for commands.\n");

    char line[64];
    while (std::fgets(line, sizeof(line), stdin) != nullptr)
    {
      if (sim_control_line_is_help(line))
      {
        std::fprintf(stderr, "%s", sim_control_help_text());
      }
      else if (sim_control_handle_line(line))
      {
        std::fprintf(stderr, "sim command accepted: %s", line);
      }
      else
      {
        std::fprintf(stderr, "sim command not recognized: %s", line);
        std::fprintf(stderr, "type 'help' for commands.\n");
      }
    }
  }).detach();

  main_cpp();
  return 0;
}
