#include "cpp_main.h"
#include "gfx.h"
#include <chrono>
#include <thread>

extern "C" void main_cpp()
{
  gfxInit();

  // Keep the host window alive for now; replace with real app loop.
  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(16));
  }
}
