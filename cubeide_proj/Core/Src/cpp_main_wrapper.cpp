
/*
 * a c-externed wrapped for cpp functions
 */

#include "main_cpp_wrapper.h"

extern int main_cpp();

int main_cpp_c()
{
  return main_cpp();
}
