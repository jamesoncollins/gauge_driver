
/*
 * a c-externed wrapped for cpp functions
 */

#include "_main_cpp.h"

extern int main_cpp();

int main_cpp_c()
{
  return main_cpp();
}
