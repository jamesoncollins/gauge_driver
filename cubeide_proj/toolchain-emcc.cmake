set(CMAKE_SYSTEM_NAME Emscripten)
set(CMAKE_SYSTEM_PROCESSOR wasm32)

set(_emscripten_root "$ENV{EMSDK}/upstream/emscripten")

find_program(CMAKE_C_COMPILER
  NAMES emcc.bat emcc
  HINTS "${_emscripten_root}"
)
find_program(CMAKE_CXX_COMPILER
  NAMES em++.bat em++ emxx.bat emxx
  HINTS "${_emscripten_root}"
)
find_program(CMAKE_AR
  NAMES emar.bat emar
  HINTS "${_emscripten_root}"
)
find_program(CMAKE_RANLIB
  NAMES emranlib.bat emranlib
  HINTS "${_emscripten_root}"
)

if(NOT CMAKE_C_COMPILER OR NOT CMAKE_CXX_COMPILER)
  message(FATAL_ERROR "Emscripten compiler not found. Install/activate emsdk so emcc and em++ are on PATH, or set EMSDK before configuring.")
endif()

set(CMAKE_EXECUTABLE_SUFFIX ".html")
