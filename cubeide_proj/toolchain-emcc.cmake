set(CMAKE_SYSTEM_NAME Emscripten)
set(CMAKE_SYSTEM_PROCESSOR wasm32)

find_program(CMAKE_C_COMPILER NAMES emcc.bat emcc)
find_program(CMAKE_CXX_COMPILER NAMES em++.bat em++ emxx.bat emxx)
find_program(CMAKE_AR NAMES emar.bat emar)
find_program(CMAKE_RANLIB NAMES emranlib.bat emranlib)

if((NOT CMAKE_C_COMPILER OR NOT CMAKE_CXX_COMPILER) AND DEFINED ENV{EMSDK})
  set(_emscripten_root "$ENV{EMSDK}/upstream/emscripten")
  find_program(CMAKE_C_COMPILER NAMES emcc.bat emcc HINTS "${_emscripten_root}" NO_DEFAULT_PATH)
  find_program(CMAKE_CXX_COMPILER NAMES em++.bat em++ emxx.bat emxx HINTS "${_emscripten_root}" NO_DEFAULT_PATH)
  find_program(CMAKE_AR NAMES emar.bat emar HINTS "${_emscripten_root}" NO_DEFAULT_PATH)
  find_program(CMAKE_RANLIB NAMES emranlib.bat emranlib HINTS "${_emscripten_root}" NO_DEFAULT_PATH)
endif()

if(NOT CMAKE_C_COMPILER OR NOT CMAKE_CXX_COMPILER)
  message(FATAL_ERROR "Emscripten compiler not found. Run scripts/setup-msys2-ucrt.sh and source scripts/env-msys2-ucrt.sh from MSYS2 UCRT64 so emcc and em++ are on PATH, or set EMSDK before configuring.")
endif()

set(CMAKE_EXECUTABLE_SUFFIX ".html")
