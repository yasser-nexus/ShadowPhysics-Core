# =============================================================================
#  FH4RPO - MinGW-w64 cross toolchain
#
#  Lets the Windows module be built and *link-checked* on a Linux CI box:
#
#      cmake -S . -B build-win -G Ninja \
#            -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-mingw-w64.cmake \
#            -DCMAKE_BUILD_TYPE=Release
#      cmake --build build-win
#
#  Note: MinGW/GCC has no MSVC style SEH, so the guards fall back to the
#  portable try/catch path.  For shipping builds use Visual Studio or
#  clang-cl, where FH4RPO_SEH == 1 and the __try/__except frames are real.
# =============================================================================
set(CMAKE_SYSTEM_NAME      Windows)
set(CMAKE_SYSTEM_PROCESSOR x86_64)

set(TOOLCHAIN_PREFIX x86_64-w64-mingw32)

set(CMAKE_C_COMPILER   ${TOOLCHAIN_PREFIX}-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_PREFIX}-g++)
set(CMAKE_RC_COMPILER  ${TOOLCHAIN_PREFIX}-windres)

set(CMAKE_FIND_ROOT_PATH /usr/${TOOLCHAIN_PREFIX})

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Static runtime so the module has no external CRT dependency inside the game.
set(CMAKE_CXX_FLAGS_INIT "-static-libgcc -static-libstdc++")
