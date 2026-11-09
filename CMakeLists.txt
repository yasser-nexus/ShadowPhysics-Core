# =============================================================================
#  FH4 Realistic Physics Overhaul (FH4RPO)
#  Root build script.
#
#  Targets
#  -------
#    fh4rpo            -> the module.  On Windows it produces FH4RPO.asi (the
#                         extension ASI loaders expect) *and* copies it to
#                         FH4RPO.dll so both loader styles work.
#    fh4rpo_core       -> portable physics kernel (static library)
#    fh4rpo_tests      -> self-contained unit tests for the kernel; builds and
#                         runs on Windows *and* on Linux/macOS, so the physics
#                         can be validated without the game.
#
#  Build (Windows, Visual Studio 2019/2022):
#      cmake -S . -B build -G "Visual Studio 17 2022" -A x64
#      cmake --build build --config Release
#
#  Build (Windows, clang-cl - what the CI image uses):
#      set CC=clang-cl
#      set CXX=clang-cl
#      cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
#      cmake --build build
#
#  Build (Linux/macOS - kernel tests only):
#      cmake -S . -B build && cmake --build build && ctest --test-dir build
# =============================================================================
cmake_minimum_required(VERSION 3.16)

project(FH4RPO
        VERSION 1.0.0
        DESCRIPTION "Forza Horizon 4 - Realistic Physics Overhaul"
        LANGUAGES C CXX)

# -----------------------------------------------------------------------------
#  Options
# -----------------------------------------------------------------------------
option(FH4RPO_BUILD_MODULE  "Build the ASI/DLL module (Windows only)" ON)
option(FH4RPO_BUILD_TESTS   "Build the portable physics unit tests"   ON)
option(FH4RPO_WARNINGS_AS_ERRORS "Treat warnings as errors"           OFF)
option(FH4RPO_USE_MINGW     "Use MinGW-w64 instead of MSVC/clang-cl"  OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Keep everything in one predictable place.
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

include(GNUInstallDirs)

# -----------------------------------------------------------------------------
#  Sources
# -----------------------------------------------------------------------------
set(FH4RPO_CORE_SOURCES
    src/Log.cpp
    src/Config.cpp
    src/PhysicsEngine.cpp
    src/Telemetry.cpp
    src/MemoryGuard.cpp
)

set(FH4RPO_MODULE_SOURCES
    src/dllmain.cpp
    src/Module.cpp
    src/Hooks.cpp
    src/GameBinds.cpp
    src/PatternScanner.cpp
    src/HookEngine.cpp
    third_party/minhook/src/buffer.c
    third_party/minhook/src/hook.c
    third_party/minhook/src/trampoline.c
    third_party/minhook/src/hde/hde64.c
)

set(FH4RPO_PUBLIC_HEADERS
    include/fh4rpo/Core.hpp
    include/fh4rpo/Config.hpp
    include/fh4rpo/Log.hpp
    include/fh4rpo/VehicleState.hpp
    include/fh4rpo/Telemetry.hpp
    include/fh4rpo/MemoryGuard.hpp
    include/fh4rpo/HookEngine.hpp
    include/fh4rpo/PatternScanner.hpp
    include/fh4rpo/GameBinds.hpp
    include/fh4rpo/PhysicsEngine.hpp
    include/fh4rpo/Module.hpp
)

# -----------------------------------------------------------------------------
#  Portable kernel
# -----------------------------------------------------------------------------
add_library(fh4rpo_core STATIC ${FH4RPO_CORE_SOURCES} ${FH4RPO_PUBLIC_HEADERS})

target_include_directories(fh4rpo_core
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/src")

target_compile_features(fh4rpo_core PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(fh4rpo_core PRIVATE /W4 /permissive- /EHsc /utf-8)
    target_compile_definitions(fh4rpo_core PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
    target_compile_definitions(fh4rpo_core PUBLIC  FH4RPO_PORTABLE=1)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(fh4rpo_core PRIVATE -Wall -Wextra -Wno-unused-parameter)
    target_compile_definitions(fh4rpo_core PUBLIC FH4RPO_PORTABLE=1)
endif()

# Telemetry needs the socket library on Windows for both the module and the
# test binary (the core static library keeps its dependencies PRIVATE, so the
# consumers must carry them).
if(WIN32)
    target_link_libraries(fh4rpo_core PUBLIC ws2_32)
endif()

if(FH4RPO_WARNINGS_AS_ERRORS)
    if(MSVC)
        target_compile_options(fh4rpo_core PRIVATE /WX)
    else()
        target_compile_options(fh4rpo_core PRIVATE -Werror)
    endif()
endif()

# -----------------------------------------------------------------------------
#  Windows only: the module itself
# -----------------------------------------------------------------------------
if(WIN32 AND FH4RPO_BUILD_MODULE)

    add_library(fh4rpo SHARED ${FH4RPO_MODULE_SOURCES} ${FH4RPO_PUBLIC_HEADERS})

    target_link_libraries(fh4rpo PRIVATE fh4rpo_core)
    target_link_libraries(fh4rpo PRIVATE psapi ws2_32 d3d11 dxgi user32)

    target_include_directories(fh4rpo
        PRIVATE
            "${CMAKE_CURRENT_SOURCE_DIR}/src"
            "${CMAKE_CURRENT_SOURCE_DIR}/third_party/minhook/include")

    set_target_properties(fh4rpo PROPERTIES
        PREFIX ""
        OUTPUT_NAME "FH4RPO"
        # MinGW produces libFH4RPO.dll by default - force the plain name.
        SUFFIX ".dll"
        WINDOWS_EXPORT_ALL_SYMBOLS OFF)

    if(MSVC)
        target_compile_options(fh4rpo PRIVATE /W4 /permissive- /EHsc /utf-8
                                             /guard:cf /Gy /Gw)
        target_compile_definitions(fh4rpo PRIVATE
            _CRT_SECURE_NO_WARNINGS
            NOMINMAX
            WIN32_LEAN_AND_MEAN
            FH4RPO_MODULE_BUILD=1)
        # The module must not pull in the C++ runtime's exception frame magic
        # from the static CRT when injected next to a game that ships its own.
        target_link_options(fh4rpo PRIVATE /DYNAMICBASE /NXCOMPAT)
        set_property(TARGET fh4rpo PROPERTY
                     MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
    else()
        target_compile_options(fh4rpo PRIVATE -Wall -Wextra -Wno-unused-parameter
                                              -fms-extensions)
        target_compile_definitions(fh4rpo PRIVATE
            WIN32_LEAN_AND_MEAN
            FH4RPO_MODULE_BUILD=1)
        target_link_options(fh4rpo PRIVATE
            -static-libgcc
            -static
            -Wl,--enable-stdcall-fixup)
    endif()

    # ---------------------------------------------------------------------
    #  Post build: produce the .asi next to the .dll and lay out a ready to
    #  use folder (module + default INI) in <build>/dist.
    # ---------------------------------------------------------------------
    add_custom_command(TARGET fh4rpo POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/dist"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:fh4rpo>"
                "${CMAKE_BINARY_DIR}/dist/FH4RPO.asi"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "$<TARGET_FILE:fh4rpo>"
                "${CMAKE_BINARY_DIR}/dist/FH4RPO.dll"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_CURRENT_SOURCE_DIR}/assets/FH4RPO.ini"
                "${CMAKE_BINARY_DIR}/dist/FH4RPO.ini"
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_CURRENT_SOURCE_DIR}/README.md"
                "${CMAKE_BINARY_DIR}/dist/README.md"
        COMMENT "FH4RPO: staging dist/ (FH4RPO.asi + FH4RPO.dll + FH4RPO.ini)")

    install(TARGETS fh4rpo RUNTIME DESTINATION ".")
    install(FILES assets/FH4RPO.ini DESTINATION ".")

endif()

# -----------------------------------------------------------------------------
#  Tests (portable)
# -----------------------------------------------------------------------------
if(FH4RPO_BUILD_TESTS)
    enable_testing()

    add_executable(fh4rpo_tests tests/PhysicsTests.cpp)
    target_link_libraries(fh4rpo_tests PRIVATE fh4rpo_core)
    target_include_directories(fh4rpo_tests PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/include")

    if(MSVC)
        target_compile_options(fh4rpo_tests PRIVATE /W4 /EHsc /utf-8)
        target_compile_definitions(fh4rpo_tests PRIVATE _CRT_SECURE_NO_WARNINGS NOMINMAX)
    else()
        target_compile_options(fh4rpo_tests PRIVATE -Wall -Wextra -Wno-unused-parameter)
    endif()

    add_test(NAME fh4rpo_physics_tests COMMAND fh4rpo_tests)
    set_tests_properties(fh4rpo_physics_tests PROPERTIES
                         PASS_REGULAR_EXPRESSION "ALL TESTS PASSED")
endif()

# -----------------------------------------------------------------------------
#  Summary
# -----------------------------------------------------------------------------
message(STATUS "")
message(STATUS "FH4RPO ${PROJECT_VERSION}")
message(STATUS "  compiler      : ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  build type    : ${CMAKE_BUILD_TYPE}")
message(STATUS "  module        : ${FH4RPO_BUILD_MODULE} (platform ${CMAKE_SYSTEM_NAME})")
message(STATUS "  tests         : ${FH4RPO_BUILD_TESTS}")
if(WIN32 AND FH4RPO_BUILD_MODULE)
    message(STATUS "  output        : ${CMAKE_BINARY_DIR}/dist/FH4RPO.asi")
endif()
message(STATUS "")

# -----------------------------------------------------------------------------
#  Developer helper: regenerate assets/FH4RPO.ini from the code defaults
# -----------------------------------------------------------------------------
if(FH4RPO_BUILD_TESTS)
    add_executable(fh4rpo_gen_ini EXCLUDE_FROM_ALL tools/GenDefaultIni.cpp)
    target_link_libraries(fh4rpo_gen_ini PRIVATE fh4rpo_core)
    add_custom_target(regenerate_ini
        COMMAND fh4rpo_gen_ini "${CMAKE_CURRENT_SOURCE_DIR}/assets/FH4RPO.ini"
        DEPENDS fh4rpo_gen_ini
        COMMENT "Regenerating assets/FH4RPO.ini from defaults")
endif()
