# CMake toolchain wrapper for cross-compiling MMDBulletPhysicsNativeContext
# for Android aarch64 (arm64-v8a) using the NDK.
#
# Selected automatically by CMakeLists.txt when ANDROID_NDK is set, so it
# does not need to be passed on the command line. The NDK root is NOT hard-coded
#  pass it via ANDROID_NDK, e.g. from the repo root:
#
#   cmake -S . -B build-android-arm64 -G Ninja ^
#       -DANDROID_NDK="PATH TO ANDROID NDK" ^
#       -DCMAKE_BUILD_TYPE=Release
#   cmake --build build-android-arm64 --config Release
#
# ANDROID_ABI / ANDROID_PLATFORM / ANDROID_STL can be overridden on the
# command line; defaults target arm64-v8a, API 24, and a self-contained
# c++_static runtime.

# Forward ANDROID_NDK into CMake's internal try_compile tests so the
# NDK toolchain include resolves during compiler ABI detection.
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES ANDROID_NDK)

if(NOT ANDROID_NDK)
    message(FATAL_ERROR
        "ANDROID_NDK must point at the Android NDK root, "
        "e.g. \".../Editor/Data/PlaybackEngines/AndroidPlayer/NDK\". "
        "Pass it on the cmake command line: -DANDROID_NDK=<path>.")
endif()

set(_mmdbullet_android_toolchain "${ANDROID_NDK}/build/cmake/android.toolchain.cmake")
if(NOT EXISTS "${_mmdbullet_android_toolchain}")
    message(FATAL_ERROR
        "Android NDK toolchain not found at: ${_mmdbullet_android_toolchain}\n"
        "Check that ANDROID_NDK points at the NDK root.")
endif()

if(NOT ANDROID_ABI)
    set(ANDROID_ABI "arm64-v8a" CACHE STRING "Android target ABI")
endif()
if(NOT ANDROID_PLATFORM)
    set(ANDROID_PLATFORM "android-24" CACHE STRING "Android minimum API level")
endif()
if(NOT ANDROID_STL)
    set(ANDROID_STL "c++_static" CACHE STRING "Android C++ STL")
endif()

include("${_mmdbullet_android_toolchain}")
