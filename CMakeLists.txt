cmake_minimum_required(VERSION 3.24)

# NDK root for Android cross-compilation, passed on the command line, e.g.
# -DANDROID_NDK=<...>.
# Defined before project() so the toolchain activation below runs first.
set(ANDROID_NDK "" CACHE PATH "Android NDK root. Set this to cross-compile for Android arm64-v8a")

# When an NDK is provided, activate the Android toolchain before project() so no -DCMAKE_TOOLCHAIN_FILE is needed on the command line. CMake re-applies this toolchain during try_compile; the wrapper forwards ANDROID_NDK there.
if(ANDROID_NDK AND NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/Toolchains/Android-aarch64.cmake" CACHE FILEPATH "Android NDK toolchain file")
endif()

project(UMTNativePlugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Fixed official 3.07 tag commit. Its upstream VERSION/BT_BULLET_VERSION still say
# 3.05/305, so verify the commit and clean source instead of trusting that string.
set(UMT_BULLET_COMMIT "272c7099d3ba8ac9e8c142e361e8d4cf5d91b8f2")
set(BulletRoot "" CACHE PATH "Clean checkout of the pinned Bullet 3.07 commit; empty fetches src into the build cache.")
find_package(Git REQUIRED)
set(_bullet_fetch FALSE)
if(NOT BulletRoot)
    set(BulletRoot "${CMAKE_CURRENT_BINARY_DIR}/_deps/bullet307-src")
    set(_bullet_fetch TRUE)
endif()
execute_process(COMMAND "${CMAKE_COMMAND}"
    "-DBULLET_SOURCE_DIR=${BulletRoot}" "-DBULLET_COMMIT=${UMT_BULLET_COMMIT}"
    "-DGIT_EXECUTABLE=${GIT_EXECUTABLE}" "-DBULLET_FETCH_ALLOWED=${_bullet_fetch}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/FetchPinnedBullet.cmake"
    RESULT_VARIABLE _bullet_result)
if(NOT _bullet_result EQUAL 0)
    message(FATAL_ERROR "Pinned Bullet preparation failed; existing source/cache files were preserved.")
endif()
message(STATUS "Using Bullet 3.07 commit ${UMT_BULLET_COMMIT} at ${BulletRoot}")

# Path to the UMT Unity package. Empty by default: the standard build produces a standalone plugin in the build tree with no Unity coupling. Set this to a UMT package folder to also copy the finished binary into its Plugins/ folder via a
# POST_BUILD step so Unity can hot-reload it:
#   -DUMT_PACKAGE_DIR=/path/to/com.candidumgames.unitymmdtools
set(UMT_PACKAGE_DIR "" CACHE PATH "UMT Unity package folder to copy the built plugin into. Empty (default) skips the copy.")

if(EMSCRIPTEN)
    set(_default_unity_plugin_output "${UMT_PACKAGE_DIR}/Plugins/Web")
elseif(ANDROID)
    set(_default_unity_plugin_output "${UMT_PACKAGE_DIR}/Plugins/Android/${ANDROID_ABI}")
else()
    set(_default_unity_plugin_output "${UMT_PACKAGE_DIR}/Plugins/Windows/x64")
endif()
set(UnityPluginOutput "${_default_unity_plugin_output}" CACHE PATH "Unity plugin output directory")

set(BULLET_VERSION 3.07)
set(BULLET_PHYSICS_SOURCE_DIR "${BulletRoot}")

add_compile_definitions(BT_USE_DOUBLE_PRECISION)

add_subdirectory("${BulletRoot}/src/LinearMath" "${CMAKE_CURRENT_BINARY_DIR}/Bullet/LinearMath")
add_subdirectory("${BulletRoot}/src/BulletCollision" "${CMAKE_CURRENT_BINARY_DIR}/Bullet/BulletCollision")
add_subdirectory("${BulletRoot}/src/BulletDynamics" "${CMAKE_CURRENT_BINARY_DIR}/Bullet/BulletDynamics")

# Static-library mode for hosts that link this plugin directly through their own build system (e.g. Unreal Engine via UBT) rather than loading a shared library at runtime. OFF by default so the Unity desktop/Android targets keep shipping a hot-loadable .dll / .so; the Web target is always static because Unity's IL2CPP/Emscripten player links native code statically at build time.
option(UMT_STATIC "Build UMTNativePlugin as a static library linked directly by the host (e.g. Unreal Engine)." OFF)

if(UMT_STATIC OR EMSCRIPTEN)
    add_library(UMTNativePlugin STATIC
        Source/NativeContext.cpp
        Source/NativeContext.h
    )
else()
    add_library(UMTNativePlugin SHARED
        Source/NativeContext.cpp
        Source/NativeContext.h
    )
endif()

# UMT_NATIVE_STATIC drops __declspec(dllexport) from the ABI macro so the archive exposes plain extern "C" symbols for the host's linker. Deliberately NOT set for the Emscripten static target, which still needs EMSCRIPTEN_KEEPALIVE to survive dead-stripping.
if(UMT_STATIC)
    target_compile_definitions(UMTNativePlugin PRIVATE UMT_NATIVE_STATIC)
endif()

target_include_directories(UMTNativePlugin SYSTEM PRIVATE
    "${BulletRoot}/src"
)

target_link_libraries(UMTNativePlugin PRIVATE
    BulletDynamics
    BulletCollision
    LinearMath
)

if(ANDROID)
    # NDK r27's -static-libstdc++ (emitted for c++_static) links static libc++
    # but omits libc++abi, leaving __cxa_guard_abort, the RTTI typeinfo vtables,
    # and __gxx_personality_v0 undefined. Link libc++ explicitly.
    target_link_libraries(UMTNativePlugin PRIVATE c++)
    target_link_options(UMTNativePlugin PRIVATE
    "-Wl,-z,max-page-size=16384"
    "-Wl,-z,common-page-size=16384"
)
endif()

if(WIN32)
    target_compile_definitions(UMTNativePlugin PRIVATE
        WIN32_LEAN_AND_MEAN
        NOMINMAX
    )
endif()

if(MSVC)
    set(_MSVC_SUPPRESSED_WARNINGS /wd4514 /wd4820 /wd4626 /wd4100 /wd4710 /wd4365 /wd5291 /wd4625 /wd5027 /wd5045 /wd4711)
    target_compile_options(UMTNativePlugin PRIVATE /WX /Wall ${_MSVC_SUPPRESSED_WARNINGS} /permissive-)
endif()

option(UMT_REPRODUCIBLE_MSVC "Normalize MSVC source paths/timestamps (tested with MSVC 14.44)." OFF)
if(MSVC AND UMT_REPRODUCIBLE_MSVC)
    # MSVC pathmap compares native backslashes; a forward-slash prefix is silently ineffective.
    file(TO_NATIVE_PATH "${CMAKE_CURRENT_SOURCE_DIR}" _umt_source_native)
    foreach(_target UMTNativePlugin LinearMath BulletCollision BulletDynamics)
        target_compile_options(${_target} PRIVATE /experimental:deterministic /Brepro
            "/pathmap:${_umt_source_native}=B:/umt")
        set_property(TARGET ${_target} APPEND PROPERTY STATIC_LIBRARY_OPTIONS /Brepro)
    endforeach()
endif()

# Artifact handed to the host: normally the target's own binary. But a static archive (the Web build, and any UMT_STATIC build) does NOT fold its Bullet dependencies in - target_link_libraries only records a usage requirement, so the target's own .lib/.a contains just NativeContext. Bundle UMTNativePlugin plus the three Bullet archives into ONE self-contained library so the host links a single artifact and needs no knowledge of Bullet.
set(_plugin_artifact "$<TARGET_FILE:UMTNativePlugin>")

if(EMSCRIPTEN OR UMT_STATIC)
    set(_bundled_dir "${CMAKE_CURRENT_BINARY_DIR}/bundled")
    set(_plugin_artifact "${_bundled_dir}/$<TARGET_FILE_NAME:UMTNativePlugin>")
    if(MSVC)
        # MSVC's librarian merges member object files when handed other .lib files: `lib /OUT:merged.lib a.lib b.lib ...`. It runs inside MSBuild's toolchain environment, so lib.exe is on PATH.
        add_custom_command(TARGET UMTNativePlugin POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E make_directory "${_bundled_dir}"
            COMMAND "${CMAKE_COMMAND}" -E rm -f "${_plugin_artifact}"
            COMMAND lib.exe /NOLOGO /Brepro "/OUT:${_plugin_artifact}" "$<TARGET_FILE:UMTNativePlugin>" "$<TARGET_FILE:LinearMath>" "$<TARGET_FILE:BulletCollision>" "$<TARGET_FILE:BulletDynamics>"
            COMMENT "Bundling Bullet archives into a self-contained ${_plugin_artifact}"
            VERBATIM
        )
    else()
        # llvm-ar / emar: the L modifier flattens member archives into the output.
        add_custom_command(TARGET UMTNativePlugin POST_BUILD
            COMMAND "${CMAKE_COMMAND}" -E make_directory "${_bundled_dir}"
            COMMAND "${CMAKE_COMMAND}" -E rm -f "${_plugin_artifact}"
            COMMAND "${CMAKE_AR}" qcsL "${_plugin_artifact}" "$<TARGET_FILE:UMTNativePlugin>" "$<TARGET_FILE:LinearMath>" "$<TARGET_FILE:BulletCollision>" "$<TARGET_FILE:BulletDynamics>"
            COMMENT "Bundling Bullet archives into a self-contained ${_plugin_artifact}"
            VERBATIM
        )
    endif()
endif()

if(UMT_PACKAGE_DIR)
    message(STATUS "POST_BUILD will copy the plugin to: ${UnityPluginOutput}")
    add_custom_command(TARGET UMTNativePlugin POST_BUILD
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${UnityPluginOutput}"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${_plugin_artifact}" "${UnityPluginOutput}/$<TARGET_FILE_NAME:UMTNativePlugin>"
    )
else()
    message(STATUS "UMT_PACKAGE_DIR not set; built plugin stays in the build tree. Set -DUMT_PACKAGE_DIR=<package> to copy into the UMT package.")
endif()

option(UMT_BUILD_TESTS "Build native ABI regression tests (requires UMT_STATIC)." OFF)
if(UMT_BUILD_TESTS)
    if(NOT UMT_STATIC)
        message(FATAL_ERROR "UMT_BUILD_TESTS currently requires UMT_STATIC=ON.")
    endif()
    enable_testing()
    add_executable(UMTWorldSnapshotTest Tests/WorldSnapshot.cpp)
    target_include_directories(UMTWorldSnapshotTest PRIVATE Source)
    target_compile_definitions(UMTWorldSnapshotTest PRIVATE UMT_NATIVE_STATIC)
    target_link_libraries(UMTWorldSnapshotTest PRIVATE UMTNativePlugin)
    add_test(NAME UMT.WorldSnapshot COMMAND UMTWorldSnapshotTest)
    add_executable(UMTSpring2Regression Tests/Spring2Regression.cpp)
    target_include_directories(UMTSpring2Regression PRIVATE Source)
    target_compile_definitions(UMTSpring2Regression PRIVATE UMT_NATIVE_STATIC)
    target_link_libraries(UMTSpring2Regression PRIVATE UMTNativePlugin)
    add_test(NAME UMT.Spring2Regression COMMAND UMTSpring2Regression)
    # Inspect real Bullet activation at the implementation seam without adding a public debug ABI.
    add_executable(UMTKinematicWakeTest Tests/KinematicWake.cpp)
    target_include_directories(UMTKinematicWakeTest PRIVATE Source "${BulletRoot}/src")
    target_compile_definitions(UMTKinematicWakeTest PRIVATE UMT_NATIVE_STATIC)
    target_link_libraries(UMTKinematicWakeTest PRIVATE BulletDynamics BulletCollision LinearMath)
    add_test(NAME UMT.KinematicWake COMMAND UMTKinematicWakeTest)
endif()
