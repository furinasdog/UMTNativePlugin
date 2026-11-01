cmake_minimum_required(VERSION 3.16)

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

# --- Bullet 2.75 acquisition --------------------------------------------------
# Bullet 2.75 was removed from the Bullet GitHub repo and survives only as an archive on the Google Code mirror, so a git submodule is not an option. Fetch the zip and verify its SHA256. The Bullet source is used unmodified: the only behavioural tweak this pipeline needs (a collision margin scaled into Unity units) is applied explicitly in code via btCollisionShape::setMargin() in Source/NativeContext.cpp, so no source patch is required.
#
# To use a local checkout instead (offline / air-gapped CI), pass
#   -DBulletRoot=/path/to/bullet-2.75
# and the download is skipped entirely.
set(BulletRoot "" CACHE PATH "Local Bullet 2.75 root. Leave empty to auto-download.")

set(BULLET_2_75_URL
    "https://storage.googleapis.com/google-code-archive-downloads/v2/code.google.com/bullet/bullet-2.75.zip"
    CACHE STRING "Bullet 2.75 archive URL")
set(BULLET_2_75_SHA256
    "fe2e369593b034c00a1245676edd8417a58c8030a94e061220f0c228ca61fe8f"
    CACHE STRING "Bullet 2.75 archive SHA256")

if(BulletRoot)
    message(STATUS "Using local Bullet 2.75 at: ${BulletRoot} (download skipped)")
else()
    include(FetchContent)
    # Download only  we do NOT want MakeAvailable to add_subdirectory() Bullet's own top-level CMakeLists (it builds demos/extras). FetchContent treats a non-existent SOURCE_SUBDIR as "no project to configure", which is the documented way to populate without configuring. We add the three needed src/ libraries ourselves below.
    set(_bullet_no_subdir "cmake/__umt_no_configure__")
    FetchContent_Declare(bullet275
        URL "${BULLET_2_75_URL}"
        URL_HASH SHA256=${BULLET_2_75_SHA256}
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SOURCE_SUBDIR "${_bullet_no_subdir}"
    )
    FetchContent_MakeAvailable(bullet275)
    set(BulletRoot "${bullet275_SOURCE_DIR}")

    # Fail loudly if a future Bullet archive ever ships this placeholder path
    # (which would make MakeAvailable configure it and build the demos).
    if(EXISTS "${BulletRoot}/${_bullet_no_subdir}/CMakeLists.txt")
        message(FATAL_ERROR
            "Bullet unexpectedly contains '${_bullet_no_subdir}/CMakeLists.txt'; "
            "the SOURCE_SUBDIR placeholder is no longer empty. Pick a different one.")
    endif()
    message(STATUS "Fetched Bullet 2.75 to: ${BulletRoot}")
endif()
# -----------------------------------------------------------------------------

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

set(BULLET_VERSION 2.75)
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

target_include_directories(UMTNativePlugin PRIVATE
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
            COMMAND lib.exe /NOLOGO "/OUT:${_plugin_artifact}" "$<TARGET_FILE:UMTNativePlugin>" "$<TARGET_FILE:LinearMath>" "$<TARGET_FILE:BulletCollision>" "$<TARGET_FILE:BulletDynamics>"
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
endif()
