# UMTNativePlugin

Native plugin backing the MMD bullet physics in [Unity MMD Tools (UMT)](https://github.com/CandidumGames/UnityMMDTools). It wraps **Bullet 2.75** behind a small, flat Cdecl ABI that the managed `UMT.MMDBulletPhysics` P/Invoke wrapper calls into.

## Targets

| Platform          | ABI         | Output                                   |
| ----------------- | ----------- | ---------------------------------------- |
| Windows           | x64         | `UMTNativePlugin.dll`             |
| Android           | arm64-v8a   | `libUMTNativePlugin.so`           |
| Web               | WebAssembly | `libUMTNativePlugin.a`            |

The build is standalone and does not require Unity. Optionally, it can also copy the finished binary into a UMT Unity package's `Plugins/` folder via a CMake POST_BUILD` step when `UMT_PACKAGE_DIR` is set (see [Output](#output)).

## Repository layout

```
UMTNativePlugin/
├── CMakeLists.txt              CMake project (Bullet subdirs + plugin target)
├── Source/
│   ├── API.h                   extern "C" exported API surface
│   ├── NativeContext.h         opaque context + blittable struct/enum layouts
│   └── NativeContext.cpp       Bullet-backed implementation
├── Toolchains/
│   └── Android-aarch64.cmake   NDK toolchain wrapper (auto-selected)
├── ThirdParty/
│   └── bullet-2.75/            Optional local Bullet checkout : NOT committed
├── LICENSE                     MIT (this project's own source)
├── .gitignore
└── README.md
```

> The struct layouts in `Source/NativeContext.h` (`RigidBodySimulationData`, `NativeJointData`) and the enums (`RigidBodyShape`, `RigidBodyMode`, `JointType`) are **byte-for-byte layout-critical** and must stay in sync with their managed counterparts in `MMDBulletPhysics.cs` / `MMDRigidBody.cs`. Do not reorder fields on one side without the other : it corrupts the marshalled data.

## Prerequisites

- **CMake** ≥ 3.16
- A **C++17** compiler
  - Windows: **Visual Studio 2022+** (MSVC14.3+)
  - Android: an **Android NDK** (r27 recommended) and **Ninja**
  - Web: an **Emscripten** SDK (3.1.x recommended) and **Ninja**
- Network access on first configure (to download Bullet) or a local checkout for offline builds

## Building

A CMake build directory is single-toolchain : use a **separate build directory per target**.

### Windows x64

```bash
cmake -S . -B build -G "Visual Studio 18 2026" -A x64
cmake --build build --config Release --target UMTNativePlugin
```

### Android arm64-v8a

Requires Ninja and an Android NDK. `CMakeLists.txt` activates the Android toolchain

```bash
cmake -S . -B build-android-arm64 -G Ninja \
    -DANDROID_NDK="/path/to/android-ndk" \
    -DCMAKE_BUILD_TYPE=Release
cmake --build build-android-arm64 --config Release
```

`ANDROID_NDK` can be any standalone NDK install (e.g. from Android Studio's SDK Manager, the `sdkmanager` CLI, a manual download or a Unity-bundled NDK `<...>/Editor/Data/PlaybackEngines/AndroidPlayer/NDK` ).

### Web (WebAssembly)

Requires Ninja and an Emscripten SDK, and builds through the Emscripten toolchain file. **The Unity-bundled Emscripten is recommended** (`<...>/Editor/Data/PlaybackEngines/WebGLSupport/BuildTools/Emscripten`) since it matches the Editor's WebGL toolchain — but it ships no `.emscripten` config and is not on PATH, so set up a generated config, a writable cache, and PATH before configuring.

```powershell

$em = "$<...>/Editor/Data/PlaybackEngines/WebGLSupport/BuildTools/Emscripten"
$env:EMSDK_PYTHON = "$em/python/python.exe"
$env:EM_CONFIG = "$env:TEMP/umt-emscripten.config"
$env:EM_CACHE = "$env:TEMP/umt-em-cache"
Set-Content $env:EM_CONFIG @("LLVM_ROOT = '$em/llvm'", "BINARYEN_ROOT = '$em/binaryen'", "NODE_JS = '$em/node/node.exe'", "EMSCRIPTEN_ROOT = '$em/emscripten'", "COMPILER_ENGINE = NODE_JS", "JS_ENGINES = [NODE_JS]")
$env:PATH = "$em/emscripten;$em/node;$env:PATH"
cmake -S . -B build-web -G Ninja `
    -DCMAKE_TOOLCHAIN_FILE="$em/emscripten/cmake/Modules/Platform/Emscripten.cmake" `
    -DCMAKE_BUILD_TYPE=Release `
    -DCMAKE_INSTALL_PREFIX="build-web/_install"
cmake --build build-web
```

`-DCMAKE_INSTALL_PREFIX` is required: the Emscripten toolchain otherwise derives the install prefix from `em-config CACHE`, whose Windows `.bat` wrapper echoes batch lines into the captured output and corrupts `CMakeCache.txt`. Passing the prefix ourselves skips that probe.

`target_link_libraries` does not fold the Bullet archives into `libUMTNativePlugin.a`, so a POST_BUILD step bundles them into one self-contained archive for Unity to link.

**Offline / air-gapped builds.**
Point at a local Bullet 2.75 checkout to skip the download entirely:

```bash
cmake -S . -B build -DBulletRoot=/path/to/bullet-2.75
```

## Output

By default the build is standalone: the binary lands in the build tree (e.g. `build/Release/UMTNativePlugin.dll`) with no Unity coupling.

To also drop the binary into a UMT Unity package so Unity can hot-reload it, set `UMT_PACKAGE_DIR` to that package folder; a `POST_BUILD` step then copies it into the package's `Plugins/` folder:

```bash
cmake -S . -B build -G "Visual Studio 18 2026" -A x64 \
    -DUMT_PACKAGE_DIR=/path/to/com.candidumgames.unitymmdtools
```

- Windows : `<UMT_PACKAGE_DIR>/Plugins/Windows/x64/UMTNativePlugin.dll`
- Android : `<UMT_PACKAGE_DIR>/Plugins/Android/arm64-v8a/libUMTNativePlugin.so`
- Web : `<UMT_PACKAGE_DIR>/Plugins/Web/libUMTNativePlugin.a`

Override the exact destination instead with `-DUnityPluginOutput=<path>`.

> If you change the managed marshalling structs, rebuild before entering Play mode.

## Build options

| Option                  | Default                                    | Purpose                                                |
| ----------------------- | ------------------------------------------ | ------------------------------------------------------ |
| `BulletRoot`            | *(empty → auto-download)*                  | Local Bullet 2.75 root; set to skip the download       |
| `BULLET_2_75_URL`       | Google Code archive zip                    | Bullet 2.75 archive URL                                |
| `BULLET_2_75_SHA256`    | `fe2e369…61fe8f`                           | Pinned archive checksum                                |
| `UMT_PACKAGE_DIR`       | *(empty → no copy)*                        | UMT package to copy the binary into; empty skips copy  |
| `UnityPluginOutput`     | `${UMT_PACKAGE_DIR}/Plugins/<platform>/<abi>` | Exact copy destination (overrides the above)        |
| `ANDROID_NDK`           | *(unset)*                                  | Android NDK root; set to cross-compile for Android     |
| `ANDROID_ABI`           | `arm64-v8a`                                | Android target ABI                                     |
| `ANDROID_PLATFORM`      | `android-24`                               | Android minimum API level                              |
| `ANDROID_STL`           | `c++_static`                               | Android C++ runtime                                    |

## API overview

The full surface is documented in [`Source/API.h`](Source/API.h).

- `MMDBulletPhysicsCreate` / `MMDBulletPhysicsDestroy` : context lifecycle
- `MMDBulletPhysicsReset` : deterministic reset from a seed
- `MMDBulletPhysicsBuildRigidBodies` / `MMDBulletPhysicsBuildJoints` : world construction
- `MMDBulletPhysicsBuildGround` / `MMDBulletPhysicsSetGroundCollisionEnabled` : ground plane
- `MMDBulletPhysicsSetRigidBodyTransforms` / `MMDBulletPhysicsGetRigidBodyMotionTransforms` : transform sync
- `MMDBulletPhysicsShiftRigidBodyPosition` : world-space position shift
- `MMDBulletPhysicsStepSimulation` : advance with fixed sub-steps

## License

This project's own source (`Source/`, `CMakeLists.txt`, `Toolchains/`) is licensed under the [MIT License](LICENSE).
Bullet is licensed under its own terms (zlib).
