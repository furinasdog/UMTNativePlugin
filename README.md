# UMTNativePlugin

Native plugin backing the MMD bullet physics in [Unity MMD Tools (UMT)](https://github.com/CandidumGames/UnityMMDTools). It wraps **Bullet 3.07** (official tag commit `272c7099d3ba8ac9e8c142e361e8d4cf5d91b8f2`) behind a small, flat Cdecl ABI that the managed `UMT.MMDBulletPhysics` P/Invoke wrapper calls into.

## Targets

| Platform          | ABI         | Output                                   |
| ----------------- | ----------- | ---------------------------------------- |
| Windows           | x64         | `UMTNativePlugin.dll`             |
| Windows (static)  | x64         | `UMTNativePlugin.lib` (`-DUMT_STATIC=ON`) |
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
│   └── bullet3/            Optional local Bullet checkout : NOT committed
├── LICENSE                     MIT (this project's own source)
├── .gitignore
└── README.md
```

> The struct layouts in `Source/NativeContext.h` (`RigidBodySimulationData`, `NativeJointData`) and the enums (`RigidBodyShape`, `RigidBodyMode`, `JointType`) are **byte-for-byte layout-critical** and must stay in sync with their managed counterparts in `MMDBulletPhysics.cs` / `MMDRigidBody.cs`. Do not reorder fields on one side without the other : it corrupts the marshalled data.

## Prerequisites

- **CMake** ≥ 3.24
- **Git** ≥ 2.25 (fixed dependency commit, partial/sparse checkout)
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

### Windows x64 (static library, for Unreal Engine)

Hosts that link the plugin directly through their own build system (e.g. Unreal Engine via UBT) consume it as a static library instead of loading a `.dll` at runtime. `-DUMT_STATIC=ON` builds `UMTNativePlugin.lib` (defining `UMT_NATIVE_STATIC`, which drops `__declspec(dllexport)` so the archive exposes plain `extern "C"` symbols). Use a separate build directory so it does not clobber the shared-library `build/`:

```bash
cmake -S . -B build-static -DUMT_STATIC=ON
cmake --build build-static --config Release --target UMTNativePlugin
```

Add `-DBulletRoot=/path/to/bullet3` (e.g. an existing `build/_deps/bullet307-src`) to reuse a local Bullet checkout and skip the download. A POST_BUILD step merges the three static Bullet archives into `UMTNativePlugin.lib` (via `lib.exe /OUT:` on MSVC), so the host links **one self-contained library** and needs no knowledge of Bullet:

- `build-static/bundled/UMTNativePlugin.lib` — the merged, self-contained archive (link this)
- `Source/API.h` + `Source/NativeContext.h` — the ABI surface (include these)

> `build-static/Release/UMTNativePlugin.lib` is the *unmerged* target output (NativeContext only) — link `bundled/UMTNativePlugin.lib` instead.

The library compiles against the dynamic CRT (`/MD`, `MSVCRT`), matching an Unreal editor build. In the Unreal MMD Tools plugin it is staged as `Plugins/UnrealMMDTools/External/UMTNativePlugin/lib/Win64/UMTNativePlugin.lib` (headers under `.../include`) and linked via `PublicAdditionalLibraries` in the editor module's `Build.cs`.

`UMT_REPRODUCIBLE_MSVC=ON` normalizes native source paths to `B:/umt` and removes build timestamps. Honkai's preparation script enables it with MSVC 14.44; independently built project copies then produce identical archives. Map that virtual directory to the local UMT checkout when debugging native source, or disable the option for a local debug build.

`MMDBulletPhysicsGetRigidBodyWorldTransforms` reads the current simulation poses for feedback operations such as mode-2 bone alignment. It uses the same indices and column-major float4x4 output layout as `GetRigidBodyMotionTransforms`. The original function keeps its motion-state interpolation behavior for existing display callers; passing an interpolated snapshot's position delta to `ShiftRigidBodyPosition`, which modifies the current body, mixes two simulation times. The new function is additive and does not change any existing structure layout or export.

The native regression exercises 120 moving mode-2 corrections and verifies exact current-world attachment, while retaining the old motion-state behavior as a negative control:

```bash
cmake -S . -B build-static -DUMT_STATIC=ON -DUMT_BUILD_TESTS=ON
cmake --build build-static --config Release
ctest --test-dir build-static -C Release --output-on-failure
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

Requires Ninja and an Emscripten SDK, and builds through the Emscripten toolchain file. **The Unity-bundled Emscripten is recommended** (`<...>/Editor/Data/PlaybackEngines/WebGLSupport/BuildTools/Emscripten`) since it matches the Editor's WebGL toolchain  but it ships no `.emscripten` config and is not on PATH, so set up a generated config, a writable cache, and PATH before configuring.

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
Point at a local pinned Bullet 3.07 Git checkout to skip the download entirely:

```bash
cmake -S . -B build -DBulletRoot=/path/to/bullet3
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
| `UMT_STATIC`            | `OFF`                                      | Build a static library (`.lib`/`.a`) with plain `extern "C"` symbols for hosts that link it directly (e.g. Unreal Engine), instead of a shared library |
| `BulletRoot`            | *(empty → auto-download)*                  | Clean pinned Bullet 3.07 Git checkout; set to skip the download       |
| `UMT_REPRODUCIBLE_MSVC` | `OFF` | Source-path/timestamp normalization; tested MSVC 14.44 |
| `UMT_BUILD_TESTS` | `OFF` | Native current-world and Spring2 behavior regressions |
| `UMT_PACKAGE_DIR`       | *(empty → no copy)*                        | UMT package to copy the binary into; empty skips copy  |
| `UnityPluginOutput`     | `${UMT_PACKAGE_DIR}/Plugins/<platform>/<abi>` | Exact copy destination (overrides the above)        |
| `ANDROID_NDK`           | *(unset)*                                  | Android NDK root; set to cross-compile for Android     |
| `ANDROID_ABI`           | `arm64-v8a`                                | Android target ABI                                     |
| `ANDROID_PLATFORM`      | `android-24`                               | Android minimum API level                              |
| `ANDROID_STL`           | `c++_static`                               | Android C++ runtime                                    |

## Spring2 behavior migration

This fork intentionally migrates the solver from Bullet 2.75 to the fixed official 3.07 commit. The C ABI names, field order and sizes remain compatible, including the existing interpolated snapshot API, but the old spring motor parameter semantics do not.

- `Spring6DOF` uses `btGeneric6DofSpring2Constraint` with XYZ rotation order. All six spring axes remain enabled, including zero-stiffness axes, so relative-velocity damping remains active. Equilibrium is captured before the first step.
- Pass the authored PMX translation and rotation stiffness values. Remove the old rotation-stiffness `lengthScale²` and inverse-square motor-damping conversions. Spring2 damping defaults to `.5` for translation and rotation; it uses viscous damping and Bullet's normal effective-mass/time-step safety limits.
- Joint-linked body pairs do not collide. Every other pair still uses both original raw allowed-group masks; this does not disable general collisions or rewrite PMX metadata.
- Convex margin defaults to `1e-6` world units. Bullet retains sphere/capsule radius semantics; it is not a uniform outward expansion for every shape. Fixed 60Hz, body damping, additional low-speed damping and the host's Mode2 alignment policy remain separate.

Dynamic bodies use Bullet's ordinary island sleeping rather than forced `DISABLE_DEACTIVATION` with zero thresholds. An unchanged kinematic target has zero velocity and a sleeping activation state, so it does not wake resting contacts indefinitely. A changed kinematic target explicitly wakes this model context's dynamic bodies before stepping; direct dynamic teleports wake their body as well. This retains authored collision masks, joints, mass and damping. `UMT.Spring2Regression` includes a Mode2 pendant resting, reacting to a moved anchor, and resuming gravity after reset. Models whose active dynamics do not meet Bullet's sleep criteria remain active.

Kinematic movement is compared within the float matrix ABI's machine epsilon rather than exact matrix equality. Rotation roundoff as small as `7e-15` to `1.5e-8` previously reactivated every dynamic body each frame after motion stopped. `UMT.KinematicWake` observes real Bullet activation: alternating sub-epsilon rotation roundoff keeps a resting body asleep, while a real 10-micrometre translation wakes it. This does not change sleeping thresholds, body velocities, damping, collision masks, or Mode2 alignment.

The solver regularizes redundant small-body loops with global CFM `0.001`. Fully zero-limit six-axis weld chains rooted at a kinematic body are evaluated from their exact relative joint frames; their source rigid-body mass/mode records and collision masks are not rewritten. The reduced solver bodies have no independent degrees of freedom. Bound Mode2 bodies are excluded from this reduction so the host's animated position remains authoritative. World inertia tensors are refreshed after prescribed transforms or reduction.

For permanent MMD Mode2 constraints, the host aligns position with `MMDBulletPhysicsShiftRigidBodyPosition` and preserves the solver velocity. An additional velocity projection was rejected after source-model geometry verification: a neutral long-hair chain curled upward despite passing finite/residual/jitter checks. A hypothetical release into free fall is not an MMD Mode2 correctness criterion. Check source-model shape as well as stability; the native suite retains permanent-Mode2 rest/wake coverage.

An isolated comparison also evaluated Three.js r170's dynamic-parent Mode2-to-Mode1 runtime topology rule. It was not adopted: the production change is limited to roundoff-safe kinematic activation, and original source modes remain in use.

The model data and any host-side unit conversion remain the host's responsibility. An existing Unity host using the old motor conversions must migrate those parameters or keep the previous native version; ABI compatibility alone does not preserve its physical response.

The default configure obtains only the official repository's source/root-license files, pins the commit and rejects dirty dependency sources. `VERSION` and `BT_BULLET_VERSION` at the official 3.07 tag still say 3.05/305; commit verification is authoritative. Existing 2.75 caches are not accepted. A local `BulletRoot` must be a clean Git checkout at the same pinned commit. The preparation does not reset local modifications or change global Git settings.

`UMT.Spring2Regression` checks real zero-stiffness damping against an unjointed world, an eccentric solid pendulum reaching and resting at an angular stop, and linked-only collision suppression with a nonlinked collision control. The old wrapper fails the zero-stiffness and linked tests; the pendulum is a bounded regression, not proof that every constrained model converges.

## API overview

The full surface is documented in [`Source/API.h`](Source/API.h).

- `MMDBulletPhysicsCreate` / `MMDBulletPhysicsDestroy` : context lifecycle
- `MMDBulletPhysicsSetConfig` : override ground-plane normal/constant, MMD→Unity unit scale, margin and Spring2 viscous damping (see migration below)
- `MMDBulletPhysicsReset` : deterministic reset from a seed
- `MMDBulletPhysicsBuildRigidBodies` / `MMDBulletPhysicsBuildJoints` : world construction
- `MMDBulletPhysicsBuildGround` / `MMDBulletPhysicsSetGroundCollisionEnabled` : ground plane
- `MMDBulletPhysicsSetRigidBodyTransforms` / `MMDBulletPhysicsGetRigidBodyMotionTransforms` : transform sync
- `MMDBulletPhysicsShiftRigidBodyPosition` : world-space position shift
- `MMDBulletPhysicsStepSimulation` : advance with fixed sub-steps

## License

This project's own source (`Source/`, `CMakeLists.txt`, `Toolchains/`) is licensed under the [MIT License](LICENSE).
Bullet is licensed under its original [zlib license](ThirdParty/Bullet-LICENSE.txt). The fixed dependency checkout also retains its upstream `LICENSE.txt`.
