# Bullet dependency

Official repository: https://github.com/bulletphysics/bullet3

Release tag: `3.07`; pinned commit: `272c7099d3ba8ac9e8c142e361e8d4cf5d91b8f2`.

The source is used unmodified. The build obtains a partial/sparse Git checkout of `src` and root files, verifies the exact commit and refuses dirty sources. Upstream's `VERSION` and `BT_BULLET_VERSION` at this tag still report `3.05` and `305`; the fixed commit identifies the dependency.

The original zlib license is retained as [Bullet-LICENSE.txt](Bullet-LICENSE.txt), SHA256 `885cd516a8420acb7c3e0b1a7119913b686495001ae564e3229740460e8cff59`. The build-cache checkout also retains its own `LICENSE.txt`. This license does not replace UMT's MIT license or any model/engine license.
