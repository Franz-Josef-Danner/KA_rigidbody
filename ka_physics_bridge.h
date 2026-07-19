# KA native Jolt bridge

This directory contains the optional ABI-v2 bridge for KA Rigid Dynamics 0.6.4.
It targets **Jolt Physics 5.6.0** and creates one native `StaticCompoundShape`
from the CoACD convex children of a logical Blender body. If no compiled bridge
is present, the add-on automatically uses the bundled Culverin compatibility
backend and its single-body oriented-box Compound Convex fallback.

## Build with automatic source download

Linux:

```bash
./native/jolt/build_linux.sh
```

Windows PowerShell:

```powershell
.\native\jolt\build_windows.ps1
```

CMake fetches the exact Jolt tag `v5.6.0` during configuration.

## Build from an existing Jolt checkout

For offline or controlled builds, download/check out Jolt 5.6.0 separately and
pass its source root:

```bash
cmake -S native/jolt -B build/jolt -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DKA_JOLT_SOURCE_DIR=/path/to/JoltPhysics
cmake --build build/jolt --config Release
```

Install the result in one of these locations:

- Windows: `vendor/jolt_bridge/win_amd64/ka_jolt_bridge.dll`
- Linux: `vendor/jolt_bridge/linux_x86_64/libka_jolt_bridge.so`

A custom library can instead be selected under Blender Preferences → Add-ons →
KA Rigid Dynamics → **Jolt bridge**.

The add-on validates ABI version 2 before loading the library. Do not rename or
substitute an unrelated Jolt library; this bridge exports a project-specific C ABI.
