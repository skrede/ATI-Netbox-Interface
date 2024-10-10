# Introduction
A small library that demonstrates Ethernet communication with the ATI Netbox, and obtaining force/torque sensor measurements using the ATI Netbox Raw Data Transfer (RDT) protocol over UDP.

# How to setup vcpkg (in manifest mode)

Call CMake with `-DCMAKE_TOOLCHAIN_FILE=[path to vcpkg]/scripts/buildsystems/vcpkg.cmake`

Add optional features by listing them with `-DVCPKG_MANIFEST_FEATURES=feature1;feature2`

Note, however, that under MinGW you'll need to specify the vcpkg triplet:
```bash
-DVCPKG_TARGET_TRIPLET=x64-mingw-[static|dynamic]  # For both of these lines, choose either 'static' or 'dynamic'.
-DVCPKG_HOST_TRIPLET=x64-mingw-[static|dynamic]    # This is needed only if MSVC cannot be found. 
```
Source: [threepp](https://github.com/markaren/threepp/blob/master/vcpkg.json).