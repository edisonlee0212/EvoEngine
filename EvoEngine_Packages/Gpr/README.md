# Gpr Runtime Package

[Back to package index](../README.md)

Gpr adds GoPro Raw support through the vendored GPR library. It wraps `.gpr`-style media as EvoEngine assets.

## Build Status

- Registered in the Windows block of `EvoEngine_Packages/CMakeLists.txt` by default.
- Builds as the shared library target `GprPackage`.
- Defines `GPR_READING=1` and `GPR_WRITING=1`.
- Links vendored GPR dependencies such as `gpr_sdk`, `vc5_decoder`, `vc5_encoder`, `dng_sdk`, and related support libraries.

## Main Responsibilities

- Load and save GPR-backed image assets.
- Maintain a preview image asset reference.
- Expose GPR asset inspection in the editor.

## Main Entry Points

| Source | Role |
| --- | --- |
| `Gpr` | `IAsset` implementation for GoPro Raw media. |

## Registered Types

The package entrypoint registers `Gpr` as an asset type for:

- `.evegpr`
- `.gpr`
- `.GPR`

## SDK Integration

Gpr uses the SDK asset lifecycle, internal save/load hooks, asset references, and editor inspection.

## Future Work Notes

Keep codec/library-specific behavior in this package. If other image formats need similar treatment, prefer a separate asset package or a shared SDK image abstraction only after duplication appears.
