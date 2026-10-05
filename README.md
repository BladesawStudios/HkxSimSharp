# HkxSimSharp

HkxSimSharp is a reusable C# simulation library for decoded Havok physics data. It is intended for
editors, preview tools, converters, and research software rather than as a replacement for Havok in
a shipping game engine.

The repository is deliberately split from file decoding:

- [HkxSharp](https://github.com/BladesawStudios/HkxSharp) reads HKX packfiles and TAG0 tagfiles into a generic object graph.
- [HkxSimSharp](https://github.com/BladesawStudios/HkxSimSharp) converts supported objects into stable simulation data and advances their runtime state.
- Applications own file wrappers, animation playback, scene transforms, rendering, and presentation.

## Current support

The first module is `HkxSimSharp.Cloth`. It currently includes:

- HkxSharp `HkObject` to HCL cloth conversion;
- particle integration and constraint solving;
- standard, stretch, compressible, bend, bend-stiffness, and spherical local-range constraints;
- capsule, tapered-capsule, sphere, and plane collision data;
- authored solve ordering, substeps, iteration counts, collision masks, and transfer-motion settings;
- object-space skinning, vertex gathering, fixed-particle movement, and mesh-bone deformation;
- embedded HKA skeleton conversion.

Unsupported HCL constraint types are retained as placeholders so authored constraint indices remain
correct. Additional Havok systems can be added as sibling modules without making the cloth API the
root abstraction of the package.

## Usage

```csharp
using HkxSharp;
using HkxSimSharp.Cloth;

HkxFile hkx = HkxFile.FromBinary(hkxBytes);
HkxClothDocument document = HkxClothAdapter.Decode(hkx);

foreach (ClothSolver solver in document.CreateSolvers())
{
    solver.Wind = wind;
    solver.Step(deltaTime, currentBoneTransforms, referenceFrameDelta);

    // Feed solver.OutputBuffers and solver.OutputBoneTransforms into your own renderer/editor.
}
```

Consumers that already decode cloth into another representation can construct the types under
`HkxSimSharp.Cloth.Data` directly and pass an `HclClothData` to `ClothSolver`. The solver has no
dependency on Marrow, OpenGL, UI frameworks, or Tears of the Kingdom file wrappers.

## Building

The sibling repositories are expected in this layout while both packages are under active development:

```text
Libraries/
├── HkxSharp/
└── HkxSimSharp/
```

Then run:

```text
dotnet build HkxSimSharp.slnx
dotnet run --project tests/HkxSimSharp.Tests
```

Passing an HKX/TAG0 file to the test executable enables the HkxSharp integration test. Its local
development harness also recognizes Zelda BPHCL wrappers, but wrapper parsing is not part of the
library API.
