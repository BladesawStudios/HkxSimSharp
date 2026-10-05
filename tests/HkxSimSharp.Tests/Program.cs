using System.Buffers.Binary;
using System.Numerics;
using HkxSharp;
using HkxSimSharp.Cloth;
using HkxSimSharp.Cloth.Data;

SmokeTestEmptyDefinition();

if (args.Length > 0)
    IntegrationTestHkxSharp(args[0]);

Console.WriteLine("HkxSimSharp tests passed.");

static void SmokeTestEmptyDefinition()
{
    var solver = new ClothSolver(new HclClothData { Name = "empty" });
    solver.Step(1f / 60f, ReadOnlySpan<Matrix4x4>.Empty);
    Assert(solver.Name == "empty", "Solver did not retain its definition name.");
    Assert(solver.ParticleSets.Count == 0, "Empty cloth unexpectedly created particle sets.");
}

static void IntegrationTestHkxSharp(string path)
{
    byte[] bytes = File.ReadAllBytes(path);
    ReadOnlySpan<byte> hkxBytes = bytes;

    // Zelda's BPHCL wrapper is intentionally handled by the application, not HkxSimSharp.
    if (bytes.AsSpan().StartsWith("Phive\0"u8))
    {
        int tagOffset = (int)BinaryPrimitives.ReadUInt32LittleEndian(bytes.AsSpan(12));
        int tagSize = (int)BinaryPrimitives.ReadUInt32LittleEndian(bytes.AsSpan(24));
        hkxBytes = bytes.AsSpan(tagOffset, tagSize);
    }

    HkxFile hkx = HkxFile.FromBinary(hkxBytes);
    HkxClothDocument document = HkxClothAdapter.Decode(hkx);
    Assert(document.Cloth.ClothDatas.Count > 0, "HkxSharp adapter found no cloth definitions.");

    foreach (ClothSolver solver in document.CreateSolvers())
    {
        Matrix4x4[] pose = solver.OutputBoneTransforms.ToArray();
        solver.Step(1f / 60f, pose);
        foreach (var particles in solver.ParticleSets)
            Assert(particles.Positions.All(IsFinite), $"{solver.Name} produced a non-finite particle.");
    }
}

static bool IsFinite(Vector3 value) =>
    float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);

static void Assert(bool condition, string message)
{
    if (!condition) throw new InvalidOperationException(message);
}
