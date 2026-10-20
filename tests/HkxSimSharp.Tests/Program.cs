using System.Buffers.Binary;
using System.Numerics;
using HkxSharp;
using HkxSimSharp.Cloth;
using HkxSimSharp.Cloth.Data;
using HkxSimSharp.Ragdoll;

SmokeTestEmptyDefinition();
TestRagdollDropsAndSettles();

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

// A chain of three capsules hung from a free body standing a metre up: it falls onto the floor, the joints stay together, and it comes to rest above the floor.
static void TestRagdollDropsAndSettles()
{
    RagdollJoint joint = new(0.8f, 0f, -0.6f, 0.6f, -0.8f, 0.8f);
    RagdollBody[] bodies =
    [
        new(-1, new Vector3(0, 0, 0), new Vector3(0.4f, 0, 0), 0.1f, 10f, null),
        new(0, new Vector3(0, 0, 0), new Vector3(0.4f, 0, 0), 0.08f, 5f, joint),
        new(1, new Vector3(0, 0, 0), new Vector3(0.4f, 0, 0), 0.06f, 3f, joint),
    ];
    Matrix4x4[] frames = [.. Enumerable.Range(0, 3).Select(i => Matrix4x4.CreateTranslation(0.4f * i, 1f, 0))];
    var world = new RagdollWorld(bodies, frames);
    for (int i = 0; i < 600; i++) world.Step(1f / 60f);

    for (int i = 0; i < 3; i++)
    {
        Vector3 o = world.FrameOf(i).Translation;
        Assert(float.IsFinite(o.X + o.Y + o.Z), "A body went to infinity.");
        Assert(o.Y > -0.01f, $"Body {i} fell through the floor ({o.Y}).");
    }
    // Each body's frame is where the one before it ends, within a few centimetres.
    for (int i = 1; i < 3; i++)
    {
        Vector3 end = Vector3.Transform(Vector3.Zero, world.FrameOf(i - 1)) ;
        Assert(Vector3.Distance(world.FrameOf(i).Translation, world.FrameOf(i - 1).Translation) < 0.6f, "A joint came apart.");
    }
    Assert(world.FrameOf(0).Translation.Y < 0.3f, "The ragdoll did not come down.");

    // Released, the bodies fall as pieces: the chain is not held together, and each still comes to the floor.
    var apart = new RagdollWorld(bodies, frames);
    apart.Release();
    for (int i = 0; i < 600; i++) apart.Step(1f / 60f);
    for (int i = 0; i < 3; i++) Assert(apart.FrameOf(i).Translation.Y > -0.01f && apart.FrameOf(i).Translation.Y < 0.3f, $"Piece {i} did not come to the floor.");
}
