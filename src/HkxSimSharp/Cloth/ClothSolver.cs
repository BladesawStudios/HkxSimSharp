using System.Numerics;
using HkxSimSharp.Cloth.Data;
using HkxSimSharp.Cloth.Data.Animation;

namespace HkxSimSharp.Cloth;

/// <summary>
/// High-level simulation API for one decoded Havok cloth definition.
/// </summary>
/// <remarks>
/// The definition may be constructed directly or produced from an <c>HkxSharp.HkxFile</c> with
/// <see cref="HkxClothAdapter"/>. The solver owns mutable runtime state but never owns or
/// mutates the decoded definition supplied by the caller.
/// </remarks>
public sealed class ClothSolver
{
    private readonly ClothInstance _instance;
    internal ClothInstance Instance => _instance;

    public ClothSolver(HclClothData definition, HkaSkeleton? skeleton = null)
    {
        ArgumentNullException.ThrowIfNull(definition);
        _instance = new ClothInstance(definition, skeleton);
    }

    /// <summary>The authored name of this cloth definition.</summary>
    public string Name => _instance.ClothData.Name;

    /// <summary>Mutable particle sets produced by each simulated-cloth block.</summary>
    public IReadOnlyList<SimClothRuntime> ParticleSets => _instance.Runtimes;

    /// <summary>Vertex buffers produced by the decoded operator pipeline.</summary>
    public IReadOnlyList<Vector3[]> OutputBuffers => _instance.Buffers;

    /// <summary>Current cloth skeleton transforms, including mesh-bone-deform output.</summary>
    public ReadOnlySpan<Matrix4x4> OutputBoneTransforms => _instance.SkeletonTransforms;

    public IReadOnlySet<int> DeformedBoneIndices => _instance.DeformedBoneIndices;

    public bool Enabled
    {
        get => _instance.IsEnabled;
        set => _instance.IsEnabled = value;
    }

    public Vector3 Wind
    {
        get => _instance.Wind;
        set => _instance.Wind = value;
    }

    public float GravityScale
    {
        get => _instance.GravityScale;
        set => _instance.GravityScale = value;
    }

    public void Reset() => _instance.Reset();

    public void Step(float deltaTime, ReadOnlySpan<Matrix4x4> inputBoneTransforms) =>
        _instance.Step(deltaTime, inputBoneTransforms);

    public void Step(
        float deltaTime,
        ReadOnlySpan<Matrix4x4> inputBoneTransforms,
        Matrix4x4 referenceFrameDelta) =>
        _instance.Step(deltaTime, inputBoneTransforms, referenceFrameDelta);
}
