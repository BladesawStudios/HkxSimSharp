using HkxSimSharp.Cloth.Data;
using HkxSimSharp.Cloth.Data.Animation;

namespace HkxSimSharp.Cloth;

/// <summary>A decoded collection of Havok cloth definitions and their optional skeletons.</summary>
public sealed class HkxClothDocument
{
    public HkxClothDocument(HclClothContainer cloth, HkaAnimationContainer? animations = null)
    {
        Cloth = cloth ?? throw new ArgumentNullException(nameof(cloth));
        Animations = animations;
    }

    public HclClothContainer Cloth { get; }
    public HkaAnimationContainer? Animations { get; }

    /// <summary>Creates a solver for a cloth definition and resolves its embedded skeleton by name.</summary>
    public ClothSolver CreateSolver(HclClothData definition)
    {
        ArgumentNullException.ThrowIfNull(definition);
        string? target = definition.TransformSetDefinitions.FirstOrDefault()?.Name;
        HkaSkeleton? skeleton = Animations?.Skeletons.FirstOrDefault(s =>
            s.Name == target || s.Name == definition.Name || s.Name == "cloth_skeleton_" + definition.Name);
        return new ClothSolver(definition, skeleton);
    }

    public IReadOnlyList<ClothSolver> CreateSolvers() => Cloth.ClothDatas.Select(CreateSolver).ToArray();
}
