using System;
using System.Collections.Generic;
using System.Numerics;

namespace HkxSimSharp.Cloth;

/// <summary>
/// World container managing active cloth simulation instances in a scene.
/// </summary>
public sealed class ClothSimulationWorld
{
    private readonly List<ClothSolver> _solvers = new();

    public IReadOnlyList<ClothSolver> Solvers => _solvers;

    public Vector3 Wind { get; set; } = Vector3.Zero;
    public float TimeScale { get; set; } = 1.0f;
    public bool IsPaused { get; set; }

    public void Add(ClothSolver solver)
    {
        ArgumentNullException.ThrowIfNull(solver);
        if (!_solvers.Contains(solver)) _solvers.Add(solver);
    }

    public bool Remove(ClothSolver solver) => _solvers.Remove(solver);

    public void Clear()
    {
        _solvers.Clear();
    }

    /// <summary>
    /// Steps all active cloth instances forward by dt seconds.
    /// </summary>
    public void Step(float dt, IReadOnlyDictionary<ClothSolver, ReadOnlyMemory<Matrix4x4>>? skeletonTransforms = null)
    {
        if (IsPaused || dt <= 0f) return;

        float effectiveDt = dt * TimeScale;

        foreach (var solver in _solvers)
        {
            if (!solver.Enabled) continue;

            solver.Wind = Wind;

            ReadOnlySpan<Matrix4x4> transforms = ReadOnlySpan<Matrix4x4>.Empty;
            if (skeletonTransforms != null && skeletonTransforms.TryGetValue(solver, out var mem))
            {
                transforms = mem.Span;
            }
            else
            {
                transforms = solver.OutputBoneTransforms;
            }

            solver.Step(effectiveDt, transforms);
        }
    }

    public void Reset()
    {
        foreach (var solver in _solvers) solver.Reset();
    }
}
