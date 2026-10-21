using System.Numerics;

namespace HkxSimSharp.Cloth;

/// <summary>Keeps the free particles of a cloth out of capsules that are not part of its own definition: the bodies of other actors.</summary>
/// <remarks>
/// A particle inside one is put back on its surface, along the line from the capsule's axis. Nothing the cloth touches is moved by it, and what the particle did in the step is left to the solver, so it slides
/// off as it would off a collider of its own.
/// </remarks>
internal static class ExternalCollision
{
    public static void Solve(SimClothRuntime runtime, Obstacle[] obstacles)
    {
        if (obstacles.Length == 0) return;

        for (int i = 0; i < runtime.ParticleCount; i++)
        {
            if (runtime.IsFixed[i]) continue;

            foreach (Obstacle o in obstacles)
            {
                Vector3 p = runtime.Positions[i], axis = o.B - o.A;
                float length = axis.LengthSquared(), t = length > 1e-9f ? Math.Clamp(Vector3.Dot(p - o.A, axis) / length, 0f, 1f) : 0f;
                Vector3 apart = p - (o.A + axis * t);
                float distance = apart.Length(), reach = o.Radius + runtime.Radii[i];
                if (distance >= reach) continue;

                // On the axis itself there is no side to read; out along the capsule's perpendicular is as good as any.
                Vector3 side = distance > 1e-6f ? apart / distance : Vector3.Normalize(Vector3.Cross(axis.LengthSquared() > 1e-9f ? axis : Vector3.UnitY, Vector3.UnitX + Vector3.UnitZ * 0.37f));
                runtime.Positions[i] = p + side * (reach - distance);
            }
        }
    }
}
