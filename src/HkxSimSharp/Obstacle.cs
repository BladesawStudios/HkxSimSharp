using System.Numerics;

namespace HkxSimSharp;

/// <summary>A capsule other than a simulation's own that its bodies or particles cannot go through, in the space the simulation works in; a sphere has both ends the same.</summary>
/// <remarks>
/// Share is how much of a contact the ragdoll gives way for: 1 for something that does not move for it (a standing actor's body, a wall), and a half for another ragdoll, which gives way the other half itself. Cloth
/// ignores it, since cloth moves nothing it touches.
/// </remarks>
public readonly record struct Obstacle(Vector3 A, Vector3 B, float Radius, float Share = 1f)
{
    // The nearest points of two segments (Ericson, Real-Time Collision Detection 5.1.9); a segment with both ends the same is a point.
    public static void Closest(Vector3 p1, Vector3 q1, Vector3 p2, Vector3 q2, out Vector3 c1, out Vector3 c2)
    {
        Vector3 d1 = q1 - p1, d2 = q2 - p2, r = p1 - p2;
        float a = Vector3.Dot(d1, d1), e = Vector3.Dot(d2, d2), f = Vector3.Dot(d2, r), s, t;
        const float Tiny = 1e-9f;

        if (a <= Tiny && e <= Tiny) { s = t = 0f; }
        else if (a <= Tiny) { s = 0f; t = Math.Clamp(f / e, 0f, 1f); }
        else
        {
            float c = Vector3.Dot(d1, r);
            if (e <= Tiny) { t = 0f; s = Math.Clamp(-c / a, 0f, 1f); }
            else
            {
                float b = Vector3.Dot(d1, d2), denominator = a * e - b * b;
                s = denominator > Tiny ? Math.Clamp((b * f - c * e) / denominator, 0f, 1f) : 0f;
                t = (b * s + f) / e;
                if (t < 0f) { t = 0f; s = Math.Clamp(-c / a, 0f, 1f); }
                else if (t > 1f) { t = 1f; s = Math.Clamp((b - c) / a, 0f, 1f); }
            }
        }
        c1 = p1 + d1 * s;
        c2 = p2 + d2 * t;
    }
}
