using System.Numerics;

namespace HkxSimSharp.Ragdoll;

/// <summary>The turn a ragdoll joint allows nearest to the turn it has, from the limits the joint is given.</summary>
/// <remarks>
/// The turn is taken as a deviation from the joint's rest pose. A hinge keeps only the part of it about its axis, within its limits. Any other joint splits it into a twist about X, kept within the twist
/// limits, and a swing: the part about Z within the plane limits, and the whole swing within the cone angle as a circle. The result is a rotation, the nearest the limits allow. The limits are read as including the rest pose, since the game's are set against its own frames and a pose outside them would be thrown to the edge at the first step.
/// </remarks>
internal static class JointLimit
{
    public static Quaternion Clamp(RagdollJoint joint, Quaternion deviation)
    {
        if (joint.Hinge)
        {
            // The whole of the turn that is about the hinge's axis, kept within its limits, and no other.
            Vector3 axis = joint.HingeAxis is { } a && a.LengthSquared() > 1e-8f ? Vector3.Normalize(a) : Vector3.UnitZ;
            return FromRotationVector(axis * Math.Clamp(Vector3.Dot(RotationVector(deviation), axis), Math.Min(joint.HingeMin, 0f), Math.Max(joint.HingeMax, 0f)));
        }

        Quaternion twist = new(deviation.X, 0f, 0f, deviation.W);
        twist = twist.LengthSquared() > 1e-10f ? Quaternion.Normalize(twist) : Quaternion.Identity;
        Quaternion swing = deviation * Quaternion.Conjugate(twist);

        float twistAngle = Math.Clamp(2f * MathF.Atan2(twist.X, twist.W), Math.Min(joint.TwistMin, 0f), Math.Max(joint.TwistMax, 0f));
        Vector3 swingVector = RotationVector(swing);

        // The plane limit bounds the turn about Z, and the cone the whole swing, as a circle: the angle between the two twist axes is at most the cone angle.
        float y = swingVector.Y, z = Math.Clamp(swingVector.Z, Math.Min(joint.PlaneMin, 0f), Math.Max(joint.PlaneMax, 0f));
        float swingAngle = MathF.Sqrt(y * y + z * z);
        if (swingAngle > joint.ConeAngle && swingAngle > 1e-6f)
        {
            float shrink = joint.ConeAngle / swingAngle;
            y *= shrink;
            z *= shrink;
        }
        return FromRotationVector(new Vector3(0f, y, z)) * Quaternion.CreateFromAxisAngle(Vector3.UnitX, twistAngle);
    }

    internal static Vector3 RotationVector(Quaternion q)
    {
        if (q.W < 0f) q = new Quaternion(-q.X, -q.Y, -q.Z, -q.W);
        float s = MathF.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z);
        if (s < 1e-6f) return Vector3.Zero;

        return new Vector3(q.X, q.Y, q.Z) / s * (2f * MathF.Atan2(s, q.W));
    }

    internal static Quaternion FromRotationVector(Vector3 v)
    {
        float angle = v.Length();
        return angle < 1e-6f ? Quaternion.Identity : Quaternion.CreateFromAxisAngle(v / angle, angle);
    }
}
