using System.Numerics;

namespace HkxSimSharp.Ragdoll;

/// <summary>The limits of a ragdoll joint, in radians: how far the body may swing from its rest pose, and twist about its own axis.</summary>
/// <remarks>
/// A joint is a ball at the origin of the body's frame, so the body hangs from its parent at a point. Twist is the turn about the body's X axis, within <see cref="TwistMin"/> and <see cref="TwistMax"/>.
/// What is left of the rotation is a swing, taken as turns about Y and Z: the one about Y is within <see cref="ConeAngle"/> either side of <see cref="ConeOffset"/>, and the one about Z within
/// <see cref="PlaneMin"/> and <see cref="PlaneMax"/>. A hinge turns only about Z, within <see cref="HingeMin"/> and <see cref="HingeMax"/>. This is how the game's ragdoll parameters are named, read as
/// the simplest limits that fit the names; Havok's own cone is not an exact match.
/// </remarks>
public sealed record RagdollJoint(float ConeAngle, float ConeOffset, float TwistMin, float TwistMax, float PlaneMin, float PlaneMax, bool Hinge = false, float HingeMin = 0f, float HingeMax = 0f);

/// <summary>One rigid body of a ragdoll: a capsule in the frame of the bone it is on, its mass, and the joint to its parent.</summary>
/// <remarks>Parent is the index of the parent body in the list given to <see cref="RagdollWorld"/>, which comes before it, or -1 for a body that is free. Joint is null for a free body.</remarks>
public sealed record RagdollBody(int Parent, Vector3 A, Vector3 B, float Radius, float Mass, RagdollJoint? Joint);

/// <summary>A ragdoll falling and settling: rigid capsules joined at their bones, under gravity, on a floor.</summary>
/// <remarks>
/// <para>
/// A drop test of an actor's ragdoll as its parameters describe it, to see what its masses, sizes and joint limits do before the game does. It is position based: each step the bodies are moved by their
/// velocities and gravity, then the joints, their limits and the floor are satisfied by moving them back, and velocities are taken from where they ended. It is not Havok: there is no contact between
/// bodies of the ragdoll, no friction model beyond a damping of what slides on the floor, and the joint limits are as <see cref="RagdollJoint"/> says. It is meant to show a ragdoll that hangs wrong, falls
/// through itself or never settles, not to match the game's motion.
/// </para>
/// <para>
/// A body's frame is its bone's; <see cref="FrameOf"/> gives where that frame is. Everything is in one space, which is the actor's.
/// </para>
/// </remarks>
public sealed class RagdollWorld
{
    private const int SubSteps = 6, Iterations = 6;

    private readonly RagdollBody[] _bodies;
    private readonly Vector3[] _centre, _position, _previous, _velocity, _spin, _jointParent;
    private readonly Quaternion[] _turn, _turnPrevious, _rest;
    private readonly float[] _inverseMass, _inverseInertia;
    private readonly bool[] _onFloor;

    public RagdollWorld(IReadOnlyList<RagdollBody> bodies, IReadOnlyList<Matrix4x4> frames)
    {
        _bodies = [.. bodies];
        int n = _bodies.Length;
        _centre = new Vector3[n];
        _position = new Vector3[n];
        _previous = new Vector3[n];
        _velocity = new Vector3[n];
        _spin = new Vector3[n];
        _jointParent = new Vector3[n];
        _turn = new Quaternion[n];
        _turnPrevious = new Quaternion[n];
        _rest = new Quaternion[n];
        _inverseMass = new float[n];
        _inverseInertia = new float[n];
        _onFloor = new bool[n];

        for (int i = 0; i < n; i++)
        {
            RagdollBody body = _bodies[i];
            _centre[i] = (body.A + body.B) * 0.5f;
            float length = Vector3.Distance(body.A, body.B), mass = MathF.Max(body.Mass, 0.01f);
            _inverseMass[i] = 1f / mass;
            _inverseInertia[i] = 1f / MathF.Max(mass * (length * length / 12f + body.Radius * body.Radius * 0.5f), mass * body.Radius * body.Radius * 0.4f);
        }
        Reset(frames);
    }

    // Gravity, and the floor: the plane of points whose dot with FloorNormal is FloorHeight.
    public Vector3 Gravity { get; set; } = new(0, -9.81f, 0);

    public Vector3 FloorNormal { get; set; } = Vector3.UnitY;

    public float FloorHeight { get; set; }

    public int Count => _bodies.Length;

    // Puts every body where a pose has its frame, at rest.
    public void Reset(IReadOnlyList<Matrix4x4> frames)
    {
        for (int i = 0; i < _bodies.Length; i++)
        {
            Matrix4x4.Decompose(frames[i], out _, out Quaternion q, out Vector3 origin);
            _turn[i] = _turnPrevious[i] = Quaternion.Normalize(q);
            _position[i] = _previous[i] = origin + Vector3.Transform(_centre[i], _turn[i]);
            _velocity[i] = _spin[i] = Vector3.Zero;
        }

        // The rest pose of each joint is the one the pose has, and where it is anchored in the parent is read from it.
        for (int i = 0; i < _bodies.Length; i++)
        {
            int parent = _bodies[i].Parent;
            if (parent < 0) continue;

            _rest[i] = Quaternion.Normalize(Quaternion.Conjugate(_turn[parent]) * _turn[i]);
            Vector3 anchor = _position[i] + Vector3.Transform(-_centre[i], _turn[i]);
            _jointParent[i] = Vector3.Transform(anchor - _position[parent], Quaternion.Conjugate(_turn[parent]));
        }
    }

    // Where a body's frame, which is its bone's, is now.
    public Matrix4x4 FrameOf(int body)
    {
        Matrix4x4 frame = Matrix4x4.CreateFromQuaternion(_turn[body]);
        frame.Translation = _position[body] + Vector3.Transform(-_centre[body], _turn[body]);
        return frame;
    }

    public void Step(float seconds)
    {
        float h = seconds / SubSteps;
        for (int s = 0; s < SubSteps; s++)
        {
            for (int i = 0; i < _bodies.Length; i++)
            {
                _previous[i] = _position[i];
                _turnPrevious[i] = _turn[i];
                _velocity[i] += Gravity * h;
                _position[i] += _velocity[i] * h;
                Quaternion spin = new(_spin[i] * 0.5f * h, 0f);
                _turn[i] = Quaternion.Normalize(_turn[i] + spin * _turn[i]);
                _onFloor[i] = false;
            }

            for (int k = 0; k < Iterations; k++)
            {
                for (int i = 0; i < _bodies.Length; i++)
                {
                    if (_bodies[i].Parent >= 0) SolveJoint(i);
                    Floor(i);
                }
            }

            for (int i = 0; i < _bodies.Length; i++)
            {
                _velocity[i] = (_position[i] - _previous[i]) / h;
                Quaternion delta = _turn[i] * Quaternion.Conjugate(_turnPrevious[i]);
                if (delta.W < 0f) delta = new Quaternion(-delta.X, -delta.Y, -delta.Z, -delta.W);
                _spin[i] = new Vector3(delta.X, delta.Y, delta.Z) * (2f / h);

                // What is lost to the air and to the joints' friction, and what slides on the floor.
                _velocity[i] *= 0.9985f;
                _spin[i] *= 0.995f;
                if (_onFloor[i])
                {
                    Vector3 along = _velocity[i] - FloorNormal * Vector3.Dot(_velocity[i], FloorNormal);
                    _velocity[i] -= along * 0.15f;
                    _spin[i] *= 0.9f;
                }
            }
        }
    }

    // Keeps a body hanging from its parent at the joint, and within the joint's limits.
    private void SolveJoint(int i)
    {
        int p = _bodies[i].Parent;
        Vector3 r1 = Vector3.Transform(_jointParent[i], _turn[p]), r2 = Vector3.Transform(-_centre[i], _turn[i]);
        Vector3 error = _position[i] + r2 - (_position[p] + r1);
        if (error.LengthSquared() > 1e-12f)
        {
            Vector3 n = Vector3.Normalize(error);
            float w1 = Weight(p, r1, n), w2 = Weight(i, r2, n);
            Vector3 push = n * (error.Length() / (w1 + w2));
            Move(p, r1, push);
            Move(i, r2, -push);
        }
        if (_bodies[i].Joint is { } joint) Limit(i, p, joint);
    }

    // Turns the child, and the parent the other way, until the joint is inside its limits.
    private void Limit(int i, int p, RagdollJoint joint)
    {
        Quaternion relative = Quaternion.Conjugate(_turn[p]) * _turn[i];
        Quaternion deviation = Quaternion.Conjugate(_rest[i]) * relative;
        if (deviation.W < 0f) deviation = new Quaternion(-deviation.X, -deviation.Y, -deviation.Z, -deviation.W);

        Quaternion twist = new(deviation.X, 0f, 0f, deviation.W);
        twist = twist.LengthSquared() > 1e-10f ? Quaternion.Normalize(twist) : Quaternion.Identity;
        Quaternion swing = deviation * Quaternion.Conjugate(twist);

        float twistAngle = 2f * MathF.Atan2(twist.X, twist.W);
        Vector3 swingVector = RotationVector(swing);
        (float y, float z) = (swingVector.Y, swingVector.Z);

        if (joint.Hinge)
        {
            twistAngle = 0f;
            y = 0f;
            z = Math.Clamp(z, joint.HingeMin, joint.HingeMax);
        }
        else
        {
            twistAngle = Math.Clamp(twistAngle, joint.TwistMin, joint.TwistMax);
            y = Math.Clamp(y, joint.ConeOffset - joint.ConeAngle, joint.ConeOffset + joint.ConeAngle);
            z = Math.Clamp(z, joint.PlaneMin, joint.PlaneMax);
        }

        Quaternion allowed = FromRotationVector(new Vector3(0f, y, z)) * Quaternion.CreateFromAxisAngle(Vector3.UnitX, twistAngle);
        Quaternion wanted = _turn[p] * _rest[i] * allowed;
        Quaternion correction = wanted * Quaternion.Conjugate(_turn[i]);
        if (Math.Abs(correction.W) > 0.99999f) return;

        // The lighter in turning does more of it.
        float child = _inverseInertia[i], parent = _inverseInertia[p], share = child / (child + parent);
        _turn[i] = Quaternion.Normalize(Quaternion.Slerp(Quaternion.Identity, correction, share) * _turn[i]);
        _turn[p] = Quaternion.Normalize(Quaternion.Slerp(Quaternion.Identity, Quaternion.Conjugate(correction), 1f - share) * _turn[p]);
    }

    // Keeps the ends of a capsule from going through the floor.
    private void Floor(int i)
    {
        RagdollBody body = _bodies[i];
        foreach (Vector3 end in new[] { body.A, body.B })
        {
            Vector3 r = Vector3.Transform(end - _centre[i], _turn[i]);
            float depth = body.Radius - (Vector3.Dot(_position[i] + r, FloorNormal) - FloorHeight);
            if (depth <= 0f) continue;

            Vector3 n = FloorNormal;
            float w = Weight(i, r, n);
            Move(i, r, n * (depth / w));
            _onFloor[i] = true;
        }
    }

    // How readily a body gives to a push along a direction at a point r from its centre: the inverse mass and the turn it would take.
    private float Weight(int i, Vector3 r, Vector3 n)
    {
        Vector3 arm = Vector3.Cross(r, n);
        return _inverseMass[i] + _inverseInertia[i] * arm.LengthSquared();
    }

    // Moves a body by a push at a point r from its centre, turning it as the push would.
    private void Move(int i, Vector3 r, Vector3 push)
    {
        _position[i] += push * _inverseMass[i];
        Vector3 turn = Vector3.Cross(r, push) * _inverseInertia[i];
        _turn[i] = Quaternion.Normalize(_turn[i] + new Quaternion(turn * 0.5f, 0f) * _turn[i]);
    }

    private static Vector3 RotationVector(Quaternion q)
    {
        if (q.W < 0f) q = new Quaternion(-q.X, -q.Y, -q.Z, -q.W);
        float s = MathF.Sqrt(q.X * q.X + q.Y * q.Y + q.Z * q.Z);
        if (s < 1e-6f) return Vector3.Zero;

        return new Vector3(q.X, q.Y, q.Z) / s * (2f * MathF.Atan2(s, q.W));
    }

    private static Quaternion FromRotationVector(Vector3 v)
    {
        float angle = v.Length();
        return angle < 1e-6f ? Quaternion.Identity : Quaternion.CreateFromAxisAngle(v / angle, angle);
    }
}
