using System.Numerics;

namespace HkxSimSharp.Ragdoll;

/// <summary>The limits of a ragdoll joint, in radians: how far the body may swing from its rest pose, and twist about its own axis.</summary>
/// <remarks>
/// A joint is a ball at the origin of the body's frame, so the body hangs from its parent at a point. Twist is the turn about the body's X axis, within <see cref="TwistMin"/> and <see cref="TwistMax"/>.
/// What is left of the rotation is a swing, taken as turns about Y and Z: the one about Z is within <see cref="PlaneMin"/> and <see cref="PlaneMax"/>, and the whole swing is within <see cref="ConeAngle"/> as a circle,
/// as Havok's ragdoll limits are described. <see cref="ConeOffset"/> is kept but not used: where the game centres the cone could not be found in its code, so the cone is centred on the rest pose. A hinge turns only about its axis (<see cref="HingeAxis"/>, Z where none is given - the game's parameter names one of six, and builds the hinge's frame from it), within <see cref="HingeMin"/> and <see cref="HingeMax"/>. This is how the game's ragdoll parameters are named, read as
/// the simplest limits that fit the names; Havok's own cone is not an exact match.
/// </remarks>
public sealed record RagdollJoint(float ConeAngle, float ConeOffset, float TwistMin, float TwistMax, float PlaneMin, float PlaneMax, bool Hinge = false, float HingeMin = 0f, float HingeMax = 0f, Vector3? HingeAxis = null);

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
public sealed partial class RagdollWorld
{
    private const int SubSteps = 6, Iterations = 6;
    private const float MaxSpeed = 120f, MaxSpin = 60f;

    private readonly RagdollBody[] _bodies;
    private readonly Vector3[] _centre, _position, _previous, _velocity, _spin, _jointParent;
    private readonly Quaternion[] _turn, _turnPrevious, _rest;
    private readonly float[] _inverseMass, _inverseInertia;
    private readonly bool[] _onFloor, _released;
    private readonly float[] _floorPush;
    private readonly Vector3[] _contactNormal;
    private int _calm;
    private float _step;

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
        _released = new bool[n];
        _floorPush = new float[n];
        _contactNormal = new Vector3[n];

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
    public Vector3 Gravity { get; set; } = new(0, -9.8f, 0);

    public Vector3 FloorNormal { get; set; } = Vector3.UnitY;

    // How much speed a body loses each second, as the game's motion properties give it: the Default set has no linear damping and 0.025 angular, a player's ragdoll 0.2 and 0.8. The game's own gravity is the
    // Havok world's default, 9.8 m/s^2 down (read from the physics module's defaults), and a ragdoll body has a gravity scale of 1.
    public float LinearDamping { get; set; } = 0f;

    public float AngularDamping { get; set; } = 0.025f;

    // What a joint takes out of the turning of the body that hangs from it, per second; the game's joints have a friction of their own which is not read here.
    public float JointDamping { get; set; } = 1f;

    // How hard the floor rubs a sliding body (the coefficient of friction), and how much it slows one that is rolling, per second.
    public float FloorFriction { get; set; } = 0.6f;

    public float FloorRolling { get; set; } = 4f;

    public float FloorHeight { get; set; }

    public int Count => _bodies.Length;

    // Lets bodies go from their parents, so they fall as pieces, as a bone enemy's arm and head do when it is knocked apart. With none named, every joint is let go.
    public void Release(IEnumerable<int>? bodies = null)
    {
        foreach (int i in bodies ?? Enumerable.Range(0, _bodies.Length)) if ((uint)i < (uint)_released.Length) _released[i] = true;
    }

    // A sudden push: an impulse (newton seconds) at a point of space. The joints answer a blow at once, so the body struck does not run off alone and be pulled back: every body held to it by joints that are not let go
    // takes the blow together, as one rigid piece, moving with the speed of its centre of mass and turning about it. A body let go of the rest is pushed alone.
    public void ApplyImpulse(int body, Vector3 point, Vector3 impulse)
    {
        Asleep = false;
        _calm = 0;

        List<int> group = Joined(body);
        if (group.Count == 1)
        {
            _velocity[body] += impulse * _inverseMass[body];
            _spin[body] += Vector3.Cross(point - _position[body], impulse) * _inverseInertia[body];
            return;
        }

        // The centre of mass and the inertia of the whole piece about it.
        float mass = 0f;
        Vector3 centre = Vector3.Zero;
        foreach (int i in group)
        {
            mass += 1f / _inverseMass[i];
            centre += _position[i] / _inverseMass[i];
        }
        centre /= mass;

        System.Numerics.Matrix4x4 inertia = System.Numerics.Matrix4x4.Identity;
        float[,] tensor = new float[3, 3];
        foreach (int i in group)
        {
            float m = 1f / _inverseMass[i], own = 1f / _inverseInertia[i];
            Vector3 d = _position[i] - centre;
            float[] a = [d.X, d.Y, d.Z];
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++) tensor[r, c] += (r == c ? own + m * d.LengthSquared() : 0f) - m * a[r] * a[c];
        }
        inertia = new System.Numerics.Matrix4x4(tensor[0, 0], tensor[0, 1], tensor[0, 2], 0, tensor[1, 0], tensor[1, 1], tensor[1, 2], 0, tensor[2, 0], tensor[2, 1], tensor[2, 2], 0, 0, 0, 0, 1);

        Vector3 moment = Vector3.Cross(point - centre, impulse), turn = Vector3.Zero;
        if (System.Numerics.Matrix4x4.Invert(inertia, out System.Numerics.Matrix4x4 inverse)) turn = Vector3.Transform(moment, inverse);

        Vector3 speed = impulse / mass;
        foreach (int i in group)
        {
            _velocity[i] += speed + Vector3.Cross(turn, _position[i] - centre);
            _spin[i] += turn;
        }
    }

    // The bodies held to a body by joints that are in force, the body among them.
    private List<int> Joined(int body)
    {
        int Top(int i)
        {
            while (_bodies[i].Parent >= 0 && !_released[i]) i = _bodies[i].Parent;
            return i;
        }

        int top = Top(body);
        List<int> group = [];
        for (int i = 0; i < _bodies.Length; i++)
            if (Top(i) == top) group.Add(i);
        return group;
    }

    // Where a body's centre is, in the space the world works in.
    public Vector3 CentreOf(int body) => _position[body];

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
        _calm = 0;
        Asleep = false;
        _settled = false;
    }

    // True once the ragdoll has lain still for half a second: it is not stepped any more, so a pile on the floor does not shiver. Reset wakes it.
    public bool Asleep { get; private set; }

    // Lifts the whole ragdoll clear of the floor, as one, for a pose whose feet are in the ground: every joint stays as it was, and no body is given any speed. Done on the first step, since the floor is
    // set after the world is made.
    private bool _settled;

    private void Settle()
    {
        _settled = true;
        float deepest = 0f;
        for (int i = 0; i < _bodies.Length; i++)
        {
            deepest = MathF.Max(deepest, Depth(i, _bodies[i].A));
            deepest = MathF.Max(deepest, Depth(i, _bodies[i].B));
        }
        for (int i = 0; i < _bodies.Length; i++)
        {
            _position[i] += FloorNormal * deepest;
            _previous[i] = _position[i];
            _turnPrevious[i] = _turn[i];
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
        if (Asleep) return;
        if (!_settled) Settle();

        float h = _step = seconds / SubSteps;
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
                _floorPush[i] = 0f;
            }

            for (int k = 0; k < Iterations; k++)
            {
                for (int i = 0; i < _bodies.Length; i++)
                {
                    if (_bodies[i].Parent >= 0 && !_released[i]) SolveJoint(i);
                    Floor(i);
                    Collide(i);
                }
            }

            for (int i = 0; i < _bodies.Length; i++)
            {
                _velocity[i] = (_position[i] - _previous[i]) / h;

                // What the solver pushed a body by is not a speed it should leave with: a pose out of its limits or a foot in the floor must not throw it.
                float fastest = _velocity[i].Length();
                if (fastest > MaxSpeed) _velocity[i] *= MaxSpeed / fastest;
                Quaternion delta = _turn[i] * Quaternion.Conjugate(_turnPrevious[i]);
                if (delta.W < 0f) delta = new Quaternion(-delta.X, -delta.Y, -delta.Z, -delta.W);
                _spin[i] = new Vector3(delta.X, delta.Y, delta.Z) * (2f / h);
                float turning = _spin[i].Length();
                if (turning > MaxSpin) _spin[i] *= MaxSpin / turning;

                // What is lost to the motion's damping, per second as the game's motion properties give it, to the joint it hangs from, and to rubbing on the floor.
                _velocity[i] *= MathF.Max(0f, 1f - LinearDamping * h);
                _spin[i] *= MathF.Max(0f, 1f - (AngularDamping + (_bodies[i].Parent >= 0 && !_released[i] ? JointDamping : 0f)) * h);
                if (_onFloor[i])
                {
                    // Coulomb friction: the speed along the floor falls by at most the friction coefficient times what the floor pushed the body up by.
                    Vector3 along = _velocity[i] - _contactNormal[i] * Vector3.Dot(_velocity[i], _contactNormal[i]);
                    float speed = along.Length(), lost = MathF.Min(speed, FloorFriction * _floorPush[i] / h);
                    if (speed > 1e-6f) _velocity[i] -= along / speed * lost;
                    _spin[i] *= MathF.Max(0f, 1f - FloorRolling * h);

                    // What is nearly still on the floor stays still, so the pile does not shiver.
                    if (_velocity[i].LengthSquared() < 0.0036f) _velocity[i] = Vector3.Zero;
                    if (_spin[i].LengthSquared() < 0.04f) _spin[i] = Vector3.Zero;
                }
            }
        }

        float movement = 0f;
        for (int i = 0; i < _bodies.Length; i++) movement = MathF.Max(movement, MathF.Max(_velocity[i].Length(), _spin[i].Length() * _bodies[i].Radius));
        _calm = movement < 0.15f ? _calm + 1 : 0;
        if (_calm >= 60) Asleep = true;
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

        Quaternion allowed = JointLimit.Clamp(joint, deviation);
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
        PushOutOfFloor(i, body.A);
        PushOutOfFloor(i, body.B);
    }

    // How far the capsule's round end at that point of a body is into the floor; negative where it is clear.
    private float Depth(int i, Vector3 end) =>
        _bodies[i].Radius - (Vector3.Dot(_position[i] + Vector3.Transform(end - _centre[i], _turn[i]), FloorNormal) - FloorHeight);

    private void PushOutOfFloor(int i, Vector3 end)
    {
        float depth = Depth(i, end);
        if (depth <= 0f) return;

        Vector3 r = Vector3.Transform(end - _centre[i], _turn[i]), n = FloorNormal;
        Move(i, r, n * (depth / Weight(i, r, n)));
        Touch(i, n, depth);
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
}
