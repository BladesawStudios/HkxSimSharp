using System.Numerics;

namespace HkxSimSharp.Ragdoll;

public sealed partial class RagdollWorld
{
    // The furthest a contact pushes a body out in one go. A ragdoll that begins inside another actor's shape (two actors placed overlapping) is eased out of it, not thrown out of it.
    private const float MostEase = 0.02f;

    // How far ahead of touching a contact is already felt, so a body moving fast enough to cross a shape in a step still meets it.
    private const float Lookahead = 0.5f;

    private Obstacle[] _obstacles = [];

    // What the bodies collide with besides the floor, from now until it is set again. The caller keeps it current as the shapes move.
    public void SetObstacles(IReadOnlyList<Obstacle> obstacles) => _obstacles = [.. obstacles];

    // The bodies as capsules, where they are now, for another world to collide with.
    public Obstacle[] Capsules(float share = 0.5f)
    {
        Obstacle[] shapes = new Obstacle[_bodies.Length];
        for (int i = 0; i < shapes.Length; i++)
            shapes[i] = new Obstacle(End(i, _bodies[i].A), End(i, _bodies[i].B), _bodies[i].Radius, share, _velocity[i]);
        return shapes;
    }

    // Where the end of a body's capsule is now.
    private Vector3 End(int i, Vector3 end) => _position[i] + Vector3.Transform(end - _centre[i], _turn[i]);

    // Records that something held a body up: how hard it pushed, along which way, so the friction knows what rubs.
    private void Touch(int i, Vector3 normal, float depth)
    {
        _onFloor[i] = true;
        if (depth < _floorPush[i]) return;

        _floorPush[i] = depth;
        _contactNormal[i] = normal;
    }

    // Keeps a body out of the obstacles.
    private void Collide(int i)
    {
        if (_obstacles.Length == 0) return;

        Vector3 a = End(i, _bodies[i].A), b = End(i, _bodies[i].B);
        float radius = _bodies[i].Radius, reach = Vector3.Distance(a, b) * 0.5f + radius;
        Vector3 middle = (a + b) * 0.5f;
        foreach (Obstacle o in _obstacles)
        {
            float near = reach + Vector3.Distance(o.A, o.B) * 0.5f + o.Radius + Lookahead;
            if (Vector3.DistanceSquared(middle, (o.A + o.B) * 0.5f) > near * near) continue;

            Obstacle.Closest(a, b, o.A, o.B, out Vector3 mine, out Vector3 theirs);
            Vector3 apart = mine - theirs;
            float distance = apart.Length(), depth = radius + o.Radius - distance;

            // Centres on one another leave no way out to read from them: out the way the floor is, which is always a way.
            Vector3 n = distance > 1e-5f ? apart / distance : FloorNormal;
            Vector3 r = mine - _position[i];

            // What the body gains or loses on meeting it: the speed it closes at, taken up as far as the obstacle's share of the meeting. A moving obstacle (another ragdoll) hands its speed on,
            // and a body about to touch it takes the blow in the step before it does, so a fast one is not through before it is felt.
            float closing = -Vector3.Dot(_velocity[i] + Vector3.Cross(_spin[i], r) - o.Velocity, n);
            float approach = MathF.Max(closing, 0f) * _step;
            if (depth + approach <= 0f) continue;

            float eased = MathF.Min(MathF.Max(depth, 0f), MostEase);
            Move(i, r, n * ((eased + approach) * o.Share / Weight(i, r, n)));
            if (depth > 0f) Touch(i, n, eased);
        }
    }
}
