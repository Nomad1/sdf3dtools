using System;
using System.Numerics;

namespace SDFTool
{
    internal class PreparedTriangle
    {
        // Vertices
        public readonly Vector3 A, B, C;

        // Center point and radius
        public readonly Vector3 Center;
        public readonly float Radius;

        // Normal
        private readonly Vector3 Normal;
        // Distance from A to (0,0,0)
        private readonly float ZeroDistance;

        private readonly Vector3 BANorm;
        private readonly Vector3 CBNorm;
        private readonly Vector3 ACNorm;

        // Triangle area
        public readonly float Area;

        private readonly float NormalLengthSquared;
        private readonly Vector3 BAxNormal, CBxNormal, ACxNormal;

        public readonly object Data;

        public readonly Vector3 LowerBound;
        public readonly Vector3 UpperBound;

        public PreparedTriangle(Vector3 a, Vector3 b, Vector3 c, object parent)
        {
            A = a;
            B = b;
            C = c;
            Data = parent;

            Vector3 ba = b - a;
            Vector3 cb = c - b;
            Vector3 ac = a - c;

            BANorm = ba / ba.LengthSquared();
            CBNorm = cb / cb.LengthSquared();
            ACNorm = ac / ac.LengthSquared();

            Normal = Vector3.Cross(ba, ac);
            NormalLengthSquared = Normal.LengthSquared();
            Area = (float)Math.Sqrt(NormalLengthSquared) * 0.5f;

            ZeroDistance = Vector3.Dot(Normal, A);

            BAxNormal = Vector3.Cross(ba, Normal);
            ACxNormal = Vector3.Cross(ac, Normal);
            CBxNormal = Vector3.Cross(cb, Normal);

            Vector3 toCircumsphereCenter = (BAxNormal * ac.LengthSquared() + ACxNormal * ba.LengthSquared()) / (2 * NormalLengthSquared);
            Radius = toCircumsphereCenter.Length();

            Center = a + toCircumsphereCenter;

            LowerBound.X = Math.Min(Math.Min(a.X, b.X), c.X);
            LowerBound.Y = Math.Min(Math.Min(a.Y, b.Y), c.Y);
            LowerBound.Z = Math.Min(Math.Min(a.Z, b.Z), c.Z);
            UpperBound.X = Math.Max(Math.Max(a.X, b.X), c.X);
            UpperBound.Y = Math.Max(Math.Max(a.Y, b.Y), c.Y);
            UpperBound.Z = Math.Max(Math.Max(a.Z, b.Z), c.Z);
        }

        private static float Clamp(float value, float min, float max)
        {
            return Math.Max(Math.Min(value, max), min);
        }

        public float DistanceSqrd(Vector3 p, out Vector3 weights)
        {
            Vector3 pa = p - A;
            Vector3 pb = p - B;
            Vector3 pc = p - C;
            Vector3 ba = B - A;
            Vector3 cb = C - B;
            Vector3 ac = A - C;

            float areaBA = Vector3.Dot(BAxNormal, pa);
            float areaCB = Vector3.Dot(CBxNormal, pb);
            float areaAC = Area - areaBA - areaCB; // /*Vector3.Dot(ACxNormal, pc)*/

            weights = new Vector3(areaBA, areaCB, areaAC);

            // inside/outside test    
            return (Math.Sign(areaBA) +
                    Math.Sign(areaCB) +
                    Math.Sign(areaAC) < 2.0f)
                    ?
                    // 3 edges    
                    Math.Min(Math.Min(
                    (ba * Clamp(Vector3.Dot(BANorm, pa), 0.0f, 1.0f) - pa).LengthSquared(),
                    (cb * Clamp(Vector3.Dot(CBNorm, pb), 0.0f, 1.0f) - pb).LengthSquared()),
                    (ac * Clamp(Vector3.Dot(ACNorm, pc), 0.0f, 1.0f) - pc).LengthSquared())
                    :
                    // 1 face    
                    (Normal - pa).LengthSquared() / NormalLengthSquared;
        }

        public bool IntersectsAABB(Vector3 lb, Vector3 ub)
        {
            return ub.X >= LowerBound.X && lb.X <= UpperBound.X && ub.Y >= LowerBound.Y && lb.Y <= UpperBound.Y && ub.Z >= LowerBound.Z && lb.Z <= UpperBound.Z;
        }

        public bool IntersectsSphere(Vector3 point, float radius)
        {
            float distance = (point - Center).Length();
            return distance < radius + Radius;
        }

        public bool PlaneIntersectsAABB(Vector3 lb, Vector3 ub)
        {
            Vector3 center = (ub + lb) * 0.5f; // Compute AABB center
            Vector3 e = ub - center; // Compute positive extents

            // Compute the projection interval radius of b onto L(t) = b.c + t * p.n
            float r = e.X * Math.Abs(Normal.X) + e.Y * Math.Abs(Normal.Y) + e.Z * Math.Abs(Normal.Z);

            // Compute distance of box center from plane
            float s = Vector3.Dot(Normal, center) - ZeroDistance;

            // Intersection occurs when distance s falls within [-r,+r] interval
            return Math.Abs(s) <= r;
        }
    }
}