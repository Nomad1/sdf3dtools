using System;
using System.Numerics;

namespace SDFTool
{
    internal class PreparedTriangle
    {
        public readonly Vector3 A, B, C;

        public readonly Vector3 Center;
        public readonly float RadiusSqrd;

        public readonly Vector3 Normal;
        public readonly Vector3 NormNormal;
        public readonly float NormDistance;
        public readonly Vector3 BA, CB, AC;
        public readonly float OneNormal2, OneBA2, OneCB2, OneAC2;
        public readonly Vector3 BANormal, CBNormal, ACNormal;

        public readonly object Parent;

        public readonly Vector3 LowerBoundary;
        public readonly Vector3 UpperBoundary;

        public PreparedTriangle(Vector3 a, Vector3 b, Vector3 c, object parent)
        {
            A = a;
            B = b;
            C = c;

            BA = b - a;
            CB = c - b;
            AC = a - c;

            Normal = Vector3.Cross(BA, AC);
            NormNormal = Vector3.Normalize(Normal);
            NormDistance = Vector3.Dot(NormNormal, A);

            OneBA2 = 1.0f / BA.LengthSquared();
            OneCB2 = 1.0f / CB.LengthSquared();
            OneAC2 = 1.0f / AC.LengthSquared();
            OneNormal2 = 1.0f / Normal.LengthSquared();

            Parent = parent;

            BANormal = Vector3.Cross(BA, Normal);
            ACNormal = Vector3.Cross(AC, Normal);
            CBNormal = Vector3.Cross(CB, Normal);

            Vector3 toCircumsphereCenter = (BANormal / OneAC2 + ACNormal / OneBA2) * OneNormal2 / 2.0f;
            RadiusSqrd = toCircumsphereCenter.LengthSquared();

            Center = a + toCircumsphereCenter;

            LowerBoundary.X = Math.Min(Math.Min(a.X, b.X), c.X);
            LowerBoundary.Y = Math.Min(Math.Min(a.Y, b.Y), c.Y);
            LowerBoundary.Z = Math.Min(Math.Min(a.Z, b.Z), c.Z);
            UpperBoundary.X = Math.Max(Math.Max(a.X, b.X), c.X);
            UpperBoundary.Y = Math.Max(Math.Max(a.Y, b.Y), c.Y);
            UpperBoundary.Z = Math.Max(Math.Max(a.Z, b.Z), c.Z);
        }

        private static float Clamp(float value, float min, float max)
        {
            return Math.Max(Math.Min(value, max), min);
        }

        public float DistanceSqrd(Vector3 p)
        {
            Vector3 pa = p - A;
            Vector3 pb = p - B;
            Vector3 pc = p - C;

            // inside/outside test    
            return (Math.Sign(Vector3.Dot(BANormal, pa)) +
                    Math.Sign(Vector3.Dot(CBNormal, pb)) +
                    Math.Sign(Vector3.Dot(ACNormal, pc)) < 2.0f)
                    ?
                    // 3 edges    
                    Math.Min(Math.Min(
                    (BA * Clamp(Vector3.Dot(BA, pa) * OneBA2, 0.0f, 1.0f) - pa).LengthSquared(),
                    (CB * Clamp(Vector3.Dot(CB, pb) * OneCB2, 0.0f, 1.0f) - pb).LengthSquared()),
                    (AC * Clamp(Vector3.Dot(AC, pc) * OneAC2, 0.0f, 1.0f) - pc).LengthSquared())
                    :
                    // 1 face    
                    (Normal - pa).LengthSquared() * OneNormal2;
        }

        public bool IntersectsAABB(Vector3 lb, Vector3 ub)
        {
            Vector3 c = (ub + lb) * 0.5f; // Compute AABB center
            Vector3 e = ub - c; // Compute positive extents

            // Compute the projection interval radius of b onto L(t) = b.c + t * p.n
            float r = e.X * Math.Abs(NormNormal.X) + e.Y * Math.Abs(NormNormal.Y) + e.Z * Math.Abs(NormNormal.Z);

            // Compute distance of box center from plane
            float s = Vector3.Dot(NormNormal, c) - NormDistance;

            // Intersection occurs when distance s falls within [-r,+r] interval
            return Math.Abs(s) <= r;
        }
    }
}