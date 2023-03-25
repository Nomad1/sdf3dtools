using System;
using System.Numerics;

namespace SDFTool
{
    public struct Vector3i : IComparable<Vector3i>
    {
        public readonly int X, Y, Z;
        public readonly int LengthSqrd;

        public Vector3i(int x, int y, int z)
        {
            X = x;
            Y = y;
            Z = z;
            LengthSqrd = X * X + Y * Y + Z * Z;
        }

        public override string ToString()
        {
            return String.Format("[{0}, {1}, {2}]", X, Y, Z);
        }

        public int CompareTo(Vector3i other)
        {
            if (X == other.X && Y == other.Y && Z == other.Z)
                return 0;

            return LengthSqrd > other.LengthSqrd ? 1 : -1; // no equality!
            //return this.ToString().CompareTo(other.ToString());
        }
    }

    public struct VectorPair
    {
        public readonly Vector3i Item1;
        public readonly Vector3i Item2;

        public VectorPair(Vector3i item1, Vector3i item2)
        {
            if (item1.CompareTo(item2) > 0)
            {
                Item1 = item2;
                Item2 = item1;
            }
            else
            {
                Item1 = item1;
                Item2 = item2;
            }
        }

        public override string ToString()
        {
            return string.Format("{0} - {1}", Item1, Item2);
        }
    }

    internal class PreparedTriangle
    {
        public readonly int Id;

        // Vertices
        private readonly Vector3 A, B, C;
        // Normal
        public readonly Vector3 N;

        // AABB
        public readonly Vector3 LowerBound;
        public readonly Vector3 UpperBound;

        // Radius - maximal distance form center to vertices
        private readonly float Radius;

        // Triangle area
        public readonly float Area;
        private readonly float NormalLength;

        public readonly object Data;

        public PreparedTriangle(int id, Vector3 a, Vector3 b, Vector3 c, object parent)
        {
            Id = id;

            A = a;
            B = b;
            C = c;
           
            Data = parent;

            N = Vector3.Cross(b - a, c - a);

            Area = N.LengthSquared();
            NormalLength = Math.Max((float)Math.Sqrt(Area), 1.0e-20f);
            N /= NormalLength;
            Area /= 2;

            Vector3 center = (a + b + c) * 0.33333333333f;
            Radius = (float)Math.Sqrt(Math.Max(Math.Max(Vector3.DistanceSquared(center, A), Vector3.DistanceSquared(center, B)), Vector3.DistanceSquared(center, C)));

            LowerBound.X = Math.Min(Math.Min(a.X, b.X), c.X);
            LowerBound.Y = Math.Min(Math.Min(a.Y, b.Y), c.Y);
            LowerBound.Z = Math.Min(Math.Min(a.Z, b.Z), c.Z);
            UpperBound.X = Math.Max(Math.Max(a.X, b.X), c.X);
            UpperBound.Y = Math.Max(Math.Max(a.Y, b.Y), c.Y);
            UpperBound.Z = Math.Max(Math.Max(a.Z, b.Z), c.Z);
        }

#if !USE_OLD_DISTANCE
        public static Vector3 ClosetPointToTriangle(PreparedTriangle t, Vector3 p, out Vector3 weights)
        {
            Vector3 pa = p - t.A;
            Vector3 pb = p - t.B;
            Vector3 pc = p - t.C;
            Vector3 ba = t.B - t.A;
            Vector3 cb = t.C - t.B;
            Vector3 ac = t.A - t.C;

            float u = Vector3.Dot(t.N, Vector3.Cross(cb, pb)) / t.NormalLength;
            float v = Vector3.Dot(t.N, Vector3.Cross(ac, pc)) / t.NormalLength;
            float w = Vector3.Dot(t.N, Vector3.Cross(ba, pa)) / t.NormalLength;

            if (Math.Sign(w) + Math.Sign(u) + Math.Sign(v) < 2.0f)
            {
                float nab = Helper.Clamp(Vector3.Dot(ba, pa) / ba.LengthSquared(), 0.0f, 1.0f);
                float nbc = Helper.Clamp(Vector3.Dot(cb, pb) / cb.LengthSquared(), 0.0f, 1.0f);
                float nca = Helper.Clamp(Vector3.Dot(ac, pc) / ac.LengthSquared(), 0.0f, 1.0f);

                Vector3 ppa = ba * nab + t.A;
                Vector3 ppb = cb * nbc + t.B;
                Vector3 ppc = ac * nca + t.C;

                float lna = (ppa - p).LengthSquared();
                float lnb = (ppb - p).LengthSquared();
                float lnc = (ppc - p).LengthSquared();

                if (lna <= lnc && lna <= lnb)
                {
                    weights = new Vector3(1.0f - nab, nab, 0);
                    return ppa;
                }
                if (lnc <= lna && lnc <= lnb)
                {
                    weights = new Vector3(nca, 0, 1.0f - nca);
                    return ppc;
                }
                if (lnb <= lna && lnb <= lnc)
                {
                    weights = new Vector3(0, 1.0f - nbc, nbc);
                    return ppb;
                }
            }

            weights = new Vector3(u, v, 1.0f - u - v);
            return t.A * weights.X + t.B * weights.Y + t.C * weights.Z;
        }
#else
        public static Vector3 ClosetPointToTriangle(PreparedTriangle t, Vector3 p, out Vector3 weights)
        {
            float snom = Vector3.Dot(p - t.A, t.B - t.A);
            float sdenom = Vector3.Dot(p - t.B, t.A - t.B);

            float tnom = Vector3.Dot(p - t.A, t.C - t.A);
            float tdenom = Vector3.Dot(p - t.C, t.A - t.C);

            float unom = Vector3.Dot(p - t.B, t.C - t.B);
            float udenom = Vector3.Dot(p - t.C, t.B - t.C);

            if (snom <= 0.0 && tnom <= 0.0)
            {
                weights = new Vector3(1.0f, 0.0f, 0.0f);
                return t.A;
            }

            if (sdenom <= 0.0 && unom <= 0.0)
            {
                weights = new Vector3(0.0f, 1.0f, 0.0f);
                return t.B;
            }

            if (tdenom <= 0.0 && udenom <= 0.0)
            {
                weights = new Vector3(0.0f, 0.0f, 1.0f);
                return t.C;
            }

            // for AB check triple scalar product [N PA PB]
            float coordsPAB = Vector3.Dot(t.N, Vector3.Cross(t.A - p, t.B - p));
            if (coordsPAB <= 0 && snom >= 0.0 && sdenom >= 0.0)
            {
                float nab = snom / (snom + sdenom);
                weights = new Vector3(1.0f - nab, nab, 0);
                return t.A + nab * (t.B - t.A);
            }

            // for BC check triple scalar product [N PB PC]
            float coordsPBC = Vector3.Dot(t.N, Vector3.Cross(t.B - p, t.C - p));
            if (coordsPBC <= 0 && unom >= 0.0 && udenom >= 0.0)
            {
                float nbc = unom / (unom + udenom);
                weights = new Vector3(0, 1.0f - nbc, nbc);
                return t.B + nbc * (t.C - t.B);
            }

            // for CA check triple scalar product [N PC PA]
            float coordsPCA = Vector3.Dot(t.N, Vector3.Cross(t.C - p, t.A - p));
            if (coordsPCA <= 0 && tnom >= 0.0 && tdenom >= 0.0)
            {
                float nca = tnom / (tnom + tdenom);
                weights = new Vector3(nca, 0, 1.0f - nca);
                return t.A + nca * (t.C - t.A);
            }

            // P is inside triangle
            // normalize barycentric coordinates
            float u = coordsPBC / (coordsPAB + coordsPBC + coordsPCA);
            float v = coordsPCA / (coordsPAB + coordsPBC + coordsPCA);

            weights = new Vector3(u, v, 1.0f - u - v);
            return t.A * weights.X + t.B * weights.Y + t.C * weights.Z;
        }
#endif
        public bool IntersectsRay(Vector3 p, Vector3 dir)
        {
            Vector3 ba = B - A;
            Vector3 ca = C - A;

            Vector3 h = Vector3.Cross(dir, ca);
            float proj = Vector3.Dot(ba, h);

            if (Math.Abs(proj) < float.Epsilon)
            {
                return false;  // ray is parallel to triangle
            }

            proj = 1.0f / proj;

            Vector3 pa = p - A;
            float u = Vector3.Dot(pa, h) * proj;

            if (u < 0.0 || u > 1.0)
            {
                return false;
            }

            Vector3 q = Vector3.Cross(pa, ba);
            float v = Vector3.Dot(dir, q) * proj;

            if (v < 0.0f || u + v > 1.0f)
            {
                return false;
            }

            float t = Vector3.Dot(ca, q) * proj;

            return t >= 0.0f;
        }

        public bool IntersectsAABB(Vector3 lb, Vector3 ub)
        {
            return ub.X >= LowerBound.X && lb.X <= UpperBound.X && ub.Y >= LowerBound.Y && lb.Y <= UpperBound.Y && ub.Z >= LowerBound.Z && lb.Z <= UpperBound.Z;
        }

        public bool IntersectsSphere(Vector3 point, float radius)
        {
            float distance = (point - (A + B + C) * 0.33333333333f).Length();
            return distance < radius + Radius;
        }

        public bool PlaneIntersectsAABB(Vector3 lb, Vector3 ub)
        {
            if (!IntersectsAABB(lb, ub))
                return false;

            Vector3 center = (ub + lb) * 0.5f; // Compute AABB center
            Vector3 e = ub - center; // Compute positive extents

            // Compute the projection interval radius of b onto L(t) = b.c + t * p.n
            float r = e.X * Math.Abs(N.X) + e.Y * Math.Abs(N.Y) + e.Z * Math.Abs(N.Z);

            // Compute distance of box center from plane
            float s = Vector3.Dot(N, center) - Vector3.Dot(N, A);

            // Intersection occurs when distance s falls within [-r,+r] interval
            return Math.Abs(s) <= r;
        }
    }
}