using System;
using System.Collections.Generic;
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

    public class PseudoNormal
    {
        private Vector3 m_value;
        private int m_count = 0;

        public Vector3 Value
        {
            get { return m_value; }
        }
        public int Count
        {
            get { return m_count; }
        }

        public void Add(Vector3 value)
        {
            m_value += value;
            m_count++;
        }
    }

    internal enum TriangleRegion
    {
        Center = 0,
        EdgeAB = 1,
        EdgeBC = 2,
        EdgeCA = 3,
        VertexA = 4,
        VertexB = 5,
        VertexC = 6,

        Max = 7
    } 

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

        // Triangle area
        private readonly float NormalLength;
        public readonly float Area;

        public readonly object Data;
#if USE_PSEUDO_NORMALS
        public readonly PseudoNormal[] PseudoNormals;
#endif

        public readonly Vector3 LowerBound;
        public readonly Vector3 UpperBound;

        public PreparedTriangle(Vector3 a, Vector3 b, Vector3 c,
            object parent
#if USE_PSEUDO_NORMALS
            , PseudoNormal[] normals
#endif
            )
        {
            A = a;
            B = b;
            C = c;
            Data = parent;

            Normal = Vector3.Cross(b - a, c - a);

            Area = Normal.LengthSquared();
            NormalLength = Math.Max((float)Math.Sqrt(Area), 1.0e-20f);
            Normal /= NormalLength;
            Area /= 2;
#if USE_PSEUDO_NORMALS
            PseudoNormals = normals;
            PseudoNormals[(int)TriangleRegion.Center].Add(Normal);
            PseudoNormals[(int)TriangleRegion.EdgeAB].Add(Normal * (float)Math.PI);
            PseudoNormals[(int)TriangleRegion.EdgeBC].Add(Normal * (float)Math.PI);
            PseudoNormals[(int)TriangleRegion.EdgeCA].Add(Normal * (float)Math.PI);
            PseudoNormals[(int)TriangleRegion.VertexA].Add((float)Math.Acos(Vector3.Dot(b - a, c - a) / ((b - a).Length() * (c - a).Length())) * Normal);
            PseudoNormals[(int)TriangleRegion.VertexB].Add((float)Math.Acos(Vector3.Dot(a - b, c - b) / ((a - b).Length() * (c - b).Length())) * Normal);
            PseudoNormals[(int)TriangleRegion.VertexC].Add((float)Math.Acos(Vector3.Dot(a - c, b - c) / ((a - c).Length() * (b - c).Length())) * Normal);
#endif
            ZeroDistance = Vector3.Dot(Normal, A);

            Center = (a + b + c) / 3;
            Radius = (float)Math.Sqrt(Math.Max(Math.Max(Vector3.DistanceSquared(Center, A), Vector3.DistanceSquared(Center, B)), Vector3.DistanceSquared(Center, C)));

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

#if USE_PSEUDO_NORMALS
        public float DistanceToPoint(Vector3 p, out Vector3 weights, out Vector3 pp, out int sign)
        {
            Vector3 pa = p - A;
            Vector3 pb = p - B;
            Vector3 pc = p - C;
            Vector3 ba = B - A;
            Vector3 cb = C - B;
            Vector3 ac = A - C;

            float u = Vector3.Dot(Vector3.Cross(cb, pb), Normal) / NormalLength;
            float v = Vector3.Dot(Vector3.Cross(ac, pc), Normal) / NormalLength;
            float w = 1.0f - u - v;
            weights = new Vector3(u, v, w);

            TriangleRegion region;
            float len;

            if (Math.Sign(w) + Math.Sign(u) + Math.Sign(v) < 2.0f)
            {
                float nab = Clamp(Vector3.Dot(ba, pa) / ba.LengthSquared(), 0.0f, 1.0f);
                float nbc = Clamp(Vector3.Dot(cb, pb) / cb.LengthSquared(), 0.0f, 1.0f);
                float nca = Clamp(Vector3.Dot(ac, pc) / ac.LengthSquared(), 0.0f, 1.0f);

                Vector3 ppa = ba * nab + A;
                Vector3 ppb = cb * nbc + B;
                Vector3 ppc = ac * nca + C;

                float lna = (ppa - p).LengthSquared();
                float lnb = (ppb - p).LengthSquared();
                float lnc = (ppc - p).LengthSquared();

                if ((u > 0 && lna < lnc) || (v > 0 && lna < lnb))
                {
                    region = nab <= 0 ? TriangleRegion.VertexA : nab >= 1 ? TriangleRegion.VertexB : TriangleRegion.EdgeAB;
                    weights = new Vector3(1.0f - nab, nab, 0);
                    pp = ppa;
                    len = lna;
                }
                else if ((u > 0 && lna > lnc) || (w > 0 && lnc < lnb))
                {
                    region = nca <= 0 ? TriangleRegion.VertexC : nca >= 1 ? TriangleRegion.VertexA : TriangleRegion.EdgeCA;
                    weights = new Vector3(nca, 0, 1.0f - nca);
                    pp = ppc;
                    len = lnc;
                }
                else if ((v > 0 && lna > lnb) || (w > 0 && lnc > lnb))
                {
                    region = nbc <= 0 ? TriangleRegion.VertexB : nbc >= 1 ? TriangleRegion.VertexC : TriangleRegion.EdgeBC;
                    weights = new Vector3(0, 1.0f - nbc, nbc);
                    pp = ppb;
                    len = lnb;
                }
                else // never happens
                {
                    sign = 0;
                    pp = Vector3.Zero;
                    if (weights.X < 0 || weights.Y < 0 || weights.Z < 0 || weights.X > 1 || weights.Y > 1 || weights.Z > 1)
                        Console.WriteLine("Weights are invalid!");
                    return float.MaxValue;
                }
            }
            else
            {
                pp = A * weights.X + B * weights.Y + C * weights.Z;
                region = TriangleRegion.Center;
                len = Vector3.DistanceSquared(p, pp);
            }

            if (weights.X < 0 || weights.Y < 0 || weights.Z < 0 || weights.X > 1 || weights.Y > 1 || weights.Z > 1)
                Console.WriteLine("Weights are invalid!");

            float dot = Vector3.Dot(PseudoNormals[(int)region].Value, p - pp);

            sign = Math.Sign(dot);

            return (float)Math.Sqrt(len);            
        }
#endif
        public float DistanceToPoint(Vector3 p, out Vector3 weights, out Vector3 pp)
        {
            Vector3 pa = p - A;
            Vector3 pb = p - B;
            Vector3 pc = p - C;
            Vector3 ba = B - A;
            Vector3 cb = C - B;
            Vector3 ac = A - C;

            float u = Vector3.Dot(Vector3.Cross(cb, pb), Normal) / NormalLength;
            float v = Vector3.Dot(Vector3.Cross(ac, pc), Normal) / NormalLength;
            float w = Vector3.Dot(Vector3.Cross(ba, pa), Normal) / NormalLength;
            //1.0f - u - v;
            weights = new Vector3(u, v, w);

            float len;

            if (Math.Sign(w) + Math.Sign(u) + Math.Sign(v) < 2.0f)
            {
                float nab = Clamp(Vector3.Dot(ba, pa) / ba.LengthSquared(), 0.0f, 1.0f);
                float nbc = Clamp(Vector3.Dot(cb, pb) / cb.LengthSquared(), 0.0f, 1.0f);
                float nca = Clamp(Vector3.Dot(ac, pc) / ac.LengthSquared(), 0.0f, 1.0f);

                Vector3 ppa = ba * nab + A;
                Vector3 ppb = cb * nbc + B;
                Vector3 ppc = ac * nca + C;

                float lna = (ppa - p).LengthSquared();
                float lnb = (ppb - p).LengthSquared();
                float lnc = (ppc - p).LengthSquared();

                if (lna <= lnc && lna <= lnb)
                {
                    weights = new Vector3(1.0f - nab, nab, 0);
                    pp = ppa;
                    len = lna;
                }
                else if (lnc <= lna && lnc <= lnb)
                {
                    weights = new Vector3(nca, 0, 1.0f - nca);
                    pp = ppc;
                    len = lnc;
                }
                else if (lnb <= lna && lnb <= lnc)
                {
                    weights = new Vector3(0, 1.0f - nbc, nbc);
                    pp = ppb;
                    len = lnb;
                }
                else // never happens
                {
                    pp = Vector3.Zero;
                    if (weights.X < 0 || weights.Y < 0 || weights.Z < 0 || weights.X > 1 || weights.Y > 1 || weights.Z > 1)
                        Console.WriteLine("Weights are invalid!");

                    return float.MaxValue;
                }
            }
            else
            {
                pp = A * weights.X + B * weights.Y + C * weights.Z;
                len = Vector3.DistanceSquared(p, pp);
            }
            if (weights.X < 0 || weights.Y < 0 || weights.Z < 0 || weights.X > 1 || weights.Y > 1 || weights.Z > 1)
                Console.WriteLine("Weights are invalid!");

            return (float)Math.Sqrt(len);
        }

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

            Vector3 pa = p - A;
            float u = Vector3.Dot(pa, h) / proj;

            if (u < 0.0 || u > 1.0)
            {
                return false;
            }

            Vector3 q = Vector3.Cross(pa, ba);
            float v = Vector3.Dot(dir, q) / proj;

            if (v < 0.0f || u + v > 1.0f)
            {
                return false;
            }

            float t = Vector3.Dot(ca, q) / proj;

            return t >= 0.0f;
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