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

    internal class PreparedTriangle
    {
        // Vertices
        public readonly Vector3 A, B, C;

        // Center point and radius
        public readonly Vector3 Center;
        public readonly float Radius;

        // Normal
        private readonly Vector3 Normal;
        //private readonly Vector3 NormNormal;
        // Distance from A to (0,0,0)
        private readonly float ZeroDistance;

        //private readonly Vector3 BANorm;
        //private readonly Vector3 CBNorm;
        //private readonly Vector3 ACNorm;

        // Triangle area
        public readonly float Area;

        private readonly float NormalLength;

        public readonly object Data;
        public readonly Vector3i[] Vertices;
        public readonly VectorPair[] Edges;
        public readonly Vector3[] PseudoNormals;

        public readonly Vector3 LowerBound;
        public readonly Vector3 UpperBound;

        public PreparedTriangle(Vector3 a, Vector3 b, Vector3 c, object parent, Vector3i[] vertices, VectorPair[] edges)
        {
            A = a;
            B = b;
            C = c;
            Data = parent;
            Vertices = vertices;
            Edges = edges;
            PseudoNormals = new Vector3[7];

            Normal = Vector3.Cross(b - a, c - a);
            NormalLength = Normal.Length();
            Normal /= NormalLength;
            PseudoNormals[0] = Normal;
            PseudoNormals[1] = Normal;
            PseudoNormals[2] = Normal;
            PseudoNormals[3] = Normal;
            PseudoNormals[4] = (float)Math.Acos(Vector3.Dot(Vector3.Normalize(b - a), Vector3.Normalize(c - a))) * Normal;
            PseudoNormals[5] = (float)Math.Acos(Vector3.Dot(Vector3.Normalize(a - b), Vector3.Normalize(c - b))) * Normal;
            PseudoNormals[6] = (float)Math.Acos(Vector3.Dot(Vector3.Normalize(b - c), Vector3.Normalize(a - c))) * Normal;
            //float normalLength = (float)Math.Sqrt(NormalLengthSquared);
            //Area = normalLength * 0.5f;
            //NormNormal = Normal / normalLength;

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

        public void UpdateNeighbors(IDictionary<Vector3i, Vector3> vertexNormals, IDictionary<VectorPair, Vector3> edgeNormals)
        {
            PseudoNormals[0] = edgeNormals[Edges[0]];
            PseudoNormals[1] = edgeNormals[Edges[1]];
            PseudoNormals[2] = edgeNormals[Edges[2]];
            PseudoNormals[4] = vertexNormals[Vertices[0]];
            PseudoNormals[5] = vertexNormals[Vertices[1]];
            PseudoNormals[6] = vertexNormals[Vertices[2]];
        }

        private static float Clamp(float value, float min, float max)
        {
            return Math.Max(Math.Min(value, max), min);
        }

        public float Distance(Vector3 p, out Vector3 weights, out int sign)
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

            int edge;
            Vector3 pp;
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
                    edge = nab <= 0 ? 4 : nab >= 1 ? 5 : 0;
                    weights = new Vector3(1.0f - nab, nab, 0);
                    pp = ppa;
                    len = lna;
                }
                else if ((u > 0 && lna > lnc) || (w > 0 && lnc < lnb))
                {
                    edge = nca <= 0 ? 6 : nca >= 1 ? 4 : 2;
                    weights = new Vector3(nca, 0, 1.0f - nca);
                    pp = ppc;
                    len = lnc;
                }
                else if ((v > 0 && lna > lnb) || (w > 0 && lnc > lnb))
                {
                    edge = nbc <= 0 ? 5 : nbc >= 1 ? 6 : 1;
                    weights = new Vector3(0, 1.0f - nbc, nbc);
                    pp = ppb;
                    len = lnb;
                }
                else // never happens
                {
                    sign = 0;
                    return float.MaxValue;
                }
            }
            else
            {
                pp = A * weights.X + B * weights.Y + C * weights.Z;
                edge = 3;
                len = Vector3.DistanceSquared(p, pp);
            }

            float dot = Vector3.Dot(PseudoNormals[edge], p - pp);

            sign = Math.Sign(dot);

            return (float)Math.Sqrt(len);
            /*

            weights = new Vector3(u, v, 1.0f - u - v);

            sign = 1;// Math.Sign(dot);

            // inside/outside test
            return (Math.Sign(w) +
                    Math.Sign(u) +
                    Math.Sign(v) < 2.0f)
                    ?
                    // 3 edges
                    Math.Min(Math.Min(
                    (ba * Clamp(Vector3.Dot(BANorm, pa), 0.0f, 1.0f) - pa).LengthSquared(),
                    (cb * Clamp(Vector3.Dot(CBNorm, pb), 0.0f, 1.0f) - pb).LengthSquared()),
                    (ac * Clamp(Vector3.Dot(ACNorm, pc), 0.0f, 1.0f) - pc).LengthSquared())
                    :
                    // 1 face
                    Vector3.Dot(Normal, pa) * Vector3.Dot(Normal, pa) / NormalLengthSquared;*/
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