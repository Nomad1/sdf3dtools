using System;
using System.Collections.Generic;
using System.Numerics;

namespace SDFTool
{
    /// <summary>
    /// 3D structure separating the space to cube based tiles
    /// </summary>
    internal class TriangleMap
    {
        private struct NibbleCell
        {
            public readonly float Length;
            public readonly int X, Y, Z;

            public NibbleCell(int x, int y, int z)
            {
                X = x;
                Y = y;
                Z = z;
                Length = (float)Math.Sqrt(x * x + y * y + z * z);
            }
        }

        /// <summary>
        /// For any given model this represents the maximum tiles for each axis
        /// </summary>
        private readonly float m_denominator;
        private readonly PreparedTriangle[][] m_triangles;
        private readonly float m_gridStep;
        private readonly Vector3 m_sceneMin;

        private readonly NibbleCell[] m_nibble;

        private readonly int m_gridx;
        private readonly int m_gridy;
        private readonly int m_gridz;

        public readonly int TriangleCount;
        public readonly int TriangleInstances;
        public readonly int CellsUsed;

        public TriangleMap(Vector3 sceneMin, Vector3 sceneMax, float denominator, IList<PreparedTriangle> triangleList)
        {
            m_denominator = denominator;

            m_sceneMin = sceneMin;
            m_gridStep = Math.Max(Math.Max(sceneMax.X - sceneMin.X, sceneMax.Y - sceneMin.Y), sceneMax.Z - sceneMin.Z) / denominator;

            m_gridx = (int)Math.Ceiling((sceneMax.X - sceneMin.X) / m_gridStep);
            m_gridy = (int)Math.Ceiling((sceneMax.Y - sceneMin.Y) / m_gridStep);
            m_gridz = (int)Math.Ceiling((sceneMax.Z - sceneMin.Z) / m_gridStep);

            List<PreparedTriangle> [] triangles = new List<PreparedTriangle>[m_gridx * m_gridy * m_gridz];

            CellsUsed = 0;
            TriangleCount = triangleList.Count;

            foreach (PreparedTriangle triangle in triangleList)
            {
                //get tile coordinates for lower and upper triangle points
                Vector3 lb = (triangle.LowerBound - m_sceneMin) / m_gridStep;
                int fromx = (int)Math.Floor(lb.X);
                int fromy = (int)Math.Floor(lb.Y);
                int fromz = (int)Math.Floor(lb.Z);
                Vector3 ub = (triangle.UpperBound - m_sceneMin) / m_gridStep;
                int tox = (int)Math.Ceiling(ub.X);
                int toy = (int)Math.Ceiling(ub.Y);
                int toz = (int)Math.Ceiling(ub.Z);

                int instances = 0;

                for (int z = fromz; z < toz; z++)
                    for (int y = fromy; y < toy; y++)
                        for (int x = fromx; x < tox; x++)
                        {
                            // check if triangle plane intersects the box
                            Vector3 tileStart = new Vector3(x, y, z) * m_gridStep + m_sceneMin;
                            Vector3 tileEnd = tileStart + new Vector3(m_gridStep, m_gridStep, m_gridStep);

                            if (!triangle.PlaneIntersectsAABB(tileStart, tileEnd))
                                continue;

                            int index = x + y * m_gridx + z * m_gridx * m_gridy;


                            if (triangles[index] == null)
                            {
                                triangles[index] = new List<PreparedTriangle>();
                                CellsUsed++;
                            }

                            triangles[index].Add(triangle);
                            instances++;
                        }

                if (instances == 0)
                    Console.WriteLine("Triangle got no instances {0} - {1}", lb, ub);
                else
                    TriangleInstances += instances;
            }

            m_triangles = new PreparedTriangle[triangles.Length][];

            for (int i = 0; i < triangles.Length; i++)
                if (triangles[i] != null)
                {
                    triangles[i].Sort((x, y) => -x.Area.CompareTo(y.Area));
                    m_triangles[i] = triangles[i].ToArray();
                }

            List<NibbleCell> nibble = new List<NibbleCell>();

            for (int z = -m_gridz; z < m_gridz; z++)
                for (int y = -m_gridy; y < m_gridy; y++)
                    for (int x = -m_gridx; x < m_gridx; x++)
                        nibble.Add(new NibbleCell(x, y, z));

            nibble.Sort((x, y) => x.Length.CompareTo(y.Length));

            m_nibble = nibble.ToArray();
        }

        public bool FindTriangles(Vector3 point, out float distance, out Vector3 weights, out object data)
        {
            distance = float.MaxValue;
            data = null;
            weights = Vector3.Zero;

            float distanceSqrd = float.MaxValue;

            Vector3 localPoint = (point - m_sceneMin) / m_gridStep;
            int pointx = (int)Math.Floor(localPoint.X);
            int pointy = (int)Math.Floor(localPoint.Y);
            int pointz = (int)Math.Floor(localPoint.Z);

            HashSet<PreparedTriangle> triangles = new HashSet<PreparedTriangle>();

            float localDist = float.MaxValue;
            Vector3 lb = Vector3.Zero;
            Vector3 ub = Vector3.Zero;
            Vector3 w;

            for (int i = 0; i < m_nibble.Length; i++)
            {
                if (m_nibble[i].Length > localDist)
                    break;

                int x = pointx + m_nibble[i].X;
                if (x < 0 || x >= m_gridx)
                    continue;
                int y = pointy + m_nibble[i].Y;
                if (y < 0 || y >= m_gridy)
                    continue;
                int z = pointz + m_nibble[i].Z;
                if (z < 0 || z >= m_gridz)
                    continue;

                int index = x + y * m_gridx + z * m_gridx * m_gridy;

                if (m_triangles[index] == null)
                    continue;

                foreach (var triangle in m_triangles[index])
                    if (!triangles.Contains(triangle))
                    {
                        if (distance != float.MaxValue)
                        {
                            if (!triangle.IntersectsAABB(lb, ub))
                                continue;

                            if (!triangle.IntersectsSphere(point, distance))
                                continue;
                        }

                        float dist = triangle.DistanceSqrd(point, out w);
                        if (dist < distanceSqrd)
                        {
                            distanceSqrd = dist;
                            distance = (float)Math.Sqrt(dist); // TODO: save triangle index to calculate color/texcoords if it wins

                            lb = point - new Vector3(distance, distance, distance);
                            ub = point + new Vector3(distance, distance, distance);

                            w = weights;
                            data = triangle.Data;
                        }
                    }

                if (distance != float.MaxValue)
                    localDist = distance / m_gridStep + 1.73205080757f / 2;
            }

            return distance != float.MaxValue;
        }
    }
}

