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
        private readonly List<PreparedTriangle>[] m_triangles;
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

            m_triangles = new List<PreparedTriangle>[m_gridx * m_gridy * m_gridz];

            CellsUsed = 0;
            TriangleCount = triangleList.Count;

            foreach (PreparedTriangle triangle in triangleList)
            {
                //get tile coordinates for lower and upper triangle points
                Vector3 lb = (triangle.LowerBoundary - m_sceneMin) / m_gridStep;
                int fromx = (int)Math.Floor(lb.X);
                int fromy = (int)Math.Floor(lb.Y);
                int fromz = (int)Math.Floor(lb.Z);
                Vector3 ub = (triangle.UpperBoundary - m_sceneMin) / m_gridStep;
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

                            if (!triangle.IntersectsAABB(tileStart, tileEnd))
                                continue;

                            int index = x + y * m_gridx + z * m_gridx * m_gridy;


                            if (m_triangles[index] == null)
                            {
                                m_triangles[index] = new List<PreparedTriangle>();
                                CellsUsed++;
                            }

                            m_triangles[index].Add(triangle);
                            instances++;
                        }

                if (instances == 0)
                    Console.WriteLine("Triangle got no instances {0} - {1}", lb, ub);
                else
                    TriangleInstances += instances;
            }

            List<NibbleCell> nibble = new List<NibbleCell>();

            for (int z = -m_gridz; z < m_gridz; z++)
                for (int y = -m_gridy; y < m_gridy; y++)
                    for (int x = -m_gridx; x < m_gridx; x++)
                        nibble.Add(new NibbleCell(x, y, z));

            nibble.Sort((x, y) => x.Length.CompareTo(y.Length));

            m_nibble = nibble.ToArray();
        }

        public bool FindTriangles(Vector3 point, out float distance)
        {
            distance = float.MaxValue;

            Vector3 localPoint = (point - m_sceneMin) / m_gridStep;
            int pointx = (int)Math.Floor(localPoint.X);
            int pointy = (int)Math.Floor(localPoint.Y);
            int pointz = (int)Math.Floor(localPoint.Z);

            HashSet<PreparedTriangle> triangles = new HashSet<PreparedTriangle>();

            float localDist = float.MaxValue;

            for (int i = 0; i < m_nibble.Length; i++)
            {
                if (localDist != float.MaxValue)
                {
                    /*Vector3 center = new Vector3(x + 0.5f, y + 0.5f, z + 0.5f);

                    if (Vector3.Distance(center, localPoint) > localDist)
                        break;*/

                    if (m_nibble[i].Length > localDist)
                        break;
                }

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
                            float distanceToSphere = (point - triangle.Center).LengthSquared() - triangle.RadiusSqrd;
                            if (distanceToSphere > distance)
                                continue;
                        }
                        float dist = triangle.DistanceSqrd(point);
                        if (dist < distance)
                        {
                            distance = dist; // TODO: save triangle index to calculate color/texcoords if it wins
                            localDist = (float)Math.Sqrt(dist) / m_gridStep + 1.73205080757f/ 2;// / 2; // cube diagonal
                        }
                    }
            }

            distance = (float)Math.Sqrt(distance);
            return distance != float.MaxValue;// triangles.Count > 0;
        }
    }
}

