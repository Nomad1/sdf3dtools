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

        public Vector3i GridSize
        {
            get { return new Vector3i(m_gridx, m_gridy, m_gridz); }
        }

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
            Vector3 result = Vector3.Zero;
            int sign;

            Vector3 localPoint = (point - m_sceneMin) / m_gridStep;
            int pointx = (int)Math.Floor(localPoint.X);
            int pointy = (int)Math.Floor(localPoint.Y);
            int pointz = (int)Math.Floor(localPoint.Z);

            HashSet<PreparedTriangle> triangles = new HashSet<PreparedTriangle>();

            float localDist = float.MaxValue;
            Vector3 lb = Vector3.Zero;
            Vector3 ub = Vector3.Zero;

            // we check farther grid cells with each step
            // stopping when a point found with a distance smaller than all remaining cells
            for (int i = 0; i < m_nibble.Length; i++)
            {
                if (m_nibble[i].Length > localDist)
                    break;

#if !DONT_USE_EARLY_EXIT
                if (i >= 27 && localDist == float.MaxValue) // if this cell is solitary, meaning all neighbors are empty, we can just put in an average distance
                {
                    distance = m_nibble[i].Length * m_gridStep;
                    result = m_sceneMin + new Vector3(pointx + 0.5f, pointy + 0.5f, pointz + 0.5f) * m_gridStep;
                    //if (weights.X < 0 || weights.Y < 0 || weights.Z < 0 || weights.X > 1 || weights.Y > 1 || weights.Z > 1)
                    //Console.WriteLine("Weights are invalid!");
                    return false;
                }
#endif
                // check is cells are outside of the grid

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

                // empty cells
                if (m_triangles[index] == null)
                    continue;

                foreach (var triangle in m_triangles[index])
                    if (!triangles.Contains(triangle))
                    {
                        triangles.Add(triangle);

                        if (distance != float.MaxValue)
                        {
                            if (!triangle.IntersectsAABB(lb, ub))
                                continue;

                            if (!triangle.IntersectsSphere(point, distance))
                                continue;
                        }

                        Vector3 tempWeights;
                        Vector3 tempResult;
#if PSEUDO_SIGN
                        int tempSign;
                        float dist = triangle.DistanceToPoint(point, out tempWeights, out tempResult, out tempSign);
#else
                        float dist = triangle.DistanceToPoint(point, out tempWeights, out tempResult);
#endif
                        if (dist < distance && dist != float.MaxValue)
                        {
                            distance = dist;

                            lb = point - new Vector3(distance, distance, distance);
                            ub = point + new Vector3(distance, distance, distance);

                            weights = tempWeights;
                            //if (weights.X < 0 || weights.Y < 0 || weights.Z < 0 || weights.X > 1 || weights.Y > 1 || weights.Z > 1)
                                //Console.WriteLine("Weights are invalid!");
#if PSEUDO_SIGN
                            sign = tempSign;
#endif
                            result = tempResult;
                            data = triangle.Data;
                        }
                    }

                if (distance != float.MaxValue)
                    localDist = distance / m_gridStep + 1.73205080757f / 2;
            }

            sign = CountIntersections(point, Vector3.Normalize(point - result)) % 2 == 0 ? 1 : -1;

            distance *= sign;

            //if (weights.X < 0 || weights.Y < 0 || weights.Z < 0 || weights.X > 1 || weights.Y > 1 || weights.Z > 1)
                //Console.WriteLine("Weights are invalid!");

            return true;
        }

        public int CountIntersections(Vector3 point, Vector3 dir)
        {
            int count = 0;

            if (float.IsNaN(dir.X) || float.IsNaN(dir.Y) || float.IsNaN(dir.Z))
                dir = new Vector3(0, 0, 1);
            Vector3 idir = new Vector3(1.0f / dir.X, 1.0f / dir.Y, 1.0f / dir.Z);
            Vector3 localPoint = (point - m_sceneMin) / m_gridStep;
            Vector3 localEndPoint = localPoint + new Vector3(Math.Sign(dir.X) * m_gridx, Math.Sign(dir.Y) * m_gridy, Math.Sign(dir.Z) * m_gridz);

            int fromx = Math.Max((int)Math.Floor(Math.Min(localPoint.X, localEndPoint.X)), 0);
            int fromy = Math.Max((int)Math.Floor(Math.Min(localPoint.Y, localEndPoint.Y)), 0);
            int fromz = Math.Max((int)Math.Floor(Math.Min(localPoint.Z, localEndPoint.Z)), 0);
            int tox = Math.Min((int)Math.Ceiling(Math.Max(localPoint.X, localEndPoint.X)), m_gridx);
            int toy = Math.Min((int)Math.Ceiling(Math.Max(localPoint.Y, localEndPoint.Y)), m_gridy);
            int toz = Math.Min((int)Math.Ceiling(Math.Max(localPoint.Z, localEndPoint.Z)), m_gridz);

            // TODO: 3dda for selecting needed grids, not a bruteforce

            HashSet<PreparedTriangle> triangles = new HashSet<PreparedTriangle>();

            for (int z = fromz; z < toz; z++)
                for (int y = fromy; y < toy; y++)
                    for (int x = fromx; x < tox; x++)
                    {
                        int index = x + y * m_gridx + z * m_gridx * m_gridy;

                        // empty cells
                        if (m_triangles[index] == null)
                            continue;

                        if (!iRayBoundIntersection(new Vector3(x, y, z), new Vector3(x + 1, y + 1, z + 1), localPoint, idir))
                            continue;

                        foreach (var triangle in m_triangles[index])
                            if (!triangles.Contains(triangle))
                            {
                                triangles.Add(triangle);

                                if (triangle.IntersectsRay(point, dir))
                                    count++;
                            }
                    }

            return count;
        }

        /// <summary>
        /// Determines Ray-Box intersection. (c) Amy Williams, Steve Barrus, R. Keith Morley, and Peter Shirley
        /// </summary>
        /// <param name="lb">The lower box boundary</param>
        /// <param name="ub">The upper box boundary</param>
        /// <param name="point">Origin point</param>
        /// <param name="dir">Inverted direction vector</param>
        /// <returns></returns>
        public static bool iRayBoundIntersection(Vector3 lb, Vector3 ub, Vector3 point, Vector3 idir)
        {
            float tmin, tmax, tymin, tymax;

            if (idir.X >= 0)
            {
                tmin = (lb.X - point.X) * idir.X;
                tmax = (ub.X - point.X) * idir.X;
            }
            else
            {
                tmin = (ub.X - point.X) * idir.X;
                tmax = (lb.X - point.X) * idir.X;
            }

            if (idir.Y >= 0)
            {
                tymin = (lb.Y - point.Y) * idir.Y;
                tymax = (ub.Y - point.Y) * idir.Y;
            }
            else
            {
                tymin = (ub.Y - point.Y) * idir.Y;
                tymax = (lb.Y - point.Y) * idir.Y;
            }

            if (tmin > tymax || tmax < tymin)
                return false;

            if (tmin < tymin)
                tmin = tymin;
            if (tmax > tymax)
                tmax = tymax;

            float tzmin, tzmax;

            if (idir.Z >= 0)
            {
                tzmin = (lb.Z - point.Z) * idir.Z;
                tzmax = (ub.Z - point.Z) * idir.Z;
            }
            else
            {
                tzmin = (ub.Z - point.Z) * idir.Z;
                tzmax = (lb.Z - point.Z) * idir.Z;
            }

            return (tmin <= tzmax) && (tmax >= tzmin);
        }

    }
}

