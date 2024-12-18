﻿//#define EXTRA_VECTORS
//#define GENERATE_UDF

using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;
using RunServer.SdfTool;

namespace SDFTool
{
    /// <summary>
    /// 3D structure separating the space to cube based tiles
    /// </summary>
    internal class TriangleGrid
    {
        /// <summary>
        /// For any given model this represents the maximum tiles for each axis
        /// </summary>
        //private readonly float m_denominator;
        private readonly PreparedTriangle[][] m_grid;
        private readonly float m_gridStep;
        private readonly Vector3 m_sceneMin;

        private readonly Vector3i[] m_cellOffsets;
        private readonly float[] m_cellLengths;

        private readonly int m_gridX;
        private readonly int m_gridY;
        private readonly int m_gridZ;

        public readonly int TriangleCount;
        public readonly int TriangleInstances;
        public readonly int CellsUsed;

        public Vector3i GridSize
        {
            get { return new Vector3i(m_gridX, m_gridY, m_gridZ); }
        }

        public TriangleGrid(Vector3 sceneMin, Vector3 sceneMax, int gridX, int gridY, int gridZ, IList<PreparedTriangle> triangleList)
        {
            //m_denominator = denominator;

            m_sceneMin = sceneMin;
            m_gridStep = Math.Max(Math.Max((sceneMax.X - sceneMin.X) / gridX, (sceneMax.Y - sceneMin.Y) / gridY), (sceneMax.Z - sceneMin.Z) / gridZ);

            m_gridX = gridX;
            m_gridY = gridY;
            m_gridZ = gridZ;
            //m_gridx = (int)Math.Ceiling((sceneMax.X - sceneMin.X) / m_gridStep) + 1;
            //m_gridy = (int)Math.Ceiling((sceneMax.Y - sceneMin.Y) / m_gridStep) + 1;
            //m_gridz = (int)Math.Ceiling((sceneMax.Z - sceneMin.Z) / m_gridStep) + 1;

            List<PreparedTriangle>[] triangles = new List<PreparedTriangle>[m_gridX * m_gridY * m_gridZ];

            CellsUsed = 0;
            TriangleCount = triangleList.Count;

            foreach (PreparedTriangle triangle in triangleList)
            {
                //get tile coordinates for lower and upper triangle points
                Vector3 lb = (triangle.LowerBound - m_sceneMin) / m_gridStep;
                int fromx = Math.Max((int)Math.Floor(lb.X), 0);
                int fromy = Math.Max((int)Math.Floor(lb.Y), 0);
                int fromz = Math.Max((int)Math.Floor(lb.Z), 0);
                Vector3 ub = (triangle.UpperBound - m_sceneMin) / m_gridStep;
                int tox = Math.Min((int)Math.Ceiling(ub.X), m_gridX - 1);
                int toy = Math.Min((int)Math.Ceiling(ub.Y), m_gridY - 1);
                int toz = Math.Min((int)Math.Ceiling(ub.Z), m_gridZ - 1);

                int instances = 0;

                for (int z = fromz; z <= toz; z++)
                    for (int y = fromy; y <= toy; y++)
                        for (int x = fromx; x <= tox; x++)
                        {
                            // check if triangle plane intersects the box
                            Vector3 tileStart = new Vector3(x, y, z) * m_gridStep + m_sceneMin;
                            Vector3 tileEnd = tileStart + new Vector3(m_gridStep, m_gridStep, m_gridStep);

                            if (!triangle.PlaneIntersectsAABB(tileStart, tileEnd))
                                continue;

                            int index = x + y * m_gridX + z * m_gridX * m_gridY;

                            if (triangles[index] == null)
                            {
                                triangles[index] = new List<PreparedTriangle>();
                                CellsUsed++;
                            }

                            triangles[index].Add(triangle);
                            instances++;
                        }

                if (instances == 0) // too small triangle or AABB intersection fails
                {
                    Console.WriteLine("Triangle got no instances {0} - {1}", lb, ub);

                    /*
                    int index = fromx + fromy * m_gridx + fromz * m_gridx * m_gridy;

                    if (triangles[index] == null)
                    {
                        triangles[index] = new List<PreparedTriangle>();
                        CellsUsed++;
                    }

                    triangles[index].Add(triangle);
                    instances++;*/
                }

                TriangleInstances += instances;
            }

            m_grid = new PreparedTriangle[triangles.Length][];

            for (int i = 0; i < triangles.Length; i++)
                if (triangles[i] != null)
                {
                    triangles[i].Sort((x, y) => -x.Area.CompareTo(y.Area));
                    m_grid[i] = triangles[i].ToArray();
                }

            List<Vector4i> nibble = new List<Vector4i>();

            for (int z = -m_gridZ; z < m_gridZ; z++)
                for (int y = -m_gridY; y < m_gridY; y++)
                    for (int x = -m_gridX; x < m_gridX; x++)
                        nibble.Add(new Vector4i(x, y, z, x * x + y * y + z * z));

            nibble.Sort((x, y) => x.W.CompareTo(y.W));

            m_cellOffsets = new Vector3i[nibble.Count];
            m_cellLengths = new float[nibble.Count];

            for (int i = 0; i < m_cellOffsets.Length; i++)
            {
                m_cellOffsets[i] = new Vector3i(nibble[i].X, nibble[i].Y, nibble[i].Z);
                m_cellLengths[i] = (float)Math.Sqrt(nibble[i].W);
            }
        }

        public void FindTriangles(Vector3 point, out float distance, out Vector3 weights, out int triangleId)
        {
            distance = float.MaxValue;
            float minDistanceSqrd = float.MaxValue;
            triangleId = -1;
            weights = Vector3.Zero;
            Vector3 result = Vector3.Zero;
            int sign;

            Vector3 localPoint = (point - m_sceneMin) / m_gridStep;
            int pointx = (int)Math.Floor(localPoint.X);
            int pointy = (int)Math.Floor(localPoint.Y);
            int pointz = (int)Math.Floor(localPoint.Z);

            HashSet<int> triangles = new HashSet<int>();

            float localDist = float.MaxValue;
            Vector3 lb = Vector3.Zero;
            Vector3 ub = Vector3.Zero;

            bool earlyExit = false;

            // we check farther grid cells with each step
            // stopping when a point found with a distance smaller than all remaining cells
            for (int i = 0; i < m_cellOffsets.Length; i++)
            {
                if (m_cellLengths[i] > localDist)
                    break;

                // check is cells are outside of the grid

                int x = pointx + m_cellOffsets[i].X;
                int y = pointy + m_cellOffsets[i].Y;
                int z = pointz + m_cellOffsets[i].Z;
                if (x < 0 || x >= m_gridX || y < 0 || y >= m_gridY || z < 0 || z >= m_gridZ)
                    continue;

                int index = x + y * m_gridX + z * m_gridX * m_gridY;


                if (i >= 27 && localDist == float.MaxValue) // if this cell is solitary, meaning all neighbors are empty, we can just put in an average distance
                {
                    earlyExit = true;
                    distance = m_cellLengths[i] * m_gridStep;
                    result = m_sceneMin + new Vector3(pointx + 0.5f, pointy + 0.5f, pointz + 0.5f) * m_gridStep;
                    //if (weights.X < 0 || weights.Y < 0 || weights.Z < 0 || weights.X > 1 || weights.Y > 1 || weights.Z > 1)
                    //Console.WriteLine("Weights are invalid!");
                    //return false;
                    break;
                }
                // empty cells
                if (m_grid[index] == null)
                    continue;

                foreach (var triangle in m_grid[index])
                    if (triangles.Add(triangle.Id))
                    {
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
                        tempResult = PreparedTriangle.ClosestPointToTriangle(triangle, point, out tempWeights);
                        Vector3 dir = point - tempResult;
                        float dist = Vector3.Dot(dir, dir);
#endif
                        if (dist < minDistanceSqrd)
                        {
                            minDistanceSqrd = dist;
                            distance = (float)Math.Sqrt(dist);

                            lb = point - new Vector3(distance, distance, distance);
                            ub = point + new Vector3(distance, distance, distance);

                            weights = tempWeights;
                            //if (weights.X < 0 || weights.Y < 0 || weights.Z < 0 || weights.X > 1 || weights.Y > 1 || weights.Z > 1)
                            //Console.WriteLine("Weights are invalid!");
#if PSEUDO_SIGN
                            sign = tempSign;
#endif
                            result = tempResult;
                            triangleId = triangle.Id;
                        }
                    }

                if (distance != float.MaxValue)
                    localDist = distance / m_gridStep + 0.63f; // half of cubic root of two
                                                        // 1.73205080757f / 2; // half of square root of three

                if (earlyExit)
                    break;
            }


#if GENERATE_UDF
            sign = Math.Sign(distance);
#else
            sign = 0;

            //if (!earlyExit && distance != float.MaxValue)
            //distance = Vector3.Distance(point, result);

            Vector3[] dirs = new Vector3[] {
                !earlyExit && distance != float.MaxValue ? Vector3.Normalize(point - result) : new Vector3(float.NaN, float.NaN, float.NaN),
#if EXTRA_VECTORS
                Vector3.Normalize(result - point),
                Vector3.Normalize(point - new Vector3(0, 0, 0)),
                Vector3.Normalize(new Vector3(1, 1, 1) - point),
                Vector3.Normalize(new Vector3(0.5f, 0.5f, 0.5f) - point),
#endif
                new Vector3(0, 0, 1),
                new Vector3(1, 0, 0),
                new Vector3(0, 1, 0),
                new Vector3(0, 0, -1),
                new Vector3(-1, 0, 0),
                new Vector3(0, -1, 0)
            };

            foreach (Vector3 dir in dirs)
            {
                Vector3 direction = dir;

                if (float.IsNaN(direction.X) || float.IsNaN(direction.Y) || float.IsNaN(direction.Z))
                    direction = Vector3.Normalize(point - new Vector3(0.5f, 0.5f, 0.5f));

                sign += CountIntersections(point, direction) % 2 == 0 ? 1 : -1;

                if (Math.Abs(sign) >= dirs.Length / 2 + 1) // no point for further checks
                    break;
            }

            sign = sign >= 0 ? 1 : -1;
#endif
            //if (data != null)
            //{
            //Vector3 p = data.GetCurvativePoint(weights.X, weights.Y, weights.Z);
            //distance = Vector3.Distance(point, p);
            //}

            distance *= sign;
        }

        public int CountIntersections(Vector3 point, Vector3 dir)
        {
            int count = 0;

            Vector3 idir = new Vector3(1.0f / dir.X, 1.0f / dir.Y, 1.0f / dir.Z);
            Vector3 localPoint = (point - m_sceneMin) / m_gridStep;

            Vector3 gridMax = new Vector3(m_gridX, m_gridY, m_gridZ);
            float boundEnter;
            float boundExit;
            float lengthMax = gridMax.Length();

            if (!SegmentBoundIntersection(new Vector3(0, 0, 0), new Vector3(m_gridX, m_gridY, m_gridZ), localPoint, idir, lengthMax, out boundEnter, out boundExit))
                return 0;

            Vector3 localEndPoint = localPoint + dir * boundExit;
           
            HashSet<int> triangles = new HashSet<int>();

            ProcessRay(localPoint, localEndPoint, delegate (int index)
            {
                if (m_grid[index] == null)
                    return;

                foreach (var triangle in m_grid[index])
                    if (triangles.Add(triangle.Id))
                    {
                        if (!iRayBoundIntersection(triangle.LowerBound, triangle.UpperBound, point, idir))
                            continue;

                        if (triangle.IntersectsRay(point, dir))
                            count++;
                    }
                return;
            });

            return count;
        }

        private bool ProcessRay(Vector3 from, Vector3 to, Action<int> action)
        {
            int fromTileX = (int)Math.Floor(Math.Max(from.X, 0));
            int fromTileY = (int)Math.Floor(Math.Max(from.Y, 0));
            int fromTileZ = (int)Math.Floor(Math.Max(from.Z, 0));
            int toTileX = (int)Math.Floor(Math.Min(to.X, m_gridX - 1));
            int toTileY = (int)Math.Floor(Math.Min(to.Y, m_gridY - 1));
            int toTileZ = (int)Math.Floor(Math.Min(to.Z, m_gridZ - 1));
            sbyte stepX = (sbyte)Math.Sign(toTileX - fromTileX);
            sbyte stepY = (sbyte)Math.Sign(toTileY - fromTileY);
            sbyte stepZ = (sbyte)Math.Sign(toTileZ - fromTileZ);

            if (stepX == 0 && stepY == 0 && stepZ == 0)
            {
                if (fromTileX >= 0 && fromTileX < m_gridX && fromTileY >= 0 && fromTileY < m_gridY && fromTileZ >= 0 && fromTileZ < m_gridZ)
                {
                    int tindex = fromTileX + fromTileY * m_gridX + fromTileZ * m_gridX * m_gridY;
                    action(tindex);
                    return true;
                }

                return false;
            }

            //Vector3 dir = Vector3.Normalize(to - from);
            Vector3 dir = Vector3.Normalize(new Vector3(toTileX, toTileY, toTileZ) - new Vector3(fromTileX, fromTileY, fromTileZ));

            float deltaX = System.Math.Abs(dir.X);
            float deltaY = System.Math.Abs(dir.Y);
            float deltaZ = System.Math.Abs(dir.Z);
            float pointX = float.MaxValue;
            float pointY = float.MaxValue;
            float pointZ = float.MaxValue;


            if (deltaX > float.Epsilon)
            {
                deltaX = 1.0f / deltaX;
                pointX = (fromTileX - from.X) / dir.X;
                if (dir.X > 0)
                    pointX += deltaX;
            }

            if (deltaY > float.Epsilon)
            {
                deltaY = 1.0f / deltaY;
                pointY = (fromTileY - from.Y) / dir.Y;
                if (dir.Y > 0)
                    pointY += deltaY;
            }

            if (deltaZ > float.Epsilon)
            {
                deltaZ = 1.0f / deltaZ;
                pointZ = (fromTileZ - from.Z) / dir.Z;
                if (dir.Z > 0)
                    pointZ += deltaZ;
            }

            int index = fromTileX + fromTileY * m_gridX + fromTileZ * m_gridX * m_gridY;

            int maxCount = Math.Max(Math.Max(m_gridX, m_gridY), m_gridZ) + 2;

            while (--maxCount >= 0)
            {
                if (fromTileX >= 0 && fromTileX < m_gridX &&
                    fromTileY >= 0 && fromTileY < m_gridY &&
                    fromTileZ >= 0 && fromTileZ < m_gridZ)
                {
                    action(index);
                    //return true;
                }
                else
                    break; // something gone wrong

                if (pointX < pointY && pointX < pointZ)
                {
                    if (fromTileX == (int)to.X)
                        break;
                    pointX += deltaX;
                    fromTileX += stepX;
                    index += stepX;
                }
                else if (pointY < pointX && pointY < pointZ)
                {
                    if (fromTileY == (int)to.Y)
                        break;
                    pointY += deltaY;
                    fromTileY += stepY;
                    index += stepY * m_gridX;
                }
                else
                {
                    if (fromTileZ == (int)to.Z)
                        break;
                    pointZ += deltaZ;
                    fromTileZ += stepZ;
                    index += stepZ * m_gridX * m_gridY;
                }
            }

            return false;
        }

        /// <summary>
        /// Returns result of intersection check of segment and AABB
        /// </summary>
        /// <param name="lb">The lb.</param>
        /// <param name="ub">The ub.</param>
        /// <param name="point">The point.</param>
        /// <param name="idir">The idir.</param>
        /// <param name="length">The length.</param>
        /// <param name="resultEnter">The result enter.</param>
        /// <param name="resultExit">The result exit.</param>
        /// <returns></returns>
        private static bool SegmentBoundIntersection(Vector3 lb, Vector3 ub, Vector3 point, Vector3 idir, float length, out float resultEnter, out float resultExit)
        {
            resultEnter = 0f;
            resultExit = length;
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

            if (tmin > tzmax || tmax < tzmin)
                return false;

            if (tmin < tymin)
                tmin = tymin;
            if (tmax > tymax)
                tmax = tymax;

            if (tmin <= 0 && tmax >= length) // we are inside the box
                return true;

            if (tmax < 0 || tmin > length)
                return false;

            if (tmin > 0)
                resultEnter = tmin;
            if (tmax < length)
                resultExit = tmax;
            return true;
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

        public void Dispatch(float[] data, Vector3 lowerBound, float pixelsToScene, float sceneToPixels, int sx, int sy, int sz, Action<float> callback)
        {
            int count = 0;
            int maxCount = sx * sy * sz;

            Parallel.For(0, maxCount, i =>
            {
                int iz = i / (sx * sy);
                int iy = (i % (sx * sy)) / sx;
                int ix = (i % (sx * sy)) % sx;

                if (ix == 0 && iy == 0)
                {
                    int c = System.Threading.Interlocked.Increment(ref count);
                    callback(c / (float)sz);
                }

                Vector3 point = lowerBound + new Vector3(ix, iy, iz) * pixelsToScene;

                float sceneDistance;
                Vector3 triangleWeights;
                int triangleId;

                float pixelDistance;

                FindTriangles(point, out sceneDistance, out triangleWeights, out triangleId);

                //pixelDistance = Math.Sign(sceneDistance) * Math.Min(Math.Abs(sceneDistance * sceneToPixels), 1.0f);
                pixelDistance = sceneDistance * sceneToPixels;

                int index = i * 4;
                // distance in brick units [-1.0;1.0] where 1 corresponds to brick size
                data[index + 0] = pixelDistance;
                data[index + 1] = triangleWeights.X;
                data[index + 2] = triangleWeights.Y;
                data[index + 3] = triangleId;
            }
            );
        }

    }
}

