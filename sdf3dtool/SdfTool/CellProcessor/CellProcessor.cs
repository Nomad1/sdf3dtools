using System;
using System.Collections.Generic;

#if UNITY_EDITOR
using Vector2 = RunServer.SdfTool.Vector2e;
using Vector3 = RunServer.SdfTool.Vector3e;
using Vector4 = RunServer.SdfTool.Vector4e;
#else
using System.Numerics;
#endif

namespace RunServer.SdfTool
{
    public static class CellProcessor
    {
        public struct BrickCellData
        {
            public readonly Vector3 Position;
            public readonly Vector3i Coord;
            public readonly Vector3 Size;
            public readonly int BrickId;
            public readonly ValueTuple<int, float>[][] BoneWeights;
            public readonly float[][] VertexDistances;

            public BrickCellData(int id, Vector3 position, Vector3i coord, Vector3 size, ValueTuple<int, float>[][] weights, float[][] distances)
            {
                Position = position;
                Coord = coord;
                BrickId = id;
                Size = size;
                BoneWeights = weights;
                VertexDistances = distances;
            }
        }

        // for internal processing
        internal struct BrickData
        {
            public readonly int Id;
            public readonly Vector3i Position;
            public readonly int CellSize;
            public readonly PixelData[] Data;
            public readonly int[] Children;

            public BrickData(int id, Vector3i position, int cellSize, PixelData[] data, int[] children = null)
            {
                Id = id;
                CellSize = cellSize;
                Data = data;
                Position = position;
                Children = children;
            }
        }

        public struct LodData
        {
            public readonly Vector3i Size;
            public readonly float[] Distances;
            public readonly Vector2[] UV;
            public readonly BrickCellData[] Bricks;
            public readonly int[] Children;
            public readonly Vector3i ChildrenSize;

            public LodData(Vector3i size, float[] distances, Vector2[] uv, BrickCellData[] bricks, int[] children, Vector3i childrenSize)
            {
                Size = size;
                Distances = distances;
                UV = uv;
                Bricks = bricks;
                Children = children;
                ChildrenSize = childrenSize;
            }
        }

        internal static bool BricksAreSimilar(PixelData[] cellData, PixelData[] enlargedCellData, double minPsnr, double multiplier = 1.0f)
        {
            // Check array sizes match
            if (cellData.Length != enlargedCellData.Length)
                return false;

            double mse = 0;
            for (int i = 0; i < cellData.Length; i++)
            {
                double diff = (1.0 / cellData[i].DistanceUV.X - 1.0 / enlargedCellData[i].DistanceUV.X) * multiplier;
                mse += diff * diff;
            }

            mse /= cellData.Length;

            // Handle perfect match case
            if (Math.Abs(mse) < double.Epsilon)
                return true;

            double psnr = 10.0 * Math.Log10(1.0 / mse);
            return psnr > minPsnr;
        }

       
        internal static PixelData[] CheckBrick(PixelData[] data, Vector3i dataStart, Vector3i dataSize, int cellSize)
        {
            int sign = 0;
            int count = 0;

            // do a first run to know if we are processing this brick at all
            for (int z = 0; z <= cellSize; z++)
                for (int y = 0; y <= cellSize; y++)
                    for (int x = 0; x <= cellSize; x++)
                    {
                        PixelData pixel = Utils.GetArrayData(data, dataSize, dataStart + new Vector3i(x, y, z));

                        float distance = pixel.DistanceUV.X;
                        sign += Math.Sign(distance);
                        count++;
                    }

            if (sign == count) // all neg or all pos is ignored
                return null;

            PixelData[] brick = new PixelData[(cellSize + 1) * (cellSize + 1) * (cellSize + 1)];

            for (int z = 0; z <= cellSize; z++)
                for (int y = 0; y <= cellSize; y++)
                    for (int x = 0; x <= cellSize; x++)
                    {
                        PixelData pixel = Utils.GetArrayData(data, dataSize, dataStart + new Vector3i(x, y, z));
                        Utils.SetArrayData(brick, pixel, new Vector3i(cellSize + 1, cellSize + 1, cellSize + 1), new Vector3i(x, y, z));
                    }

            return brick;

        }

        /// <summary>
        /// We are taking array @data of size @dataSize and looking for cells of size @cellSize^3
        /// If @targetArray is provided, it should be of size @cellsx*@cellsy*@cellsz and we put new cell index there
        /// </summary>
        /// <param name="data"></param>
        /// <param name="dataSize"></param>
        /// <param name="cellSize"></param>
        /// <param name="cells"></param>
        /// <param name="shift"></param>
        /// <param name="targetArray"></param>
        /// <returns></returns>
        internal static int FindFixedBricks(
            PixelData[] data, Vector3i dataSize,
            int cellSize,
            IList<BrickData> cells, Vector3i shift,
            int[] targetArray, Vector3i targetSize, int childCellSize)
        {
            int cellsx = dataSize.X / cellSize;
            int cellsy = dataSize.Y / cellSize;
            int cellsz = dataSize.Z / cellSize;

            int childCell = childCellSize;

            int cellCount = 0;

            for (int iz = 0; iz < cellsz; iz++)
            {
                for (int iy = 0; iy < cellsy; iy++)
                {
                    for (int ix = 0; ix < cellsx; ix++)
                    {
                        Vector3i coord = new Vector3i(ix, iy, iz);
                        PixelData [] brick = CheckBrick(data, coord * cellSize, dataSize, cellSize);

                        if (brick != null)
                        {
                            // add result to `cells`

                            int index = cells.Count;

                            if (targetArray != null)
                                Utils.SetArrayData(targetArray, index, targetSize, coord);

                            cellCount++;
                            cells.Add(new BrickData(cellCount, coord * cellSize + shift, cellSize, brick, new int[childCell * childCell * childCell]));
                        }
                    }
                }
            }
            return cellCount;
        }
        
    }
}

