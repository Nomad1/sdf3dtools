using System;
using System.Collections.Generic;
using System.Drawing;

#if UNITY_EDITOR
using Vector2 = RunServer.SdfTool.Vector2e;
using Vector3 = RunServer.SdfTool.Vector3e;
using Vector4 = RunServer.SdfTool.Vector4e;
#else
using System.Numerics;
using System.Runtime.CompilerServices;
#endif

namespace RunServer.SdfTool
{
    public struct PixelData
    {
        public readonly Vector3 DistanceUV;
        public readonly Vector4i Bones;
        public readonly Vector4 BoneWeights;

        public PixelData(float distance, float u, float v, Vector4i bones, Vector4 weights)
        {
            DistanceUV = new Vector3(distance, u, v);
            Bones = bones;
            BoneWeights = weights;
        }
    }

    public struct DistanceData
    {
        [Flags]
        private enum PointFlags : uint
        {
            None = 0,
            Uvs = 1,
            Bones = 2,
            Lods = 4
        }

        private static readonly uint Signature = (('P' | ('T' << 8) | ('0' << 16) | ('3' << 24)));

        public readonly int CellSize;
        public readonly Vector3i Size;
        public readonly Vector3 LowerBound;
        public readonly Vector3 UpperBound;
        public readonly PixelData[][] LodData;

        public PixelData[] Data
        {
            get { return LodData[0]; }
        }
        public DistanceData(int cellSize, PixelData[] data, Vector3i size, Vector3 lowerBound, Vector3 upperBound)
        {
            CellSize = cellSize;
            LodData = new PixelData[][] { data };
            Size = size;
            LowerBound = lowerBound;
            UpperBound = upperBound;
        }

        public DistanceData(System.IO.Stream stream)
        {
            using (System.IO.BinaryReader reader = new System.IO.BinaryReader(stream))
            {
                uint signature = reader.ReadUInt32();

                if (signature != Signature)
                    throw new Exception("Invalid points v2 file signature: " + signature);

                int sx = reader.ReadInt32();
                int sy = reader.ReadInt32();
                int sz = reader.ReadInt32();
                Size = new Vector3i(sx, sy, sz);
                CellSize = reader.ReadInt32();
                float lx = reader.ReadSingle();
                float ly = reader.ReadSingle();
                float lz = reader.ReadSingle();
                float ux = reader.ReadSingle();
                float uy = reader.ReadSingle();
                float uz = reader.ReadSingle();
                LowerBound = new Vector3(lx, ly, lz);
                UpperBound = new Vector3(ux, uy, uz);

                PointFlags flags = (PointFlags)reader.ReadUInt32();

                int nlods = reader.ReadInt32();

                LodData = new PixelData[nlods][];

                for (int l = 0; l < nlods; l++)
                {
                    int length = reader.ReadInt32();

                    LodData[l] = new PixelData[length];

                    for (int i = 0; i < length; i++)
                    {
                        float dx = reader.ReadSingle();

                        float dy = 0.0f;
                        float dz = 0.0f;
                        if ((flags & PointFlags.Uvs) != 0)
                        {
                            dy = reader.ReadSingle();
                            dz = reader.ReadSingle();
                        }

                        int bx = 0;
                        int by = 0;
                        int bz = 0;
                        int bw = 0;
                        float wx = 1.0f;
                        float wy = 0.0f;
                        float wz = 0.0f;
                        float ww = 0.0f;

                        if ((flags & PointFlags.Bones) != 0)
                        {
                            bx = reader.ReadInt32();
                            by = reader.ReadInt32();
                            bz = reader.ReadInt32();
                            bw = reader.ReadInt32();
                            wx = reader.ReadSingle();
                            wy = reader.ReadSingle();
                            wz = reader.ReadSingle();
                            ww = reader.ReadSingle();
                        }

                        LodData[l][i] = new PixelData(dx, dy, dz, new Vector4i(bx, by, bz, bw), new Vector4(wx, wy, wz, ww));
                    }
                }
            }
        }

        public void SaveTo(System.IO.Stream stream)
        {
            using (System.IO.BinaryWriter writer = new System.IO.BinaryWriter(stream))
            {
                writer.Write(Size.X);
                writer.Write(Size.Y);
                writer.Write(Size.Z);
                writer.Write(CellSize);
                writer.Write(LowerBound.X);
                writer.Write(LowerBound.Y);
                writer.Write(LowerBound.Z);
                writer.Write(UpperBound.X);
                writer.Write(UpperBound.Y);
                writer.Write(UpperBound.Z);
                writer.Write(Data.Length);

                for (int i = 0; i < Data.Length; i++)
                {
                    writer.Write(Data[i].DistanceUV.X);
                    writer.Write(Data[i].DistanceUV.Y);
                    writer.Write(Data[i].DistanceUV.Z);
                    writer.Write(Data[i].Bones.X);
                    writer.Write(Data[i].Bones.Y);
                    writer.Write(Data[i].Bones.Z);
                    writer.Write(Data[i].Bones.W);
                    writer.Write(Data[i].BoneWeights.X);
                    writer.Write(Data[i].BoneWeights.Y);
                    writer.Write(Data[i].BoneWeights.Z);
                    writer.Write(Data[i].BoneWeights.W);
                }
            }
        }
    }

    public static class CellProcessor
    {
        // to return data
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
        private struct BrickData
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


        /// <summary>
        /// Splits SDF pixel data to uniform bricks
        /// </summary>
        /// <param name="data">input data</param>
        /// <param name="dataSize">data size</param>
        /// <param name="topLodCellSize"></param>
        /// <param name="lowerBound"></param>
        /// <param name="upperBound"></param>
        /// <param name="nlods"></param>
        /// <param name="distanceLods"></param>
        /// <param name="topLoduv"></param>
        /// <param name="zeroLodData"></param>
        /// <param name="boxArray"></param>
        /// <returns>cell count</returns>
        public static int ProcessCells(
            DistanceData data,
            int nlods,

            out Vector3i topLodTextureSize,
            out float[][] distanceLods,
            out Vector2[] topLoduv,
            out Vector4[] zeroLodData,
            IList<BrickCellData> boxes)
        {
            Dictionary<Vector3i, ValueTuple<int, float>[]> weightCache = new Dictionary<Vector3i, ValueTuple<int, float>[]>();

            int usedCells = 0;

            int cellsx = data.Size.X / data.CellSize;
            int cellsy = data.Size.Y / data.CellSize;
            int cellsz = data.Size.Z / data.CellSize;

            int paddedTopLodCellSize = data.CellSize + 1;

            int totalCells = cellsx * cellsy * cellsz;

            Vector3i blockSize = new Vector3i(paddedTopLodCellSize, paddedTopLodCellSize, paddedTopLodCellSize);

            float[] cells = new float[totalCells];
            usedCells = CheckCells(data.Data, data.Size, data.CellSize, cells);

            DebugLog("Used cells: {0}, cellsx: {1}, cellsy: {2}, cellsz: {3}, blockSize: {4}", usedCells, cellsx, cellsy, cellsz, paddedTopLodCellSize);

            int packx;
            int packy;
            int packz;
            FindBestDividers(usedCells + 1, out packx, out packy, out packz, 256);

            topLodTextureSize = new Vector3i(packx * paddedTopLodCellSize, packy * paddedTopLodCellSize, packz * paddedTopLodCellSize);

            distanceLods = new float[nlods][];

            {
                int size = paddedTopLodCellSize;
                for (int i = 0; i < nlods; i++)
                {
                    distanceLods[i] = new float[packx * size * packy * size * packz * size];
                    size /= 2;
                }
            }

            topLoduv = new Vector2[packx * paddedTopLodCellSize * packy * paddedTopLodCellSize * packz * paddedTopLodCellSize];

            zeroLodData = new Vector4[cellsx * cellsy * cellsz];

            Vector3 boxStep = (data.UpperBound - data.LowerBound) / new Vector3(cellsx, cellsy, cellsz);

            int brickId = 0;

            int pxy = packx * packy;
            for (int iz = 0; iz < cellsz; iz++)
                for (int iy = 0; iy < cellsy; iy++)
                    for (int ix = 0; ix < cellsx; ix++)
                    {
                        int index = ix + iy * cellsx + iz * cellsx * cellsy;
                        Vector3i vindex = new Vector3i(ix, iy, iz);


                        int atlasX = 0;
                        int atlasY = 0;
                        int atlasZ = 0;

                        float distancePercentage = cells[index];

                        if (distancePercentage < 0)
                            distancePercentage = -distancePercentage;
                        else
                        {
                            // Add used cell to 3D atlas TODO: move this code to separate function
                            brickId++;

                            if (brickId >= usedCells + 1)
                                throw new Exception("Too many cells for partial LOD: " + brickId + "!");

                            atlasZ = ((brickId / pxy));
                            atlasY = ((brickId % pxy) / packx);
                            atlasX = ((brickId % pxy) % packx);

                            if (atlasX >= 256 || atlasY >= 256 || atlasZ >= 256)
                                throw new Exception("Too big atlas index for partial LOD: " + brickId + "!");

                            Vector3i blockStart = new Vector3i(atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize);

                            float[] distanceBlock = new float[paddedTopLodCellSize * paddedTopLodCellSize * paddedTopLodCellSize];

                            Vector3i dataStart = vindex * data.CellSize;

                            for (int z = 0; z < paddedTopLodCellSize; z++)
                                for (int y = 0; y < paddedTopLodCellSize; y++)
                                    for (int x = 0; x < paddedTopLodCellSize; x++)
                                    {
                                        Vector3i coord = new Vector3i(x, y, z);

                                        PixelData pixel = GetArrayData(data.Data, data.Size, dataStart + coord);
#if USE_LODS
                                        // block for lower LOD calculation
                                        SetArrayData(distanceBlock, pixel.DistanceUV.X, blockSize, coord);
#endif
                                        // higher LOD
                                        SetArrayData(distanceLods[0], pixel.DistanceUV.X, topLodTextureSize, blockStart + coord);

                                        // texture UV coords
                                        SetArrayData(topLoduv, new Vector2(pixel.DistanceUV.Y, pixel.DistanceUV.Z), topLodTextureSize, blockStart + coord);
                                    }

                            float[] distancePercentageArr = GetTrilinear(distanceBlock, paddedTopLodCellSize, 1, new Vector3(data.CellSize / 2.0f, data.CellSize / 2.0f, data.CellSize / 2.0f));

                            distancePercentage = distancePercentageArr[0]; // central point


#if USE_LODS
                            int lodSize = paddedTopLodCellSize;
                            for (int i = 1; i < nlods; i++)
                            {
                                lodSize /= 2;
                                float[] lodDistanceBlock = GenerateLod(distanceBlock, blockWidth, blockHeight, blockDepth, lodSize);

#if LOD2_8BIT
                                lodDistance[i].PutBlock(lodDistanceBlock, lodSize, lodSize, lodSize, 1, atlasX * lodSize, atlasY * lodSize, atlasZ * lodSize, (k, l) => PackFloatToSByte((k - dist) * packLodCoef));
#else
                                lodDistance[i].PutBlock(lodDistanceBlock, lodSize, lodSize, lodSize, 1, atlasX * lodSize, atlasY * lodSize, atlasZ * lodSize, (k, l) => Helper.PackFloatToUShort(k - dist));
#endif
                            }
#endif

                            ValueTuple<int, float>[][] boxBones = GetCubeWeights(weightCache, data.Data, data.Size, dataStart, data.CellSize, vindex);

                            float[][] vertexDistances = GetCubeDistances(data.Data, data.Size, dataStart, data.CellSize);

                            boxes.Add(new BrickCellData(brickId, data.LowerBound + new Vector3(ix, iy, iz) * boxStep, vindex, boxStep, boxBones, vertexDistances));
                        }


                        SetArrayData(zeroLodData, new Vector4(distancePercentage, atlasX, atlasY, atlasZ), new Vector3i(cellsx, cellsy, cellsz), vindex);
                    }

            return usedCells;
        }

        /// <summary>
        /// Splits SDF data to non uniform bricks with sizes data.CellSize and it's multiplies of 2
        /// </summary>
        /// <param name="data"></param>
        /// <param name="topLodTextureSize"></param>
        /// <param name="topLodDistances"></param>
        /// <param name="topLoduv"></param>
        /// <param name="boxes"></param>
        /// <param name="targetPsnr"></param>
        /// <returns></returns>
        public static int ProcessBricks(
            DistanceData data,
            float targetPsnr,
            out LodData topLod
            )
        {
            Dictionary<Vector3i, ValueTuple<int, float>[]> weightCache = new Dictionary<Vector3i, ValueTuple<int, float>[]>();

            int cellsx = data.Size.X / data.CellSize;
            int cellsy = data.Size.Y / data.CellSize;
            int cellsz = data.Size.Z / data.CellSize;

            int paddedTopLodCellSize = data.CellSize + 1;

            Vector3i blockSize = new Vector3i(paddedTopLodCellSize, paddedTopLodCellSize, paddedTopLodCellSize);

            //DebugLog("Used cells: {0}, cellsx: {1}, cellsy: {2}, cellsz: {3}, blockSize: {4}", usedCells, cellsx, cellsy, cellsz, paddedTopLodCellSize);

            List<BrickData> bricks = new List<BrickData>();
            FindBricks(data.Data, data.Size, data.CellSize, targetPsnr, bricks);

            int packx;
            int packy;
            int packz;
            FindBestDividers(bricks.Count, out packx, out packy, out packz, 256);

            Vector3i topLodTextureSize = new Vector3i(packx, packy, packz) * paddedTopLodCellSize;

            float[] topLodDistances = new float[topLodTextureSize.X * topLodTextureSize.Y * topLodTextureSize.Z];

            Vector2[] topLoduv = new Vector2[packx * paddedTopLodCellSize * packy * paddedTopLodCellSize * packz * paddedTopLodCellSize];

            Vector3 boxStep = (data.UpperBound - data.LowerBound) / new Vector3(cellsx, cellsy, cellsz);

            BrickCellData[] boxes = new BrickCellData[bricks.Count];

            int pxy = packx * packy;

            for (int i = 0; i < bricks.Count; i++)
            {
                BrickData brick = bricks[i];

                int atlasZ = ((i / pxy));
                int atlasY = ((i % pxy) / packx);
                int atlasX = ((i % pxy) % packx);

                Vector3i blockStart = new Vector3i(atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize);

                for (int z = 0; z < paddedTopLodCellSize; z++)
                    for (int y = 0; y < paddedTopLodCellSize; y++)
                        for (int x = 0; x < paddedTopLodCellSize; x++)
                        {
                            Vector3i coord = new Vector3i(x, y, z);

                            PixelData pixel = GetArrayData(brick.Data, blockSize, coord);

                            // higher LOD
                            SetArrayData(topLodDistances, pixel.DistanceUV.X * data.CellSize / brick.CellSize, topLodTextureSize, blockStart + coord);

                            // texture UV coords
                            SetArrayData(topLoduv, new Vector2(pixel.DistanceUV.Y, pixel.DistanceUV.Z), topLodTextureSize, blockStart + coord);
                        }

                //ValueTuple<int, float>[][] boxBones = GetBoxWeights(weightCache, brick.Item3, blockSize, new Vector3i(0,0,0), data.CellSize, brick.Item1 / data.CellSize);
                ValueTuple<int, float>[][] boxBones = GetCubeWeights(weightCache, data.Data, data.Size, brick.Position, data.CellSize, brick.Position / data.CellSize);
                float[][] vertexDistances = GetCubeDistances(data.Data, data.Size, brick.Position, data.CellSize);

                boxes[i] = new BrickCellData(i, data.LowerBound + boxStep * new Vector3(brick.Position.X, brick.Position.Y, brick.Position.Z) / data.CellSize, brick.Position, boxStep * brick.CellSize / data.CellSize, boxBones, vertexDistances);
            }

            topLod = new LodData(topLodTextureSize, topLodDistances, topLoduv, boxes, null, new Vector3i(0, 0, 0));
            return bricks.Count;
        }

        /// <summary>
        /// Marks cells where distance changes from negative to positive and vice versa
        /// </summary>
        /// <param name="data"></param>
        /// <param name="dataSize"></param>
        /// <param name="cellSize"></param>
        /// <param name="cells"></param>
        /// <returns>number of cells</returns>
        private static int CheckCells(PixelData[] data, Vector3i dataSize, int cellSize, float[] cells)
        {
            int usedCells = 0;
            int cellsx = dataSize.X / cellSize;
            int cellsy = dataSize.Y / cellSize;
            int cellsz = dataSize.Z / cellSize;

            for (int iz = 0; iz < cellsz; iz++)
            {
                for (int iy = 0; iy < cellsy; iy++)
                {
                    for (int ix = 0; ix < cellsx; ix++)
                    {
                        Vector3i dataStart = new Vector3i(ix * cellSize, iy * cellSize, iz * cellSize);

                        float minDistance = float.MaxValue;

                        int sign = 0;
                        int count = 0;

                        for (int z = 0; z <= cellSize; z++)
                            for (int y = 0; y <= cellSize; y++)
                                for (int x = 0; x <= cellSize; x++)
                                {
                                    PixelData pixel = GetArrayData(data, dataSize, dataStart + new Vector3i(x, y, z));

                                    float distance = pixel.DistanceUV.X;
                                    sign += Math.Sign(distance);
                                    count++;

                                    if (Math.Abs(distance) < minDistance)
                                    {
                                        minDistance = Math.Abs(distance);
                                    }
                                }


                        if (sign != count) // all neg or all pos is ignored
                        {
                            usedCells++;
                        }
                        else
                        {
                            minDistance = -minDistance;
                        }

                        cells[ix + iy * cellsx + iz * cellsx * cellsy] = minDistance;
                    }
                }
            }

            return usedCells;
        }

        /// <summary>
        /// This method analyzes the pixel data and tries to find biggest possible bricks with some quality loss
        /// </summary>
        /// <param name="data"></param>
        /// <param name="dataSize"></param>
        /// <param name="minCellSize"></param>
        /// <param name="minPsnr"></param>
        /// <param name="cells"></param>
        private static void FindBricks(
            PixelData[] data, Vector3i dataSize,
            int minCellSize, float minPsnr,
            IList<BrickData> cells)
        {
            int minCellsx = dataSize.X / minCellSize;
            int minCellsy = dataSize.Y / minCellSize;
            int minCellsz = dataSize.Z / minCellSize;
            // we are having a boolean array of cellsx*cellsy*cellsz size to indicate non-usable bricks

            Vector3i minCellDimensions = new Vector3i(minCellsx, minCellsy, minCellsz);
            bool[] usedCells = new bool[minCellsx * minCellsy * minCellsz];

            int steps = 1;
            int cellSize = minCellSize;
            int cellsx = minCellsx;
            int cellsy = minCellsy;
            int cellsz = minCellsz;

            while (cellsx >= 4 && cellsy >= 4 && cellsz >= 4)
            {
                steps++;
                cellSize *= 2;

                cellsx = dataSize.X / cellSize;
                cellsy = dataSize.Y / cellSize;
                cellsz = dataSize.Z / cellSize;
            }; // faster and easier than log calculation


            // first we start with largest cell size that is (minCellSize >> steps) and go down to minCellSize
            for (int s = steps; s >= 1; s--)
            {
                int ncells = cellSize / minCellSize; // how many small cells fit in this cell. 2^steps for the first step, 1 for the last
                cellsx = dataSize.X / cellSize;
                cellsy = dataSize.Y / cellSize;
                cellsz = dataSize.Z / cellSize;

                Vector3i cellDimensions = new Vector3i(cellSize + 1, cellSize + 1, cellSize + 1);

                int cellCount = 0;

                for (int iz = 0; iz < cellsz; iz++)
                {
                    for (int iy = 0; iy < cellsy; iy++)
                    {
                        for (int ix = 0; ix < cellsx; ix++)
                        {
                            // skip cells that are marked at taken by previous passes
                            if (GetArrayData(usedCells, minCellDimensions, new Vector3i(ix * ncells, iy * ncells, iz * ncells)))
                                continue;

                            Vector3i dataStart = new Vector3i(ix * cellSize, iy * cellSize, iz * cellSize);

                            //float minDistance = float.MaxValue;

                            int countPositive = 0;
                            int countNegative = 0;

                            PixelData[] cellData = new PixelData[(cellSize + 1) * (cellSize + 1) * (cellSize + 1)];

                            for (int z = 0; z <= cellSize; z++)
                                for (int y = 0; y <= cellSize; y++)
                                    for (int x = 0; x <= cellSize; x++)
                                    {
                                        PixelData pixel = GetArrayData(data, dataSize, dataStart + new Vector3i(x, y, z));

                                        float distance = pixel.DistanceUV.X;

                                        if (distance > 0)
                                            countPositive++;
                                        else
                                            if (distance < 0)
                                            countNegative++;

                                        //if (Math.Abs(distance) < minDistance)
                                        //{
                                        //    minDistance = Math.Abs(distance);
                                        //}

                                        SetArrayData(cellData, pixel, cellDimensions, new Vector3i(x, y, z));
                                    }

                            if (countPositive == 0 || countNegative == 0) // all neg or all pos is ignored
                            {
                                if (s > 1)
                                    for (int cz = 0; cz < ncells; cz++)
                                        for (int cy = 0; cy < ncells; cy++)
                                            for (int cx = 0; cx < ncells; cx++)
                                            {
                                                Vector3i cellPos = new Vector3i(ix * ncells + cx, iy * ncells + cy, iz * ncells + cz);

                                                SetArrayData(usedCells, true, minCellDimensions, cellPos);
                                            }
                                continue;
                            }

                            PixelData[] brick = cellData;

                            if (s > 1)
                            {
                                // check if big cell can be used instead of small. If so - mark the usedCells block just as we did above and write result to `cells`
                                // to do so we skip some pixels, enlarge the brick to the full cell size and check if there is a significant difference.
                                // if not, we add the brick to collection and use it instead of lots of small bricks

                                // we take only each 'step' pixels including first and last one

                                /*PixelData[] reducedCellData = new PixelData[(minCellSize + 1) * (minCellSize + 1) * (minCellSize + 1)];
                                float[] reducedCellDistances = new float[(minCellSize + 1) * (minCellSize + 1) * (minCellSize + 1)];

                                for (int z = 0; z <= minCellSize; z++)
                                    for (int y = 0; y <= minCellSize; y++)
                                        for (int x = 0; x <= minCellSize; x++)
                                        {
                                            PixelData pixel = GetArrayData(cellData, cellDimensions, new Vector3i(x * ncells, y * ncells, z * ncells));
                                            float distance = pixel.DistanceUV.X;
                                            SetArrayData(reducedCellData, pixel, new Vector3i(minCellSize + 1, minCellSize + 1, minCellSize + 1), new Vector3i(x, y, z));
                                            SetArrayData(reducedCellDistances, distance, new Vector3i(minCellSize + 1, minCellSize + 1, minCellSize + 1), new Vector3i(x, y, z));
                                        }

                                //float[] reducedCellDistances = GenerateLod(cellData, cellSize + 1, minCellSize + 1);

                                float[] enlargedCellDistances = GenerateLod(reducedCellDistances, minCellSize + 1, cellSize + 1);*/

                                PixelData[] reducedCellData = GenerateLod(cellData, cellSize + 1, minCellSize + 1);
                                PixelData[] enlargedCellData = GenerateLod(reducedCellData, minCellSize + 1, cellSize + 1);


                                float mse = 0;

                                for (int i = 0; i < cellData.Length; i++)
                                {
                                    float diff = Math.Abs(cellData[i].DistanceUV.X - enlargedCellData[i].DistanceUV.X);

                                    mse += diff * diff;
                                }

                                mse /= cellData.Length;

                                float psnr = (float)(10.0 * Math.Log10(1.0 / mse));

                                if (psnr < minPsnr)
                                    continue;

                                brick = reducedCellData;

                                // mark data as already used

                                for (int cz = 0; cz < ncells; cz++)
                                    for (int cy = 0; cy < ncells; cy++)
                                        for (int cx = 0; cx < ncells; cx++)
                                        {
                                            Vector3i cellPos = new Vector3i(ix * ncells + cx, iy * ncells + cy, iz * ncells + cz);

                                            SetArrayData(usedCells, true, minCellDimensions, cellPos);
                                        }

                            }

                            // add result to `cells`

                            cellCount++;
                            cells.Add(new BrickData(cellCount, dataStart, cellSize, brick));
                        }
                    }

                }

                DebugLog("Found {0} cells of size {1}x{1}x{1}", cellCount, cellSize + 1);

                cellSize /= 2;
            }
        }

        public static int ProcessBricksWithLods(
            DistanceData data,
            int baseCellSize,
            bool processWeights,

            out LodData[] lods
            )
        {
            int cellSize = data.CellSize;
            if (cellSize != 4 && cellSize != 16 && cellSize != 64 && cellSize != 256 &&
                cellSize != 3 && cellSize != 9 && cellSize != 27 && cellSize != 81)
            {
                throw new Exception("Only cell size 4^x and 3^x are supported at the moment!");
            }

            Dictionary<Vector3i, ValueTuple<int, float>[]> weightCache = new Dictionary<Vector3i, ValueTuple<int, float>[]>();

            int cellsx = data.Size.X / baseCellSize;
            int cellsy = data.Size.Y / baseCellSize;
            int cellsz = data.Size.Z / baseCellSize;

            int paddedTopLodCellSize = baseCellSize + 1;

            Vector3i blockSize = new Vector3i(paddedTopLodCellSize, paddedTopLodCellSize, paddedTopLodCellSize);

            //DebugLog("Used cells: {0}, cellsx: {1}, cellsy: {2}, cellsz: {3}, blockSize: {4}", usedCells, cellsx, cellsy, cellsz, paddedTopLodCellSize);

            //DebugLog("Used cells: {0}, cellsx: {1}, cellsy: {2}, cellsz: {3}, blockSize: {4}", 0, cellsx, cellsy, cellsz, paddedTopLodCellSize);

            int lodLevels = 0;
            int ncellSize = cellSize;
            while (ncellSize >= baseCellSize)
            {
                ncellSize /= baseCellSize;
                lodLevels++;
            }

            DebugLog("Processing {0} lod levels, from {1} to {2}", lodLevels, baseCellSize, cellSize);

            List<BrickData>[] resultBricks = new List<BrickData>[lodLevels];

            // LOD 0
            int[] lod0 = new int[cellsx * cellsy * cellsz];
            resultBricks[0] = new List<BrickData>();
            ncellSize = cellSize;


            DebugLog("Processing lod {0}, cell size {1}", 0, ncellSize);
            // lod 0 is added as is2z3zxv4cb5v68789m,,,,,,,,0.-/////////.0,9m.p8ionuobyi j  hf g f
            FindFixedBricks(data.Data, data.Size, ncellSize, resultBricks[0], new Vector3i(0, 0, 0), lod0, new Vector3i(cellsx, cellsy, cellsz), (int)Math.Round(Math.Sqrt(cellSize)));
            DebugLog("Brick count: {0}", resultBricks[0].Count);

            for (int lod = 1; lod < lodLevels; lod++)
            {
                resultBricks[lod] = new List<BrickData>();
                ncellSize /= baseCellSize;

                DebugLog("Processing lod {0}, cell size {1}", lod, ncellSize);

                foreach (var brick in resultBricks[lod - 1])
                {
                    int index = resultBricks[lod].Count;

                    FindFixedBricks(brick.Data, new Vector3i(brick.CellSize + 1, brick.CellSize + 1, brick.CellSize + 1), ncellSize,
                        resultBricks[lod], brick.Position, brick.Children, new Vector3i(baseCellSize, baseCellSize, baseCellSize), (int)Math.Round(Math.Sqrt(ncellSize)));
                }

                DebugLog("Brick count: {0}", resultBricks[lod].Count);
            }

            Vector3i[] packing = new Vector3i[lodLevels];
            for (int lod = 0; lod < lodLevels; lod++)
            {
                int packx;
                int packy;
                int packz;
                FindBestDividers(resultBricks[lod].Count, out packx, out packy, out packz, 256);
                packing[lod] = new Vector3i(packx, packy, packz);
            }

            Vector3 boxStep = (data.UpperBound - data.LowerBound) / (new Vector3(cellsx, cellsy, cellsz) * baseCellSize);

            lods = new LodData[lodLevels];

            for (int lod = 0; lod < lodLevels; lod++)
            {
                DebugLog("Packing lod {0}", lod);

                Vector3i pack = packing[lod];

                Vector3i lodTextureSize = pack * paddedTopLodCellSize;
                float[] lodDistances = new float[lodTextureSize.X * lodTextureSize.Y * lodTextureSize.Z];
                Vector2[] lodUv = new Vector2[lodTextureSize.X * lodTextureSize.Y * lodTextureSize.Z];

                Vector3i lodChildrenSize = pack * baseCellSize;
                int[] lodChildren = new int[lodChildrenSize.X * lodChildrenSize.Y * lodChildrenSize.Z];

                BrickCellData[] lodBoxes = new BrickCellData[resultBricks[lod].Count];

                int pxy = pack.X * pack.Y;

                for (int i = 0; i < resultBricks[lod].Count; i++)
                {
                    BrickData brick = resultBricks[lod][i];

                    PixelData[] pixelData = brick.CellSize == baseCellSize ? brick.Data : GenerateLod(brick.Data, brick.CellSize + 1, paddedTopLodCellSize);

                    int atlasZ = ((i / pxy));
                    int atlasY = ((i % pxy) / pack.X);
                    int atlasX = ((i % pxy) % pack.X);

                    Vector3i blockStart = new Vector3i(atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize);

                    for (int z = 0; z < paddedTopLodCellSize; z++)
                        for (int y = 0; y < paddedTopLodCellSize; y++)
                            for (int x = 0; x < paddedTopLodCellSize; x++)
                            {
                                Vector3i coord = new Vector3i(x, y, z);

                                PixelData pixel = GetArrayData(pixelData, blockSize, coord);

                                // distance in X coord
                                SetArrayData(lodDistances, pixel.DistanceUV.X * data.CellSize / brick.CellSize, lodTextureSize, blockStart + coord);

                                // texture UV coords in YZ
                                SetArrayData(lodUv, new Vector2(pixel.DistanceUV.Y, pixel.DistanceUV.Z), lodTextureSize, blockStart + coord);

                            }

                    if (brick.Children != null && brick.Children.Length == (baseCellSize * baseCellSize * baseCellSize))
                    {
                        blockStart = new Vector3i(atlasX * baseCellSize, atlasY * baseCellSize, atlasZ * baseCellSize);

                        for (int z = 0; z < baseCellSize; z++)
                            for (int y = 0; y < baseCellSize; y++)
                                for (int x = 0; x < baseCellSize; x++)
                                {
                                    Vector3i coord = new Vector3i(x, y, z);

                                    int childData = GetArrayData(brick.Children, new Vector3i(baseCellSize, baseCellSize, baseCellSize), coord);
                                    SetArrayData(lodChildren, childData, lodChildrenSize, blockStart + coord);
                                }
                    }

                    //ValueTuple<int, float>[][] boxBones = GetBoxWeights(weightCache, brick.Item3, blockSize, new Vector3i(0,0,0), data.CellSize, brick.Item1 / data.CellSize);
                    ValueTuple<int, float>[][] boxBones = processWeights ? GetCubeWeights(weightCache, data.Data, data.Size, brick.Position, baseCellSize, brick.Position / data.CellSize) : null;
                    float[][] vertexDistances = GetCubeDistances(pixelData, blockSize, new Vector3i(0, 0, 0), baseCellSize);

                    lodBoxes[i] = new BrickCellData(i, data.LowerBound + boxStep * new Vector3(brick.Position.X, brick.Position.Y, brick.Position.Z), brick.Position, boxStep * brick.CellSize, boxBones, vertexDistances);
                }

                lods[lod] = new LodData(lodTextureSize, lodDistances, lodUv, lodBoxes, lodChildren, lodChildrenSize);
            }

            return lodLevels;
        }

        public static int ProcessBricksWithNestedLods(
            DistanceData data,
            int baseCellSize,
            bool processWeights,

            out LodData[] lods
            )
        {
            int cellSize = data.CellSize;
            if (cellSize != 4)
            {
                throw new Exception("Only cell size 4 is supported at the moment!");
            }

            Dictionary<Vector3i, ValueTuple<int, float>[]> weightCache = new Dictionary<Vector3i, ValueTuple<int, float>[]>();

            int cellsx = data.Size.X / baseCellSize;
            int cellsy = data.Size.Y / baseCellSize;
            int cellsz = data.Size.Z / baseCellSize;

            int paddedTopLodCellSize = baseCellSize + 1;

            Vector3i blockSize = new Vector3i(paddedTopLodCellSize, paddedTopLodCellSize, paddedTopLodCellSize);

            int lodLevels = 1;// (data.LodData.Length - 1) / 2 + 1;

            Tuple<PixelData[], Vector3i>[] lodData = new Tuple<PixelData[], Vector3i>[lodLevels];

            Vector3i lodSize = data.Size;

            for (int l = 0; l < lodLevels; l++)
            {
                lodData[l] = new Tuple<PixelData[], Vector3i>(data.LodData[l * 2], lodSize);
                lodSize = new Vector3i((lodSize.X - 1) * cellSize + 1, (lodSize.Y - 1) * cellSize + 1, (lodSize.Z - 1) * cellSize + 1);
            }

            DebugLog("Processing {0} lod levels, from {1} to {2}", lodLevels, baseCellSize, cellSize);

            List<BrickData>[] resultBricks = new List<BrickData>[lodLevels];
            resultBricks[0] = new List<BrickData>();

            DebugLog("Processing lod {0}, cell size {1}", 0, cellSize);
            int totalBricks = FindFixedBricks(lodData[0].Item1, lodData[0].Item2, cellSize, resultBricks[0], new Vector3i(0, 0, 0), null, new Vector3i(cellsx, cellsy, cellsz), cellSize + 1);
            DebugLog("LOD0 Brick count: {0}", totalBricks);

            for (int lod = 1; lod < lodLevels; lod++)
            {
                DebugLog("Processing lod {0}", lod);
                resultBricks[lod] = new List<BrickData>();

                //Dictionary<int, int> readyBricks = new Dictionary<int, int>();

                foreach(var brick in resultBricks[lod - 1])
                {
                    Vector3i position = brick.Position;

                    for (int z = 0; z < cellSize + 1; z++)
                        for (int y = 0; y < cellSize + 1; y++)
                            for (int x = 0; x < cellSize + 1; x++)
                            {
                                Vector3i brickPos = (position + new Vector3i(x, y, z)) * cellSize;
                                /*int index = brickPos.X + brickPos.Y * lodData[lod].Item2.X + brickPos.Z * lodData[lod].Item2.X * lodData[lod].Item2.Y;

                                int readyBrick;
                                if (readyBricks.TryGetValue(index, out readyBrick))
                                {
                                    brick.Children[x + y * (cellSize + 1) + z * (cellSize + 1) * (cellSize + 1)] = readyBrick;
                                    continue;
                                }*/

                                PixelData[] newBrick = CheckBrick(lodData[lod].Item1, brickPos, lodData[lod].Item2, cellSize);

                                if (newBrick == null)
                                    continue;

                                int id = ++totalBricks;
                                resultBricks[lod].Add(new BrickData(id, brickPos, cellSize, newBrick, new int[(cellSize + 1) * (cellSize + 1) * (cellSize + 1)]));

                                brick.Children[x + y * (cellSize + 1) + z * (cellSize + 1) * (cellSize + 1)] = id;
                                //readyBricks[index] = id;
                            }
                }

                DebugLog("Brick count: {0}", resultBricks[lod].Count);
            }

            List<BrickData> allBricks = new List<BrickData>();
            for (int lod = 0; lod < lodLevels; lod++)
                allBricks.AddRange(resultBricks[lod]);

            int packx;
            int packy;
            int packz;
            FindBestDividers(allBricks.Count, out packx, out packy, out packz, 256);
            Vector3i pack = new Vector3i(packx, packy, packz);

            Vector3 boxStep = (data.UpperBound - data.LowerBound) / (new Vector3(cellsx, cellsy, cellsz) * baseCellSize);

            lods = new LodData[1];

            {
                DebugLog("Generating LOD1 boxes");

                BrickCellData[] lodBoxes = new BrickCellData[resultBricks[0].Count + 1];

                foreach(BrickData brick in resultBricks[0])
                {
                    PixelData[] pixelData = brick.Data;

                    ValueTuple<int, float>[][] boxBones = processWeights ? GetCubeWeights(weightCache, data.Data, data.Size, brick.Position, baseCellSize, brick.Position / data.CellSize) : null;
                    float[][] vertexDistances = GetCubeDistances(pixelData, blockSize, new Vector3i(0, 0, 0), baseCellSize);

                    lodBoxes[brick.Id] = new BrickCellData(brick.Id, data.LowerBound + boxStep * new Vector3(brick.Position.X, brick.Position.Y, brick.Position.Z), brick.Position, boxStep * brick.CellSize, boxBones, vertexDistances);
                }

                DebugLog("Packing");

                Vector3i lodTextureSize = pack * paddedTopLodCellSize;
                float[] lodDistances = new float[lodTextureSize.X * lodTextureSize.Y * lodTextureSize.Z];
                Vector2[] lodUv = new Vector2[lodTextureSize.X * lodTextureSize.Y * lodTextureSize.Z];
                int[] lodChildren = new int[lodTextureSize.X * lodTextureSize.Y * lodTextureSize.Z];

                int pxy = pack.X * pack.Y;

                for (int i = 0; i < allBricks.Count; i++)
                {
                    BrickData brick = allBricks[i];

                    PixelData[] pixelData = brick.Data;

                    int id = brick.Id;

                    int atlasZ = ((id / pxy));
                    int atlasY = ((id % pxy) / pack.X);
                    int atlasX = ((id % pxy) % pack.X);

                    Vector3i blockStart = new Vector3i(atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize);

                    for (int z = 0; z < paddedTopLodCellSize; z++)
                        for (int y = 0; y < paddedTopLodCellSize; y++)
                            for (int x = 0; x < paddedTopLodCellSize; x++)
                            {
                                Vector3i coord = new Vector3i(x, y, z);

                                PixelData pixel = GetArrayData(pixelData, blockSize, coord);

                                // distance in X coord
                                SetArrayData(lodDistances, pixel.DistanceUV.X * data.CellSize / brick.CellSize, lodTextureSize, blockStart + coord);

                                // texture UV coords in YZ
                                SetArrayData(lodUv, new Vector2(pixel.DistanceUV.Y, pixel.DistanceUV.Z), lodTextureSize, blockStart + coord);

                                if (brick.Children != null)
                                {
                                    int childData = GetArrayData(brick.Children, blockSize, coord);
                                    SetArrayData(lodChildren, childData, lodTextureSize, blockStart + coord);
                                }
                            }

                    lods[0] = new LodData(lodTextureSize, lodDistances, lodUv, lodBoxes, lodChildren, lodTextureSize);
                }
            }
            
            return lodLevels;
        }

        private static Vector3i Unpack(int i, Vector3i pack)
        {
            int pxy = pack.X * pack.Y;


            int atlasZ = ((i / pxy));
            int atlasY = ((i % pxy) / pack.X);
            int atlasX = ((i % pxy) % pack.X);

            return new Vector3i(atlasX, atlasY, atlasZ);
        }

        private static PixelData[] CheckBrick(PixelData[] data, Vector3i dataStart, Vector3i dataSize, int cellSize)
        {
            int sign = 0;
            int count = 0;

            // do a first run to know if we are processing this brick at all
            for (int z = 0; z <= cellSize; z++)
                for (int y = 0; y <= cellSize; y++)
                    for (int x = 0; x <= cellSize; x++)
                    {
                        PixelData pixel = GetArrayData(data, dataSize, dataStart + new Vector3i(x, y, z));

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
                        PixelData pixel = GetArrayData(data, dataSize, dataStart + new Vector3i(x, y, z));
                        SetArrayData(brick, pixel, new Vector3i(cellSize + 1, cellSize + 1, cellSize + 1), new Vector3i(x, y, z));
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
        private static int FindFixedBricks(
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
                                SetArrayData(targetArray, index, targetSize, coord);

                            cellCount++;
                            cells.Add(new BrickData(cellCount, coord * cellSize + shift, cellSize, brick, new int[childCell * childCell * childCell]));
                        }
                    }
                }
            }
            return cellCount;
        }


        #region Interpolation and LODs

        private static readonly Vector3i[] s_cubeIndices =
        {
            new Vector3i(0,0,0),
            new Vector3i(1,0,0),
            new Vector3i(1,1,0),
            new Vector3i(0,1,0),
            new Vector3i(0,1,1),
            new Vector3i(1,1,1),
            new Vector3i(1,0,1),
            new Vector3i(0,0,1),
        };

        private static float[][] GetCubeDistances(
            PixelData[] data, Vector3i dataSize, Vector3i dataStart,
            int blockSize)
        {
            float[][] vertexDistances = new float[8][];

            for (int i = 0; i < vertexDistances.Length; i++)
                vertexDistances[i] = new float[] { GetArrayData(data, dataSize, dataStart + s_cubeIndices[i] * blockSize).DistanceUV.X };

            return vertexDistances;
        }

        private static ValueTuple<int, float>[][] GetCubeWeights(
            Dictionary<Vector3i, ValueTuple<int, float>[]> cache,
            PixelData[] data, Vector3i dataSize, Vector3i dataStart,
            int blockSize,
            Vector3i index)
        {
            int ubx = blockSize - 1;
            int uby = blockSize - 1;
            int ubz = blockSize - 1;

            ValueTuple<int, float>[][] result = new ValueTuple<int, float>[8][];

            List<ValueTuple<Vector3, Vector4i, Vector4>>[] vertexWeights = new List<ValueTuple<Vector3, Vector4i, Vector4>>[8];

            for (int i = 0; i < 8; i++)
                vertexWeights[i] = new List<ValueTuple<Vector3, Vector4i, Vector4>>();

            for (int z = 0; z < ubz; z++)
                for (int y = 0; y < uby; y++)
                    for (int x = 0; x < ubx; x++)
                    {
                        Vector3i coord = dataStart + new Vector3i(x, y, z);
                        PixelData pixelData = GetArrayData(data, dataSize, dataStart + new Vector3i(x, y, z));

                        float dist = pixelData.DistanceUV.X;
                        int sign = Math.Sign(dist);

                        // checks if a surface crosses nearest 2x2x2 texel cube
                        if (Math.Abs(dist) <= 1 ||
                            sign != Math.Sign(GetArrayData(data, dataSize, coord + s_cubeIndices[1]).DistanceUV.X) ||
                            sign != Math.Sign(GetArrayData(data, dataSize, coord + s_cubeIndices[2]).DistanceUV.X) ||
                            sign != Math.Sign(GetArrayData(data, dataSize, coord + s_cubeIndices[3]).DistanceUV.X) ||
                            sign != Math.Sign(GetArrayData(data, dataSize, coord + s_cubeIndices[4]).DistanceUV.X) ||
                            sign != Math.Sign(GetArrayData(data, dataSize, coord + s_cubeIndices[5]).DistanceUV.X) ||
                            sign != Math.Sign(GetArrayData(data, dataSize, coord + s_cubeIndices[6]).DistanceUV.X) ||
                            sign != Math.Sign(GetArrayData(data, dataSize, coord + s_cubeIndices[7]).DistanceUV.X)
                            )
                        {
                            vertexWeights[0].Add(new ValueTuple<Vector3, Vector4i, Vector4>(new Vector3(x, y, z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[1].Add(new ValueTuple<Vector3, Vector4i, Vector4>(new Vector3(ubx - x, y, z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[2].Add(new ValueTuple<Vector3, Vector4i, Vector4>(new Vector3(ubx - x, uby - y, z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[3].Add(new ValueTuple<Vector3, Vector4i, Vector4>(new Vector3(x, uby - y, z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[4].Add(new ValueTuple<Vector3, Vector4i, Vector4>(new Vector3(x, uby - y, ubz - z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[5].Add(new ValueTuple<Vector3, Vector4i, Vector4>(new Vector3(ubx - x, uby - y, ubz - z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[6].Add(new ValueTuple<Vector3, Vector4i, Vector4>(new Vector3(ubx - x, y, ubz - z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[7].Add(new ValueTuple<Vector3, Vector4i, Vector4>(new Vector3(x, y, ubz - z), pixelData.Bones, pixelData.BoneWeights));
                        }
                    }

            for (int i = 0; i < vertexWeights.Length; i++)
            {
                Dictionary<int, ValueTuple<float, int>> weightValues = new Dictionary<int, ValueTuple<float, int>>();

                foreach (var pair in vertexWeights[i])
                {
                    //Euclidian distance to the pixel
                    //float weight = Vector3.DistanceSquared(new Vector3(1, 1, 1), pair.Item1 / new Vector3(ubx, uby, ubz));
                    float weight = Vector3.Distance(new Vector3(1, 1, 1), pair.Item1 / new Vector3(ubx, uby, ubz));
                    //Manhattan distance to the pixel
                    //float weight = Vector3.Dot(new Vector3(1, 1, 1) - pair.Item1 / new Vector3(ubx, uby, ubz), new Vector3(1, 1, 1));

                    for (int b = 0; b < 4; b++)
                        if (pair.Item2[b] != 0)
                        {
                            ValueTuple<float, int> oldValue;
                            if (!weightValues.TryGetValue(pair.Item2[b], out oldValue))
                                oldValue = new ValueTuple<float, int>(0, 0);

                            float w = 0.0f;
                            switch (b)
                            {
                                case 0:
                                    w = pair.Item3.X;
                                    break;
                                case 1:
                                    w = pair.Item3.Y;
                                    break;
                                case 2:
                                    w = pair.Item3.Z;
                                    break;
                                case 3:
                                    w = pair.Item3.W;
                                    break;
                            }

                            oldValue.Item1 += w * weight;
                            oldValue.Item2++;

                            weightValues[pair.Item2[b]] = oldValue;
                        }
                }

                ValueTuple<int, float>[] oldWeights;

                Vector3i nindex = index + s_cubeIndices[i];

                if (cache.TryGetValue(nindex, out oldWeights))
                {
                    foreach (var pair in oldWeights)
                    {
                        ValueTuple<float, int> oldValue;
                        if (!weightValues.TryGetValue(pair.Item1, out oldValue))
                            oldValue = new ValueTuple<float, int>(pair.Item2, 1);
                        else
                        {
                            // add old data with the same weight as current
                            oldValue.Item1 = oldValue.Item1 + pair.Item2;
                            oldValue.Item2 *= 2;
                        }

                        weightValues[pair.Item1] = oldValue;
                    }
                }

                List<ValueTuple<int, float>> vertexResult = new List<ValueTuple<int, float>>(weightValues.Count);

                foreach (var pair in weightValues)
                {
                    int bone = pair.Key;
                    float weigth = pair.Value.Item1 / pair.Value.Item2;
                    vertexResult.Add(new ValueTuple<int, float>(bone, weigth));
                }

                vertexResult.Sort((x, y) => y.Item2.CompareTo(x.Item2));

                if (oldWeights == null)
                    oldWeights = new (int, float)[4];

                for (int j = 0; j < vertexResult.Count && j < oldWeights.Length; j++)
                    oldWeights[j] = vertexResult[j];

                result[i] = oldWeights;

                cache[nindex] = oldWeights;
            }

            return result;
        }

        private static readonly Vector3i[] s_trilinearShifts =
        {
            new Vector3i(0, 0, 0),
            new Vector3i(1, 0, 0),
            new Vector3i(0, 1, 0),
            new Vector3i(1, 1, 0),
            new Vector3i(0, 0, 1),
            new Vector3i(1, 0, 1),
            new Vector3i(0, 1, 1),
            new Vector3i(1, 1, 1)
        };

        /// <summary>
        /// Gets the value of pixel with trilinear interpolation 
        /// </summary>
        /// <param name="block"></param>
        /// <param name="blockSize"></param>
        /// <param name="stride"></param>
        /// <param name="coord"></param>
        /// <returns>array of size <paramref name="stride"/></returns>
        private static float[] GetTrilinear(float[] block, int blockSize, int stride, Vector3 coord)
        {
            int ix = (int)coord.X;
            int iy = (int)coord.Y;
            int iz = (int)coord.Z;
            float frx = coord.X - ix;
            float fry = coord.Y - iy;
            float frz = coord.Z - iz;

            float[] coef =
            {
                (1.0f - frx) * (1.0f - fry) * (1.0f - frz),
                (frx) * (1.0f - fry) * (1.0f - frz),
                (1.0f - frx) * (fry) * (1.0f - frz),
                (frx) * (fry) * (1.0f - frz),
                (1.0f - frx) * (1.0f - fry) * (frz),
                (frx) * (1.0f - fry) * (frz),
                (1.0f - frx) * (fry) * (frz),
                (frx) * (fry) * (frz)
            };


            Vector3i blockDimensions = new Vector3i(blockSize * stride, blockSize * stride, blockSize * stride);

            float[] result = new float[stride];
            for (int i = 0; i < coef.Length; i++)
            {
                Vector3i ip = new Vector3i(ix + s_trilinearShifts[i].X, iy + s_trilinearShifts[i].Y, iz + s_trilinearShifts[i].Z);

                if (ip.X >= blockSize || ip.Y >= blockSize || ip.Z >= blockSize)
                    continue;

                for (int c = 0; c < stride; c++)
                    result[c] += coef[i] * GetArrayData(block, blockDimensions, ip, c);
            }
            return result;
        }

        private static PixelData GetTrilinear(PixelData[] block, int blockSize, Vector3 coord)
        {
            int ix = (int)coord.X;
            int iy = (int)coord.Y;
            int iz = (int)coord.Z;
            float frx = coord.X - ix;
            float fry = coord.Y - iy;
            float frz = coord.Z - iz;

            float[] coef =
            {
                (1.0f - frx) * (1.0f - fry) * (1.0f - frz),
                (frx) * (1.0f - fry) * (1.0f - frz),
                (1.0f - frx) * (fry) * (1.0f - frz),
                (frx) * (fry) * (1.0f - frz),
                (1.0f - frx) * (1.0f - fry) * (frz),
                (frx) * (1.0f - fry) * (frz),
                (1.0f - frx) * (fry) * (frz),
                (frx) * (fry) * (frz)
            };


            Vector3i blockDimensions = new Vector3i(blockSize, blockSize, blockSize);

            float dx = 0;
            float dy = 0;
            float dz = 0;

            for (int i = 0; i < coef.Length; i++)
            {
                Vector3i ip = new Vector3i(ix + s_trilinearShifts[i].X, iy + s_trilinearShifts[i].Y, iz + s_trilinearShifts[i].Z);

                if (ip.X >= blockSize || ip.Y >= blockSize || ip.Z >= blockSize)
                    continue;

                PixelData data = GetArrayData(block, blockDimensions, ip);

                dx += coef[i] * data.DistanceUV.X;
                dy += coef[i] * data.DistanceUV.Y;
                dz += coef[i] * data.DistanceUV.Z;
            }
            PixelData nearest = GetArrayData(block, blockDimensions, new Vector3i((int)Math.Round(frx), (int)Math.Round(fry), (int)Math.Round(frz)));

            return new PixelData(dx, dy, dz, nearest.Bones, nearest.BoneWeights);
        }

        private static float[] GenerateLod(float[] sourceData, int sourceSize, int targetSize)
        {
            float[] result = new float[targetSize * targetSize * targetSize];

            float step = (sourceSize - 1.0f) / (targetSize - 1.0f);

            for (int nz = 0; nz < targetSize; nz++)
            {
                for (int ny = 0; ny < targetSize; ny++)
                {
                    for (int nx = 0; nx < targetSize; nx++)
                    {
                        float[] value = GetTrilinear(sourceData, sourceSize, 1, new Vector3(nx, ny, nz) * step);

                        result[nx + ny * targetSize + nz * targetSize * targetSize] = value[0];
                    }
                }
            }

            return result;
        }

        private static PixelData[] GenerateLod(PixelData[] sourceData, int sourceSize, int targetSize)
        {
            PixelData[] result = new PixelData[targetSize * targetSize * targetSize];

            float step = (sourceSize - 1.0f) / (targetSize - 1.0f);

            for (int nz = 0; nz < targetSize; nz++)
            {
                for (int ny = 0; ny < targetSize; ny++)
                {
                    for (int nx = 0; nx < targetSize; nx++)
                    {
                        result[nx + ny * targetSize + nz * targetSize * targetSize] = GetTrilinear(sourceData, sourceSize, new Vector3(nx, ny, nz) * step);
                    }
                }
            }

            return result;
        }

        #endregion

        #region Utils

        private static T GetArrayData<T>(T[] array, Vector3i dataSize, Vector3i coord, int shift = 0)
        {
            return array[coord.X + coord.Y * dataSize.X + coord.Z * dataSize.X * dataSize.Y + shift];
        }

        private static void SetArrayData<T>(T[] array, T data, Vector3i dataSize, Vector3i coord, int shift = 0)
        {
            array[coord.X + coord.Y * dataSize.X + coord.Z * dataSize.X * dataSize.Y + shift] = data;
        }


        /// <summary>
        /// Tries to find a best way to divide value by three numbers
        /// </summary>
        /// <param name="value"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <param name="max"></param>
        public static void FindBestDividers(int value, out int x, out int y, out int z, int max)
        {
            int root = (int)Math.Ceiling(Math.Pow(value, 1 / 3.0));

            x = root; y = root; z = root;
            int closest = x * y * z;

            for (int nz = 1; nz <= root * 4; nz++)
            {
                int lz = root + nz / 2 * Math.Sign(nz % 2 - 0.5f);
                if (lz > max || lz <= 0)
                    continue;

                for (int ny = 1; ny <= root * 4; ny++)
                {
                    int ly = root + ny / 2 * Math.Sign(ny % 2 - 0.5f);
                    if (ly > max || ly <= 0)
                        continue;

                    for (int nx = 1; nx <= root * 4; nx++)
                    {
                        int lx = root + nx / 2 * Math.Sign(nx % 2 - 0.5f);

                        if (lx > max || lx <= 0)
                            continue;

                        int nvalue = lx * ly * lz;

                        if (nvalue < value)
                            continue;

                        if (nvalue < closest)
                        {
                            x = lx;
                            y = ly;
                            z = lz;

                            closest = nvalue;

                            if (nvalue == value)
                                return;
                        }
                    }
                }
            }
        }

        #endregion

        private static void DebugLog(string format, params object[] parameters)
        {
#if UNITY_EDITOR
            UnityEngine.Debug.Log(string.Format(format, parameters));
#else
            Console.WriteLine(format, parameters);
#endif
        }
    }
}

