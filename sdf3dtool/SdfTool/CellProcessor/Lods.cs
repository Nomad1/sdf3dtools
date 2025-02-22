using System;
using System.Collections.Generic;
using System.Numerics;
using static RunServer.SdfTool.CellProcessor;
using static RunServer.SdfTool.Utils;

namespace RunServer.SdfTool
{
    public static class Lods
    {
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
                                SetArrayData(lodDistances, pixel.DistanceUV.X /* data.CellSize*/ / brick.CellSize, lodTextureSize, blockStart + coord);

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
    }
}

