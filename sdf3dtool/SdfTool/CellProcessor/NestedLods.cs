using System;
using System.Collections.Generic;
using System.Numerics;
using static RunServer.SdfTool.CellProcessor;
using static RunServer.SdfTool.Utils;

namespace RunServer.SdfTool
{
    public static class NestedLods
    {
        public static int ProcessBricksWithNestedLods(
             DistanceData data,
             int baseCellSize,
             bool processWeights,
             float minPsnr,
             out LodData[] lods
             )
        {
            int cellSize = data.CellSize;
            if (cellSize != 4)
            {
                throw new Exception("Cell size " + cellSize + " is not supported at the moment!");
            }

            Dictionary<Vector3i, ValueTuple<int, float>[]> weightCache = new Dictionary<Vector3i, ValueTuple<int, float>[]>();

            int cellsx = data.Size.X / baseCellSize;
            int cellsy = data.Size.Y / baseCellSize;
            int cellsz = data.Size.Z / baseCellSize;

            int paddedTopLodCellSize = baseCellSize + 1;

            Vector3i blockSize = new Vector3i(paddedTopLodCellSize, paddedTopLodCellSize, paddedTopLodCellSize);

            int lodLevels = (data.LodData.Length - 1) / 2 + 1;

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

                Dictionary<int, int> readyBricks = new Dictionary<int, int>();

                foreach (var brick in resultBricks[lod - 1])
                {
                    Vector3i position = brick.Position;

                    for (int z = 0; z < cellSize + 1; z++)
                        for (int y = 0; y < cellSize + 1; y++)
                            for (int x = 0; x < cellSize + 1; x++)
                            {
                                Vector3i brickPos = (position + new Vector3i(x, y, z)) * cellSize;
                                int index = brickPos.X + brickPos.Y * lodData[lod].Item2.X + brickPos.Z * lodData[lod].Item2.X * lodData[lod].Item2.Y;

                                int readyBrick;
                                if (readyBricks.TryGetValue(index, out readyBrick))
                                {
                                    brick.Children[x + y * (cellSize + 1) + z * (cellSize + 1) * (cellSize + 1)] = readyBrick;
                                    continue;
                                }

                                PixelData[] newBrick = CheckBrick(lodData[lod].Item1, brickPos, lodData[lod].Item2, cellSize);

                                if (newBrick == null)
                                    continue;

                                int cs = cellSize + 1;

                                // Get corners of 3D cube (x,y,z coordinates)
                                PixelData[] reducedCellData =
                                {
                                    // Origin corner (0,0,0)
                                    newBrick[0],
                                    // Corner (0,cs-1,0)
                                    newBrick[(cs-1)],
                                    // Corner (cs-1,0,0)  
                                    newBrick[(cs-1) * cs],
                                    // Corner (cs-1,cs-1,0)
                                    newBrick[(cs-1) + (cs-1) * cs],
                                    // Corner (0,0,cs-1)
                                    newBrick[(cs-1) * cs * cs],
                                    // Corner (0,cs-1,cs-1)
                                    newBrick[(cs-1) + (cs-1) * cs * cs],
                                    // Corner (cs-1,0,cs-1)
                                    newBrick[(cs-1) * cs + (cs-1) * cs * cs],
                                    // Corner (cs-1,cs-1,cs-1)
                                    newBrick[(cs-1) + (cs-1) * cs + (cs-1) * cs * cs]
                                };

                                PixelData[] enlargedCellData = GenerateLod(reducedCellData, 2, cs);

                                if (BricksAreSimilar(newBrick, enlargedCellData, minPsnr, (1 << (lod - 1)) / (double)Math.Max(Math.Max(cellsx, cellsy), cellsz))) // there is no reason to use more detailed brick here
                                {
                                    brick.Children[x + y * (cellSize + 1) + z * (cellSize + 1) * (cellSize + 1)] = 0xffffff;
                                    readyBricks[index] = 0xffffff;
                                    continue;
                                }

                                int id = ++totalBricks;
                                resultBricks[lod].Add(new BrickData(id, brickPos, cellSize, newBrick, new int[(cellSize + 1) * (cellSize + 1) * (cellSize + 1)]));

                                brick.Children[x + y * (cellSize + 1) + z * (cellSize + 1) * (cellSize + 1)] = id;
                                readyBricks[index] = id;
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
            FindBestDividers(allBricks.Count + 1, out packx, out packy, out packz, 256);
            Vector3i pack = new Vector3i(packx, packy, packz);

            Vector3 boxStep = (data.UpperBound - data.LowerBound) / (new Vector3(cellsx, cellsy, cellsz) * baseCellSize);

            lods = new LodData[1];

            {
                DebugLog("Generating LOD1 boxes");

                BrickCellData[] lodBoxes = new BrickCellData[resultBricks[0].Count + 1];

                foreach (BrickData brick in resultBricks[0])
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
                                SetArrayData(lodDistances, pixel.DistanceUV.X /* data.CellSize*/ / brick.CellSize, lodTextureSize, blockStart + coord);

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
    }
}

