using System;
using System.Collections.Generic;
using System.Numerics;
using static RunServer.SdfTool.CellProcessor;
using static RunServer.SdfTool.Utils;

namespace RunServer.SdfTool
{
    public static class Cells
    {
        /// <summary>
        /// Marks cells where distance changes from negative to positive and vice versa
        /// </summary>
        /// <param name="data"></param>
        /// <param name="dataSize"></param>
        /// <param name="cellSize"></param>
        /// <param name="cells"></param>
        /// <returns>number of cells</returns>
        internal static int CheckCells(PixelData[] data, Vector3i dataSize, int cellSize, float[] cells)
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
                                    PixelData pixel = Utils.GetArrayData(data, dataSize, dataStart + new Vector3i(x, y, z));

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
    }
}

