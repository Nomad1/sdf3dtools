using System;
using System.Collections.Generic;
using System.Numerics;
using static RunServer.SdfTool.CellProcessor;
using static RunServer.SdfTool.Utils;

namespace RunServer.SdfTool
{
    public class Bricks
    {
        /// <summary>
        /// This method analyzes the pixel data and tries to find biggest possible bricks with some quality loss
        /// </summary>
        /// <param name="data"></param>
        /// <param name="dataSize"></param>
        /// <param name="minCellSize"></param>
        /// <param name="minPsnr"></param>
        /// <param name="cells"></param>
        internal static void FindBricks(
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
                            if (Utils.GetArrayData(usedCells, minCellDimensions, new Vector3i(ix * ncells, iy * ncells, iz * ncells)))
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
                                        PixelData pixel = Utils.GetArrayData(data, dataSize, dataStart + new Vector3i(x, y, z));

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

                                        Utils.SetArrayData(cellData, pixel, cellDimensions, new Vector3i(x, y, z));
                                    }

                            if (countPositive == 0 || countNegative == 0) // all neg or all pos is ignored
                            {
                                if (s > 1)
                                    for (int cz = 0; cz < ncells; cz++)
                                        for (int cy = 0; cy < ncells; cy++)
                                            for (int cx = 0; cx < ncells; cx++)
                                            {
                                                Vector3i cellPos = new Vector3i(ix * ncells + cx, iy * ncells + cy, iz * ncells + cz);

                                                Utils.SetArrayData(usedCells, true, minCellDimensions, cellPos);
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

                                PixelData[] reducedCellData = Utils.GenerateLod(cellData, cellSize + 1, minCellSize + 1);
                                PixelData[] enlargedCellData = Utils.GenerateLod(reducedCellData, minCellSize + 1, cellSize + 1);

                                if (BricksAreSimilar(cellData, enlargedCellData, minPsnr))
                                    continue;

                                brick = reducedCellData;

                                // mark data as already used

                                for (int cz = 0; cz < ncells; cz++)
                                    for (int cy = 0; cy < ncells; cy++)
                                        for (int cx = 0; cx < ncells; cx++)
                                        {
                                            Vector3i cellPos = new Vector3i(ix * ncells + cx, iy * ncells + cy, iz * ncells + cz);

                                            Utils.SetArrayData(usedCells, true, minCellDimensions, cellPos);
                                        }

                            }

                            // add result to `cells`

                            cellCount++;
                            cells.Add(new BrickData(cellCount, dataStart, cellSize, brick));
                        }
                    }

                }

                Utils.DebugLog("Found {0} cells of size {1}x{1}x{1}", cellCount, cellSize + 1);

                cellSize /= 2;
            }
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
                            SetArrayData(topLodDistances, pixel.DistanceUV.X /* data.CellSize*/ / brick.CellSize, topLodTextureSize, blockStart + coord);

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
    }
}

