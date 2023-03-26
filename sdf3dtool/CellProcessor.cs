using System;
using System.Collections.Generic;
using System.Numerics;
using RunMobile.Utility;

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

    public static class CellProcessor
    {
        /// <summary>
        /// Splits SDF pixel data to bricks
        /// </summary>
        /// <param name="data">input data</param>
        /// <param name="dataSize">data size</param>
        /// <param name="topLodCellSize"></param>
        /// <param name="paddedTopLodCellSize"></param>
        /// <param name="lowerBound"></param>
        /// <param name="upperBound"></param>
        /// <param name="nlods"></param>
        /// <param name="distanceLods"></param>
        /// <param name="topLoduv"></param>
        /// <param name="zeroLodData"></param>
        /// <param name="boxArray"></param>
        /// <returns>cell count</returns>
        public static int ProcessCells(
            PixelData[] data, Vector3i dataSize,

            int topLodCellSize, int paddedTopLodCellSize,
            Vector3 lowerBound, Vector3 upperBound,
            int nlods,
            out Vector3i topLodTextureSize,
            out float[][] distanceLods, out Vector2[] topLoduv, out Vector4[] zeroLodData,
            out ValueTuple<Matrix4x4, int, ValueTuple<int, float>[][]>[] boxArray)
        {
            List<ValueTuple<Matrix4x4, int, ValueTuple<int, float>[][]>> boxes = new List<ValueTuple<Matrix4x4, int, ValueTuple<int, float>[][]>>();
            Dictionary<Vector3i, ValueTuple<int, float>[]> weightCache = new Dictionary<Vector3i, ValueTuple<int, float>[]>();

            int usedCells = 0;

            int cellsx = dataSize.X / topLodCellSize;
            int cellsy = dataSize.Y / topLodCellSize;
            int cellsz = dataSize.Z / topLodCellSize;

            int totalCells = cellsx * cellsy * cellsz;

            Vector3i blockSize = new Vector3i(paddedTopLodCellSize, paddedTopLodCellSize, paddedTopLodCellSize);

            float[] cells = new float[totalCells];
            usedCells = CheckCells(data, dataSize, cellsx, cellsy, cellsz, topLodCellSize, paddedTopLodCellSize, cells);

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

            Vector3 boxStep = (upperBound - lowerBound) / new Vector3(cellsx, cellsy, cellsz);

            int brickId = 0;

            int pxy = packx * packy;
            for (int iz = 0; iz < cellsz; iz++)
                for (int iy = 0; iy < cellsy; iy++)
                    for (int ix = 0; ix < cellsx; ix++)
                    {
                        int index = ix + iy * cellsx + iz * cellsx * cellsy;

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

                            Vector3i dataStart = new Vector3i(ix * topLodCellSize, iy * topLodCellSize, iz * topLodCellSize);

                            for (int z = 0; z < paddedTopLodCellSize; z++)
                                for (int y = 0; y < paddedTopLodCellSize; y++)
                                    for (int x = 0; x < paddedTopLodCellSize; x++)
                                    {
                                        Vector3i coord = new Vector3i(x, y, z);

                                        PixelData pixel = GetArrayData(data, dataSize, dataStart + coord);
#if USE_LODS
                                        // block for lower LOD calculation
                                        SetArrayData(distanceBlock, pixel.DistanceUV.X, blockSize, coord);
#endif
                                        // higher LOD
                                        SetArrayData(distanceLods[0], pixel.DistanceUV.X, topLodTextureSize, blockStart + coord);

                                        // texture UV coords
                                        SetArrayData(topLoduv, new Vector2(pixel.DistanceUV.Y, pixel.DistanceUV.Z), topLodTextureSize, blockStart + coord);
                                    }

                            float[] distancePercentageArr = GetTrilinear(distanceBlock, paddedTopLodCellSize, 1, topLodCellSize / 2.0f, topLodCellSize / 2.0f, topLodCellSize / 2.0f);

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

                            Vector3 boxStart = lowerBound + new Vector3(ix, iy, iz) * boxStep;

                            ValueTuple<int, float>[][] boxBones = GetBoxWeights(weightCache, data, dataSize, dataStart, topLodCellSize, ix, iy, iz);//, topLodCellSize * 0.25f);

                            Vector3 boxCenter = boxStart + boxStep / 2;

                            Matrix4x4 boxMatrix = Matrix4x4.CreateScale(boxStep) * Matrix4x4.CreateTranslation(boxStart);

                            boxes.Add(new ValueTuple<Matrix4x4, int, ValueTuple<int, float>[][]>(boxMatrix, brickId, boxBones));

                            //boxes.Add(new MeshGenerator.Shape(
                            //    boxMatrix,
                            //    boxTextureMatrix,
                            //    MeshGenerator.ShapeType.Cube,
                            //    MeshGenerator.ShapeFlags.NoNormals,
                            //    new float[] {
                            //        brickId
                            //    }, boxBones));
                        }


                        SetArrayData(zeroLodData, new Vector4(distancePercentage, atlasX, atlasY, atlasZ), new Vector3i(cellsx, cellsy, cellsz), new Vector3i(ix, iy, iz));
                    }

            boxArray = boxes.ToArray();

            return usedCells;
        }

        private static int CheckCells(PixelData[] data, Vector3i dataSize, int cellsx, int cellsy, int cellsz, int topLodCellSize, int blockSize, float[] cells)
        {
            int usedCells = 0;

            for (int iz = 0; iz < cellsz; iz++)
            {
                for (int iy = 0; iy < cellsy; iy++)
                {
                    for (int ix = 0; ix < cellsx; ix++)
                    {
                        Vector3i dataStart = new Vector3i(ix * topLodCellSize, iy * topLodCellSize, iz * topLodCellSize);

                        float minDistance = float.MaxValue;

                        int countPositive = 0;
                        int countNegative = 0;

                        for (int z = 0; z < blockSize; z++)
                            for (int y = 0; y < blockSize; y++)
                                for (int x = 0; x < blockSize; x++)
                                {
                                    PixelData pixel = GetArrayData(data, dataSize, dataStart + new Vector3i(x, y, z));

                                    float distance = pixel.DistanceUV.X;

                                    if (distance > 0)
                                        countPositive++;
                                    else
                                        if (distance < 0)
                                        countNegative++;

                                    if (Math.Abs(distance) < minDistance)
                                    {
                                        minDistance = Math.Abs(distance);
                                    }
                                }


                        if ((countPositive != 0 && countNegative != 0)) // all neg or all pos is ignored
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

        #region Interpolation and LODs

        private static readonly Vector3i[] s_boxIndices =
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

        private static ValueTuple<int, float>[][] GetBoxWeights(
            Dictionary<Vector3i, ValueTuple<int, float>[]> cache,
            PixelData[] data, Vector3i dataSize, Vector3i dataStart,
            int blockSize,
            int ix, int iy, int iz)
        {
            int ubx = blockSize - 1;
            int uby = blockSize - 1;
            int ubz = blockSize - 1;

            ValueTuple<int, float>[][] result = new ValueTuple<int, float>[8][];

            List<ValueTuple<Vector3, Vector4i, Vector4>>[] vertexWeights = new List<ValueTuple<Vector3, Vector4i, Vector4>>[8];

            for (int i = 0; i < 8; i++)
                vertexWeights[i] = new List<ValueTuple<Vector3, Vector4i, Vector4>>();

            for (int z = 0; z < blockSize - 1; z++)
                for (int y = 0; y < blockSize - 1; y++)
                    for (int x = 0; x < blockSize - 1; x++)
                    {
                        PixelData pixelData = GetArrayData(data, dataSize, dataStart + new Vector3i(x, y, z));

                        float dist = pixelData.DistanceUV.X;
                        int sign = Math.Sign(dist);

                        // checks if a surface crosses nearest 2x2x2 texel cube
                        if (Math.Abs(dist) <= 1 ||
                            sign != Math.Sign(GetArrayData(data, dataSize, dataStart + new Vector3i(x + 1, y + 0, z + 0)).DistanceUV.X) ||
                            sign != Math.Sign(GetArrayData(data, dataSize, dataStart + new Vector3i(x + 0, y + 1, z + 0)).DistanceUV.X) ||
                            sign != Math.Sign(GetArrayData(data, dataSize, dataStart + new Vector3i(x + 1, y + 1, z + 0)).DistanceUV.X) ||
                            sign != Math.Sign(GetArrayData(data, dataSize, dataStart + new Vector3i(x + 1, y + 0, z + 1)).DistanceUV.X) ||
                            sign != Math.Sign(GetArrayData(data, dataSize, dataStart + new Vector3i(x + 0, y + 1, z + 1)).DistanceUV.X) ||
                            sign != Math.Sign(GetArrayData(data, dataSize, dataStart + new Vector3i(x + 1, y + 1, z + 1)).DistanceUV.X) ||
                            sign != Math.Sign(GetArrayData(data, dataSize, dataStart + new Vector3i(x + 1, y + 1, z + 1)).DistanceUV.X)
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

                Vector3i index = new Vector3i(ix + s_boxIndices[i].X, iy + s_boxIndices[i].Y, iz + s_boxIndices[i].Z);

                ValueTuple<int, float>[] oldWeights;

                if (cache.TryGetValue(index, out oldWeights))
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

                            // old data has 25% of weigth
                            //oldValue.Item1 = oldValue.Item1 + pair.Item2 * 3;
                            //oldValue.Item2 = oldValue.Item2 * 2 + oldValue.Item2 * 2;
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

                cache[index] = oldWeights;
            }

            return result;
        }

        private static float[] GetTrilinear(float[] block, int blockSize, int components, float px, float py, float pz)
        {
            int ix = (int)px;
            int iy = (int)py;
            int iz = (int)pz;
            float frx = px - ix;
            float fry = py - iy;
            float frz = pz - iz;

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

            Vector3i[] shifts =
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


            float[] result = new float[components];
            for (int i = 0; i < coef.Length; i++)
            {
                Vector3i ip = new Vector3i(ix + shifts[i].X, iy + shifts[i].Y, iz + shifts[i].Z);

                if (ip.X >= blockSize || ip.Y >= blockSize || ip.Z >= blockSize)
                    continue;

                for (int c = 0; c < components; c++)
                    result[c] += coef[i] * GetArrayData(block, new Vector3i(blockSize, blockSize, blockSize), ip);
            }
            return result;
        }

        private static float[] GenerateLod(float[] topLodDistance, int blockSize, int lodSize)
        {
            float[] result = new float[lodSize * lodSize * lodSize];

            float step = (blockSize - 1.0f) / (lodSize - 1.0f);

            for (int nz = 0; nz < lodSize; nz++)
            {
                for (int ny = 0; ny < lodSize; ny++)
                {
                    for (int nx = 0; nx < lodSize; nx++)
                    {
                        float[] value = GetTrilinear(topLodDistance, blockSize, 1, nx * step, ny * step, nz * step);

                        result[nx + ny * lodSize + nz * lodSize * lodSize] = value[0];
                    }
                }
            }

            return result;
        }

        #endregion

#region Utils

        private static T GetArrayData<T>(T[] array, Vector3i dataSize, Vector3i coord)
        {
            return array[coord.X + coord.Y * dataSize.X + coord.Z * dataSize.X * dataSize.Y];
        }

        private static void SetArrayData<T>(T[] array, T data, Vector3i dataSize, Vector3i coord)
        {
            array[coord.X + coord.Y * dataSize.X + coord.Z * dataSize.X * dataSize.Y] = data;
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
    }
}

