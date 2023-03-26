using System;
using System.Collections.Generic;
using System.Numerics;
using RunMobile.Utility;

namespace SDFTool
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
       /// <param name="lodDistance"></param>
       /// <param name="topLoduv"></param>
       /// <param name="zeroLodData"></param>
       /// <param name="boxArray"></param>
       /// <returns>cell count</returns>
        public static int ProcessCells(
            PixelData[] data, Vector3i dataSize,

            int topLodCellSize, int paddedTopLodCellSize,
            Vector3 lowerBound, Vector3 upperBound,
            int nlods,
            // TODO: return plain arrays, not Array3D
            out Array3D<ushort>[] lodDistance, out Array3D<ushort> topLoduv, out Array3D<ushort> zeroLodData,
            // TODO: return structure for box creation
            out MeshGenerator.Shape [] boxArray)
        {
            List<MeshGenerator.Shape> boxes = new List<MeshGenerator.Shape>();
            Dictionary<Vector3i, ValueTuple<int, float>[]> weightCache = new Dictionary<Vector3i, ValueTuple<int, float>[]>();

            int usedCells = 0;

            int cellsx = dataSize.X / topLodCellSize;
            int cellsy = dataSize.Y / topLodCellSize;
            int cellsz = dataSize.Z / topLodCellSize;

            int totalCells = cellsx * cellsy * cellsz;

            int blockSize = paddedTopLodCellSize;

            float[] cells = new float[totalCells];
            usedCells = CheckCells(data, dataSize, cellsx, cellsy, cellsz, topLodCellSize, blockSize, cells);

            int packx;
            int packy;
            int packz;
            Helper.FindBestDividers(usedCells + 1, out packx, out packy, out packz, 256);


#if LOD2_8BIT
            Array3D<byte>[] lodDistance = new Array3D<byte>[nlods];
            {
                int size = paddedTopLodCellSize;
                for (int i = 0; i < nlods; i++)
                {
                    lodDistance[i] = new Array3D<byte>(1, packx * size, packy * size, packz * size);
                    size /= 2;
                }
            }

            Array3D<byte> topLodDistance = lodDistance[0];
#else
            lodDistance = new Array3D<ushort>[nlods];
            {
                int size = paddedTopLodCellSize;
                for (int i = 0; i < nlods; i++)
                {
                    lodDistance[i] = new Array3D<ushort>(1, packx * size, packy * size, packz * size);
                    size /= 2;
                }
            }

            Array3D<ushort> topLodDistance = lodDistance[0];
#endif

            topLoduv = new Array3D<ushort>(2, packx * paddedTopLodCellSize, packy * paddedTopLodCellSize, packz * paddedTopLodCellSize);
            Array3D<byte> topLodTexture = new Array3D<byte>(4, packx * paddedTopLodCellSize, packy * paddedTopLodCellSize, packz * paddedTopLodCellSize);

            int brickId = 0;

#if LOD0_8BIT
            float packLodCoef = lod0pixels;
            Array3D<byte> zeroLodData = new Array3D<byte>(4, cellsx, cellsy, cellsz);
#else
            zeroLodData = new Array3D<ushort>(4, cellsx, cellsy, cellsz);
#endif
            Vector3 boxStep = (upperBound - lowerBound) / new Vector3(cellsx, cellsy, cellsz);

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

                            float[] distanceBlock = new float[blockSize * blockSize * blockSize];

                            Vector3i dataStart = new Vector3i(ix * topLodCellSize, iy * topLodCellSize, iz * topLodCellSize);

                            for (int z = 0; z < blockSize; z++)
                                for (int y = 0; y < blockSize; y++)
                                    for (int x = 0; x < blockSize; x++)
                                    {
                                        PixelData pixel = GetArrayData(data, dataSize, dataStart + new Vector3i(x, y, z));
                                        int blockIndex = x + y * blockSize + z * blockSize * blockSize;

                                        distanceBlock[blockIndex] = pixel.DistanceUV.X;

                                        topLoduv[atlasX * paddedTopLodCellSize + x, atlasY * paddedTopLodCellSize + y, atlasZ * paddedTopLodCellSize + z, 0] = Helper.PackFloatToUShort(pixel.DistanceUV.Y);
                                        topLoduv[atlasX * paddedTopLodCellSize + x, atlasY * paddedTopLodCellSize + y, atlasZ * paddedTopLodCellSize + z, 1] = Helper.PackFloatToUShort(pixel.DistanceUV.Z);
                                    }

                            float[] distancePercentageArr = GetTrilinear(distanceBlock, blockSize, 1, topLodCellSize / 2.0f, topLodCellSize / 2.0f, topLodCellSize / 2.0f);

                            distancePercentage = distancePercentageArr[0]; // central point


#if LOD0_8BIT
                            byte distByte = PackFloatToSByte(cell.Item2);
                            float dist = (distByte / 255.0f) * 2.0f - 1.0f;
#else
                            float dist = 0;// cell.Item2;
#endif
#if LOD2_8BIT
                            topLodDistance.PutBlock(distanceBlock, atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize, (k) => PackFloatToSByte((k - dist) * packLodCoef));
                            //topLodDistance.PutBlock(distanceBlock, atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize, (k) => PackFloatToSByte(k * (maximumDistance / step) / topLodCellSize));
#else
                            //dist = 0;
                            topLodDistance.PutBlock(distanceBlock, blockSize, blockSize, blockSize, 1, atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize, (k, l) => Helper.PackFloatToUShort(k - dist));
#endif

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
                            Matrix4x4 boxTextureMatrix = Matrix4x4.Identity;

                            ValueTuple<int, float>[][] boxBones = GetBoxWeights(weightCache, data, dataSize, dataStart, blockSize, ix, iy, iz);//, topLodCellSize * 0.25f);

                            Vector3 boxCenter = boxStart + boxStep / 2;

                            Matrix4x4 boxMatrix = Matrix4x4.CreateScale(boxStep) * Matrix4x4.CreateTranslation(boxStart);

                            boxes.Add(new MeshGenerator.Shape(
                                boxMatrix,
                                boxTextureMatrix,
                                MeshGenerator.ShapeType.Cube,
                                MeshGenerator.ShapeFlags.NoNormals,
                                new float[] {
                                    brickId
                                }, boxBones));
                        }

#if LOD0_8BIT
                        zeroLodData[ix, iy, iz, 0] = PackFloatToSByte(distancePercentage);

                        zeroLodData[ix, iy, iz, 1] = (byte)(atlasX);
                        zeroLodData[ix, iy, iz, 2] = (byte)(atlasY);
                        zeroLodData[ix, iy, iz, 3] = (byte)(atlasZ);
#else
                        zeroLodData[ix, iy, iz, 0] = (new HalfFloat(distancePercentage)).Data;
                        zeroLodData[ix, iy, iz, 1] = (new HalfFloat(atlasX)).Data;
                        zeroLodData[ix, iy, iz, 2] = (new HalfFloat(atlasY)).Data;
                        zeroLodData[ix, iy, iz, 3] = (new HalfFloat(atlasZ)).Data;
#endif
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

        private static T GetArrayData<T>(T[] data, Vector3i dataSize, Vector3i coord)
        {
            return data[coord.X + coord.Y * dataSize.X + coord.Z * dataSize.X * dataSize.Y];
        }
    }
}

