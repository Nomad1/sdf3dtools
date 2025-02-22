using System;
using System.Collections.Generic;
using System.Numerics;

namespace RunServer.SdfTool
{
    
    public static class Utils
    {
        public static void DebugLog(string format, params object[] parameters)
        {
#if UNITY_EDITOR
            UnityEngine.Debug.Log(string.Format(format, parameters));
#else
            Console.WriteLine(format, parameters);
#endif
        }

        public static T GetArrayData<T>(T[] array, Vector3i dataSize, Vector3i coord, int shift = 0)
        {
            return array[coord.X + coord.Y * dataSize.X + coord.Z * dataSize.X * dataSize.Y + shift];
        }

        public static void SetArrayData<T>(T[] array, T data, Vector3i dataSize, Vector3i coord, int shift = 0)
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

        public static Vector3i Unpack(int i, Vector3i pack)
        {
            int pxy = pack.X * pack.Y;


            int atlasZ = ((i / pxy));
            int atlasY = ((i % pxy) / pack.X);
            int atlasX = ((i % pxy) % pack.X);

            return new Vector3i(atlasX, atlasY, atlasZ);
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

        public static float[][] GetCubeDistances(
            PixelData[] data, Vector3i dataSize, Vector3i dataStart,
            int blockSize)
        {
            float[][] vertexDistances = new float[8][];

            for (int i = 0; i < vertexDistances.Length; i++)
                vertexDistances[i] = new float[] { GetArrayData(data, dataSize, dataStart + s_cubeIndices[i] * blockSize).DistanceUV.X };

            return vertexDistances;
        }

        public static ValueTuple<int, float>[][] GetCubeWeights(
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
        public static float[] GetTrilinear(float[] block, int blockSize, int stride, Vector3 coord)
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

        public static float[] GenerateLod(float[] sourceData, int sourceSize, int targetSize)
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

        public static PixelData[] GenerateLod(PixelData[] sourceData, int sourceSize, int targetSize)
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

    }
}

