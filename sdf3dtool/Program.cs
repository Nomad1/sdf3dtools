﻿#define LOD0_8BIT
#define LOD2_8BIT

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading.Tasks;
using Assimp;
using Assimp.Configs;
using RunMobile.Utility;

using Vector = System.Numerics.Vector3;

namespace SDFTool
{
    static class Program
    {
        private static readonly string s_syntax = "Syntax: {0} input.dae output.ktx [grid_cells] [lod_0_size] [lod_1_cell_size]\n" +
        "\n";

        private static void ProcessAssimpImport(string fileName, string outFile, string textureFile = null, int gridCellCount = 64, int lod0pixels = 32, int lod1cellSize = 4)
        {
            Debug.Assert(lod1cellSize % 2 != 0, "Lod 1 cell size should be even!");

            Stopwatch sw = new Stopwatch();
            sw.Start();

            Console.WriteLine("[{0}] Processing file {1}", sw.Elapsed, fileName);

            AssimpContext importer = new AssimpContext();
            importer.SetConfig(new ACSeparateBackfaceCullConfig(false));
            Scene scene = importer.ImportFile(fileName, PostProcessPreset.TargetRealTimeMaximumQuality);

            Console.WriteLine("[{0}] File loaded", sw.Elapsed);

            int textureWidth = 0;
            int textureHeight = 0;
            int[] texture = textureFile == null ? null : Helper.LoadBitmap(textureFile, out textureWidth, out textureHeight);

            float scale = 1;

            Vector sceneMin = new Vector(float.MaxValue, float.MaxValue, float.MaxValue);
            Vector sceneMax = new Vector(float.MinValue, float.MinValue, float.MinValue);
            Matrix4x4 matrix = Matrix4x4.FromScaling(new Vector3D(scale));
            List<PreparedTriangle> triangleList = new List<PreparedTriangle>();

            List<Tuple<Vector, Vector, Bone>> bones = new List<Tuple<Vector, Vector, Bone>>();

            Preprocess(scene, scene.RootNode, matrix, triangleList, bones);

            //List<float> xvalues = new List<float>();
            //List<float> yvalues = new List<float>();
            //List<float> zvalues = new List<float>();

            foreach (PreparedTriangle triangle in triangleList)
            {
                //xvalues.Add(triangle.UpperBound.X - triangle.LowerBound.X);
                //yvalues.Add(triangle.UpperBound.Y - triangle.LowerBound.Y);
                //zvalues.Add(triangle.UpperBound.Z - triangle.LowerBound.Z);

                sceneMax.X = Math.Max(sceneMax.X, triangle.UpperBound.X);
                sceneMax.Y = Math.Max(sceneMax.Y, triangle.UpperBound.Y);
                sceneMax.Z = Math.Max(sceneMax.Z, triangle.UpperBound.Z);
                sceneMin.X = Math.Min(sceneMin.X, triangle.LowerBound.X);
                sceneMin.Y = Math.Min(sceneMin.Y, triangle.LowerBound.Y);
                sceneMin.Z = Math.Min(sceneMin.Z, triangle.LowerBound.Z);
            }

            //xvalues.Sort();
            //yvalues.Sort();
            //zvalues.Sort();

            //// use median triangle size as a step
            //float step = Math.Min(Math.Min(xvalues[xvalues.Count / 2], yvalues[yvalues.Count / 2]), zvalues[zvalues.Count / 2]);

            //if ((sceneMax.Z - sceneMin.Z) / step > 256)
            //    step = (sceneMax.Z - sceneMin.Z) / 256;

            //step *= stepScale;

            float maxSide = Math.Max(Math.Max(sceneMax.Z - sceneMin.Z, sceneMax.Y - sceneMin.Y), sceneMax.X - sceneMin.X);

            // maximum pixel size of lod 0 in any dimension

            // lod 0 cell size is 1
            int lod2cellSize = lod1cellSize * 2;

            // number of extra pixels on the each size
            int lod0padding = 1;
            int lod1padding = lod0padding * lod1cellSize;
            int lod2padding = lod0padding * lod2cellSize;


            float step = maxSide / ((lod0pixels - lod0padding * 2) * lod1cellSize * 2); // step for lod 2


            //creating grid structure for faster triangle search
            TriangleMap triangleMap = new TriangleMap(sceneMin, sceneMax, gridCellCount, triangleList);

            Console.WriteLine("Bounding box: {0} - {1}, step {2}, triangles {3}, cells {4}, instances {5}", sceneMin, sceneMax, step, triangleMap.TriangleCount, triangleMap.CellsUsed, triangleMap.TriangleInstances);

            int sx = (int)Math.Ceiling(sceneMax.X / step) - (int)Math.Floor(sceneMin.X / step) + lod2padding * 2;
            int sy = (int)Math.Ceiling(sceneMax.Y / step) - (int)Math.Floor(sceneMin.Y / step) + lod2padding * 2;
            int sz = (int)Math.Ceiling(sceneMax.Z / step) - (int)Math.Floor(sceneMin.Z / step) + lod2padding * 2;

            if (sx % lod2cellSize != 0)
                sx += lod2cellSize - sx % lod2cellSize;
            sx++;
            if (sy % lod2cellSize != 0)
                sy += lod2cellSize - sy % lod2cellSize;
            sy++;
            if (sz % lod1cellSize != 0)
                sz += lod2cellSize - sz % lod2cellSize;
            sz++;

            Vector lowerBound = new Vector(-step * lod2padding) + sceneMin;
            Vector upperBound = new Vector(step * lod2padding) + sceneMax; // new Vector(sx, sy, sz) * step + lowerBound;

            float maximumDistance =
                //Math.Max(Math.Max(sceneMax.X - sceneMin.X, sceneMax.Y - sceneMin.Y), sceneMax.Z - sceneMin.Z);
                Vector.Distance(sceneMax, sceneMin);

            Console.WriteLine("[{0}] File preprocessed. X: {1}, Y: {2}, Z: {3}", sw.Elapsed, sx, sy, sz);

            float emptyCellCheckDistance = step * 1.73205080757f / maximumDistance;
            float emptyCellDistance = step * 0.5f / maximumDistance;
            //step * 0.5f / maximumDistance;// * 0.5f;// (float)Math.Sqrt(2) * 0.5f;// cellSize * step * 0.05f;// * (float)Math.Sqrt(2) * 0.5f;

            Dictionary<int, List<ValueTuple<int, float>>> tempBoneDictionary = new Dictionary<int, List<ValueTuple<int, float>>>();

            for (int b = 0; b < bones.Count; b++)
            {
                Tuple<Vector, Vector, Bone> btuple = bones[b];

                for (int j = 0; j < btuple.Item3.VertexWeightCount; j++)
                {
                    int vid = btuple.Item3.VertexWeights[j].VertexID; // vertex id
                    int bid = b + 1; // bone id
                    float weight = btuple.Item3.VertexWeights[j].Weight; // weight

                    List<ValueTuple<int, float>> list;

                    if (!tempBoneDictionary.TryGetValue(vid, out list))
                    {
                        list = new List<ValueTuple<int, float>>();
                        tempBoneDictionary.Add(vid, list);
                    }

                    list.Add(new ValueTuple<int, float>(bid, weight));
                }
            }

            Dictionary<int, ValueTuple<int, float>[]> boneDictionary = new Dictionary<int, ValueTuple<int, float>[]>();

            foreach (var pair in tempBoneDictionary)
                boneDictionary[pair.Key] = pair.Value.ToArray();

            Array3D<float> data = new Array3D<float>(6, sx, sy, sz); // we are using lod 2 data for processing

            Iterate(0, sz * sy * sx, (i) =>
            {
                int iz = i / (sx * sy);
                int iy = (i % (sx * sy)) / sx;
                int ix = (i % (sx * sy)) % sx;

                if (ix == 0 && iy == 0)
                    Console.WriteLine("[{0}] Processing {1}", sw.Elapsed, new Vector(ix, iy, iz));

#if DEBUG_IMAGES
                byte[] testData = new byte[sx * sy * 4];
#endif
                Vector point = lowerBound + new Vector(ix, iy, iz) * step;

                float distance;
                Vector triangleWeights;
                PreparedTriangle triangle;

                float distancePercentage;

                bool empty = !triangleMap.FindTriangles(point, out distance, out triangleWeights, out triangle);

                distancePercentage = Math.Sign(distance) * Math.Min(Math.Abs(distance / maximumDistance), 1.0f);

                // the distance
                //lock (data)
                {
                    data[ix, iy, iz, 0] = distancePercentage;

#if DEBUG_IMAGES
                        // temporary test images. I had to scale the distance 4x to make them visible. Not sure it would work
                        // for all the meshes
                        testData[(ix + iy * sx) * 4] = distancePercentage > 0 ? (byte)(1024.0f * distancePercentage) : (byte)0;
                        testData[(ix + iy * sx) * 4 + 1] = distancePercentage < 0 ? (byte)(-1024.0f * distancePercentage) : (byte)0;
                        //testData[(ix + iy * sx) * 4 + 2] = (byte)(triangleData == null ? 128 : 0);
#endif

                    if (triangle != null)
                    {
                        // Saved triangle data
                        Tuple<Mesh, Face> tuple = (Tuple<Mesh, Face>)triangle.Data;
                        Mesh mesh = tuple.Item1;
                        Face face = tuple.Item2;


                        if (mesh.TextureCoordinateChannelCount > 0 && mesh.TextureCoordinateChannels[0].Count > 0)
                        {
                            Vector3D tca = mesh.TextureCoordinateChannels[0][face.Indices[0]];
                            Vector3D tcb = mesh.TextureCoordinateChannels[0][face.Indices[1]];
                            Vector3D tcc = mesh.TextureCoordinateChannels[0][face.Indices[2]];

                            Vector3D tc = tca * triangleWeights.X + tcb * triangleWeights.Y + tcc * triangleWeights.Z;

                            if (tc.X < 0 || tc.Y < 0 || tc.X > 1 || tc.Y > 1 || tc.Z != 0)
                                Console.WriteLine("Result weights are invalid: {0}!", tc);

                            // texture coords
                            data[ix, iy, iz, 1] = tc.X;
                            data[ix, iy, iz, 2] = tc.Y;
                        }

                        //Vector.Dot(triangle.Normal, point -
                        Vector normal = triangle.Normal;

                        Dictionary<int, float> topBones = new Dictionary<int, float>(8);

                        //float[] mainBones = new float[bones.Count];

                        float maxWeight = 0;
                        int maxBoneId = 0;

                        for (int k = 0; k < face.IndexCount; k++)
                        {
                            ValueTuple<int, float>[] list;
                            if (boneDictionary.TryGetValue(face.Indices[k], out list))
                            {
                                foreach (var pair in list)
                                {
                                    float weight;
                                    topBones.TryGetValue(pair.Item1, out weight);
                                    weight += pair.Item2;

                                    topBones[pair.Item1] = weight;

                                    if (weight > maxWeight)
                                    {
                                        maxWeight = weight;
                                        maxBoneId = pair.Item1;
                                    }
                                }
                            }
                        }

                        data[ix, iy, iz, 3] = normal.X;
                        data[ix, iy, iz, 4] = normal.Y; // no need for Z, it can be easily calculated
                        data[ix, iy, iz, 5] = maxBoneId;

                        //data[ix,iy,iz,3]

                        // TODO: 3,4,5,6 are RGBA colors
                        // TODO: indices 7+ should be weights for the bones and bone numbers



                        //float uvdist = Math.Min(Math.Min(triangleWeights.X, triangleWeights.Y), triangleWeights.Z);
                        //data[ix, iy, iz, 3] = 0.5f - uvdist;
                    }
                }

#if DEBUG_IMAGES
                if (ix == sx - 1 && iy == sy - 1)
                    Helper.SaveBitmap(testData, sx, sy, Path.GetFileNameWithoutExtension(outFile) + "_" + iz);
#endif
            }
            );

            // split to cells

            int usedCells = 0;
            int cellsx = (int)Math.Floor(sx / (float)(lod2cellSize));
            int cellsy = (int)Math.Floor(sy / (float)(lod2cellSize));
            int cellsz = (int)Math.Floor(sz / (float)(lod2cellSize));

            int totalCells = cellsx * cellsy * cellsz;
            int paddedLod1cellSize = lod1cellSize + 1;
            int paddedLod2cellSize = lod2cellSize + 1;


            Tuple<float, Array3D<float>>[] cells = new Tuple<float, Array3D<float>>[totalCells];
            //List<Tuple<float, float>> values = new List<Tuple<float, float>>(paddedCellSize * paddedCellSize * paddedCellSize);

            for (int iz = 0; iz < cellsz; iz++)
                for (int iy = 0; iy < cellsy; iy++)
                    for (int ix = 0; ix < cellsx; ix++)
                    {
                        int index = ix + iy * cellsx + iz * cellsx * cellsy;

                        Array3D<float> block = data.GetBlock(ix * lod2cellSize, iy * lod2cellSize, iz * lod2cellSize, paddedLod2cellSize, paddedLod2cellSize, paddedLod2cellSize, new float[] { float.MaxValue, 0, 0, 0, 0, 0, 0, 0, 0, 0 });

                        float minDistance = float.MaxValue;

                        for (int z = 0; z < block.Depth; z++)
                            for (int y = 0; y < block.Height; y++)
                                for (int x = 0; x < block.Width; x++)
                                //for (int i = 0; i < block.Data.Length; i += block.Components)
                                {
                                    float distance = block[x, y, z, 0];

                                    if (Math.Abs(distance) < Math.Abs(minDistance))
                                        minDistance = distance;
                                }
                                    /*
                                    for (int i = 0; i < block.Data.Length; i += block.Components)
                        {
                            float distance = block.Data[i];

                            if (Math.Abs(distance) < Math.Abs(minDistance))
                                minDistance = distance;
                        }*/

                        for (int i = 0; i < block.Data.Length; i += block.Components)
                            if (block[i] == float.MaxValue)
                                block[i] = minDistance;

                        float distancePercentage = block[paddedLod2cellSize / 2, paddedLod2cellSize / 2, paddedLod2cellSize / 2, 0]; // central point

                        if (Math.Abs(minDistance) < emptyCellCheckDistance)
                        {
                            usedCells++;
                        }
                        else
                        {
                            block = null;
                        }

                        cells[index] = new Tuple<float, Array3D<float>>(distancePercentage, block);
                    }


            int packx;
            int packy;
            int packz;
            FindBestDividers(usedCells + 1, out packx, out packy, out packz, 256);

            Array3D<byte> lod1distance = new Array3D<byte>(1, packx * paddedLod1cellSize, packy * paddedLod1cellSize, packz * paddedLod1cellSize);
#if LOD2_8BIT
            Array3D<byte> lod2distance = new Array3D<byte>(1, packx * paddedLod2cellSize, packy * paddedLod2cellSize, packz * paddedLod2cellSize);
#else
            Array3D<ushort> lod2distance = new Array3D<ushort>(1, packx * paddedLod2cellSize, packy * paddedLod2cellSize, packz * paddedLod2cellSize);
#endif

            Array3D<ushort> lod0uv = new Array3D<ushort>(2, cellsx, cellsy, cellsz);
            Array3D<ushort> lod1uv = new Array3D<ushort>(2, packx * paddedLod1cellSize, packy * paddedLod1cellSize, packz * paddedLod1cellSize);
            Array3D<ushort> lod2uv = new Array3D<ushort>(2, packx * paddedLod2cellSize, packy * paddedLod2cellSize, packz * paddedLod2cellSize);

            Array3D<byte> lod2texture = new Array3D<byte>(4, packx * paddedLod2cellSize, packy * paddedLod2cellSize, packz * paddedLod2cellSize);
            Array3D<byte> lod2normal = new Array3D<byte>(3, packx * paddedLod2cellSize, packy * paddedLod2cellSize, packz * paddedLod2cellSize);

            Console.WriteLine("[{0}] Got {1} empty cells, cell grid size {2}, {3:P}, total {4} of {5}x{5}x{5} cells, size {6} vs {7}, grid {8}x{9}x{10}",
                sw.Elapsed, totalCells - usedCells, new Vector3i(cellsx, cellsy, cellsz), usedCells / (float)totalCells,
                usedCells,
                paddedLod1cellSize,
                lod1distance.Bytes,
                data.Bytes,
                packx, packy, packz
                );
            //Console.WriteLine("[{0}] SDF finished, saving KTX", sw.Elapsed);

            //Helper.SaveKTX(Helper.KTX_RGBA16F, data.Width, data.Height, data.Depth, data.Data, outFile);

            Console.WriteLine("[{0}] Full size KTX saved, saving lower LOD", sw.Elapsed);

            //ushort[] partialData = new ushort[paddedCellSize * paddedCellSize * paddedCellSize * usedCells.Count * 4];
            int partialCells = 0;

            float packLod1Coef = lod0pixels;
            float packLod2Coef = lod0pixels;

#if LOD0_8BIT
            Array3D<byte> zeroLodData = new Array3D<byte>(4, cellsx, cellsy, cellsz);
#else
            Array3D<ushort> zeroLodData = new Array3D<ushort>(4, cellsx, cellsy, cellsz);
#endif
            for (int iz = 0; iz < cellsz; iz++)
                for (int iy = 0; iy < cellsy; iy++)
                    for (int ix = 0; ix < cellsx; ix++)
                    {
                        int index = ix + iy * cellsx + iz * cellsx * cellsy;

                        int atlasX = 0;
                        int atlasY = 0;
                        int atlasZ = 0;

                        Tuple<float, Array3D<float>> cell = cells[index];

                        if (cell.Item2 != null)
                        {
                            // Add used cell to 3D atlas TODO: move this code to separate function
                            partialCells++;

                            if (partialCells >= usedCells + 1)
                                throw new Exception("Too many cells for partial LOD: " + partialCells + "!");

                            atlasZ = ((partialCells / (packy * packx)));
                            atlasY = ((partialCells % (packy * packx)) / packx);
                            atlasX = ((partialCells % (packy * packx)) % packx);


                            if (atlasX >= 256 || atlasY >= 256 || atlasZ >= 256)
                                throw new Exception("Too big atlas index for partial LOD: " + partialCells + "!");

                            Array3D<float> lod1distanceBlock = new Array3D<float>(1, paddedLod1cellSize, paddedLod1cellSize, paddedLod1cellSize);
                            Array3D<float> lod2distanceBlock = new Array3D<float>(1, paddedLod2cellSize, paddedLod2cellSize, paddedLod2cellSize);

                            Array3D<float> lod1uvBlock = new Array3D<float>(2, paddedLod1cellSize, paddedLod1cellSize, paddedLod1cellSize);
                            Array3D<float> lod2uvBlock = new Array3D<float>(2, paddedLod2cellSize, paddedLod2cellSize, paddedLod2cellSize);

                            Array3D<byte> lod2textureBlock = new Array3D<byte>(4, paddedLod2cellSize, paddedLod2cellSize, paddedLod2cellSize);
                            Array3D<byte> lod2normalBlock = new Array3D<byte>(3, paddedLod2cellSize, paddedLod2cellSize, paddedLod2cellSize);

                            for (int z = 0; z < cell.Item2.Depth; z++)
                                for (int y = 0; y < cell.Item2.Height; y++)
                                    for (int x = 0; x < cell.Item2.Width; x++)
                                    {
                                        float distance = cell.Item2[x, y, z, 0];
                                        float u = cell.Item2[x, y, z, 1];
                                        float v = cell.Item2[x, y, z, 2];

                                        lod2uvBlock[x, y, z, 0] = u;
                                        lod2distanceBlock[x, y, z, 0] = distance;
                                        lod2uvBlock[x, y, z, 1] = v;

                                        if (z % 2 == 0 && y % 2 == 0 && x % 2 == 0) // half the resolution
                                        {
                                            lod1distanceBlock[x / 2, y / 2, z / 2, 0] = distance;
                                            lod1uvBlock[x / 2, y / 2, z / 2, 0] = u;
                                            lod1uvBlock[x / 2, y / 2, z / 2, 1] = v;
                                        }

                                        if (texture != null)
                                        {
                                            int textureColor = texture != null ? texture[(int)(Clamp(u * textureWidth, 0.0f, textureWidth - 1)) + ((int)Clamp((1.0f - v) * textureHeight, 0.0f, textureHeight - 1)) * textureWidth] : 0;

                                            lod2textureBlock[x, y, z, 0] = (byte)(textureColor >> 16);
                                            lod2textureBlock[x, y, z, 1] = (byte)(textureColor >> 8);
                                            lod2textureBlock[x, y, z, 2] = (byte)(textureColor);
                                            lod2textureBlock[x, y, z, 3] = (byte)(textureColor >> 24);
                                        }

                                        lod2normalBlock[x, y, z, 0] = PackFloatToSByte(cell.Item2[x, y, z, 3]);
                                        lod2normalBlock[x, y, z, 1] = PackFloatToSByte(cell.Item2[x, y, z, 4]);
                                        lod2normalBlock[x, y, z, 2] = (byte)cell.Item2[x, y, z, 5];
                                    }
#if LOD0_8BIT
                            byte distByte = PackFloatToSByte(cell.Item1);
                            float dist = (distByte / 255.0f) * 2.0f - 1.0f;
#else
                            float dist = cell.Item1;
#endif
                            lod1distance.PutBlock(lod1distanceBlock, atlasX * paddedLod1cellSize, atlasY * paddedLod1cellSize, atlasZ * paddedLod1cellSize, (k) => PackFloatToSByte((k - dist) * packLod1Coef));
#if LOD2_8BIT
                            lod2distance.PutBlock(lod2distanceBlock, atlasX * paddedLod2cellSize, atlasY * paddedLod2cellSize, atlasZ * paddedLod2cellSize, (k) => PackFloatToSByte((k - dist) * packLod2Coef));
#else
                            lod2distance.PutBlock(lod2distanceBlock, atlasX * paddedLod2cellSize, atlasY * paddedLod2cellSize, atlasZ * paddedLod2cellSize, (k) => PackFloatToUShort((k - dist) * packLod2Coef/* * 0.5f + 0.5f*/));
#endif

                            lod1uv.PutBlock(lod1uvBlock, atlasX * paddedLod1cellSize, atlasY * paddedLod1cellSize, atlasZ * paddedLod1cellSize, PackFloatToUShort);
                            lod2uv.PutBlock(lod2uvBlock, atlasX * paddedLod2cellSize, atlasY * paddedLod2cellSize, atlasZ * paddedLod2cellSize, PackFloatToUShort);

                            if (texture != null)
                                lod2texture.PutBlock(lod2textureBlock, atlasX * paddedLod2cellSize, atlasY * paddedLod2cellSize, atlasZ * paddedLod2cellSize);

                            lod2normal.PutBlock(lod2normalBlock, atlasX * paddedLod2cellSize, atlasY * paddedLod2cellSize, atlasZ * paddedLod2cellSize);

                            lod0uv[ix, iy, iz, 0] = PackFloatToUShort(cell.Item2[paddedLod2cellSize / 2, paddedLod2cellSize / 2, paddedLod2cellSize / 2, 1]);
                            lod0uv[ix, iy, iz, 1] = PackFloatToUShort(cell.Item2[paddedLod2cellSize / 2, paddedLod2cellSize / 2, paddedLod2cellSize / 2, 2]);
                        }

#if LOD0_8BIT
                        zeroLodData[ix, iy, iz, 0] = PackFloatToSByte(cell.Item1);

                        zeroLodData[ix, iy, iz, 1] = (byte)(atlasX);
                        zeroLodData[ix, iy, iz, 2] = (byte)(atlasY);
                        zeroLodData[ix, iy, iz, 3] = (byte)(atlasZ);
#else
                        zeroLodData[ix, iy, iz, 0] = (new HalfFloat(cell.Item1/* * 0.5f + 0.5f*/)).Data;//(new HalfFloat(cell.Item1 * 0.5f + 0.5f)).Data;
                        zeroLodData[ix, iy, iz, 1] = (new HalfFloat(atlasX / 255.0f)).Data;
                        zeroLodData[ix, iy, iz, 2] = (new HalfFloat(atlasY / 255.0f)).Data;
                        zeroLodData[ix, iy, iz, 3] = (new HalfFloat(atlasZ / 255.0f)).Data;
#endif
                    }

            Helper.SaveKTX(
#if LOD0_8BIT
                Helper.KTX_RGBA8,
#else
                Helper.KTX_RGBA16F,
#endif
                zeroLodData.Width, zeroLodData.Height, zeroLodData.Depth, zeroLodData.Data, Path.GetFileNameWithoutExtension(outFile) + "_lod_0.3d.ktx");

            Helper.SaveKTX(Helper.KTX_R8, lod1distance.Width, lod1distance.Height, lod1distance.Depth, lod1distance.Data, Path.GetFileNameWithoutExtension(outFile) + "_lod_1.3d.ktx");
            Helper.SaveKTX(
#if LOD2_8BIT
                Helper.KTX_R8,
#else
                Helper.KTX_R16F,
#endif
                lod2distance.Width, lod2distance.Height, lod2distance.Depth, lod2distance.Data, Path.GetFileNameWithoutExtension(outFile) + "_lod_2.3d.ktx");

            Helper.SaveKTX(Helper.KTX_RG16F, lod0uv.Width, lod0uv.Height, lod0uv.Depth, lod0uv.Data, Path.GetFileNameWithoutExtension(outFile) + "_lod_0_uv.3d.ktx");
            Helper.SaveKTX(Helper.KTX_RG16F, lod1uv.Width, lod1uv.Height, lod1uv.Depth, lod1uv.Data, Path.GetFileNameWithoutExtension(outFile) + "_lod_1_uv.3d.ktx");
            Helper.SaveKTX(Helper.KTX_RG16F, lod2uv.Width, lod2uv.Height, lod2uv.Depth, lod2uv.Data, Path.GetFileNameWithoutExtension(outFile) + "_lod_2_uv.3d.ktx");

            if (texture != null)
                Helper.SaveKTX(Helper.KTX_RGBA8, lod2texture.Width, lod2texture.Height, lod2texture.Depth, lod2texture.Data, Path.GetFileNameWithoutExtension(outFile) + "_lod_2_texture.3d.ktx");

            Helper.SaveKTX(Helper.KTX_RGB8, lod2normal.Width, lod2normal.Height, lod2normal.Depth, lod2normal.Data, Path.GetFileNameWithoutExtension(outFile) + "_lod_2_normal.3d.ktx");

            Console.WriteLine("[{0}] KTX saved, saving boundary mesh", sw.Elapsed);

            //Vector uvwScale = Vector.Normalize(new Vector(1.0f/sx, 1.0f / sy, 1.0f / sz));

            MeshGenerator.Surface surface = MeshGenerator.CreateBoxMesh(lowerBound, upperBound, true, "main_box");//, Matrix.CreateScale(uvwScale));

            Helper.SaveUnigineMesh(new MeshGenerator.Surface[] { surface }, outFile);
            //Helper.SaveObjMesh(new MeshGenerator.Surface[] { surface }, outFile);

            List<MeshGenerator.Surface> surfaces = new List<MeshGenerator.Surface>();

            surfaces.Add(surface);

            foreach (var tuple in bones)
            {
                MeshGenerator.Surface bone = MeshGenerator.CreateBoxMesh(tuple.Item1, tuple.Item2, true, tuple.Item3.Name);
                surfaces.Add(bone);
            }

            Helper.SaveUnigineMesh(surfaces.ToArray(), Path.GetFileNameWithoutExtension(outFile) + ".bones");

            Console.WriteLine("[{0}] All done", sw.Elapsed);

            sw.Stop();
        }

        private static float Clamp(float value, float min, float max)
        {
            return Math.Max(min, Math.Min(max, value));
        }

        private static byte PackFloatToSByte(float value)
        {
            return (byte)Clamp((value * 0.5f + 0.5f) * 255.0f, 0.0f, 255.0f);
        }

        private static byte PackFloatToByte(float value)
        {
            return (byte)Clamp(value * 255.0f, 0.0f, 255.0f);
        }

        private static ushort PackFloatToUShort(float value)
        {
            return new HalfFloat(value).Data;
        }

        private static void Iterate(int from, int to, Action<int> action)
        {
#if !DEBUG
            Parallel.For(from, to, action);
#else
            for (int i = from; i < to; i++)
                action(i);
#endif
        }

        private static void FindBestDividers(int value, out int x, out int y, out int z, int max)
        {
            int root = (int)Math.Ceiling(Math.Pow(value, 1/3.0));

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

        private static void Preprocess(Scene scene, Node node, Matrix4x4 matrix, IList<PreparedTriangle> allTriangles, IList<Tuple<Vector, Vector, Bone>> bones)
        {
            matrix = node.Transform * matrix;

            if (node.HasMeshes)
            {
#if USE_PSEUDO_NORMALS
                Dictionary<VectorPair, PseudoNormal> edgeNormals = new Dictionary<VectorPair, PseudoNormal>();
                Dictionary<Vector3i, PseudoNormal> vertexNormals = new Dictionary<Vector3i, PseudoNormal>();
#endif
                foreach (int index in node.MeshIndices)
                {
                    Mesh mesh = scene.Meshes[index];

                    if (mesh.HasBones)
                    {
                        for (int i = 0; i < mesh.BoneCount; i++)
                        {
                            Bone bone = mesh.Bones[i];

                            if (bone.HasVertexWeights)
                            {

                                float minx = float.MaxValue;
                                float miny = float.MaxValue;
                                float minz = float.MaxValue;
                                float maxx = float.MinValue;
                                float maxy = float.MinValue;
                                float maxz = float.MinValue;

                                for (int j = 0; j < bone.VertexWeightCount; j++)
                                {
                                    Vector3D v = /*bone.OffsetMatrix **/ mesh.Vertices[bone.VertexWeights[j].VertexID];
                                    minx = Math.Min(minx, v.X);
                                    miny = Math.Min(miny, v.Y);
                                    minz = Math.Min(minz, v.Z);
                                    maxx = Math.Max(maxx, v.X);
                                    maxy = Math.Max(maxy, v.Y);
                                    maxz = Math.Max(maxz, v.Z);
                                }

                                Vector3D min = matrix * new Vector3D(minx, miny, minz);
                                Vector3D max = matrix * new Vector3D(maxx, maxy, maxz);

                                bones.Add(new Tuple<Vector, Vector, Bone>(new Vector(min.X, min.Y, min.Z), new Vector(max.X, max.Y, max.Z), bone));
                            }
                        }
                    }

                    for (int i = 0; i < mesh.FaceCount; i++)
                    {
                        Face face = mesh.Faces[i];

                        if (face.HasIndices && face.IndexCount == 3) // only process triangles. Don't have a clue what to do with other primitives
                        {
                            Vector3D va = matrix * mesh.Vertices[face.Indices[0]];
                            Vector3D vb = matrix * mesh.Vertices[face.Indices[1]];
                            Vector3D vc = matrix * mesh.Vertices[face.Indices[2]];

                            Vector a = new Vector(va.X, va.Y, va.Z);
                            Vector b = new Vector(vb.X, vb.Y, vb.Z);
                            Vector c = new Vector(vc.X, vc.Y, vc.Z);

#if USE_PSEUDO_NORMALS
#if !USE_INT_COORDS
                            int multiplier = 10000;
                            //Vector3i ia = new Vector3i((int)(mesh.Vertices[face.Indices[0]].X * multiplier), (int)(mesh.Vertices[face.Indices[0]].Y * multiplier), (int)(mesh.Vertices[face.Indices[0]].Z * multiplier));
                            //Vector3i ib = new Vector3i((int)(mesh.Vertices[face.Indices[1]].X * multiplier), (int)(mesh.Vertices[face.Indices[1]].Y * multiplier), (int)(mesh.Vertices[face.Indices[1]].Z * multiplier));
                            //Vector3i ic = new Vector3i((int)(mesh.Vertices[face.Indices[2]].X * multiplier), (int)(mesh.Vertices[face.Indices[2]].Y * multiplier), (int)(mesh.Vertices[face.Indices[2]].Z * multiplier));
                            Vector3i ia = new Vector3i((int)(mesh.Vertices[face.Indices[0]].X * multiplier), (int)(mesh.Vertices[face.Indices[0]].Y * multiplier), (int)(mesh.Vertices[face.Indices[0]].Z * multiplier));
                            Vector3i ib = new Vector3i((int)(mesh.Vertices[face.Indices[1]].X * multiplier), (int)(mesh.Vertices[face.Indices[1]].Y * multiplier), (int)(mesh.Vertices[face.Indices[1]].Z * multiplier));
                            Vector3i ic = new Vector3i((int)(mesh.Vertices[face.Indices[2]].X * multiplier), (int)(mesh.Vertices[face.Indices[2]].Y * multiplier), (int)(mesh.Vertices[face.Indices[2]].Z * multiplier));
#else
                            Vector3i ia = new Vector3i(face.Indices[0], 0, 0);
                            Vector3i ib = new Vector3i(face.Indices[1], 0, 0);
                            Vector3i ic = new Vector3i(face.Indices[2], 0, 0);
#endif
                            VectorPair edgeab = new VectorPair(ia, ib);
                            VectorPair edgebc = new VectorPair(ib, ic);
                            VectorPair edgeca = new VectorPair(ic, ia);
#endif

                            // This tuple will be used to get back the triangle and vertices when needed
                            Tuple<Mesh, Face> data = new Tuple<Mesh, Face>(mesh, face);

                            PreparedTriangle triangle = new PreparedTriangle(a, b, c, data
#if USE_PSEUDO_NORMALS
                                ,
                                new PseudoNormal[]
                                {
                                    new PseudoNormal(),
                                    GetValue(edgeNormals, edgeab),
                                    GetValue(edgeNormals, edgebc),
                                    GetValue(edgeNormals, edgeca),
                                    GetValue(vertexNormals, ia),
                                    GetValue(vertexNormals, ib),
                                    GetValue(vertexNormals, ic)
                                }
#endif
                                );
                            allTriangles.Add(triangle);

                        }
                        else
                            Console.WriteLine("Not a triangle!");
                    }
                }

#if USE_PSEUDO_NORMALS
                foreach (var pair in edgeNormals)
                    if (pair.Value.Count == 1)
                    {
                        Console.WriteLine("Edge {0} has only 1 triangle!", pair.Key);
                        pair.Value.Add(pair.Value.Value);
                    } else
                        if (pair.Value.Count > 2)
                        Console.WriteLine("Edge {0} has {1} triangles!", pair.Key, pair.Value.Count);
#endif
            }

            for (int i = 0; i < node.ChildCount; i++)
                Preprocess(scene, node.Children[i], matrix, allTriangles, bones);
        }

#if USE_PSEUDO_NORMALS
        private static PseudoNormal GetValue<K>(IDictionary<K, PseudoNormal> dictionary, K key)
        {
            PseudoNormal value;
            if (dictionary.TryGetValue(key, out value))
                return value;

            dictionary[key] = value = new PseudoNormal();
            return value;
        }
#endif
        static void Main(string[] args)
        {
            if (args.Length < 2)
            {
                Console.Error.WriteLine(s_syntax, Path.GetFileNameWithoutExtension(System.Reflection.Assembly.GetExecutingAssembly().Location));
                return;
            }

            string fileName = args[0];
            string outFileName = args[1];

            if (!File.Exists(fileName))
            {
                Console.Error.WriteLine("File {0} not found", fileName);
                return;
            }

            int gridSize = args.Length > 2 ? int.Parse(args[2]) : 64;
            int size = args.Length > 3 ? int.Parse(args[3]) : 32;
            int cellSize = args.Length > 4 ? int.Parse(args[4]) : 4;

            string textureFileName = args.Length > 5 ? args[5] : null;

            if (!string.IsNullOrEmpty(textureFileName) && !File.Exists(textureFileName))
            {
                Console.Error.WriteLine("Texture file {0} not found", textureFileName);
                return;
            }

            ProcessAssimpImport(fileName, outFileName, textureFileName, gridSize, size, cellSize);
        }
    }
}

