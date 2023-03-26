//#define LOD0_8BIT
//#define LOD2_8BIT

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using Assimp;
using RunMobile.Utility;

using Vector = System.Numerics.Vector3;
using Vector4 = System.Numerics.Vector4;

namespace SDFTool
{
    static class Program
    {
        private static readonly string s_syntax = "Syntax: {0} input.dae output.ktx [grid_cells] [lod_0_size] [top_lod_cell_size]\n" +
        "\n";

        #region Assimp specific

        private static Node FindMeshNode(Node node, int meshIndex)
        {
            if (node.HasMeshes && node.MeshIndices[0] == meshIndex)
                return node;

            foreach (var child in node.Children)
            {
                Node result = FindMeshNode(child, meshIndex);
                if (result != null)
                    return result;
            }

            return null;
        }

        private static PreparedTriangle[] PrepareScene(Scene scene, float scale, out Vector sceneMin, out Vector sceneMax, out Matrix4x4 matrix, out IDictionary<int, ValueTuple<int, float>[]> boneDictionary, out ValueTuple<string, Bone>[] boneArray)
        {
            sceneMin = new Vector(float.MaxValue, float.MaxValue, float.MaxValue);
            sceneMax = new Vector(float.MinValue, float.MinValue, float.MinValue);

            matrix = Matrix4x4.Identity;

            if (scene.Metadata != null)
            {
                int UpAxis = 1, UpAxisSign = 1, FrontAxis = 2, FrontAxisSign = 1, CoordAxis = 0, CoordAxisSign = 1;
                float UnitScaleFactor = 1.0f;

                foreach (var prop in scene.Metadata)
                {
                    if (prop.Key == "UpAxis")
                    {
                        UpAxis = prop.Value.DataAs<int>().Value;
                    }
                    if (prop.Key == "UpAxisSign")
                    {
                        UpAxisSign = prop.Value.DataAs<int>().Value;
                    }
                    if (prop.Key == "FrontAxis")
                    {
                        FrontAxis = prop.Value.DataAs<int>().Value;
                    }
                    if (prop.Key == "FrontAxisSign")
                    {
                        FrontAxisSign = prop.Value.DataAs<int>().Value;
                    }
                    if (prop.Key == "CoordAxis")
                    {
                        CoordAxis = prop.Value.DataAs<int>().Value;
                    }
                    if (prop.Key == "CoordAxisSign")
                    {
                        CoordAxisSign = prop.Value.DataAs<int>().Value;
                    }
                    if (prop.Key == "UnitScaleFactor")
                    {
                        UnitScaleFactor = (float)prop.Value.DataAs<float>().Value;
                        //if (UnitScaleFactor == 0)
                        //UnitScaleFactor = 1.0f;
                    }
                }

                Vector3D upVec = new Vector3D();
                Vector3D forwardVec = new Vector3D();
                Vector3D rightVec = new Vector3D();

                upVec[UpAxis] = UpAxisSign * (float)UnitScaleFactor * scale;
                forwardVec[FrontAxis] = FrontAxisSign * (float)UnitScaleFactor * scale;
                rightVec[CoordAxis] = CoordAxisSign * (float)UnitScaleFactor * scale;

                matrix = new Matrix4x4(
                    rightVec.X, rightVec.Y, rightVec.Z, 0.0f,
                    upVec.X, upVec.Y, upVec.Z, 0.0f,
                    forwardVec.X, forwardVec.Y, forwardVec.Z, 0.0f,
                    0.0f, 0.0f, 0.0f, 1.0f);
            }
            else
                matrix = Matrix4x4.FromScaling(new Vector3D(scale));

            List<PreparedTriangle> triangleList = new List<PreparedTriangle>();

            List<ValueTuple<string, Bone>> bones = new List<ValueTuple<string, Bone>>();

            for (int meshIndex = 0; meshIndex < scene.Meshes.Count; meshIndex++)
            {
                Mesh mesh = scene.Meshes[meshIndex];
                Node node = FindMeshNode(scene.RootNode, meshIndex);

                Matrix4x4 vertexMatrix = Matrix4x4.Identity;

                if (node != null)
                {
                    while (node != null)
                    {
                        vertexMatrix = vertexMatrix * node.Transform;
                        node = node.Parent;
                    }
                    matrix = vertexMatrix;
                    vertexMatrix = Matrix4x4.Identity;
                }

                if (mesh.HasBones)
                {
                    foreach (Bone bone in mesh.Bones)
                    {
                        if (bone.HasVertexWeights)
                            //for (int j = 0; j < bone.VertexWeightCount; j++)
                            bones.Add(new ValueTuple<string, Bone>(mesh.Name, bone));
                    }
                }

                for (int i = 0; i < mesh.FaceCount; i++)
                {
                    Face face = mesh.Faces[i];

                    if (face.HasIndices && face.IndexCount == 3) // only process triangles. Don't have a clue what to do with other primitives
                    {
                        Vector3D va = vertexMatrix * mesh.Vertices[face.Indices[0]];
                        Vector3D vb = vertexMatrix * mesh.Vertices[face.Indices[1]];
                        Vector3D vc = vertexMatrix * mesh.Vertices[face.Indices[2]];

                        //Vector3D na = vertexMatrix * mesh.Normals[face.Indices[0]];
                        //Vector3D nb = vertexMatrix * mesh.Normals[face.Indices[1]];
                        //Vector3D nc = vertexMatrix * mesh.Normals[face.Indices[2]];

                        // This tuple will be used to get back the triangle and vertices when needed
                        Tuple<Mesh, Face> triangleData = new Tuple<Mesh, Face>(mesh, face);

                        PreparedTriangle triangle = new PreparedTriangle(triangleList.Count,
                            new Vector(va.X, va.Y, va.Z), new Vector(vb.X, vb.Y, vb.Z), new Vector(vc.X, vc.Y, vc.Z),
                            triangleData
                            );
                        triangleList.Add(triangle);

                    }
                    else
                        Console.WriteLine("Not a triangle!");
                }
                // let's stick with first mesh for now
                break;
            }

            foreach (PreparedTriangle triangle in triangleList)
            {
                sceneMax.X = Math.Max(sceneMax.X, triangle.UpperBound.X);
                sceneMax.Y = Math.Max(sceneMax.Y, triangle.UpperBound.Y);
                sceneMax.Z = Math.Max(sceneMax.Z, triangle.UpperBound.Z);
                sceneMin.X = Math.Min(sceneMin.X, triangle.LowerBound.X);
                sceneMin.Y = Math.Min(sceneMin.Y, triangle.LowerBound.Y);
                sceneMin.Z = Math.Min(sceneMin.Z, triangle.LowerBound.Z);
            }


            Dictionary<int, List<ValueTuple<int, float>>> tempBoneDictionary = new Dictionary<int, List<ValueTuple<int, float>>>();

            for (int b = 0; b < bones.Count; b++)
            {
                ValueTuple<string, Bone> btuple = bones[b];

                for (int j = 0; j < btuple.Item2.VertexWeightCount; j++)
                {
                    int vid = btuple.Item2.VertexWeights[j].VertexID; // vertex id
                    int bid = b + 1; // bone id
                    float weight = btuple.Item2.VertexWeights[j].Weight; // weight

                    List<ValueTuple<int, float>> list;

                    if (!tempBoneDictionary.TryGetValue(vid, out list))
                    {
                        list = new List<ValueTuple<int, float>>();
                        tempBoneDictionary.Add(vid, list);
                    }

                    list.Add(new ValueTuple<int, float>(bid, weight));
                }
            }

            boneDictionary = new Dictionary<int, ValueTuple<int, float>[]>();

            foreach (var pair in tempBoneDictionary)
                boneDictionary[pair.Key] = pair.Value.ToArray();

            boneArray = bones.ToArray();

            return triangleList.ToArray();
        }


        private static PixelData[] GetPixelData(float[] distanceData, IDictionary<int, (int, float)[]> boneDictionary, PreparedTriangle[] triangleList)
        {
            PixelData[] data = new PixelData[distanceData.Length / 4];

            for (int i = 0; i < data.Length; i++)
            {
                float pixelDistance = distanceData[i * 4 + 0];
                float u = distanceData[i * 4 + 1];
                float v = distanceData[i * 4 + 2];
                int triangleId = (int)distanceData[i * 4 + 3];

                float textureU = 0;
                float textureV = 0;
                Vector4i pixelBones = default;
                Vector4 pixelBoneWeights = default;

                if (triangleId != -1)
                {
                    Vector triangleWeights = new Vector(u, v, 1.0f - u - v);

                    PreparedTriangle triangle = triangleList[triangleId];

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
                            textureU = tc.X;
                            textureV = tc.Y;
                        }

                        if (Math.Abs(pixelDistance) <= 1.0f) // no need to calculate weight for non-surface points
                        {
                            Dictionary<int, float[]> weights = new Dictionary<int, float[]>();

                            for (int k = 0; k < face.IndexCount; k++)
                            {
                                ValueTuple<int, float>[] list;
                                if (boneDictionary.TryGetValue(face.Indices[k], out list))
                                {
                                    foreach (var pair in list)
                                    {
                                        float[] bweight;
                                        if (!weights.TryGetValue(pair.Item1, out bweight))
                                            weights[pair.Item1] = bweight = new float[face.IndexCount];

                                        bweight[k] = pair.Item2;
                                    }
                                }
                            }

                            List<ValueTuple<float, int>> boneWeights = new List<ValueTuple<float, int>>();

                            foreach (var pair in weights)
                            {
                                float weight = pair.Value[0] * triangleWeights.X + pair.Value[1] * triangleWeights.Y + pair.Value[2] * triangleWeights.Z;
                                if (weight > 0.01f)
                                    boneWeights.Add(new ValueTuple<float, int>(weight, pair.Key));
                            }

                            boneWeights.Sort((x, y) => y.Item1.CompareTo(y.Item1)); // sort descending by weight

                            float maxWeight = boneWeights.Count > 0 ? boneWeights[0].Item1 : 0;
                            int maxBoneId = boneWeights.Count > 0 ? boneWeights[0].Item2 : 0;
                            float secondWeight = boneWeights.Count > 1 ? boneWeights[1].Item1 : 0;
                            int secondBoneId = boneWeights.Count > 1 ? boneWeights[1].Item2 : 0;
                            float thirdWeight = boneWeights.Count > 2 ? boneWeights[2].Item1 : 0;
                            int thirdBoneId = boneWeights.Count > 2 ? boneWeights[2].Item2 : 0;
                            float fourthWeight = boneWeights.Count > 3 ? boneWeights[3].Item1 : 0;
                            int fourthBoneId = boneWeights.Count > 3 ? boneWeights[3].Item2 : 0;

                            pixelBones = new Vector4i(maxBoneId, secondBoneId, thirdBoneId, fourthBoneId);
                            pixelBoneWeights = new Vector4(maxWeight, secondWeight, thirdWeight, fourthWeight);
                        }
                    }

                }
                data[i] = new PixelData(pixelDistance, textureU, textureV, pixelBones, pixelBoneWeights);
            }

            return data;
        }

        #endregion

        struct PixelData
        {
            public readonly Vector DistanceUV;
            public readonly Vector4i Bones;
            public readonly Vector4 BoneWeights;

            public PixelData(float distance, float u, float v, Vector4i bones, Vector4 weights)
            {
                DistanceUV = new Vector(distance, u, v);
                Bones = bones;
                BoneWeights = weights;
            }
        }

        private static T GetArrayData<T>(T[] data, Vector3i dataSize, Vector3i coord)
        {
            return data[coord.X + coord.Y * dataSize.X + coord.Z * dataSize.X * dataSize.Y];
        }

        private static void ProcessAssimpImport(string fileName, string outFile, int gridCellCount = 64, int lod0pixels = 32, int topLodCellSize = 4)
        {
            if (topLodCellSize <= 2)//(topLodCellSize & (topLodCellSize - 1)) != 0)
            {
                Console.Error.WriteLine("Top LOD cell size should be a power of two and greater than 2 (4, 8, 16, 32, etc.)!");
                return;
            }

            int nlods = 0;
            int lod = topLodCellSize;

            while (lod >= 2 && lod % 2 == 0)
            {
                nlods++;
                lod /= 2;
            }


            Stopwatch sw = new Stopwatch();
            sw.Start();

            Console.WriteLine("[{0}] Processing file {1}", sw.Elapsed, fileName);

            AssimpContext importer = new AssimpContext();
            Scene scene = importer.ImportFile(fileName, PostProcessSteps.GenerateSmoothNormals);

            Console.WriteLine("[{0}] File loaded", sw.Elapsed);

            float scale = 1;

            Vector sceneMin;
            Vector sceneMax;
            Matrix4x4 matrix;

            IDictionary<int, ValueTuple<int, float>[]> boneDictionary;
            ValueTuple<string, Bone>[] bones;

            PreparedTriangle[] triangleList = PrepareScene(scene, scale, out sceneMin, out sceneMax, out matrix, out boneDictionary, out bones);

            float maxSide = Math.Max(Math.Max(sceneMax.Z - sceneMin.Z, sceneMax.Y - sceneMin.Y), sceneMax.X - sceneMin.X);

            // maximum pixel size of lod 0 in any dimension

            // lod 0 cell size is 1
            int paddedTopLodCellSize = topLodCellSize;
            topLodCellSize--;

            // number of extra pixels on the each size
            int lodPadding = 1;

            // multiply by this value to convert scene units to pixels
            float sceneToPixels = ((lod0pixels - lodPadding * 2)) / maxSide; // step for lod 2

            // multiply by this value to convert pixels to scene units
            float pixelsToScene = 1.0f / sceneToPixels;

            // divide by this value to convert scene units to 0..1
            float sceneToRelative = maxSide;


            // exact number of pixels 

            int sx = (int)Math.Ceiling((sceneMax.X - sceneMin.X) * sceneToPixels) + topLodCellSize * 2;
            int sy = (int)Math.Ceiling((sceneMax.Y - sceneMin.Y) * sceneToPixels) + topLodCellSize * 2;
            int sz = (int)Math.Ceiling((sceneMax.Z - sceneMin.Z) * sceneToPixels) + topLodCellSize * 2;

            if (sx % topLodCellSize != 0)
                sx += topLodCellSize - sx % topLodCellSize;
            if (sy % topLodCellSize != 0)
                sy += topLodCellSize - sy % topLodCellSize;
            if (sz % topLodCellSize != 0)
                sz += topLodCellSize - sz % topLodCellSize;

            // we need to increase the size by one since each cell takes (topLodCellSize + 1) pixels and last one will be left without it
            sx++;
            sy++;
            sz++;

            Vector3i dataSize = new Vector3i(sx, sy, sz);

            float padding = topLodCellSize * pixelsToScene;

            Vector lowerBound = sceneMin - new Vector(padding);
            Vector upperBound = sceneMax + new Vector(padding);

            Console.WriteLine("[{0}] File preprocessed. X: {1}, Y: {2}, Z: {3}, maximum distance: {4}", sw.Elapsed, sx, sy, sz, sceneToRelative);

            int maxcount = sz * sy * sx;

            float[] distanceData = new float[maxcount * 4];

            //creating grid structure for faster triangle search
            TriangleMap triangleMap = new TriangleMap(sceneMin, sceneMax, gridCellCount, triangleList);

            Console.WriteLine("Bounding box: {0} - {1}, step {2}, triangles {3}, cells {4}, instances {5}", sceneMin, sceneMax, sceneToPixels, triangleMap.TriangleCount, triangleMap.CellsUsed, triangleMap.TriangleInstances);

            triangleMap.Dispatch(distanceData, lowerBound, pixelsToScene, sceneToPixels / topLodCellSize, sx, sy, sz, (progress) => Console.WriteLine("[{0}] Processing {1:P2}", sw.Elapsed, progress));

            PixelData[] data = GetPixelData(distanceData, boneDictionary, triangleList);

            Console.WriteLine("[{0}] SDF data ready", sw.Elapsed);

            // find non-empty cells

            int usedCells = 0;
            int cellsx = sx / topLodCellSize;
            int cellsy = sy / topLodCellSize;
            int cellsz = sz / topLodCellSize;

            int totalCells = cellsx * cellsy * cellsz;

            int blockWidth = paddedTopLodCellSize;
            int blockHeight = paddedTopLodCellSize;
            int blockDepth = paddedTopLodCellSize;

            float[] cells = new float[totalCells];

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

                        for (int z = 0; z < blockDepth; z++)
                            for (int y = 0; y < blockHeight; y++)
                                for (int x = 0; x < blockWidth; x++)
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
            Array3D<ushort>[] lodDistance = new Array3D<ushort>[nlods];
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

            Array3D<ushort> topLoduv = new Array3D<ushort>(2, packx * paddedTopLodCellSize, packy * paddedTopLodCellSize, packz * paddedTopLodCellSize);

            Array3D<byte> topLodTexture = new Array3D<byte>(4, packx * paddedTopLodCellSize, packy * paddedTopLodCellSize, packz * paddedTopLodCellSize);
            Console.WriteLine("[{0}] Got {1} empty cells, cell grid size {2}, {3:P}, total {4} of {5}x{5}x{5} cells, size {6} vs {7}, grid {8}x{9}x{10}",
                sw.Elapsed, totalCells - usedCells, new Vector3i(cellsx, cellsy, cellsz), usedCells / (float)totalCells,
                usedCells,
                topLodCellSize,
                topLodDistance.Bytes,
                maxcount * Marshal.SizeOf<PixelData>(),
                packx, packy, packz
                );

            Console.WriteLine("[{0}] Full size KTX saved, saving lower LOD", sw.Elapsed);

            int brickId = 0;

            float packLodCoef = lod0pixels;

#if LOD0_8BIT
            Array3D<byte> zeroLodData = new Array3D<byte>(4, cellsx, cellsy, cellsz);
#else
            Array3D<ushort> zeroLodData = new Array3D<ushort>(4, cellsx, cellsy, cellsz);
#endif
            Vector boxStep = (upperBound - lowerBound) / new Vector(cellsx, cellsy, cellsz);

            List<MeshGenerator.Shape> boxes = new List<MeshGenerator.Shape>();

            Dictionary<Vector3i, ValueTuple<int, float>[]> weightCache = new Dictionary<Vector3i, ValueTuple<int, float>[]>();

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

                            float[] distanceBlock = new float[blockWidth * blockHeight * blockDepth];

                            Vector3i dataStart = new Vector3i(ix * topLodCellSize, iy * topLodCellSize, iz * topLodCellSize);

                            for (int z = 0; z < blockDepth; z++)
                                for (int y = 0; y < blockHeight; y++)
                                    for (int x = 0; x < blockWidth; x++)
                                    {
                                        PixelData pixel = GetArrayData(data, dataSize, dataStart + new Vector3i(x, y, z));
                                        int blockIndex = x + y * blockWidth + z * blockWidth * blockHeight;

                                        distanceBlock[blockIndex] = pixel.DistanceUV.X;

                                        topLoduv[atlasX * paddedTopLodCellSize + x, atlasY * paddedTopLodCellSize + y, atlasZ * paddedTopLodCellSize + z, 0] = Helper.PackFloatToUShort(pixel.DistanceUV.Y);
                                        topLoduv[atlasX * paddedTopLodCellSize + x, atlasY * paddedTopLodCellSize + y, atlasZ * paddedTopLodCellSize + z, 1] = Helper.PackFloatToUShort(pixel.DistanceUV.Z);
                                    }

                            float[] distancePercentageArr = GetTrilinear(distanceBlock, blockWidth, blockHeight, blockDepth, 1, topLodCellSize / 2.0f, topLodCellSize / 2.0f, topLodCellSize / 2.0f);

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
                            topLodDistance.PutBlock(distanceBlock, blockWidth, blockHeight, blockDepth, 1, atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize, (k, l) => Helper.PackFloatToUShort(k - dist));
#endif

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
                            

                            Vector boxStart = lowerBound + new Vector(ix, iy, iz) * boxStep;
                            System.Numerics.Matrix4x4 boxTextureMatrix =
                                System.Numerics.Matrix4x4.Identity;

                            ValueTuple<int, float>[][] boxBones = GetBoxWeights(weightCache, data, dataSize, dataStart, blockWidth, blockHeight, blockDepth, ix, iy, iz);//, topLodCellSize * 0.25f);

                            Vector boxCenter = boxStart + boxStep / 2;

                            System.Numerics.Matrix4x4 boxMatrix =
                                System.Numerics.Matrix4x4.CreateScale(boxStep) * System.Numerics.Matrix4x4.CreateTranslation(boxStart);

                            boxes.Add(new MeshGenerator.Shape(
                                boxMatrix,
                                boxTextureMatrix,
                                MeshGenerator.ShapeType.Cube,
                                MeshGenerator.ShapeFlags.NoNormals,
                                new float[] {
                                    //0, 0, 0,// padding
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

            Helper.SaveKTX(
#if LOD0_8BIT
                Helper.KTX_RGBA8,
#else
                Helper.KTX_RGBA16F,
#endif
                zeroLodData, outFile, "_lod_0.3d.ktx");

#if LOD2_8BIT
            SaveKTX(Helper.KTX_R8, lodDistance, outFile, "_lod.3d.ktx");
#else
            Helper.SaveKTX(Helper.KTX_R16F, lodDistance, outFile, "_lod.3d.ktx");
#endif

            Helper.SaveKTX(Helper.KTX_RG16F, topLoduv, outFile, "_lod_2_uv.3d.ktx");

            Console.WriteLine("[{0}] KTX saved, saving boundary mesh", sw.Elapsed);

            MeshGenerator.Surface[] boxesSurface = new MeshGenerator.Surface[] { MeshGenerator.CreateBoxesMesh(boxes.ToArray(), "main_box") };

            //Helper.SaveAssimpMesh(boxesSurface, outFile, new[] { bones }, scene.Animations, scene);
            Helper.SaveUMesh(boxesSurface, outFile, new[] { bones }, scene.Animations, scene, matrix);

            Console.WriteLine("[{0}] All done", sw.Elapsed);

            sw.Stop();
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
            PixelData [] data, Vector3i dataSize, Vector3i dataStart,
            int blockWidth, int blockHeight, int blockDepth,
            int ix, int iy, int iz)
        {
            int ubx = blockWidth - 1;
            int uby = blockHeight - 1;
            int ubz = blockDepth - 1;

            ValueTuple<int, float>[][] result = new ValueTuple<int, float>[8][];

            List<ValueTuple<Vector, Vector4i, Vector4>> [] vertexWeights = new List<ValueTuple<Vector, Vector4i, Vector4>>[8];

            for (int i = 0; i < 8; i++)
                vertexWeights[i] = new List<ValueTuple<Vector, Vector4i, Vector4>>();

            for (int z = 0; z < blockDepth - 1; z++)
                for (int y = 0; y < blockHeight - 1; y++)
                    for (int x = 0; x < blockWidth - 1; x++)
                    {
                        PixelData pixelData = GetArrayData(data, dataSize, dataStart + new Vector3i(x,y,z));

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
                            vertexWeights[0].Add(new ValueTuple<Vector, Vector4i, Vector4>(new Vector(x, y, z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[1].Add(new ValueTuple<Vector, Vector4i, Vector4>(new Vector(ubx - x, y, z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[2].Add(new ValueTuple<Vector, Vector4i, Vector4>(new Vector(ubx - x, uby - y, z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[3].Add(new ValueTuple<Vector, Vector4i, Vector4>(new Vector(x, uby - y, z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[4].Add(new ValueTuple<Vector, Vector4i, Vector4>(new Vector(x, uby - y, ubz - z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[5].Add(new ValueTuple<Vector, Vector4i, Vector4>(new Vector(ubx - x, uby - y, ubz - z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[6].Add(new ValueTuple<Vector, Vector4i, Vector4>(new Vector(ubx - x, y, ubz - z), pixelData.Bones, pixelData.BoneWeights));
                            vertexWeights[7].Add(new ValueTuple<Vector, Vector4i, Vector4>(new Vector(x, y, ubz - z), pixelData.Bones, pixelData.BoneWeights));
                        }
                    }

            for (int i = 0; i < vertexWeights.Length; i++)
            {
                Dictionary<int, ValueTuple<float, int>> weightValues = new Dictionary<int, ValueTuple<float, int>>();

                foreach (var pair in vertexWeights[i])
                {
                    //Euclidian distance to the pixel
                    //float weight = Vector.DistanceSquared(new Vector(1, 1, 1), pair.Item1 / new Vector(ubx, uby, ubz));
                    float weight = Vector.Distance(new Vector(1, 1, 1), pair.Item1 / new Vector(ubx, uby, ubz));
                    //Manhattan distance to the pixel
                    //float weight = Vector.Dot(new Vector(1, 1, 1) - pair.Item1 / new Vector(ubx, uby, ubz), new Vector(1, 1, 1));

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

        private static float[] GetTrilinear(float[] block, int blockWidth, int blockHeight, int blockDepth, int components, float px, float py, float pz)
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

                if (ip.X >= blockWidth || ip.Y >= blockHeight || ip.Z >= blockDepth)
                    continue;

                for (int c = 0; c < components; c++)
                    result[c] += coef[i] * block[ip.X + ip.Y * blockWidth + ip.Z * blockWidth * blockHeight];
            }
            return result;
        }

        private static float [] GenerateLod(float [] topLodDistance, int blockWidth, int blockHeight, int blockDepth, int lodSize)
        {
            float [] result = new float[lodSize * lodSize * lodSize];

            float step = (blockDepth - 1.0f) / (lodSize - 1.0f);

            for (int nz = 0; nz < lodSize; nz++)
            {
                for (int ny = 0; ny < lodSize; ny++)
                {
                    for (int nx = 0; nx < lodSize; nx++)
                    {
                        float[] value = GetTrilinear(topLodDistance, blockWidth, blockHeight, blockDepth, 1, nx * step, ny * step, nz * step);

                        result[nx + ny * lodSize + nz * lodSize * lodSize] = value[0];
                    }
                }
            }

            return result;
        }

        #endregion

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

            ProcessAssimpImport(fileName, outFileName, gridSize, size, cellSize);
        }
    }
}

