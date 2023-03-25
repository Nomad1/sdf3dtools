//#define LOD0_8BIT
//#define LOD2_8BIT

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using Assimp;
using RunMobile.Utility;

using Vector = System.Numerics.Vector3;

namespace SDFTool
{
    static class Program
    {
        private static readonly string s_syntax = "Syntax: {0} input.dae output.ktx [grid_cells] [lod_0_size] [top_lod_cell_size]\n" +
        "\n";

        #region Assimp scene to triangles

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

        #endregion

        private static void ProcessAssimpImport(string fileName, string outFile, string textureFile = null, int gridCellCount = 64, int lod0pixels = 32, int topLodCellSize = 4)
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

#if USE_TEXTURE
            int textureWidth = 0;
            int textureHeight = 0;
            int[] texture = textureFile == null ? null : Helper.LoadBitmap(textureFile, out textureWidth, out textureHeight);
#endif
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

            float padding = topLodCellSize * pixelsToScene;

            Vector lowerBound = sceneMin - new Vector(padding);
            Vector upperBound = sceneMax + new Vector(padding);

            Console.WriteLine("[{0}] File preprocessed. X: {1}, Y: {2}, Z: {3}, maximum distance: {4}", sw.Elapsed, sx, sy, sz, sceneToRelative);

            //float emptyCellCheckDistance = topLodCellSize * 0.5f;

            int maxcount = sz * sy * sx;

            float[] distanceData = new float[maxcount * 4];

            //creating grid structure for faster triangle search
            TriangleMap triangleMap = new TriangleMap(sceneMin, sceneMax, gridCellCount, triangleList);

            Console.WriteLine("Bounding box: {0} - {1}, step {2}, triangles {3}, cells {4}, instances {5}", sceneMin, sceneMax, sceneToPixels, triangleMap.TriangleCount, triangleMap.CellsUsed, triangleMap.TriangleInstances);

            triangleMap.Dispatch(distanceData, lowerBound, pixelsToScene, sx, sy, sz, (progress) => Console.WriteLine("[{0}] Processing {1:P2}", sw.Elapsed, progress));

            Array3D<float> data = new Array3D<float>(13, sx, sy, sz); // we are using lod 2 data for processing

            for (int i = 0; i < maxcount; i++)
            {
                float pixelDistance = distanceData[i * 4 + 0] / topLodCellSize;
                float u = distanceData[i * 4 + 1];
                float v = distanceData[i * 4 + 2];
                int triangleId = (int)distanceData[i * 4 + 3];


                data[i * 13 + 0] = pixelDistance;

                if (triangleId == -1)
                    continue;

                Vector triangleWeights = new Vector(u, v, 1.0f - u - v);

                PreparedTriangle triangle = triangleList[triangleId];

                if (triangle == null)
                    continue;
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
                        data[i * 13 + 1] = tc.X;
                        data[i * 13 + 2] = tc.Y;
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

                        data[i * 13 + 5] = maxBoneId;
                        data[i * 13 + 6] = maxWeight;
                        data[i * 13 + 7] = secondBoneId;
                        data[i * 13 + 8] = secondWeight;
                        data[i * 13 + 9] = thirdBoneId;
                        data[i * 13 + 10] = thirdWeight;
                        data[i * 13 + 11] = fourthBoneId;
                        data[i * 13 + 12] = fourthWeight;
                    }
                    // indices 5+ should be weights for the bones and bone numbers
                }
            }

            Console.WriteLine("[{0}] SDF data ready", sw.Elapsed);

            // split to cells

            int usedCells = 0;
            int cellsx = sx / topLodCellSize;
            int cellsy = sy / topLodCellSize;
            int cellsz = sz / topLodCellSize;

            int totalCells = cellsx * cellsy * cellsz;

            Tuple<Array3D<float>, float, float, Vector>[] cells = new Tuple<Array3D<float>, float, float, Vector>[totalCells];

            for (int iz = 0; iz < cellsz; iz++)
            {
                for (int iy = 0; iy < cellsy; iy++)
                {
                    for (int ix = 0; ix < cellsx; ix++)
                    {
                        int index = ix + iy * cellsx + iz * cellsx * cellsy;

                        Array3D<float> block = data.GetBlock(ix * topLodCellSize, iy * topLodCellSize, iz * topLodCellSize, paddedTopLodCellSize, paddedTopLodCellSize, paddedTopLodCellSize, new float[] { float.MaxValue, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 });

                        float minDistance = float.MaxValue;

                        Vector minPoint = new Vector();

                        int countPositive = 0;
                        int countNegative = 0;

                        for (int z = 0; z < block.Depth; z++)
                            for (int y = 0; y < block.Height; y++)
                                for (int x = 0; x < block.Width; x++)
                                //for (int i = 0; i < block.Data.Length; i += block.Components)
                                {
                                    float distance = block[x, y, z, 0];

                                    if (distance > 0)
                                        countPositive++;
                                    else
                                        if (distance < 0)
                                        countNegative++;

                                    if (Math.Abs(distance) < minDistance)
                                    {
                                        minDistance = Math.Abs(distance);
                                        minPoint = new Vector((float)x / block.Width, (float)y / block.Height, (float)z / block.Depth);
                                    }
                                }

                        float[] distancePercentageArr = GetTrilinear(block, topLodCellSize / 2.0f, topLodCellSize / 2.0f, topLodCellSize / 2.0f);

                        float distancePercentage = distancePercentageArr[0]; // central point

                        if ((countPositive != 0 && countNegative != 0))// || minDistance < emptyCellCheckDistance) // all neg or all pos is ignored
                        {
                            usedCells++;
                        }
                        else
                        {
                            block = null;
                        }
                        cells[index] = new Tuple<Array3D<float>, float, float, Vector>(block, distancePercentage, minDistance, minPoint);
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
                data.Bytes,
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

                        Tuple<Array3D<float>, float, float, Vector> cell = cells[index];

                        if (cell.Item1 != null)
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

                            Array3D<float> distanceBlock = new Array3D<float>(1, paddedTopLodCellSize, paddedTopLodCellSize, paddedTopLodCellSize);

                            Array3D<float> uvBlock = new Array3D<float>(2, paddedTopLodCellSize, paddedTopLodCellSize, paddedTopLodCellSize);

                            Array3D<byte> textureBlock = new Array3D<byte>(4, paddedTopLodCellSize, paddedTopLodCellSize, paddedTopLodCellSize);

                            for (int z = 0; z < cell.Item1.Depth; z++)
                                for (int y = 0; y < cell.Item1.Height; y++)
                                    for (int x = 0; x < cell.Item1.Width; x++)
                                    {
                                        float distance = cell.Item1[x, y, z, 0];
                                        float u = cell.Item1[x, y, z, 1];
                                        float v = cell.Item1[x, y, z, 2];

                                        distanceBlock[x, y, z, 0] = distance;

                                        uvBlock[x, y, z, 0] = u;
                                        uvBlock[x, y, z, 1] = v;
#if USE_TEXTURE
                                        if (texture != null)
                                        {
                                            int textureColor = texture != null ? texture[(int)(Helper.Clamp(u * textureWidth, 0.0f, textureWidth - 1)) + ((int)Helper.Clamp((1.0f - v) * textureHeight, 0.0f, textureHeight - 1)) * textureWidth] : 0;

                                            textureBlock[x, y, z, 0] = (byte)(textureColor >> 16);
                                            textureBlock[x, y, z, 1] = (byte)(textureColor >> 8);
                                            textureBlock[x, y, z, 2] = (byte)(textureColor);
                                            textureBlock[x, y, z, 3] = (byte)(textureColor >> 24);
                                        }
#endif
                                    }

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
                            topLodDistance.PutBlock(distanceBlock, atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize, (k) => Helper.PackFloatToUShort(k - dist));
#endif

                            int lodSize = paddedTopLodCellSize;
                            for (int i = 1; i < nlods; i++)
                            {
                                lodSize /= 2;
                                Array3D<float> lodDistanceBlock = GenerateLod(distanceBlock, lodSize);

#if LOD2_8BIT
                                lodDistance[i].PutBlock(lodDistanceBlock, atlasX * lodSize, atlasY * lodSize, atlasZ * lodSize, (k) => PackFloatToSByte((k - dist) * packLodCoef));
#else
                                lodDistance[i].PutBlock(lodDistanceBlock, atlasX * lodSize, atlasY * lodSize, atlasZ * lodSize, (k) => Helper.PackFloatToUShort(k - dist));
#endif
                            }

                            topLoduv.PutBlock(uvBlock, atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize, Helper.PackFloatToUShort);
#if USE_TEXTURE
                            if (texture != null)
                                topLodTexture.PutBlock(textureBlock, atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize);
#endif

                            Vector boxStart = lowerBound + new Vector(ix, iy, iz) * boxStep;
                            System.Numerics.Matrix4x4 boxTextureMatrix =
                                //System.Numerics.Matrix4x4.CreateTranslation(new Vector(-atlasX * paddedTopLodCellSize, -atlasY * paddedTopLodCellSize, -atlasZ * paddedTopLodCellSize));
                                System.Numerics.Matrix4x4.Identity;

                            ValueTuple<int, float>[][] boxBones = GetBoxWeights(weightCache, cell.Item1, ix, iy, iz, topLodCellSize * 0.25f);

                            Vector boxCenter = boxStart + boxStep / 2;

                            System.Numerics.Matrix4x4 boxMatrix =
                                System.Numerics.Matrix4x4.CreateScale(boxStep) * System.Numerics.Matrix4x4.CreateTranslation(boxStart);

                            boxes.Add(new MeshGenerator.Shape(
                                boxMatrix,
                                boxTextureMatrix,
                                MeshGenerator.ShapeType.Cube,
                                /*MeshGenerator.ShapeFlags.GroupNumber | */MeshGenerator.ShapeFlags.NoNormals/* | MeshGenerator.ShapeFlags.VertexNumber *//*| MeshGenerator.ShapeFlags.ShapeNumber*/,
                                new float[] {
                                    //0, 0, 0,// padding
                                    brickId
                                    //atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize, brickId // box coord
                                }, boxBones));
                        }

#if LOD0_8BIT
                        zeroLodData[ix, iy, iz, 0] = PackFloatToSByte(cell.Item2);

                        zeroLodData[ix, iy, iz, 1] = (byte)(atlasX);
                        zeroLodData[ix, iy, iz, 2] = (byte)(atlasY);
                        zeroLodData[ix, iy, iz, 3] = (byte)(atlasZ);
#else
                        zeroLodData[ix, iy, iz, 0] = (new HalfFloat(cell.Item2/* * 0.5f + 0.5f*/)).Data;//(new HalfFloat(cell.Item1 * 0.5f + 0.5f)).Data;
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

#if USE_TEXTURE
            if (texture != null)
                Helper.SaveKTX(Helper.KTX_RGBA8, topLodTexture, outFile, "_lod_2_texture.3d.ktx");
#endif
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


        private static ValueTuple<int, float>[][] GetBoxWeights(Dictionary<Vector3i, ValueTuple<int, float>[]> cache, Array3D<float> data, int ix, int iy, int iz, float step)
        {
            int ubx = data.Width - 1;
            int uby = data.Height - 1;
            int ubz = data.Depth - 1;

            ValueTuple<int, float>[][] result = new ValueTuple<int, float>[8][];

            List<ValueTuple<Vector, ValueTuple<int, float>[]>>[] vertexWeights = new List<ValueTuple<Vector, ValueTuple<int, float>[]>>[8];

            for (int i = 0; i < 8; i++)
                vertexWeights[i] = new List<(Vector, (int, float)[])>();

            for (int z = 0; z < data.Depth - 1; z++)
                for (int y = 0; y < data.Height - 1; y++)
                    for (int x = 0; x < data.Width - 1; x++)
                    {
                        float dist = data[x, y, z, 0];
                        int sign = Math.Sign(dist);

                        // checks if a surface crosses nearest 2x2x2 texel cube
                        if (Math.Abs(dist) <= 1 ||
                            sign != Math.Sign(data[x + 1, y + 0, z + 0, 0]) ||
                            sign != Math.Sign(data[x + 0, y + 1, z + 0, 0]) ||
                            sign != Math.Sign(data[x + 1, y + 1, z + 0, 0]) ||
                            sign != Math.Sign(data[x + 1, y + 0, z + 1, 0]) ||
                            sign != Math.Sign(data[x + 0, y + 1, z + 1, 0]) ||
                            sign != Math.Sign(data[x + 1, y + 1, z + 1, 0]) ||
                            sign != Math.Sign(data[x + 1, y + 1, z + 1, 0])
                            )
                        {
                            var weigths = GetWeights(data, x, y, z, 5, 6, 7, 8, 9, 10, 11, 12);

                            vertexWeights[0].Add(new ValueTuple<Vector, ValueTuple<int, float>[]>(new Vector(x, y, z), weigths));
                            vertexWeights[1].Add(new ValueTuple<Vector, ValueTuple<int, float>[]>(new Vector(ubx - x, y, z), weigths));
                            vertexWeights[2].Add(new ValueTuple<Vector, ValueTuple<int, float>[]>(new Vector(ubx - x, uby - y, z), weigths));
                            vertexWeights[3].Add(new ValueTuple<Vector, ValueTuple<int, float>[]>(new Vector(x, uby - y, z), weigths));
                            vertexWeights[4].Add(new ValueTuple<Vector, ValueTuple<int, float>[]>(new Vector(x, uby - y, ubz - z), weigths));
                            vertexWeights[5].Add(new ValueTuple<Vector, ValueTuple<int, float>[]>(new Vector(ubx - x, uby - y, ubz - z), weigths));
                            vertexWeights[6].Add(new ValueTuple<Vector, ValueTuple<int, float>[]>(new Vector(ubx - x, y, ubz - z), weigths));
                            vertexWeights[7].Add(new ValueTuple<Vector, ValueTuple<int, float>[]>(new Vector(x, y, ubz - z), weigths));
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

                    foreach (var bone in pair.Item2)
                    {
                        ValueTuple<float, int> oldValue;
                        if (!weightValues.TryGetValue(bone.Item1, out oldValue))
                            oldValue = new ValueTuple<float, int>(0, 0);

                        oldValue.Item1 += bone.Item2 * weight;
                        oldValue.Item2++;

                        weightValues[bone.Item1] = oldValue;
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

        private static ValueTuple<int, float>[] GetWeights(Dictionary<Vector3i, ValueTuple<int, float>[]> cache, Vector3i index, Array3D<float> data, int ix, int iy, int iz, params int[] indices)
        {
            ValueTuple<int, float>[] result;

            if (!cache.TryGetValue(index, out result))
            {
                result = GetWeights(data, ix, iy, iz, indices);
                cache[index] = result;
            }
            return result;
        }

        private static ValueTuple<int, float>[] GetWeights(Array3D<float> arr, int ix, int iy, int iz, params int[] indices)
        {
            ValueTuple<int, float>[] result;

            result = new ValueTuple<int, float>[indices.Length / 2];

            for (int i = 0; i < result.Length; i++)
                result[i] = new ValueTuple<int, float>((int)arr[ix, iy, iz, indices[i * 2]], arr[ix, iy, iz, indices[i * 2 + 1]]);

            return result;
        }

        private static float[] GetTrilinear(Array3D<float> arr, float px, float py, float pz)
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


            float[] result = new float[arr.Components];
            for (int c = 0; c < result.Length; c++)
            {
                for (int i = 0; i < coef.Length; i++)
                {
                    Vector3i ip = new Vector3i(ix + shifts[i].X, iy + shifts[i].Y, iz + shifts[i].Z);

                    if (ip.X >= arr.Width || ip.Y >= arr.Height || ip.Z >= arr.Depth)
                        continue;

                    result[c] += coef[i] * arr[ip.X, ip.Y, ip.Z, c];
                }
            }
            return result;
        }

        private static Array3D<float> GenerateLod(Array3D<float> topLodDistance, int lodSize)
        {
            Array3D<float> result = new Array3D<float>(topLodDistance.Components, lodSize, lodSize, lodSize);

            float step = (topLodDistance.Depth - 1.0f) / (lodSize - 1.0f);

            for (int nz = 0; nz < lodSize; nz++)
            {
                for (int ny = 0; ny < lodSize; ny++)
                {
                    for (int nx = 0; nx < lodSize; nx++)
                    {
                        float[] value = GetTrilinear(topLodDistance, nx * step, ny * step, nz * step);

                        for (int c = 0; c < value.Length; c++)
                        {
                            result[nx, ny, nz, c] = value[c];
                        }
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

