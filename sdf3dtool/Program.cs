//#define LOD0_8BIT
//#define LOD2_8BIT
#define FAST_CELL_CHECK

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing.Drawing2D;
using System.IO;
using System.Reflection;
using System.Security.Policy;
using System.Threading.Tasks;
using Assimp;
using Assimp.Unmanaged;
using RunMobile.Utility;

using Vector = System.Numerics.Vector3;

namespace SDFTool
{
    static class Program
    {
        private static readonly string s_syntax = "Syntax: {0} input.dae output.ktx [grid_cells] [lod_0_size] [top_lod_cell_size]\n" +
        "\n";

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
            //importer.SetConfig(new Assimp.Configs.BooleanPropertyConfig(AiConfigs.AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false));
            //importer.SetConfig(new Assimp.Configs.BooleanPropertyConfig(AiConfigs.AI_CONFIG_IMPORT_FBX_OPTIMIZE_EMPTY_ANIMATION_CURVES, false));
            //importer.SetConfig(new Assimp.Configs.BooleanPropertyConfig(AiConfigs.AI_CONFIG_IMPORT_REMOVE_EMPTY_BONES, false));
            //importer.SetConfig(new ACSeparateBackfaceCullConfig(false));
            //importer.SetConfig(new KeepSceneHierarchyConfig(true));
            //importer.SetConfig(new NormalSmoothingAngleConfig(80));
            Scene scene = importer.ImportFile(fileName,
                0
                | PostProcessSteps.GenerateSmoothNormals
                //| PostProcessSteps.Triangulate
                //| PostProcessSteps.PreTransformVertices
                //| PostProcessSteps.JoinIdenticalVertices
                //| PostProcessSteps.OptimizeGraph
                //| PostProcessSteps.Debone
                );

            Console.WriteLine("[{0}] File loaded", sw.Elapsed);

            int textureWidth = 0;
            int textureHeight = 0;
            int[] texture = textureFile == null ? null : Helper.LoadBitmap(textureFile, out textureWidth, out textureHeight);

            float scale = 1;

            Vector sceneMin = new Vector(float.MaxValue, float.MaxValue, float.MaxValue);
            Vector sceneMax = new Vector(float.MinValue, float.MinValue, float.MinValue);
            
            Matrix4x4 matrix = Matrix4x4.Identity;

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
            } else
                matrix = Matrix4x4.FromScaling(new Vector3D(scale));
            
            List<PreparedTriangle> triangleList = new List<PreparedTriangle>();

            List<ValueTuple<string, Bone>> bones = new List<ValueTuple<string, Bone>>();

            //Preprocess(scene, scene.RootNode, matrix, triangleList, bones);

            //Matrix4x4 vertexMatrix =
            //matrix;
            //Matrix4x4.Identity;

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

                        Vector3D na = vertexMatrix * mesh.Normals[face.Indices[0]];
                        Vector3D nb = vertexMatrix * mesh.Normals[face.Indices[1]];
                        Vector3D nc = vertexMatrix * mesh.Normals[face.Indices[2]];

                        // This tuple will be used to get back the triangle and vertices when needed
                        Tuple<Mesh, Face> triangleData = new Tuple<Mesh, Face>(mesh, face);

                        PreparedTriangle triangle = new PreparedTriangle(
                            new Vector(va.X, va.Y, va.Z), new Vector(vb.X, vb.Y, vb.Z), new Vector(vc.X, vc.Y, vc.Z),
                            triangleData,
                            new Vector(na.X, na.Y, na.Z), new Vector(nb.X, nb.Y, nb.Z), new Vector(nc.X, nc.Y, nc.Z)
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

            float maxSide = Math.Max(Math.Max(sceneMax.Z - sceneMin.Z, sceneMax.Y - sceneMin.Y), sceneMax.X - sceneMin.X);

            // maximum pixel size of lod 0 in any dimension

            // lod 0 cell size is 1
            int paddedTopLodCellSize = topLodCellSize;
            topLodCellSize--;

            // number of extra pixels on the each size
            int lodPadding = 1;

            // multiply by this value to convert scene units to pixels
            float sceneToPixels = ((lod0pixels - lodPadding * 2) * topLodCellSize) / maxSide; // step for lod 2

            // multiply by this value to convert pixels to scene units
            float pixelsToScene = 1.0f / sceneToPixels;

            // divide by this value to convert scene units to 0..1
            float sceneToRelative = maxSide;


            //creating grid structure for faster triangle search
            TriangleMap triangleMap = new TriangleMap(sceneMin, sceneMax, gridCellCount, triangleList);

            Console.WriteLine("Bounding box: {0} - {1}, step {2}, triangles {3}, cells {4}, instances {5}", sceneMin, sceneMax, sceneToPixels, triangleMap.TriangleCount, triangleMap.CellsUsed, triangleMap.TriangleInstances);

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

            Vector lowerBound = new Vector(-topLodCellSize * pixelsToScene) + sceneMin;
            Vector upperBound = new Vector(topLodCellSize * pixelsToScene) + sceneMax; // new Vector(sx, sy, sz) * step + lowerBound;

            // re-calculate max side

            //float maximumDistance = sceneToPixels;
            //Math.Max(Math.Max(sx, sy), sz) * step;
            //Math.Max(Math.Max(sceneMax.X - sceneMin.X, sceneMax.Y - sceneMin.Y), sceneMax.Z - sceneMin.Z);
            //Vector.Distance(sceneMax, sceneMin);

            Console.WriteLine("[{0}] File preprocessed. X: {1}, Y: {2}, Z: {3}, maximum distance: {4}", sw.Elapsed, sx, sy, sz, sceneToRelative);

            //float emptyTextureCellCheckDistance = step * 1.73205080757f / maximumDistance; // value to make sure that every point is surrounded by at least two empty bricks
            //float emptyCellCheckDistance = 0.5f * step / maximumDistance; // acceptable value
            float emptyCellCheckDistance = topLodCellSize * 0.5f;// (sceneToPixels / topLodCellSize) / sceneToRelative; // acceptable value

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

            Dictionary<int, ValueTuple<int, float>[]> boneDictionary = new Dictionary<int, ValueTuple<int, float>[]>();

            foreach (var pair in tempBoneDictionary)
                boneDictionary[pair.Key] = pair.Value.ToArray();

            Array3D<float> data = new Array3D<float>(13, sx, sy, sz); // we are using lod 2 data for processing
            Array2D<byte> normalData = new Array2D<byte>(3, 1024, 1024);

            int count = 0;
            int maxcount = sz * sy * sx;

            Iterate(0, maxcount, (i) =>
            {
                int iz = i / (sx * sy);
                int iy = (i % (sx * sy)) / sx;
                int ix = (i % (sx * sy)) % sx;

                if (ix == 0 && iy == 0)
                {
                    int c = System.Threading.Interlocked.Increment(ref count);
                    Console.WriteLine("[{0}] Processing {2:P2} (Z = {1})", sw.Elapsed, iz, count * sx * sy/(float)maxcount);
                }

#if DEBUG_IMAGES
                byte[] testData = new byte[sx * sy * 4];
#endif
                Vector point = lowerBound + new Vector(ix, iy, iz) * pixelsToScene;

                float sceneDistance;
                Vector triangleWeights;
                PreparedTriangle triangle;

                float pixelDistance;

                bool empty = !triangleMap.FindTriangles(point, out sceneDistance, out triangleWeights, out triangle);

                pixelDistance = Math.Sign(sceneDistance) * Math.Min(Math.Abs(sceneDistance * sceneToPixels), 1.0f);

                // the distance
                //lock (data)
                {
                    // distance in brick units [-1.0;1.0] where 1 corresponds to brick size
                    data[ix, iy, iz, 0] = pixelDistance / topLodCellSize;

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

                        Vector normal = triangle.Normal;

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

                            int tcx = (int)(tc.X * normalData.Width);
                            int tcy = (int)(tc.Y * normalData.Height);

                            for (int ry = -4; ry < 4; ry++)
                            {
                                int iry = tcy + ry;
                                if (iry < 0 || iry >= normalData.Height)
                                    continue;

                                for (int rx = -4; rx < 4; rx++)
                                {
                                    int irx = tcx + rx;
                                    if (irx < 0 || irx >= normalData.Width)
                                        continue;

                                    if (rx != 0 && ry != 0 && normalData[irx, iry, 0] == 0)
                                        continue;

                                    normalData[irx, iry, 0] = PackFloatToByte(normal.X);
                                    normalData[irx, iry, 1] = PackFloatToByte(normal.Y);
                                    normalData[irx, iry, 2] = PackFloatToByte(normal.Z);
                                }
                            }
                        }

                        data[ix, iy, iz, 3] = normal.X;
                        data[ix, iy, iz, 4] = normal.Y; // no need for Z, it can be easily calculated

                        if (Math.Abs(pixelDistance) <= 1.0f) // no need to calculate weight for non-surface points
                        {
                            //Vector.Dot(triangle.Normal, point -

                            //Dictionary<int, float> topBones = new Dictionary<int, float>(8);

                            //float[] mainBones = new float[bones.Count];

                            //float cellSizeScene = topLodCellSize * pixelsToScene * 0.5f;

                            //Vector3D np = new Vector3D(point.X, point.Y, point.Z);

                            //if ((np - mesh.Vertices[face.Indices[0]]).Length() > cellSizeScene)
                            //    triangleWeights.X = 0;

                            //if ((np - mesh.Vertices[face.Indices[1]]).Length() > cellSizeScene)
                            //    triangleWeights.Y = 0;

                            //if ((np - mesh.Vertices[face.Indices[2]]).Length() > cellSizeScene)
                            //    triangleWeights.Z = 0;

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

                                //if (distance > 0)
                                //{
                                //weight /= Math.Min(1.0f, 1.0f + distance / topLodCellSize);
                                //weight /= 1.0f + Math.Abs(distance) / topLodCellSize;
                                //}

                                if (weight > 0.01f)
                                    boneWeights.Add(new ValueTuple<float, int>(weight, pair.Key));

                                //if (weight > maxWeight)
                                //{
                                //    secondBoneId = maxBoneId;
                                //    secondWeight = maxWeight;
                                //    maxWeight = weight;
                                //    maxBoneId = pair.Key;
                                //}
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

                            data[ix, iy, iz, 5] = maxBoneId;
                            data[ix, iy, iz, 6] = maxWeight;
                            data[ix, iy, iz, 7] = secondBoneId;
                            data[ix, iy, iz, 8] = secondWeight;
                            data[ix, iy, iz, 9] = thirdBoneId;
                            data[ix, iy, iz, 10] = thirdWeight;
                            data[ix, iy, iz, 11] = fourthBoneId;
                            data[ix, iy, iz, 12] = fourthWeight;
                        }
                        // indices 5+ should be weights for the bones and bone numbers
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
            int cellsx = (int)Math.Floor(sx / (float)(topLodCellSize));
            int cellsy = (int)Math.Floor(sy / (float)(topLodCellSize));
            int cellsz = (int)Math.Floor(sz / (float)(topLodCellSize));

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

#if FAST_CELL_CHECK
                        //if (minDistance < emptyCellCheckDistance)//emptyTextureCellCheckDistance)//
                        if ((countPositive !=0 && countNegative != 0))// || minDistance < emptyCellCheckDistance) // all neg or all pos is ignored
                        {
                            usedCells++;
                        }
                        else
                        {
                            block = null;
                        }
#endif
                        cells[index] = new Tuple<Array3D<float>, float, float, Vector>(block, distancePercentage, minDistance, minPoint);
                    }
                }
            }
#if !FAST_CELL_CHECK
            for (int iz = 0; iz < cellsz; iz++)
            {
                for (int iy = 0; iy < cellsy; iy++)
                {
                    for (int ix = 0; ix < cellsx; ix++)
                    {
                        int index = ix + iy * cellsx + iz * cellsx * cellsy;

                        int fromz = cells[index].Item4.Z < 0.5 ? Math.Max(iz - 1, 0) : iz;
                        int toz = cells[index].Item4.Z > 0.5 ? Math.Min(iz + 1, cellsz - 1) : iz;
                        int fromy = cells[index].Item4.Y < 0.5 ? Math.Max(iy - 1, 0) : iy;
                        int toy = cells[index].Item4.Y > 0.5 ? Math.Min(iy + 1, cellsy - 1) : iy;
                        int fromx = cells[index].Item4.X < 0.5 ? Math.Max(ix - 1, 0) : ix;
                        int tox = cells[index].Item4.X > 0.5 ? Math.Min(ix + 1, cellsx - 1) : ix;

                        bool hasData = false;

                        for (int nz = fromz; nz <= toz; nz++)
                        {
                            for (int ny = fromy; ny <= toy; ny++)
                            {
                                for (int nx = fromx; nx <= tox; nx++)
                                {
                                    int nindex = nx + ny * cellsx + nz * cellsx * cellsy;

                                    if (Math.Abs(cells[nindex].Item3) < emptyCellCheckDistance)
                                    {
                                        hasData = true;
                                        break;
                                    }
                                }

                                if (hasData)
                                    break;
                            }
                            if (hasData)
                                break;
                        }

                        if (!hasData)
                            cells[index] = new Tuple<Array3D<float>, float, float, Vector>(null, cells[index].Item2, cells[index].Item3, cells[index].Item4);
                        else
                            usedCells++;
                    }
                }
            }
#endif

            int packx;
            int packy;
            int packz;
            FindBestDividers(usedCells + 1, out packx, out packy, out packz, 256);

            //FindBestDividers2D(usedCells + 1, out packx, out packy, 256);

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
            Array3D<byte> topLodNormal = new Array3D<byte>(3, packx * paddedTopLodCellSize, packy * paddedTopLodCellSize, packz * paddedTopLodCellSize);

            Console.WriteLine("[{0}] Got {1} empty cells, cell grid size {2}, {3:P}, total {4} of {5}x{5}x{5} cells, size {6} vs {7}, grid {8}x{9}x{10}",
                sw.Elapsed, totalCells - usedCells, new Vector3i(cellsx, cellsy, cellsz), usedCells / (float)totalCells,
                usedCells,
                topLodCellSize,
                topLodDistance.Bytes,
                data.Bytes,
                packx, packy, packz
                );
            //Console.WriteLine("[{0}] SDF finished, saving KTX", sw.Elapsed);

            //Helper.SaveKTX(Helper.KTX_RGBA16F, data.Width, data.Height, data.Depth, data.Data, outFile);

            Console.WriteLine("[{0}] Full size KTX saved, saving lower LOD", sw.Elapsed);

            //ushort[] partialData = new ushort[paddedCellSize * paddedCellSize * paddedCellSize * usedCells.Count * 4];
            int brickId = 0;

            float packLodCoef = lod0pixels;

#if LOD0_8BIT
            Array3D<byte> zeroLodData = new Array3D<byte>(4, cellsx, cellsy, cellsz);
#else
            Array3D<ushort> zeroLodData = new Array3D<ushort>(4, cellsx, cellsy, cellsz);
#endif
            //for (int iz = 0; iz < paddedTopLodCellSize; iz++)
            //    for (int iy = 0; iy < paddedTopLodCellSize; iy++)
            //        for (int ix = 0; ix < paddedTopLodCellSize; ix++)
            //            topLodDistance[ix, iy, iz, 0] = 127;

            Vector boxStep = (upperBound - lowerBound) / new Vector(cellsx, cellsy, cellsz);
            //Vector boxUVStep = new Vector(1.0f, 1.0f, 1.0f) / new Vector(cellsx - 1, cellsy - 1, cellsz - 1);

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
                            Array3D<byte> normalBlock = new Array3D<byte>(3, paddedTopLodCellSize, paddedTopLodCellSize, paddedTopLodCellSize);

                            //float minu = 1.0f;
                            //float maxu = 0.0f;
                            //float minv = 1.0f;
                            //float maxv = 0.0f;

                            for (int z = 0; z < cell.Item1.Depth; z++)
                                for (int y = 0; y < cell.Item1.Height; y++)
                                    for (int x = 0; x < cell.Item1.Width; x++)
                                    {
                                        float distance = cell.Item1[x, y, z, 0];
                                        float u = cell.Item1[x, y, z, 1];
                                        float v = cell.Item1[x, y, z, 2];

                                        //if (u < minu)
                                        //    minu = u;
                                        //if (v < minv)
                                        //    minv = v;
                                        //if (u > maxu)
                                        //    maxu = u;
                                        //if (v > maxv)
                                        //    maxv = v;

                                        distanceBlock[x, y, z, 0] = distance;

                                        uvBlock[x, y, z, 0] = u;
                                        uvBlock[x, y, z, 1] = v;

                                        if (texture != null)
                                        {
                                            int textureColor = texture != null ? texture[(int)(Clamp(u * textureWidth, 0.0f, textureWidth - 1)) + ((int)Clamp((1.0f - v) * textureHeight, 0.0f, textureHeight - 1)) * textureWidth] : 0;

                                            textureBlock[x, y, z, 0] = (byte)(textureColor >> 16);
                                            textureBlock[x, y, z, 1] = (byte)(textureColor >> 8);
                                            textureBlock[x, y, z, 2] = (byte)(textureColor);
                                            textureBlock[x, y, z, 3] = (byte)(textureColor >> 24);
                                        }

                                        normalBlock[x, y, z, 0] = PackFloatToSByte(cell.Item1[x, y, z, 3]);
                                        normalBlock[x, y, z, 1] = PackFloatToSByte(cell.Item1[x, y, z, 4]);
                                        //normalBlock[x, y, z, 2] = (byte)cell.Item1[x, y, z, 5];
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
                            topLodDistance.PutBlock(distanceBlock, atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize, (k) => PackFloatToUShort(k - dist));
#endif

                            int lodSize = paddedTopLodCellSize;
                            for (int i = 1; i < nlods; i++)
                            {
                                lodSize /= 2;
                                Array3D<float> lodDistanceBlock = GenerateLod(distanceBlock, lodSize);

#if LOD2_8BIT
                                lodDistance[i].PutBlock(lodDistanceBlock, atlasX * lodSize, atlasY * lodSize, atlasZ * lodSize, (k) => PackFloatToSByte((k - dist) * packLodCoef));
#else
                                lodDistance[i].PutBlock(lodDistanceBlock, atlasX * lodSize, atlasY * lodSize, atlasZ * lodSize, (k) => PackFloatToUShort(k - dist));
#endif
                            }

                            //lod0uv[ix, iy, iz, 0] = PackFloatToUShort(minu);
                            //lod0uv[ix, iy, iz, 1] = PackFloatToUShort(maxu-minu);
                            //lod0uv[ix, iy, iz, 2] = PackFloatToUShort(minv);
                            //lod0uv[ix, iy, iz, 3] = PackFloatToUShort(maxv - minv);

                            //lod1uv.PutBlock(lod1uvBlock, atlasX * paddedLod1cellSize, atlasY * paddedLod1cellSize, atlasZ * paddedLod1cellSize, (k, i) =>
                            //{
                            //    switch (i)
                            //    {
                            //        case 0:
                            //return PackFloatToByte((k - minu) / (maxu - minu));
                            //        case 1:
                            //            return PackFloatToByte((k - minv) / (maxv - minv));
                            //        default:
                            //            return 0;
                            //    }
                            //});
                            topLoduv.PutBlock(uvBlock, atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize, PackFloatToUShort);

                            if (texture != null)
                                topLodTexture.PutBlock(textureBlock, atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize);

                            topLodNormal.PutBlock(normalBlock, atlasX * paddedTopLodCellSize, atlasY * paddedTopLodCellSize, atlasZ * paddedTopLodCellSize);

                            //lod0uv[ix, iy, iz, 0] = PackFloatToUShort(cell.Item2[paddedLod2cellSize / 2, paddedLod2cellSize / 2, paddedLod2cellSize / 2, 1]);
                            //lod0uv[ix, iy, iz, 1] = PackFloatToUShort(cell.Item2[paddedLod2cellSize / 2, paddedLod2cellSize / 2, paddedLod2cellSize / 2, 2]);

                            //surfaces.Add(MeshGenerator.CreateBoxMesh(new  Vector(ix, iy, iz) * cellSize, new Vector(ix + 1, iy + 1, iz + 1) * cellSize, true, "box_" + ix + "_" + iy + "_" + iz, default(System.Numerics.Matrix4x4), false));

                            Vector boxStart = lowerBound + new Vector(ix, iy, iz) * boxStep;
                            System.Numerics.Matrix4x4 boxTextureMatrix =
                                //System.Numerics.Matrix4x4.CreateTranslation(new Vector(-atlasX * paddedTopLodCellSize, -atlasY * paddedTopLodCellSize, -atlasZ * paddedTopLodCellSize));
                                System.Numerics.Matrix4x4.Identity;

                            //CreateTranslation(new Vector(ix + 0.5f, iy + 0.5f, iz + 0.5f)) * System.Numerics.Matrix4x4.CreateScale(1.0f / cellsx, 1.0f / cellsy, 1.0f / cellsz);

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

            SaveKTX(
#if LOD0_8BIT
                Helper.KTX_RGBA8,
#else
                Helper.KTX_RGBA16F,
#endif
                zeroLodData, outFile, "_lod_0.3d.ktx");

#if LOD2_8BIT
            SaveKTX(Helper.KTX_R8, lodDistance, outFile, "_lod.3d.ktx");
#else
            SaveKTX(Helper.KTX_R16F, lodDistance, outFile, "_lod.3d.ktx");
#endif

            //SaveKTX(Helper.KTX_RG16F, lod0uv, outFile, "_lod_0_uv.3d.ktx");
            //SaveKTX(Helper.KTX_RG16F, lod1uv, outFile, "_lod_1_uv.3d.ktx");
            //SaveKTX(Helper.KTX_RG16F, Array3Dto2D(lod1uv, paddedLod1cellSize), outFile, "_lod_1_uv.ktx");
            SaveKTX(Helper.KTX_RG16F, topLoduv, outFile, "_lod_2_uv.3d.ktx");
            //SaveKTX(Helper.KTX_RG16F, Array3Dto2D(lod2uv, paddedLod2cellSize), outFile, "_lod_2_uv.ktx");

            if (texture != null)
                SaveKTX(Helper.KTX_RGBA8, topLodTexture, outFile, "_lod_2_texture.3d.ktx");

            //SaveKTX(Helper.KTX_RGB8, topLodNormal, outFile, "_lod_2_normal.3d.ktx");

            SaveKTX(Helper.KTX_RGB8, normalData, outFile, "_normal.ktx");

            Console.WriteLine("[{0}] KTX saved, saving boundary mesh", sw.Elapsed);

            //Vector uvwScale = Vector.Normalize(new Vector(1.0f/sx, 1.0f / sy, 1.0f / sz));

            //MeshGenerator.Surface surface = MeshGenerator.CreateBoxMesh(lowerBound, upperBound, true, "main_box", default(System.Numerics.Matrix4x4), false);//, Matrix.CreateScale(uvwScale));

            //Helper.SaveUnigineMesh(new MeshGenerator.Surface[] { MeshGenerator.CreateBoxMesh(lowerBound, upperBound, System.Numerics.Matrix4x4.Identity, "main_box", true) }, Path.GetFileNameWithoutExtension(outFile) + "_box");

            //MeshGenerator.Surface[] surfaces = new MeshGenerator.Surface[boxes.Count];
            //for (int l = 0; l < surfaces.Length; l++)
            //{
            //    surfaces[l] = MeshGenerator.CreateBoxMesh(boxes[l], "box_" + l);
            //}

            //Helper.SaveUnigineMesh(surfaces, outFile);

            MeshGenerator.Surface [] boxesSurface = new MeshGenerator.Surface[] { MeshGenerator.CreateBoxesMesh(boxes.ToArray(), "main_box") };

            //Helper.SaveUnigineMesh(boxesSurface, outFile);
            //Helper.SaveObjMesh(boxesSurface, outFile);

            Helper.SaveAssimpMesh(boxesSurface, outFile, new [] { bones.ToArray() }, scene.Animations, scene);
            Helper.SaveUMesh(boxesSurface, outFile, new[] { bones.ToArray() }, scene.Animations, scene, matrix);

            //List<MeshGenerator.Surface> surfaces = new List<MeshGenerator.Surface>();

            //surfaces.Add(surface);

            //foreach (var tuple in bones)
            //{
            //MeshGenerator.Surface bone = MeshGenerator.CreateBoxMesh(tuple.Item1, tuple.Item2, true, tuple.Item3.Name);
            //surfaces.Add(bone);
            //}

            //Helper.SaveUnigineMesh(surfaces.ToArray(), Path.GetFileNameWithoutExtension(outFile) + ".bones");

            Console.WriteLine("[{0}] All done", sw.Elapsed);

            sw.Stop();
        }

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
            /*
            return new ValueTuple<int, float>[][]
                            {
                                //new[]{ new ValueTuple<int, float>((int)cell.Item1[ubx/2,   uby/2,   ubz/2, 5], cell.Item1[ubx/2,   uby/2,   ubz/2, 6]), new ValueTuple<int, float>((int)cell.Item1[ubx / 2, uby / 2, ubz / 2, 7], cell.Item1[ubx / 2, uby / 2, ubz / 2, 8]) },
                                //new[]{ new ValueTuple<int, float>((int)cell.Item1[ubx/2,   uby/2,   ubz/2, 5], cell.Item1[ubx/2,   uby/2,   ubz/2, 6]), new ValueTuple<int, float>((int)cell.Item1[ubx / 2, uby / 2, ubz / 2, 7], cell.Item1[ubx / 2, uby / 2, ubz / 2, 8]) },
                                //new[]{ new ValueTuple<int, float>((int)cell.Item1[ubx/2,   uby/2,   ubz/2, 5], cell.Item1[ubx/2,   uby/2,   ubz/2, 6]), new ValueTuple<int, float>((int)cell.Item1[ubx / 2, uby / 2, ubz / 2, 7], cell.Item1[ubx / 2, uby / 2, ubz / 2, 8]) },
                                //new[]{ new ValueTuple<int, float>((int)cell.Item1[ubx/2,   uby/2,   ubz/2, 5], cell.Item1[ubx/2,   uby/2,   ubz/2, 6]), new ValueTuple<int, float>((int)cell.Item1[ubx / 2, uby / 2, ubz / 2, 7], cell.Item1[ubx / 2, uby / 2, ubz / 2, 8]) },
                                //new[]{ new ValueTuple<int, float>((int)cell.Item1[ubx/2,   uby/2,   ubz/2, 5], cell.Item1[ubx/2,   uby/2,   ubz/2, 6]), new ValueTuple<int, float>((int)cell.Item1[ubx / 2, uby / 2, ubz / 2, 7], cell.Item1[ubx / 2, uby / 2, ubz / 2, 8]) },
                                //new[]{ new ValueTuple<int, float>((int)cell.Item1[ubx/2,   uby/2,   ubz/2, 5], cell.Item1[ubx/2,   uby/2,   ubz/2, 6]), new ValueTuple<int, float>((int)cell.Item1[ubx / 2, uby / 2, ubz / 2, 7], cell.Item1[ubx / 2, uby / 2, ubz / 2, 8]) },
                                //new[]{ new ValueTuple<int, float>((int)cell.Item1[ubx/2,   uby/2,   ubz/2, 5], cell.Item1[ubx/2,   uby/2,   ubz/2, 6]), new ValueTuple<int, float>((int)cell.Item1[ubx / 2, uby / 2, ubz / 2, 7], cell.Item1[ubx / 2, uby / 2, ubz / 2, 8]) },
                                //new[]{ new ValueTuple<int, float>((int)cell.Item1[ubx/2,   uby/2,   ubz/2, 5], cell.Item1[ubx/2,   uby/2,   ubz/2, 6]), new ValueTuple<int, float>((int)cell.Item1[ubx / 2, uby / 2, ubz / 2, 7], cell.Item1[ubx / 2, uby / 2, ubz / 2, 8]) },

                                GetWeights(cache, new Vector3i(ix, iy, iz), data, 0, 0, 0, 5, 6, 7, 8, 9, 10, 11, 12),
                                GetWeights(cache, new Vector3i(ix+1, iy, iz), data, ubx, 0, 0, 5, 6, 7, 8, 9, 10, 11, 12),
                                GetWeights(cache, new Vector3i(ix+1, iy+1, iz), data, ubx, uby, 0, 5, 6, 7, 8, 9, 10, 11, 12),
                                GetWeights(cache, new Vector3i(ix, iy+1, iz), data, 0, uby, 0, 5, 6, 7, 8, 9, 10, 11, 12),
                                GetWeights(cache, new Vector3i(ix, iy+1, iz+1), data, 0, uby, ubz, 5, 6, 7, 8, 9, 10, 11, 12),
                                GetWeights(cache, new Vector3i(ix+1, iy+1, iz+1), data, ubx, uby, ubz, 5, 6, 7, 8, 9, 10, 11, 12),
                                GetWeights(cache, new Vector3i(ix+1, iy, iz+1), data, ubx, 0, ubz, 5, 6, 7, 8, 9, 10, 11, 12),
                                GetWeights(cache, new Vector3i(ix, iy, iz+1), data, 0, 0, ubz, 5, 6, 7, 8, 9, 10, 11, 12),
                            };*/
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
                        float [] value = GetTrilinear(topLodDistance, nx * step, ny * step, nz * step);
                        
                        for (int c = 0; c < value.Length; c++)
                        {
                            result[nx, ny, nz, c] = value[c];
                        }
                    }
                }
            }

            return result;
        }

        private static void SaveKTX(int format, Array3D<ushort> [] arrays, string outFile, string extension)
        {
            ushort[][] data = new ushort[arrays.Length][];
            for (int i = 0; i < arrays.Length; i++)
                data[i] = arrays[i].Data;

            Helper.SaveKTX(format, arrays[0].Width, arrays[0].Height, arrays[0].Depth, data, Path.GetFileNameWithoutExtension(outFile) + extension);
        }

        private static void SaveKTX(int format, Array3D<byte>[] arrays, string outFile, string extension)
        {
            byte[][] data = new byte[arrays.Length][];
            for (int i = 0; i < arrays.Length; i++)
                data[i] = arrays[i].Data;

            Helper.SaveKTX(format, arrays[0].Width, arrays[0].Height, arrays[0].Depth, data, Path.GetFileNameWithoutExtension(outFile) + extension);
        }

        private static void SaveKTX(int format, Array3D<ushort> array3d, string outFile, string extension)
        {
            Helper.SaveKTX(format, array3d.Width, array3d.Height, array3d.Depth, new ushort[][] { array3d.Data }, Path.GetFileNameWithoutExtension(outFile) + extension);
        }

        private static void SaveKTX(int format, Array3D<byte> array3d, string outFile, string extension)
        {
            Helper.SaveKTX(format, array3d.Width, array3d.Height, array3d.Depth, new byte[][] { array3d.Data }, Path.GetFileNameWithoutExtension(outFile) + extension);
        }

        private static void SaveKTX(int format, Array2D<ushort> array2d, string outFile, string extension)
        {
            Helper.SaveKTX(format, array2d.Width, array2d.Height, 0, new ushort[][] { array2d.Data }, Path.GetFileNameWithoutExtension(outFile) + extension);
        }

        private static void SaveKTX(int format, Array2D<byte> array2d, string outFile, string extension)
        {
            Helper.SaveKTX(format, array2d.Width, array2d.Height, 0, new byte[][] { array2d.Data }, Path.GetFileNameWithoutExtension(outFile) + extension);
        }

        private static Array2D<T> Array3Dto2D<T>(Array3D<T> array3d, int blockSize) where T:struct
        {
            Debug.Assert(array3d.Depth == blockSize);

            Array2D<T> array2d = new Array2D<T>(array3d.Components, array3d.Width * blockSize, array3d.Height);

            for (int nz = 0; nz < array3d.Depth; nz += blockSize)
            {
                for (int ny = 0; ny < array3d.Height; ny += blockSize)
                {
                    for (int nx = 0; nx < array3d.Width; nx += blockSize)
                    {

                        for (int z = 0; z < blockSize; z++)
                        {
                            for (int y = 0; y < blockSize; y++)
                            {
                                for (int x = 0; x < blockSize; x++)
                                {
                                    for (int c = 0; c < array3d.Components; c++)
                                        array2d[(nx + z) * blockSize + x, ny + y, c] = array3d[nx + x, ny + y, nz + z, c];
                                }
                            }
                        }
                    }
                }
            }

            return array2d;
        }

        private static float Clamp(float value, float min, float max)
        {
            return Math.Max(min, Math.Min(max, value));
        }

        private static byte PackFloatToSByte(float value)
        {
            value = Clamp(value, -1.0f, 1.0f);
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
            int cpus = Environment.ProcessorCount;

            cpus /= 2;
#if !CUSTOM_FOR
            Parallel.For(from, to, new ParallelOptions { MaxDegreeOfParallelism = cpus }, action);
#else
            if (cpus <= 1 || from >= to)
            {
                for (int j = from; j < to; j++)
                    action(j);
                return;
            }

            int split = (to - from) / cpus;
            if (split == 0)
            {
                split = 1;
                cpus = to - from;
            }

            int done = 0;

            for (int i = 0; i < cpus - 1; i++)
            {
                int jfrom = i * split;
                int jto = (i + 1) * split;
                System.Threading.Thread thread = new System.Threading.Thread(delegate ()
                {
                    for (int j = jfrom; j < jto; j++)
                        action(j);

                    System.Threading.Interlocked.Increment(ref done);
                });


                thread.Start();
            }

            int ifrom = (cpus - 1) * split;
            for (int j = ifrom; j < to; j++)
                action(j);

            System.Threading.Interlocked.Increment(ref done);

            while (done < cpus)
                System.Threading.Thread.Sleep(1);
#endif
#else
            for (int i = from; i < to; i++)
                action(i);
#endif
        }

        /// <summary>
        /// Tries to find a best way to divide value by three numbers
        /// </summary>
        /// <param name="value"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="z"></param>
        /// <param name="max"></param>
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

        /// <summary>
        /// Tries to find a best way to divide value by two numbers
        /// </summary>
        /// <param name="value"></param>
        /// <param name="widthCoef"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="max"></param>
        private static void FindBestDividers2D(int value, out int x, out int y, int max)
        {
            int root = (int)Math.Ceiling(Math.Pow(value, 1 / 2.0));

            x = root; y = root;
            int closest = x * y;

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

                    int nvalue = lx * ly;

                    if (nvalue < value)
                        continue;

                    if (nvalue < closest)
                    {
                        x = lx;
                        y = ly;

                        closest = nvalue;

                        if (nvalue == value)
                            return;
                    }
                }
            }
        }

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

        private static void Preprocess(Scene scene, Node node, Matrix4x4 matrix, IList<PreparedTriangle> allTriangles, IList<Tuple<Vector, Vector, Bone, string>> bones)
        {
            Matrix4x4 nmatrix =
                //Matrix4x4.Identity;
                node.Transform * matrix;

            if (node.HasMeshes)
            {
#if USE_PSEUDO_NORMALS
                Dictionary<VectorPair, PseudoNormal> edgeNormals = new Dictionary<VectorPair, PseudoNormal>();
                Dictionary<Vector3i, PseudoNormal> vertexNormals = new Dictionary<Vector3i, PseudoNormal>();
#endif
                foreach (int index in node.MeshIndices)
                {
                    Mesh mesh = scene.Meshes[index];

                    Vector3D [] vertices = new Vector3D[mesh.VertexCount];
                    Vector3D[] normals = new Vector3D[mesh.VertexCount];
                    //Matrix4x4[] transforms = new Matrix4x4[mesh.VertexCount];

                    for (int i = 0; i < vertices.Length; i++)
                    {
                        vertices[i] = nmatrix * mesh.Vertices[i];
                        normals[i] = mesh.HasNormals ? nmatrix * mesh.Normals[i] : new Vector3D();
                        //    transforms[i] = /*mesh.HasBones ? new Matrix4x4() : */Matrix4x4.Identity;
                    }

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
                                    Vector3D v = /*bone.OffsetMatrix **/ vertices[bone.VertexWeights[j].VertexID];
                                    float w = bone.VertexWeights[j].Weight;
                                    minx = Math.Min(minx, v.X);
                                    miny = Math.Min(miny, v.Y);
                                    minz = Math.Min(minz, v.Z);
                                    maxx = Math.Max(maxx, v.X);
                                    maxy = Math.Max(maxy, v.Y);
                                    maxz = Math.Max(maxz, v.Z);

                                    //if (w >= 0.5f)
                                    //{
                                    //    transforms[i] = new Matrix4x4(bone.OffsetMatrix);
                                    //    transforms[i].Inverse();
                                    //}
                                }

                                Vector3D min = new Vector3D(minx, miny, minz);
                                Vector3D max = new Vector3D(maxx, maxy, maxz);

                                bones.Add(new Tuple<Vector, Vector, Bone, string>(new Vector(min.X, min.Y, min.Z), new Vector(max.X, max.Y, max.Z), bone, node.Name));
                            }
                        }
                    }

                    for (int i = 0; i < mesh.FaceCount; i++)
                    {
                        Face face = mesh.Faces[i];

                        if (face.HasIndices && face.IndexCount == 3) // only process triangles. Don't have a clue what to do with other primitives
                        {
                            Vector3D va = vertices[face.Indices[0]];
                            Vector3D vb = vertices[face.Indices[1]];
                            Vector3D vc = vertices[face.Indices[2]];

                            Vector3D na = normals[face.Indices[0]];
                            Vector3D nb = normals[face.Indices[1]];
                            Vector3D nc = normals[face.Indices[2]];

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

                            PreparedTriangle triangle = new PreparedTriangle(a, b, c, data,
                                new Vector(na.X, na.Y, na.Z), new Vector(nb.X, nb.Y, nb.Z),new Vector(nc.X, nc.Y, nc.Z)
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
                Preprocess(scene, node.Children[i], nmatrix, allTriangles, bones);
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

