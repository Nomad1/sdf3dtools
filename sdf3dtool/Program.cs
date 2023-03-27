//#define LOD0_8BIT
//#define LOD2_8BIT

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Runtime.InteropServices;
using Assimp;
using RunMobile.Utility;
using RunServer.SdfTool;
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

        private static void ProcessAssimpImport(string fileName, string outFile, int gridCellCount = 64, int lod0pixels = 32, int topLodCellSize = 4)
        {
            if (topLodCellSize <= 2)//(topLodCellSize & (topLodCellSize - 1)) != 0)
            {
                Console.Error.WriteLine("Top LOD cell size should be a power of two and greater than 2 (4, 8, 16, 32, etc.)!");
                return;
            }

#if USE_LODS
            int nlods = 0;
            int lod = topLodCellSize;

            while (lod >= 2 && lod % 2 == 0)
            {
                nlods++;
                lod /= 2;
            }
#else
            int numberOfLods = 1;
#endif


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

            Console.WriteLine("[{0}] File preprocessed. X: {1}, Y: {2}, Z: {3}, maximum distance: {4}", sw.Elapsed, sx, sy, sz, maxSide);

            int maxcount = sz * sy * sx;

            float[] distanceData = new float[maxcount * 4];

            // Use TriangleMap to generate raw SDF grid
            TriangleMap triangleMap = new TriangleMap(sceneMin, sceneMax, gridCellCount, triangleList);

            Console.WriteLine("Bounding box: {0} - {1}, step {2}, triangles {3}, cells {4}, instances {5}", sceneMin, sceneMax, sceneToPixels, triangleMap.TriangleCount, triangleMap.CellsUsed, triangleMap.TriangleInstances);

            triangleMap.Dispatch(distanceData, lowerBound, pixelsToScene, sceneToPixels / topLodCellSize, sx, sy, sz, (progress) => Console.WriteLine("[{0}] Processing {1:P2}", sw.Elapsed, progress));

            // Fill in data structure with SDF data and additional parameters from original model
            PixelData[] data = GetPixelData(distanceData, boneDictionary, triangleList);

            Console.WriteLine("[{0}] SDF data ready", sw.Elapsed);

            ValueTuple<Vector, Vector, int, ValueTuple<int, float>[][]> [] nboxes;
            float[][] alods;
            System.Numerics.Vector2[] nuv;
            Vector4[] nzeroLod;
            Vector3i topLodTextureSize;

            // find non-empty cells
            int usedCells = CellProcessor.ProcessCells(data, dataSize, topLodCellSize, paddedTopLodCellSize, lowerBound, upperBound, numberOfLods,
                out topLodTextureSize,
                out alods, out nuv, out nzeroLod, out nboxes);

            Array3D<ushort>[] lods = new Array3D<ushort>[alods.Length];
            for (int i = 0; i < alods.Length; i++)
            {
                lods[i] = new Array3D<ushort>(1, topLodTextureSize.X, topLodTextureSize.Y, topLodTextureSize.Z);
                for (int j = 0; j < alods[i].Length; j++)
                    lods[i][j] = Helper.PackFloatToUShort(alods[i][j]);

                break; // only top lod is supported for now
            }

            Array3D<ushort> uv = new Array3D<ushort>(2, topLodTextureSize.X, topLodTextureSize.Y, topLodTextureSize.Z);
            for (int j = 0; j < nuv.Length; j++)
            {
                uv[j * 2 + 0] = Helper.PackFloatToUShort(nuv[j].X);
                uv[j * 2 + 1] = Helper.PackFloatToUShort(nuv[j].Y);
            }

            Array3D<ushort> zeroLod = new Array3D<ushort>(4, dataSize.X / topLodCellSize, dataSize.Y / topLodCellSize, dataSize.Z / topLodCellSize);
            for (int j = 0; j < nzeroLod.Length; j++)
            {
                zeroLod[j * 2 + 0] = Helper.PackFloatToUShort(nzeroLod[j].X);
                zeroLod[j * 2 + 1] = Helper.PackFloatToUShort(nzeroLod[j].Y);
                zeroLod[j * 2 + 2] = Helper.PackFloatToUShort(nzeroLod[j].Z);
                zeroLod[j * 2 + 3] = Helper.PackFloatToUShort(nzeroLod[j].W);
            }


            MeshGenerator.Shape[] boxes = new MeshGenerator.Shape[nboxes.Length];

            for (int i = 0; i < boxes.Length; i++)
                boxes[i] = new MeshGenerator.Shape(
                    System.Numerics.Matrix4x4.CreateScale(nboxes[i].Item1) * System.Numerics.Matrix4x4.CreateTranslation(nboxes[i].Item2),
                    System.Numerics.Matrix4x4.Identity,
                    MeshGenerator.ShapeType.Cube,
                    MeshGenerator.ShapeFlags.NoNormals,
                    new float[] {
                        nboxes[i].Item3
                    }, nboxes[i].Item4);


            //Console.WriteLine("[{0}] Got {1} empty cells, cell grid size {2}, {3:P}, total {4} of {5}x{5}x{5} cells, size {6} vs {7}, grid {8}x{9}x{10}",
            //sw.Elapsed, totalCells - usedCells, new Vector3i(cellsx, cellsy, cellsz), usedCells / (float)totalCells,
            //usedCells,
            //topLodCellSize,
            //topLodDistance.Bytes,
            //data.Length * Marshal.SizeOf<PixelData>(),
            //packx, packy, packz
            //);

            Console.WriteLine("[{0}] Saving LODs", sw.Elapsed);

            Helper.SaveKTX(
#if LOD0_8BIT
                Helper.KTX_RGBA8,
#else
                Helper.KTX_RGBA16F,
#endif
                zeroLod, outFile, "_lod_0.3d.ktx");

#if LOD2_8BIT
            Helper.SaveKTX(Helper.KTX_R8, lodDistance, outFile, "_lod.3d.ktx");
#else
            Helper.SaveKTX(Helper.KTX_R16F, lods, outFile, "_lod.3d.ktx");
#endif

            Helper.SaveKTX(Helper.KTX_RG16F, uv, outFile, "_lod_2_uv.3d.ktx");

            Console.WriteLine("[{0}] KTX saved, saving boundary mesh", sw.Elapsed);

            MeshGenerator.Surface[] boxesSurface = new MeshGenerator.Surface[] { MeshGenerator.CreateBoxesMesh(boxes, "main_box") };

            //Helper.SaveAssimpMesh(boxesSurface, outFile, new[] { bones }, scene.Animations, scene);
            Helper.SaveUMesh(boxesSurface, outFile, new[] { bones }, scene.Animations, scene, matrix);

            Console.WriteLine("[{0}] All done", sw.Elapsed);

            sw.Stop();
        }

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

