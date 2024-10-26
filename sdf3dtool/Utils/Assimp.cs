using System;
using System.IO;
using Assimp;
using System.Collections.Generic;
using RunServer.SdfTool;
using System.Diagnostics;

using Vector = System.Numerics.Vector3;
using Vector4 = System.Numerics.Vector4;
using Matrix4x4 = Assimp.Matrix4x4;

namespace SDFTool.Utils
{
    public class Assimp
    {
        /// <summary>
        /// Saves a mesh to Unigine .mesh format
        /// </summary>
        /// <param name="outFile"></param>
        public static void SaveAssimpMesh(MeshGenerator.Surface[] surfaces, string outFile, ValueTuple<string, Bone> [][] bones = null, IList<Animation> animations = null, Scene oldScene = null)
        {
            string file = Path.GetFileNameWithoutExtension(outFile) + ".assimp.fbx";

            Scene scene = new Scene();

            //if (oldScene != null)
            //    foreach (var pair in oldScene.Metadata)
            //        scene.Metadata.Add(pair.Key, pair.Value);

            Material mat = new Material();
            mat.Name = "Material";
            scene.Materials.Add(mat);

            Node meshNode = null;

            if (oldScene != null)
            {
                ProcessNode(oldScene.RootNode, null, scene, ref meshNode);
            }
            else
            {
                scene.RootNode = new Node("Scene Root");
            }

            if (meshNode == null)
                meshNode = scene.RootNode;

            Matrix4x4 meshMatrix = Matrix4x4.Identity;
            //Node node = meshNode;

            //do
            //{
            //    meshMatrix = meshMatrix * node.Transform;
            //    node = node.Parent;
            //} while (node != null);

            //meshMatrix.Inverse();

            for (int j = 0; j < surfaces.Length; j++)
            {
                MeshGenerator.Surface surface = surfaces[j];
                Mesh mesh = new Mesh(surface.Name, surface.Faces[0].Length == 3 ? PrimitiveType.Triangle : PrimitiveType.Polygon);

                Dictionary<int, List<VertexWeight>> vertexWeights = new Dictionary<int, List<VertexWeight>>();

                for (int i = 0; i < surface.VertexWeights.Count; i++)
                {
                    for (int k = 0; k < surface.VertexWeights[i].Length; k++)
                    {
                        ValueTuple<int, float> pair = surface.VertexWeights[i][k];

                        List<VertexWeight> list;

                        if (!vertexWeights.TryGetValue(pair.Item1, out list))
                            vertexWeights[pair.Item1] = list = new List<VertexWeight>();

                        list.Add(new VertexWeight(mesh.Vertices.Count + i, pair.Item2));
                    }
                }

                for (int i = 0; i < surface.Vertices.Count; i++)
                    mesh.Vertices.Add(meshMatrix * new Vector3D(surface.Vertices[i].X, surface.Vertices[i].Y, surface.Vertices[i].Z));

                if (surface.Normals != null && surface.Normals.Count == surface.Vertices.Count)
                    for (int i = 0; i < surface.Normals.Count; i++)
                        mesh.Normals.Add(meshMatrix * new Vector3D(surface.Normals[i].X, surface.Normals[i].Y, surface.Normals[i].Z));

                for (int i = 0; i < surface.TexCoords.Count; i++)
                {
                    // Unity ignores Z coordinate for Collada files and there is no way to specify W coordinate, so let's split everything to many UV channels

                    for (int k = 0; k < surface.TexCoords[i].Length / 2 + surface.TexCoords[i].Length % 2; k++)
                    {
                        mesh.TextureCoordinateChannels[k].Add(new Vector3D(surface.TexCoords[i].Length > k * 2 ? surface.TexCoords[i][k * 2] : 0, surface.TexCoords[i].Length > k * 2 + 1 ? surface.TexCoords[i][k * 2 + 1] : 0, 0));
                        mesh.UVComponentCount[k] = Math.Min(surface.TexCoords[i].Length - k * 2, 2);
                    }
                }

                for (int i = 0; i < surface.Faces.Count; i++)
                    mesh.Faces.Add(new Face(surface.Faces[i]));

                mesh.MaterialIndex = 0;

                if (bones != null)
                {
                    for (int i = 0; i < bones[j].Length; i++)
                        if (vertexWeights.ContainsKey(i + 1))
                            mesh.Bones.Add(new Bone(bones[j][i].Item2.Name, bones[j][i].Item2.OffsetMatrix, vertexWeights[i + 1].ToArray()));
                }

                scene.Meshes.Add(mesh);
                meshNode.MeshIndices.Add(j);
            }

            if (animations != null)
            {
                foreach (Animation oldAnimation in animations)
                {
                    Animation animation = new Animation();
                    animation.Name = oldAnimation.Name;
                    animation.TicksPerSecond = oldAnimation.TicksPerSecond;
                    animation.DurationInTicks = oldAnimation.DurationInTicks;

                    foreach (NodeAnimationChannel oldChannel in oldAnimation.NodeAnimationChannels)
                    {
                        string nodename = oldChannel.NodeName;

                        if (scene.RootNode.FindNode(nodename) == null)
                            continue;

                        NodeAnimationChannel animationChannel = new NodeAnimationChannel();
                        animationChannel.NodeName = nodename;
                        animationChannel.PreState = oldChannel.PreState;
                        animationChannel.PostState = oldChannel.PostState;
                        animationChannel.PositionKeys.AddRange(oldChannel.PositionKeys);
                        animationChannel.RotationKeys.AddRange(oldChannel.RotationKeys);
                        animationChannel.ScalingKeys.AddRange(oldChannel.ScalingKeys);
                        animation.NodeAnimationChannels.Add(animationChannel);
                    }
                    scene.Animations.Add(animation);
                }
            }

            AssimpContext exporter = new AssimpContext();

            //if (!exporter.ExportFile(scene, file, "collada", PostProcessSteps.ValidateDataStructure))// | PostProcessSteps.OptimizeGraph))
            if (!exporter.ExportFile(scene, file, "fbxa"))
                Console.WriteLine("Assimp export failed!");

            //if (!exporter.ExportFile(oldScene, file + ".tmp", "collada"))
                //Console.WriteLine("Assimp old export failed!");
        }

        private static void ProcessNode(Node oldNode, Node parent, Scene scene, ref Node meshNode)
        {
            Node node = new Node(oldNode.Name, parent);
            if (parent == null)
            {
                scene.RootNode = node;
            } else
                parent.Children.Add(node);

            node.Transform = oldNode.Transform;

            foreach (var pair in oldNode.Metadata)
                node.Metadata.Add(pair.Key, pair.Value);

            
            if (meshNode == null && oldNode.HasMeshes)
                meshNode = node;

            foreach (Node child in oldNode.Children)
            {
                ProcessNode(child, node, scene, ref meshNode);
            }
        }


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

        public static DistanceData ProcessAssimpImport(
            string fileName, int gridCellCount, int lod0pixels, int topLodCellSize,
            Stopwatch sw, out Scene scene, out ValueTuple<string, Bone>[][] obones, out Matrix4x4 matrix
            )
        {
            Console.WriteLine("[{0}] Processing file {1}", sw.Elapsed, fileName);

            AssimpContext importer = new AssimpContext();
            scene = importer.ImportFile(fileName, PostProcessSteps.GenerateSmoothNormals | PostProcessSteps.Triangulate);

            Console.WriteLine("[{0}] File loaded", sw.Elapsed);

            float scale = 1;

            Vector sceneMin;
            Vector sceneMax;

            IDictionary<int, ValueTuple<int, float>[]> boneDictionary;
            ValueTuple<string, Bone>[] bones;

            PreparedTriangle[] triangleList = PrepareScene(scene, scale, out sceneMin, out sceneMax, out matrix, out boneDictionary, out bones);

            float maxSide = Math.Max(Math.Max(sceneMax.Z - sceneMin.Z, sceneMax.Y - sceneMin.Y), sceneMax.X - sceneMin.X);

            // maximum pixel size of lod 0 in any dimension

            // lod 0 cell size is 1
            int paddedTopLodCellSize = topLodCellSize;
            topLodCellSize--;


            // multiply by this value to convert scene units to pixels
            float sceneToPixels = lod0pixels / maxSide; // step for lod 2

            // multiply by this value to convert pixels to scene units
            float pixelsToScene = 1.0f / sceneToPixels;

            // exact number of pixels 

            int sx = (int)Math.Ceiling((sceneMax.X - sceneMin.X) * sceneToPixels);
            int sy = (int)Math.Ceiling((sceneMax.Y - sceneMin.Y) * sceneToPixels);
            int sz = (int)Math.Ceiling((sceneMax.Z - sceneMin.Z) * sceneToPixels);

            int padx = sx % topLodCellSize != 0 ? topLodCellSize - sx % topLodCellSize : topLodCellSize;
            int pady = sy % topLodCellSize != 0 ? topLodCellSize - sy % topLodCellSize : topLodCellSize;
            int padz = sz % topLodCellSize != 0 ? topLodCellSize - sz % topLodCellSize : topLodCellSize;

            sx += padx + 1;
            sy += pady + 1;
            sz += padz + 1;

            // we need to increase the size by one since each cell takes (topLodCellSize + 1) pixels and last one will be left without it

            Vector padding = new Vector(padx , pady, padz) * pixelsToScene * 0.5f;

            Vector lowerBound = sceneMin - padding;
            Vector upperBound = sceneMax + padding;

            Console.WriteLine("[{0}] File preprocessed. X: {1}, Y: {2}, Z: {3}, maximum distance: {4}", sw.Elapsed, sx, sy, sz, maxSide);

            int maxcount = sz * sy * sx;
                
            float[] distanceData = new float[maxcount * 4];

            // Use TriangleMap to generate raw SDF grid
            TriangleGrid triangleMap = new TriangleGrid(sceneMin, sceneMax,
                sx / topLodCellSize + ((sx % topLodCellSize != 0) ? 1 : 0),
                sy / topLodCellSize + ((sy % topLodCellSize != 0) ? 1 : 0),
                sz / topLodCellSize + ((sz % topLodCellSize != 0) ? 1 : 0),
                triangleList);

            triangleMap.Dispatch(distanceData, lowerBound, pixelsToScene, sceneToPixels / paddedTopLodCellSize, sx, sy, sz, (progress) => Console.WriteLine("[{0}] Processing {1:P2}", sw.Elapsed, progress));

            Console.WriteLine("Bounding box: {0} - {1}, step {2}, triangles {3}, cells {4}, instances {5}", sceneMin, sceneMax, sceneToPixels, triangleMap.TriangleCount, triangleMap.CellsUsed, triangleMap.TriangleInstances);

            // temporary! Remove it

            string file = (Path.GetFileNameWithoutExtension("out.sdf"));

            using (Stream stream = File.Open(file, FileMode.Create, FileAccess.Write, FileShare.None))
            using (BinaryWriter writer = new BinaryWriter(stream))
            {
                foreach (float fdata in distanceData)
                    writer.Write(fdata);
            }

            PixelData[] data = GetPixelData(distanceData, boneDictionary, triangleList);

            Console.WriteLine("[{0}] SDF data ready", sw.Elapsed);

            obones = new[] { bones };

            return new DistanceData(topLodCellSize, data, new Vector3i(sx, sy, sz), lowerBound, upperBound);
        }

    }
}

