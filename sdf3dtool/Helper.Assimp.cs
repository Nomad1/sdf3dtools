using System;
using SDFTool;
using System.IO;
using Assimp;
using System.Collections.Generic;

namespace SDFTool
{
    public partial class Helper
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
    }
}

