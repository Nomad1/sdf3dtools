using System;
using SDFTool;
using System.IO;
using System.Collections.Generic;
using Assimp;
using System.Numerics;
using System.Text;

namespace SDFTool
{
    public static class BinaryWriterExt
    {
        public static void WriteStringExt(this BinaryWriter writer, string str)
        {
            if (string.IsNullOrEmpty(str))
                str = "(empty)";

            byte[] bytes = Encoding.Default.GetBytes(str);

            int length = Math.Min(bytes.Length, 64);

            for (int i = 0; i < bytes.Length; i++)
                if (bytes[i] == 0)
                {
                    length = i;
                    break;
                }

            writer.Write((ushort)length);
            for (int i = 0; i < length; i++)
                if (bytes[i] != 0)
                    writer.Write(bytes[i]);
        }

        public static string ReadStringExt(this BinaryReader reader)
        {
            int length = reader.ReadUInt16();

            byte[] bytes = reader.ReadBytes(length);

            return Encoding.Default.GetString(bytes);
        }
    }

    public partial class Helper
    {
        public static int Signature = (int)(('u' | ('m' << 8) | ('0' << 16) | ('3' << 24)));
      
        /// <summary>
        /// Saves a mesh to custom .umesh format
        /// </summary>
        /// <param name="outFile"></param>
        public static void SaveUMesh(MeshGenerator.Surface[] surfaces, string outFile, ValueTuple<string, Bone>[][] bones = null, IList<Animation> animations = null, Scene oldScene = null, Assimp.Matrix4x4 matrix = default)
        {
            string file = Path.GetFileNameWithoutExtension(outFile) + ".umesh";

            using (Stream stream = File.Open(file, FileMode.Create, FileAccess.Write, FileShare.None))
            using (BinaryWriter writer = new BinaryWriter(stream))
            {
                writer.Write(Signature);

                writer.Write(surfaces.Length); // surfaces

                for (int s = 0; s < surfaces.Length; s++)
                {
                    MeshGenerator.Surface surface = surfaces[s];

                    writer.WriteStringExt(surface.Name);

                    {
                        Vector3D scaling;
                        Assimp.Quaternion rotation;
                        Vector3D translation;
                        matrix.Decompose(out scaling, out rotation, out translation);

                        writer.Write(translation.X);
                        writer.Write(translation.Y);
                        writer.Write(translation.Z);

                        writer.Write(rotation.X);
                        writer.Write(rotation.Y);
                        writer.Write(rotation.Z);
                        writer.Write(rotation.W);

                        writer.Write(scaling.X);
                        writer.Write(scaling.Y);
                        writer.Write(scaling.Z);
                    }

                    writer.Write(surface.Vertices.Count); // vertices
                    writer.Write(surface.TCLength); // number of bytes in TC channels

                    bool useNormals = surface.Normals != null && surface.Normals.Count == surface.Vertices.Count;
                    bool useWeights = surface.VertexWeights != null && surface.VertexWeights.Count == surface.Vertices.Count;

                    int flags = 0;
                    if (useNormals)
                        flags |= 1;
                    if (useWeights)
                        flags |= 2;
                    writer.Write(flags); // TODO: enum

                    for (int i = 0; i < surface.Vertices.Count; i++)
                    {
                        Vector3D v = new Vector3D(surface.Vertices[i].X, surface.Vertices[i].Y, surface.Vertices[i].Z);
                        writer.Write(v.X);
                        writer.Write(v.Y);
                        writer.Write(v.Z);

                        if (useNormals)
                        {
                            Vector3D n = new Vector3D(surface.Normals[i].X, surface.Normals[i].Y, surface.Normals[i].Z);
                            writer.Write(n.X);
                            writer.Write(n.Y);
                            writer.Write(n.Z);
                        }
                       
                        for (int j = 0; j < surface.TCLength; j++)
                            writer.Write(j < surface.TexCoords[i].Length ? surface.TexCoords[i][j] : 0.0f);

                        if (useWeights)
                        {
                            if (surface.VertexWeights[i] != null)
                            {
                                List<ValueTuple<byte, float>> nbones = new List<(byte, float)>(surface.VertexWeights[i].Length);
                                foreach (var pair in surface.VertexWeights[i])
                                    if (pair.Item1 > 0 && pair.Item2 > 0)
                                        nbones.Add(new ValueTuple<byte, float>((byte)(pair.Item1 - 1), pair.Item2));

                                writer.Write(nbones.Count);

                                foreach (var pair in nbones)
                                {
                                    writer.Write(pair.Item1); // bone index as byte
                                    writer.Write(pair.Item2);
                                }
                            } else
                                writer.Write(0);
                        }
                    }

                    writer.Write(surface.Faces.Count); // triangles

                    if (surface.Vertices.Count < 65535) // 16 bit indices
                    {
                        for (int i = 0; i < surface.Faces.Count; i++)
                            for (int j = 0; j < surface.Faces[i].Length; j++)
                                writer.Write((ushort)surface.Faces[i][j]);
                    }
                    else
                    {
                        for (int i = 0; i < surface.Faces.Count; i++)
                            for (int j = 0; j < surface.Faces[i].Length; j++)
                                writer.Write(surface.Faces[i][j]);
                    }

                    Assimp.Matrix4x4 rootTransform = Assimp.Matrix4x4.Identity;

                    if (bones != null)
                    {
                        Dictionary<string, int> bonesDict = new Dictionary<string, int>();

                        for (int i = 0; i < bones[s].Length; i++)
                            bonesDict[bones[s][i].Item2.Name] = i;

                        if (animations != null)
                        {
                            foreach (Animation animation in animations)
                                foreach (NodeAnimationChannel channel in animation.NodeAnimationChannels)
                                    if (!bonesDict.ContainsKey(channel.NodeName))
                                        bonesDict.Add(channel.NodeName, bonesDict.Count);

                            //Dictionary<string, int> newBones = new Dictionary<string, int>();

                            //foreach (string bname in bonesDict.Keys)
                            //{
                            //    Node boneNode = oldScene.RootNode.FindNode(bname);
                            //    while (boneNode != null)
                            //    {
                            //        if (!bonesDict.ContainsKey(boneNode.Name) && !newBones.ContainsKey(boneNode.Name))
                            //            newBones.Add(boneNode.Name, bonesDict.Count + newBones.Count);

                            //        boneNode = boneNode.Parent;
                            //    }
                            //}

                            //foreach (var pair in newBones)
                            //    bonesDict.Add(pair.Key, pair.Value);
                        }

                        writer.Write(bonesDict.Count);

                        foreach(var pair in bonesDict)
                        {
                            int index = pair.Value;
                            string bname = pair.Key;

                            writer.Write(index); // bone index
                            writer.WriteStringExt(bname);

                            Node boneNode = oldScene.RootNode.FindNode(bname);

                            if (boneNode == null)
                            {
                                Console.Error.WriteLine("Invalid bone {0}", bname);
                            }

                            bool parentFound = false;
                            if (boneNode != null && boneNode.Parent != null)
                            {
                                int parent;
                                if (bonesDict.TryGetValue(boneNode.Parent.Name, out parent))
                                {
                                    writer.Write(parent);
                                    parentFound = true;
                                }
                            }

                            Assimp.Matrix4x4 transform = boneNode == null ? Assimp.Matrix4x4.Identity : boneNode.Transform;

                            if (!parentFound)
                            {
                                writer.Write(-1);

                                /*Node node = boneNode.Parent;

                                rootTransform = boneNode.Transform;

                                while (node != null)
                                {
                                    rootTransform = rootTransform * node.Transform;
                                    node = node.Parent;
                                }

                                //transform = rootTransform * transform;
                                */
                            }

                            Vector3D scaling;
                            Assimp.Quaternion rotation;
                            Vector3D translation;

                            //Assimp.Matrix4x4 transform = //bname == oldScene.RootNode.Name ? matrix :
                            //boneNode.Transform;

                            if (index < bones[s].Length || boneNode == null)
                            {
                                transform = bones[s][index].Item2.OffsetMatrix;
                                transform.Inverse();
                            }

                            transform.Decompose(out scaling, out rotation, out translation);
                            //
                            //bones[s][i].Item2.OffsetMatrix.Decompose(out scaling, out rotation, out translation);

                            writer.Write(translation.X);
                            writer.Write(translation.Y);
                            writer.Write(translation.Z);

                            writer.Write(rotation.X);
                            writer.Write(rotation.Y);
                            writer.Write(rotation.Z);
                            writer.Write(rotation.W);

                            writer.Write(scaling.X);
                            writer.Write(scaling.Y);
                            writer.Write(scaling.Z);
                            /*
                            writer.Write(bones[s][i].Item2.OffsetMatrix.A1);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.A2);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.A3);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.A4);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.B1);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.B2);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.B3);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.B4);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.C1);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.C2);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.C3);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.C4);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.D1);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.D2);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.D3);
                            writer.Write(bones[s][i].Item2.OffsetMatrix.D4);*/
                        }

                        if (animations != null)
                        {
                            writer.Write(animations.Count);

                            foreach (Animation animation in animations)
                            {
                                writer.WriteStringExt(animation.Name);
                                writer.Write((double)(animation.DurationInTicks / animation.TicksPerSecond));

                                int acount = 0;
                                foreach (NodeAnimationChannel channel in animation.NodeAnimationChannels)
                                {
                                    int bone;
                                    if (!bonesDict.TryGetValue(channel.NodeName, out bone))
                                    {
                                        Console.Error.WriteLine("Bone {0} not found for animation channel", channel.NodeName);
                                        continue;
                                    }
                                    acount++;
                                }
                                writer.Write(acount);

                                foreach (NodeAnimationChannel channel in animation.NodeAnimationChannels)
                                {
                                    int bone;
                                    if (!bonesDict.TryGetValue(channel.NodeName, out bone))
                                    {
                                        Console.Error.WriteLine("Bone {0} not found for animation channel", channel.NodeName);
                                        continue;
                                    }

                                    writer.Write(bone);

                                    // only for mode when keys are distributed equally


                                    int keys = channel.PositionKeys.Count;
                                    ValueTuple<Vector3D, Assimp.Quaternion, Vector3D>[] newKeys = new ValueTuple<Vector3D, Assimp.Quaternion, Vector3D>[keys];

                                    for (int i = 0; i < keys; i++)
                                    {
                                        Assimp.Matrix4x4 transform =
                                            //rootTransform *
                                            new Assimp.Matrix4x4(channel.RotationKeys[i].Value.GetMatrix()) *
                                            Assimp.Matrix4x4.FromTranslation(channel.PositionKeys[i].Value) *
                                            Assimp.Matrix4x4.FromScaling(channel.ScalingKeys[i].Value);
                                        //    ;
                                        Vector3D scaling;
                                        Assimp.Quaternion rotation;
                                        Vector3D translation;

                                        transform.Decompose(out scaling, out rotation, out translation);

                                        newKeys[i] = new ValueTuple<Vector3D, Assimp.Quaternion, Vector3D>(translation, rotation, scaling);
                                        //newKeys[i] = new ValueTuple<Vector3D, Assimp.Quaternion, Vector3D>(channel.PositionKeys[i].Value, channel.RotationKeys[i].Value, channel.ScalingKeys[i].Value);
                                    }

                                    writer.Write(keys);
                                    for (int i = 0; i < keys; i++)
                                    {
                                        writer.Write(channel.PositionKeys[i].Time);
                                        writer.Write(newKeys[i].Item1.X);
                                        writer.Write(newKeys[i].Item1.Y);
                                        writer.Write(newKeys[i].Item1.Z);
                                    }
                                    writer.Write(keys);
                                    for (int i = 0; i < keys; i++)
                                    {
                                        writer.Write(channel.RotationKeys[i].Time);
                                        writer.Write(newKeys[i].Item2.X);
                                        writer.Write(newKeys[i].Item2.Y);
                                        writer.Write(newKeys[i].Item2.Z);
                                        writer.Write(newKeys[i].Item2.W);
                                    }
                                    writer.Write(keys);
                                    for (int i = 0; i < keys; i++)
                                    {
                                        writer.Write(channel.ScalingKeys[i].Time);
                                        writer.Write(newKeys[i].Item3.X);
                                        writer.Write(newKeys[i].Item3.Y);
                                        writer.Write(newKeys[i].Item3.Z);
                                    }
                                    /*
                                writer.Write(channel.PositionKeys.Count);
                                foreach (VectorKey key in channel.PositionKeys)
                                {
                                    writer.Write(key.Time);
                                    writer.Write(key.Value.X);
                                    writer.Write(key.Value.Y);
                                    writer.Write(key.Value.Z);
                                }
                                writer.Write(channel.RotationKeys.Count);
                                foreach (QuaternionKey key in channel.RotationKeys)
                                {
                                    writer.Write(key.Time);
                                    writer.Write(key.Value.X);
                                    writer.Write(key.Value.Y);
                                    writer.Write(key.Value.Z);
                                    writer.Write(key.Value.W);
                                }

                                writer.Write(channel.ScalingKeys.Count);
                                foreach (VectorKey key in channel.ScalingKeys)
                                {
                                    writer.Write(key.Time);
                                    writer.Write(key.Value.X);
                                    writer.Write(key.Value.Y);
                                    writer.Write(key.Value.Z);
                                }*/
                                }
                            }
                        }
                        else
                        {
                            writer.Write(0);
                            writer.Write(0);
                        }
                    }
                }
            }
        }
    }
}

