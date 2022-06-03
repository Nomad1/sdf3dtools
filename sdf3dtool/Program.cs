using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using Assimp;
using Assimp.Configs;
using RunMobile.Utility;

using Vector = System.Numerics.Vector3;

namespace SDFTool
{
    static class Program
    {
        private static readonly string s_syntax = "Syntax: {0} input.dae output.png\n" +
        "\n";

        private static void ProcessCollada(string fileName, string outFile)
        {
            Stopwatch sw = new Stopwatch();
            sw.Start();

            Console.WriteLine("[{0}] Processing file {1}", sw.Elapsed, fileName);

            AssimpContext importer = new AssimpContext();
            importer.SetConfig(new NormalSmoothingAngleConfig(66.0f));
            Scene scene = importer.ImportFile(fileName, PostProcessPreset.TargetRealTimeMaximumQuality);

            Console.WriteLine("[{0}] File loaded", sw.Elapsed);

            Vector sceneMin = new Vector(float.MaxValue, float.MaxValue, float.MaxValue);
            Vector sceneMax = new Vector(float.MinValue, float.MinValue, float.MinValue);
            Matrix4x4 matrix = Matrix4x4.Identity;
            List<PreparedTriangle> triangleList = new List<PreparedTriangle>();

            Preprocess(scene, scene.RootNode, ref sceneMin, ref sceneMax, ref matrix, triangleList);

            TriangleMap triangleMap = new TriangleMap(sceneMin, sceneMax, 32, triangleList);

            List<float> xvalues = new List<float>();
            List<float> yvalues = new List<float>();
            List<float> zvalues = new List<float>();

            foreach (PreparedTriangle triangle in triangleList)
            {
                xvalues.Add(triangle.UpperBound.X - triangle.LowerBound.X);
                yvalues.Add(triangle.UpperBound.Y - triangle.LowerBound.Y);
                zvalues.Add(triangle.UpperBound.Z - triangle.LowerBound.Z);
            }

            xvalues.Sort();
            yvalues.Sort();
            zvalues.Sort();

            // use median triangle size as a step
            float step = Math.Min(Math.Min(xvalues[xvalues.Count / 2], yvalues[yvalues.Count / 2]), zvalues[zvalues.Count / 2]);
            //Vector step = new Vector(xvalues[xvalues.Count / 2], yvalues[yvalues.Count / 2], zvalues[zvalues.Count / 2]) * 0.5f;

            Console.WriteLine("Bounding box: {0} - {1}, step {2}, triangles {3}, cells {4}, instances {5}", sceneMin, sceneMax, step, triangleMap.TriangleCount, triangleMap.CellsUsed, triangleMap.TriangleInstances);

            int sx = (int)Math.Ceiling((sceneMax.X - sceneMin.X) / step) + 2;
            int sy = (int)Math.Ceiling((sceneMax.Y - sceneMin.Y) / step) + 2;
            int sz = (int)Math.Ceiling((sceneMax.Z - sceneMin.Z) / step) + 2;

            float maximumDistance = Vector.Distance(sceneMin, sceneMax);

            Console.WriteLine("[{0}] File preprocessed. X: {1}, Y: {2}, Z: {3}", sw.Elapsed, sx, sy, sz);

            ushort[] data = new ushort[sx * sy * sz * 4];

            for (int iz = 0; iz < sz; iz++)
            {
                Console.WriteLine("[{0}] Processing depth {1}", sw.Elapsed, iz);
                byte[] testData = new byte[sx * sy];

                for (int iy = 0; iy < sy; iy++)
                {
                    for (int ix = 0; ix < sx; ix++)
                    {
                        Vector point = new Vector(ix * step + sceneMin.X, iy * step + sceneMin.Y, iz * step + sceneMin.Z);

                        float distance;
                        Vector triangleWeights;
                        object triangleData;

                        if (triangleMap.FindTriangles(point, out distance, out triangleWeights, out triangleData))
                        {
                            int index = (ix + iy * sx + iz * sx * sy) * 4;

                            data[index + 0] = (new HalfFloat(distance)).Data;


                            testData[ix + iy * sx] = (byte)(128 + 4.0f * 255.0f * distance / maximumDistance);

                            Tuple<Mesh, Face> tuple = (Tuple<Mesh, Face>)triangleData;
                            Mesh mesh = tuple.Item1;
                            Face face = tuple.Item2;

                            Vector3D tca = mesh.TextureCoordinateChannels[0][face.Indices[0]];
                            Vector3D tcb = mesh.TextureCoordinateChannels[0][face.Indices[1]];
                            Vector3D tcc = mesh.TextureCoordinateChannels[0][face.Indices[2]];

                            Vector tc = new Vector(
                                tca.X * triangleWeights.X + tcb.X * triangleWeights.Y + tcc.X * triangleWeights.Z,
                                tca.Y * triangleWeights.Y + tcb.Y * triangleWeights.Y + tcc.Y * triangleWeights.Z,
                                tca.Z * triangleWeights.Z + tcb.Z * triangleWeights.Y + tcc.Z * triangleWeights.Z);

                            data[index + 1] = (new HalfFloat(tc.X)).Data;
                            data[index + 2] = (new HalfFloat(tc.Y)).Data;
                        }

                        /*
                        float distance = triangles[0].DistanceSqrd(point);

                        for (int i = 1; i < triangles.Length; i++)
                        {
                            float distanceToSphere = (point - triangles[i].Center).LengthSquared() - triangles[i].RadiusSqrd;
                            if (distanceToSphere > distance)
                                continue;

                            float dist = triangles[i].DistanceSqrd(point);
                            if (dist < distance)
                                distance = dist; // TODO: save triangle index to calculate color/texcoords if it wins
                        }
                        */


                        // TODO: index + 1 and index + 2 should be texture coordinates. index + 3 is empty
                    }
                }

                SaveGrayscaleBitmap(testData, sx, sy, Path.GetFileNameWithoutExtension(outFile) + "_" + iz);
            }

            Console.WriteLine("[{0}] SDF finished, saving KTX", sw.Elapsed);

            SaveKTX(0x881A, sx, sy, sz, data, outFile);

            Console.WriteLine("[{0}] KTX saved", sw.Elapsed);

            sw.Stop();
        }

        public static void SaveGrayscaleBitmap(byte[] data, int imageWidth, int imageHeight, string outFile)
        {
            string file = Path.GetFileNameWithoutExtension(outFile) + ".ppm";
            Console.WriteLine("Converting data to PPM bitmap {0}", file);
            using (FileStream stream = File.Open(file, FileMode.Create, FileAccess.Write, FileShare.None))
            using (StreamWriter writer = new StreamWriter(stream))
            {
                writer.WriteLine("P3");
                writer.WriteLine("{0} {1}", imageWidth, imageHeight);
                writer.WriteLine("255");

                for (int y = 0; y < imageHeight; y++)
                {
                    for (int x = 0; x < imageWidth; x++)
                    {
                        byte bvalue = data[x + y * imageWidth];

                        writer.Write(bvalue);
                        writer.Write(' ');
                        writer.Write(bvalue);
                        writer.Write(' ');
                        writer.Write(bvalue);
                        writer.Write(' ');
                    }
                    writer.WriteLine();
                }
            }

        }


        private static void Preprocess(Scene scene, Node node, ref Vector min, ref Vector max, ref Matrix4x4 matrix, IList<PreparedTriangle> triangles)
        {
            Matrix4x4 prev = matrix;
            matrix = prev * node.Transform;

            if (node.HasMeshes)
            {
                foreach (int index in node.MeshIndices)
                {
                    Mesh mesh = scene.Meshes[index];
                    for (int i = 0; i < mesh.VertexCount; i++)
                    {
                        Vector3D tmp = matrix * mesh.Vertices[i];

                        min.X = Math.Min(min.X, tmp.X);
                        min.Y = Math.Min(min.Y, tmp.Y);
                        min.Z = Math.Min(min.Z, tmp.Z);

                        max.X = Math.Max(max.X, tmp.X);
                        max.Y = Math.Max(max.Y, tmp.Y);
                        max.Z = Math.Max(max.Z, tmp.Z);
                    }

                    for (int i = 0; i < mesh.FaceCount; i++)
                    {
                        Face face = mesh.Faces[i];

                        if (face.HasIndices && face.IndexCount == 3) // only process triangles. Don't have a clue what to do with other primitives
                        {
                            Vector3D a = matrix * mesh.Vertices[face.Indices[0]];
                            Vector3D b = matrix * mesh.Vertices[face.Indices[1]];
                            Vector3D c = matrix * mesh.Vertices[face.Indices[2]];

                            Tuple<Mesh, Face> data = new Tuple<Mesh, Face>(mesh, face);

                            triangles.Add(new PreparedTriangle(new Vector(a.X, a.Y, a.Z), new Vector(b.X, b.Y, b.Z), new Vector(c.X, c.Y, c.Z), data));
                        }
                    }
                }
            }

            for (int i = 0; i < node.ChildCount; i++)
            {
                Preprocess(scene, node.Children[i], ref min, ref max, ref matrix, triangles);
            }
            matrix = prev;
        }


        private static void SaveKTX(int format, int width, int height, int depth, ushort[] data, string outfile)
        {
            using (Stream stream = File.Open(outfile, FileMode.Create, FileAccess.Write, FileShare.None))
            using (BinaryWriter writer = new BinaryWriter(stream))
            {
                writer.Write(new byte[] { 0xAB, 0x4B, 0x54, 0x58, 0x20, 0x31, 0x31, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A });
                writer.Write(0x04030201);
                writer.Write(0x140B); // raw type
                writer.Write(2); // raw size
                writer.Write(format); // raw format
                writer.Write(format); // format
                writer.Write(0x1908); // rgba?
                writer.Write(width);
                writer.Write(height);
                writer.Write(depth);
                writer.Write(0); // elements
                writer.Write(1); // faces
                writer.Write(1); // mipmaps
                writer.Write(0); // metadata

                while (writer.BaseStream.Length < 64 + 0) // header + metadata size
                    writer.Write(0);

                writer.Write(data.Length); // current mipmap size

                for (int i = 0; i < data.Length; i++)
                    writer.Write(data[i]);
            }
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

            ProcessCollada(fileName, outFileName);
        }
    }
}

