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

            float scale = 1.0f; // 10000

            Vector sceneMin = new Vector(float.MaxValue, float.MaxValue, float.MaxValue);
            Vector sceneMax = new Vector(float.MinValue, float.MinValue, float.MinValue);
            Matrix4x4 matrix = Matrix4x4.FromScaling(new Vector3D(scale));
            List<PreparedTriangle> triangleList = new List<PreparedTriangle>();

            Preprocess(scene, scene.RootNode, ref matrix, triangleList);

            List<float> xvalues = new List<float>();
            List<float> yvalues = new List<float>();
            List<float> zvalues = new List<float>();

            foreach (PreparedTriangle triangle in triangleList)
            {
                xvalues.Add(triangle.UpperBound.X - triangle.LowerBound.X);
                yvalues.Add(triangle.UpperBound.Y - triangle.LowerBound.Y);
                zvalues.Add(triangle.UpperBound.Z - triangle.LowerBound.Z);

                sceneMax.X = Math.Max(sceneMax.X, triangle.UpperBound.X);
                sceneMax.Y = Math.Max(sceneMax.Y, triangle.UpperBound.Y);
                sceneMax.Z = Math.Max(sceneMax.Z, triangle.UpperBound.Z);
                sceneMin.X = Math.Min(sceneMin.X, triangle.LowerBound.X);
                sceneMin.Y = Math.Min(sceneMin.Y, triangle.LowerBound.Y);
                sceneMin.Z = Math.Min(sceneMin.Z, triangle.LowerBound.Z);
            }

            xvalues.Sort();
            yvalues.Sort();
            zvalues.Sort();

            // use median triangle size as a step
            float step = Math.Min(Math.Min(xvalues[xvalues.Count / 2], yvalues[yvalues.Count / 2]), zvalues[zvalues.Count / 2]);
            //step *= 0.5f;

            //creating tiled structure for faster triangle search
            TriangleMap triangleMap = new TriangleMap(sceneMin, sceneMax, 32, triangleList);

            Console.WriteLine("Bounding box: {0} - {1}, step {2}, triangles {3}, cells {4}, instances {5}", sceneMin, sceneMax, step, triangleMap.TriangleCount, triangleMap.CellsUsed, triangleMap.TriangleInstances);

            // number of extra pixels on the each size
            int padding = 2;

            int sx = (int)Math.Ceiling((sceneMax.X - sceneMin.X) / step) + padding * 2;
            int sy = (int)Math.Ceiling((sceneMax.Y - sceneMin.Y) / step) + padding * 2;
            int sz = (int)Math.Ceiling((sceneMax.Z - sceneMin.Z) / step) + padding * 2;

            Vector lowerBound = new Vector(-step * padding) + sceneMin;
            Vector upperBound = new Vector(sx, sy, sz) * step + sceneMin;

            float maximumDistance =
                //Math.Max(Math.Max(sceneMax.X - sceneMin.X, sceneMax.Y - sceneMin.Y), sceneMax.Z - sceneMin.Z);
                Vector.Distance(sceneMax, sceneMin);

            ushort[] data = new ushort[sx * sy * sz * 4];

            Console.WriteLine("[{0}] File preprocessed. X: {1}, Y: {2}, Z: {3}", sw.Elapsed, sx, sy, sz);

            Parallel.For(0, sz, (iz) =>
            //for (int iz = 0; iz < sz; iz++)
            {
                Console.WriteLine("[{0}] Processing depth {1}", sw.Elapsed, iz);
#if DEBUG
                byte[] testData = new byte[sx * sy * 4];
#endif
                for (int iy = 0; iy < sy; iy++)
                {
                    for (int ix = 0; ix < sx; ix++)
                    {
                        Vector point = lowerBound + new Vector(ix, iy, iz) * step;

                        float distance;
                        Vector triangleWeights;
                        object triangleData;

                        if (triangleMap.FindTriangles(point, out distance, out triangleWeights, out triangleData))
                        {
                            int index = (ix + iy * sx + iz * sx * sy) * 4;

                            float distancePercentage = distance / maximumDistance;

                            // the distance
                            data[index + 0] = (new HalfFloat(distancePercentage)).Data;


#if DEBUG
                            // temporary test images. I had to scale the distance 4x to make them visible. Not sure it would work
                            // for all the meshes
                            testData[(ix + iy * sx) * 4] = distancePercentage > 0 ? (byte)(1024.0f * distancePercentage) : (byte)0;
                            testData[(ix + iy * sx) * 4 + 1] = distancePercentage < 0 ? (byte)(-1024.0f * distancePercentage) : (byte)0;
#endif
                            // don't store tex coords for empty space. Note that right now "empty" means N% from the surface
                            // it could be altered for different situations
                            //if (distancePercentage < 0.1f)
                            {
                                // Saved triangle data
                                Tuple<Mesh, Face> tuple = (Tuple<Mesh, Face>)triangleData;
                                Mesh mesh = tuple.Item1;
                                Face face = tuple.Item2;

                                Vector3D tca = mesh.TextureCoordinateChannels[0][face.Indices[0]];
                                Vector3D tcb = mesh.TextureCoordinateChannels[0][face.Indices[1]];
                                Vector3D tcc = mesh.TextureCoordinateChannels[0][face.Indices[2]];

                                Vector3D tc =
                                //new Vector3D(ix / (float)sx, iy / (float)sz, 0);
                                tca* triangleWeights.X + tcb * triangleWeights.Y + tcc * triangleWeights.Z;

                                // texture coords
                                data[index + 1] = (new HalfFloat(tc.X)).Data;
                                data[index + 2] = (new HalfFloat(tc.Y)).Data;
                            }
                        }

                        // TODO: index + 1 and index + 2 should be texture coordinates. index + 3 is empty
                    }
                }

#if DEBUG
                SaveBitmap(testData, sx, sy, Path.GetFileNameWithoutExtension(outFile) + "_" + iz);
#endif
            }
            );

            Console.WriteLine("[{0}] SDF finished, saving KTX", sw.Elapsed);

            SaveKTX(0x881A, sx, sy, sz, data, outFile);

            Console.WriteLine("[{0}] KTX saved, saving boundary mesh", sw.Elapsed);

            SaveBoxMesh(lowerBound, upperBound, outFile);

            Console.WriteLine("[{0}] All done", sw.Elapsed);

            sw.Stop();
        }

        private static void Preprocess(Scene scene, Node node, ref Matrix4x4 matrix, IList<PreparedTriangle> allTriangles)
        {
            Matrix4x4 prev = matrix;
            matrix = prev * node.Transform;

            if (node.HasMeshes)
            {
                foreach (int index in node.MeshIndices)
                {
                    Mesh mesh = scene.Meshes[index];

                    List<PreparedTriangle> triangles = new List<PreparedTriangle>(mesh.FaceCount);

                    Dictionary<VectorPair, Vector> edgeNormals = new Dictionary<VectorPair, Vector>();

                    Dictionary<Vector3i, Vector> vertexNormals = new Dictionary<Vector3i, Vector>();

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

                            int multiplier = 10000;
                            Vector3i ia = new Vector3i((int)(mesh.Vertices[face.Indices[0]].X * multiplier), (int)(mesh.Vertices[face.Indices[0]].Y * multiplier), (int)(mesh.Vertices[face.Indices[0]].Z * multiplier));
                            Vector3i ib = new Vector3i((int)(mesh.Vertices[face.Indices[1]].X * multiplier), (int)(mesh.Vertices[face.Indices[1]].Y * multiplier), (int)(mesh.Vertices[face.Indices[1]].Z * multiplier));
                            Vector3i ic = new Vector3i((int)(mesh.Vertices[face.Indices[2]].X * multiplier), (int)(mesh.Vertices[face.Indices[2]].Y * multiplier), (int)(mesh.Vertices[face.Indices[2]].Z * multiplier));
                            VectorPair edgeab = new VectorPair(ia, ib);
                            VectorPair edgebc = new VectorPair(ib, ic);
                            VectorPair edgeca = new VectorPair(ic, ia);


                            // This tuple will be used to get back the triangle and vertices when needed
                            Tuple<Mesh, Face> data = new Tuple<Mesh, Face>(mesh, face);

                            PreparedTriangle triangle = new PreparedTriangle(a, b, c, data, new Vector3i[] { ia, ib, ic }, new VectorPair[] { edgeab, edgebc, edgeca });
                            triangles.Add(triangle);
                            allTriangles.Add(triangle);

                            if (!vertexNormals.ContainsKey(ia))
                                vertexNormals.Add(ia, triangle.PseudoNormals[4]);
                            else
                                vertexNormals[ia] += triangle.PseudoNormals[4];

                            if (!vertexNormals.ContainsKey(ib))
                                vertexNormals.Add(ib, triangle.PseudoNormals[5]);
                            else
                                vertexNormals[ib] += triangle.PseudoNormals[5];

                            if (!vertexNormals.ContainsKey(ic))
                                vertexNormals.Add(ic, triangle.PseudoNormals[6]);
                            else
                                vertexNormals[ic] += triangle.PseudoNormals[6];

                            if (!edgeNormals.ContainsKey(edgeab))
                                edgeNormals.Add(edgeab, triangle.PseudoNormals[0]);
                            else
                                edgeNormals[edgeab] += triangle.PseudoNormals[0];

                            if (!edgeNormals.ContainsKey(edgebc))
                                edgeNormals.Add(edgebc, triangle.PseudoNormals[1]);
                            else
                                edgeNormals[edgebc] += triangle.PseudoNormals[1];

                            if (!edgeNormals.ContainsKey(edgeca))
                                edgeNormals.Add(edgeca, triangle.PseudoNormals[2]);
                            else
                                edgeNormals[edgeca] += triangle.PseudoNormals[2];
                        }
                    }

                    
                    foreach (PreparedTriangle triangle in triangles)
                    {
                        triangle.UpdateNeighbors(vertexNormals, edgeNormals);
                    }

                }
            }

            for (int i = 0; i < node.ChildCount; i++)
                Preprocess(scene, node.Children[i], ref matrix, allTriangles);

            matrix = prev;
        }

        /// <summary>
        /// Saves a 3d texture to KTX format with each pixel being a set of 4 ushort values
        /// </summary>
        /// <param name="format"></param>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <param name="depth"></param>
        /// <param name="data"></param>
        /// <param name="outfile"></param>
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

                writer.Write(data.Length * 2); // current mipmap size

                for (int i = 0; i < data.Length; i++)
                    writer.Write(data[i]);
            }
        }

        /// <summary>
        /// Saves a bitmap to 3 channel PPM format with all the channels being a grayscale
        /// </summary>
        /// <param name="data"></param>
        /// <param name="imageWidth"></param>
        /// <param name="imageHeight"></param>
        /// <param name="outFile"></param>
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

        public static void SaveBitmap(byte[] data, int imageWidth, int imageHeight, string outFile)
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
                        byte rvalue = data[(x + y * imageWidth) * 4 + 0];
                        byte gvalue = data[(x + y * imageWidth) * 4 + 1];
                        byte bvalue = data[(x + y * imageWidth) * 4 + 2];

                        writer.Write(rvalue);
                        writer.Write(' ');
                        writer.Write(gvalue);
                        writer.Write(' ');
                        writer.Write(bvalue);
                        writer.Write(' ');
                    }
                    writer.WriteLine();
                }
            }
        }

        private struct Triangle
        {
            public readonly ValueTuple<int,int,int>[] Points;
            public readonly int[] Vertices;

            public Triangle(int av, int atc, int an, int bv, int btc, int bn, int cb, int ctc, int cn)
            {
                Vertices = new int[3];
                Points = new[] {
                    new ValueTuple<int, int, int>(av - 1, atc - 1, an - 1),
                    new ValueTuple<int, int, int>(bv - 1, btc - 1, bn - 1),
                    new ValueTuple<int, int, int>(cb - 1, ctc - 1, cn - 1)
                };
            }
        }

        /// <summary>
        /// Saves a box to Unigine .mesh format with lower bound having 0,0 tex coord and upper having 1,1
        /// </summary>
        /// <param name="lb"></param>
        /// <param name="ub"></param>
        /// <param name="outFile"></param>
        public static void SaveBoxMesh(Vector lb, Vector ub, string outFile)
        {
            string file = Path.GetFileNameWithoutExtension(outFile) + ".mesh";
            Console.WriteLine("Saving bounding box mesh to {0}", file);

            Vector center = (lb + ub) / 2;
            float radius = Vector.Distance(center, lb);

            // 8 vertices
            Vector[] vertices = {
                lb,
                new Vector(ub.X, lb.Y, lb.Z),
                new Vector(lb.X, ub.Y, lb.Z),
                new Vector(ub.X, ub.Y, lb.Z),
                new Vector(lb.X, lb.Y, ub.Z),
                new Vector(ub.X, lb.Y, ub.Z),
                new Vector(lb.X, ub.Y, ub.Z),
                ub
            };

            // 12 normals
            Vector[] normals = {
                new Vector(0.0f, 0.0f, -1.0f),
                new Vector(0.0f, 0.0f, 1.0f),
                new Vector(0.0f, -1.0f, 0.0f),
                new Vector(1.0f, 0.0f, 0.0f),
                new Vector(0.0f, 1.0f, 0.0f),
                new Vector(-1.0f, 0.0f, 0.0f),
            };
            // 12 tex coords
            Vector[] texcoords = {
                new Vector(0.0f, 0.0f, 0.0f),
                new Vector(1.0f, 0.0f, 0.0f),

                new Vector(0.0f, 1.0f, 0.0f),
                new Vector(1.0f, 1.0f, 0.0f),

                new Vector(0.0f, 0.0f, 0.0f),
                new Vector(1.0f, 0.0f, 0.0f),

                new Vector(0.0f, 1.0f, 0.0f),
                new Vector(1.0f, 1.0f, 0.0f),

                new Vector(0.0f, 0.0f, 0.0f),
                new Vector(1.0f, 0.0f, 0.0f),

                new Vector(0.0f, 1.0f, 0.0f),
                new Vector(1.0f, 1.0f, 0.0f)
            };
            // 12 triangles
            Triangle[] faces = {
                new Triangle(1, 10, 1, 3, 12, 1, 4, 11, 1),
                new Triangle(4, 11, 1, 2, 9, 1, 1, 10, 1),

                new Triangle(5, 9, 2, 6, 10, 2, 8, 12, 2),
                new Triangle(8, 12, 2, 7, 11, 2, 5, 9, 2),

                new Triangle(1, 5, 3, 2, 6, 3, 6, 8, 3),
                new Triangle(6, 8, 3, 5, 7, 3, 1, 5, 3),

                new Triangle(2, 1, 4, 4, 2, 4, 8, 4, 4),
                new Triangle(8, 4, 4, 6, 3, 4, 2, 1, 4),

                new Triangle(4, 5, 5, 3, 6, 5, 7, 8, 5),
                new Triangle(7, 8, 5, 8, 7, 5, 4, 5, 5),

                new Triangle(3, 1, 6, 1, 2, 6, 5, 4, 6),
                new Triangle(5, 4, 6, 7, 3, 6, 3, 1, 6)
            };

            // here goes deduplication: every vertex should have unique normal and texcoord

            List<ValueTuple<int, int, int>> vertexPairs = new List<ValueTuple<int, int, int>>();

            for (int i = 0; i < faces.Length; i++)
                for (int j = 0; j < faces[i].Points.Length; j++)
                {
                    int index = vertexPairs.IndexOf(faces[i].Points[j]);
                    if (index == -1)
                    {
                        index = vertexPairs.Count;
                        vertexPairs.Add(faces[i].Points[j]);
                    }
                    faces[i].Vertices[j] = index;
                }

            using (Stream stream = File.Open(file, FileMode.Create, FileAccess.Write, FileShare.None))
            using (BinaryWriter writer = new BinaryWriter(stream))
            {
                writer.Write((int)(('m' | ('i' << 8) | ('0' << 16) | ('8' << 24))));
                writer.Write(lb.X);
                writer.Write(lb.Y);
                writer.Write(lb.Z);
                writer.Write(ub.X);
                writer.Write(ub.Y);
                writer.Write(ub.Z);
                writer.Write(center.X);
                writer.Write(center.Y);
                writer.Write(center.Z);
                writer.Write(radius);

                writer.Write(1); // surfaces

                string surfaceName = "box";

                writer.Write(surfaceName.Length + 1);
                writer.Write(surfaceName.ToCharArray());
                writer.Write((byte)0);
                writer.Write(lb.X);
                writer.Write(lb.Y);
                writer.Write(lb.Z);
                writer.Write(ub.X);
                writer.Write(ub.Y);
                writer.Write(ub.Z);
                writer.Write(center.X);
                writer.Write(center.Y);
                writer.Write(center.Z);
                writer.Write(radius);

                writer.Write(vertexPairs.Count); // vertices

                for (int i = 0; i < vertexPairs.Count; i++)
                {
                    writer.Write(vertices[vertexPairs[i].Item1].X);
                    writer.Write(vertices[vertexPairs[i].Item1].Y);
                    writer.Write(vertices[vertexPairs[i].Item1].Z);
                    writer.Write((ushort)((normals[vertexPairs[i].Item3].X / 2.0f + 0.5f) * 65535.0f));
                    writer.Write((ushort)((normals[vertexPairs[i].Item3].Y / 2.0f + 0.5f) * 65535.0f));
                    writer.Write((ushort)((normals[vertexPairs[i].Item3].Z / 2.0f + 0.5f) * 65535.0f));
                }

                writer.Write(vertexPairs.Count); // texture coordinates

                for (int i = 0; i < vertexPairs.Count; i++)
                {
                    writer.Write(texcoords[vertexPairs[i].Item2].X);
                    writer.Write(texcoords[vertexPairs[i].Item2].Y);
                }

                writer.Write(0); // second texcoords

                writer.Write(faces.Length); // triangles

                for (int i = 0; i < faces.Length; i++)
                {
                    writer.Write((ushort)faces[i].Vertices[0]);
                    writer.Write((ushort)faces[i].Vertices[1]);
                    writer.Write((ushort)faces[i].Vertices[2]);
                }
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

