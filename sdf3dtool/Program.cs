using System;
using System.Collections.Concurrent;
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

        private static void ProcessAssimpImport(string fileName, string outFile, int gridCellCount = 32, float stepScale = 1.0f, int cellSize = 8, int cellPadding = 1)
        {
            Stopwatch sw = new Stopwatch();
            sw.Start();

            Console.WriteLine("[{0}] Processing file {1}", sw.Elapsed, fileName);

            AssimpContext importer = new AssimpContext();
            importer.SetConfig(new ACSeparateBackfaceCullConfig(false));
            Scene scene = importer.ImportFile(fileName, PostProcessPreset.TargetRealTimeMaximumQuality);

            Console.WriteLine("[{0}] File loaded", sw.Elapsed);

            float scale = 1;

            Vector sceneMin = new Vector(float.MaxValue, float.MaxValue, float.MaxValue);
            Vector sceneMax = new Vector(float.MinValue, float.MinValue, float.MinValue);
            Matrix4x4 matrix = Matrix4x4.FromScaling(new Vector3D(scale));
            List<PreparedTriangle> triangleList = new List<PreparedTriangle>();

            List<Tuple<Vector, Vector, Bone>> bones = new List<Tuple<Vector, Vector, Bone>>();

            Preprocess(scene, scene.RootNode, ref matrix, triangleList, bones);

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

            if ((sceneMax.Z - sceneMin.Z) / step > 256)
                step = (sceneMax.Z - sceneMin.Z) / 256;

            step *= stepScale;

            //creating grid structure for faster triangle search
            TriangleMap triangleMap = new TriangleMap(sceneMin, sceneMax, gridCellCount, triangleList);

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

            Texture3D<ushort> data = new Texture3D<ushort>(4, sx, sy, sz);

            Console.WriteLine("[{0}] File preprocessed. X: {1}, Y: {2}, Z: {3}", sw.Elapsed, sx, sy, sz);

            float emptyCellCheckDistance = step * 0.5f / maximumDistance;// * 0.5f;// (float)Math.Sqrt(2) * 0.5f;// cellSize * step * 0.05f;// * (float)Math.Sqrt(2) * 0.5f;

            Iterate(0, sz * sy * sx, (i) =>
            {
                int iz = i / (sx * sy);
                int iy = (i % (sx * sy)) / sx;
                int ix = (i % (sx * sy)) % sx;

                if (ix == 0 && iy == 0)
                    Console.WriteLine("[{0}] Processing {1}", sw.Elapsed, new Vector(ix, iy, iz));

#if DEBUG
                byte[] testData = new byte[sx * sy * 4];
#endif
                Vector point = lowerBound + new Vector(ix, iy, iz) * step;

                float distance;
                Vector triangleWeights;
                object triangleData;
                Vector3i cellId = new Vector3i(ix / cellSize, iy / cellSize, iz / cellSize);

                float distancePercentage;

                bool empty = !triangleMap.FindTriangles(point, out distance, out triangleWeights, out triangleData);

                distancePercentage = Math.Sign(distance) * Math.Min(Math.Abs(distance / maximumDistance), 1.0f);

                // the distance
                //lock (data)
                {
                    data[ix, iy, iz, 0] = (new HalfFloat(distancePercentage)).Data;

#if DEBUG
                        // temporary test images. I had to scale the distance 4x to make them visible. Not sure it would work
                        // for all the meshes
                        testData[(ix + iy * sx) * 4] = distancePercentage > 0 ? (byte)(1024.0f * distancePercentage) : (byte)0;
                        testData[(ix + iy * sx) * 4 + 1] = distancePercentage < 0 ? (byte)(-1024.0f * distancePercentage) : (byte)0;
                        //testData[(ix + iy * sx) * 4 + 2] = (byte)(triangleData == null ? 128 : 0);
#endif

                    if (triangleData != null)
                    {
                        // Saved triangle data
                        Tuple<Mesh, Face> tuple = (Tuple<Mesh, Face>)triangleData;
                        Mesh mesh = tuple.Item1;
                        Face face = tuple.Item2;


                        if (mesh.TextureCoordinateChannelCount > 0 && mesh.TextureCoordinateChannels[0].Count > 0)
                        {
                            Vector3D tca = mesh.TextureCoordinateChannels[0][face.Indices[0]];
                            Vector3D tcb = mesh.TextureCoordinateChannels[0][face.Indices[1]];
                            Vector3D tcc = mesh.TextureCoordinateChannels[0][face.Indices[2]];

                            Vector3D tc = tca * triangleWeights.X + tcb * triangleWeights.Y + tcc * triangleWeights.Z;

                            if (tc.X < 0 || tc.Y < 0 || tc.X > 1 || tc.Y > 1 || tc.Z != 0)
                                Console.WriteLine("Result weights are invalid!");

                            // texture coords
                            data[ix, iy, iz, 1] = (new HalfFloat(tc.X)).Data;
                            data[ix, iy, iz, 2] = (new HalfFloat(tc.Y)).Data;
                        }

                        float uvdist = Math.Min(Math.Min(triangleWeights.X, triangleWeights.Y), triangleWeights.Z);
                        data[ix, iy, iz, 3] = (new HalfFloat(0.5f - uvdist)).Data;
                    }

                    // TODO: index + 1 and index + 2 should be texture coordinates. index + 3 is empty
                }

#if DEBUG
                if (ix == sx - 1 && iy == sy - 1)
                    Helper.SaveBitmap(testData, sx, sy, Path.GetFileNameWithoutExtension(outFile) + "_" + iz);
#endif
            }
            );

            // split to cells

            int usedCells = 0;
            int cellsx = (int)Math.Ceiling(sx / (float)cellSize);
            int cellsy = (int)Math.Ceiling(sy / (float)cellSize);
            int cellsz = (int)Math.Ceiling(sz / (float)cellSize);

            int totalCells = cellsx * cellsy * cellsz;
            int paddedCellSize = cellSize + cellPadding;


            Tuple<float, Texture3D<ushort>>[] cells = new Tuple<float, Texture3D<ushort>>[totalCells];

            for (int iz = 0; iz < cellsz; iz++)
                for (int iy = 0; iy < cellsy; iy++)
                    for (int ix = 0; ix < cellsx; ix++)
                    {
                        int index = ix + iy * cellsx + iz * cellsx * cellsy;

                        Texture3D<ushort> block = data.GetBlock(ix * cellSize, iy * cellSize, iz * cellSize, paddedCellSize, paddedCellSize, paddedCellSize, new HalfFloat(emptyCellCheckDistance).Data, 0, 0, 0);

                        float distancePercentage = float.MaxValue;

                        for (int i = 0; i < block.Data.Length; i += block.Components)
                        {
                            float distance = new HalfFloat(block.Data[i]).ToSingle();
                            if (Math.Abs(distance) < Math.Abs(distancePercentage))
                                distancePercentage = distance;
                        }

                        if (Math.Abs(distancePercentage) < emptyCellCheckDistance)
                        {
                            usedCells++;
                        }
                        else
                            block = null;

                        cells[index] = new Tuple<float, Texture3D<ushort>>(distancePercentage, block);
                    }


            int gridx;
            int gridy;
            int gridz;
            FindBestDividers(usedCells + 1, out gridx, out gridy, out gridz, 16384 / paddedCellSize);

            Texture3D<ushort> partialData = new Texture3D<ushort>(4, gridx * paddedCellSize, gridy * paddedCellSize, gridz * paddedCellSize);

            Console.WriteLine("[{0}] Got {1} empty cells, cell grid size {2}, {3:P}, total {4} of {5}x{5}x{5} cells, size {6} vs {7}, grid {8}x{9}x{10}",
                sw.Elapsed, totalCells - usedCells, new Vector3i(cellsx, cellsy, cellsz), usedCells / (float)totalCells,
                usedCells,
                paddedCellSize,
                partialData.Bytes,
                data.Bytes,
                gridx, gridy, gridz
                );
            Console.WriteLine("[{0}] SDF finished, saving KTX", sw.Elapsed);

            Helper.SaveKTX(0x881A, data.Width, data.Height, data.Depth, data.Data, outFile);

            Console.WriteLine("[{0}] Full size KTX saved, saving lower LOD", sw.Elapsed);

            //ushort[] partialData = new ushort[paddedCellSize * paddedCellSize * paddedCellSize * usedCells.Count * 4];
            int partialCells = 0;

            Texture3D<ushort> zeroLodData = new Texture3D<ushort>(4, cellsx, cellsy, cellsz);

            for (int iz = 0; iz < cellsz; iz++)
                for (int iy = 0; iy < cellsy; iy++)
                    for (int ix = 0; ix < cellsx; ix++)
                    {
                        int index = ix + iy * cellsx + iz * cellsx * cellsy;

                        int atlasX = 0;
                        int atlasY = 0;
                        int atlasZ = 0;

                        Tuple<float, Texture3D<ushort>> cell = cells[index];

                        if (cell.Item2 != null)
                        {
                            // Add used cell to 3D atlas TODO: move this code to separate function
                            partialCells++;

                            if (partialCells >= usedCells + 1)
                                throw new Exception("Too many cells for partial LOD: "+ partialCells + "!");

                            atlasZ = (partialCells / (gridy * gridx)) * paddedCellSize;
                            atlasY = ((partialCells % (gridy * gridx)) / gridx) * paddedCellSize;
                            atlasX = ((partialCells % (gridy * gridx)) % gridx) * paddedCellSize;
                           
                            partialData.PutBlock(cell.Item2, atlasX, atlasY, atlasZ);
                        }
                        else
                        {
                            atlasX = 0;
                            atlasY = 0;
                            atlasZ = 0;
                        }

                        zeroLodData[ix, iy, iz, 0] = (new HalfFloat(cell.Item1)).Data;
                        zeroLodData[ix, iy, iz, 1] = (new HalfFloat((atlasX) / (float)partialData.Width)).Data;
                        zeroLodData[ix, iy, iz, 2] = (new HalfFloat((atlasY) / (float)partialData.Height)).Data;
                        zeroLodData[ix, iy, iz, 3] = (new HalfFloat((atlasZ) / (float)partialData.Depth)).Data;
                    }

            Helper.SaveKTX(0x881A/*0x822C*/, zeroLodData.Width, zeroLodData.Height, zeroLodData.Depth, zeroLodData.Data, Path.GetFileNameWithoutExtension(outFile) + "_lod_0.ktx"); ; // GL_RGBA16 /*GL_RG16*/
            Helper.SaveKTX(0x881A, partialData.Width, partialData.Height, partialData.Depth, partialData.Data, Path.GetFileNameWithoutExtension(outFile) + "_lod_hi.ktx");

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

        private static void Preprocess(Scene scene, Node node, ref Matrix4x4 matrix, IList<PreparedTriangle> allTriangles, IList<Tuple<Vector, Vector, Bone>> bones)
        {
            Matrix4x4 prev = matrix;
            matrix = prev * node.Transform;

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
                Preprocess(scene, node.Children[i], ref matrix, allTriangles, bones);

            matrix = prev;
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
            float scale = args.Length > 3 ? 1.0f / int.Parse(args[3]) : 1.0f;
            int cellSize = args.Length > 4 ? int.Parse(args[4]) : 6;
            int cellPadding = args.Length > 5 ? int.Parse(args[5]) : 1;

            ProcessAssimpImport(fileName, outFileName, gridSize, scale, cellSize, cellPadding);
        }
    }
}

