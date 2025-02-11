//#define LOD0_8BIT
//#define LOD2_8BIT
#define LOD2_16BIT
//#define OLD_MODE

//#define SINGLE_LOD

using System;
using System.Diagnostics;
using System.IO;
using System.Numerics;
using RunServer.SdfTool;

using SDFTool.Utils;

namespace SDFTool
{
    static class Program
    {
        private static readonly string s_syntax = "Syntax: {0} input.dae output.ktx [grid_cells] [lod_0_size] [top_lod_cell_size] [psnr]\n" +
        "\n";

        private static void ProcessPixelData(
            DistanceData data,
            string outFile, float psnr,
            Stopwatch sw,
            Assimp.Scene scene = null, ValueTuple<string, Assimp.Bone>[][] bones = null, Assimp.Matrix4x4 matrix = default(Assimp.Matrix4x4)
            )
        {

            CellProcessor.LodData[] lodData;

#if SINGLE_LOD
            lodData = new CellProcessor.LodData[1];
            CellProcessor.ProcessBricks(data, psnr, out lodData[0]);
#else
            // find non-empty cells
            
            CellProcessor.ProcessBricksWithLods(data, data.CellSize % 4 == 0 ? 4 : data.CellSize % 3 == 0 ? 3 : data.CellSize, scene != null, out lodData);
#endif

            for (int l = 0; l < lodData.Length; l++)
            {
                int lodNumber = l + 1;

                Console.WriteLine("[{0}] Saving LOD {1}: size {2}", sw.Elapsed, lodNumber, lodData[l].Size);

#if LOD2_16BIT
                Array3D<ushort>[] lods = new Array3D<ushort>[1];
                {
                    lods[0] = new Array3D<ushort>(1, lodData[l].Size.X, lodData[l].Size.Y, lodData[l].Size.Z);
                    for (int j = 0; j < lodData[l].Distances.Length; j++)
                        lods[0][j] = Utils.Utils.PackFloatToUShort(lodData[l].Distances[j]);
                }
#else
                Array3D<float>[] lods = new Array3D<float>[1];
                {
                    lods[0] = new Array3D<float>(1, topLodTextureSize.X, topLodTextureSize.Y, topLodTextureSize.Z);
                    for (int j = 0; j < alods.Length; j++)
                        lods[0][j] = alods[j];
                }
#endif

                Array3D<ushort> uv = new Array3D<ushort>(2, lodData[l].Size.X, lodData[l].Size.Y, lodData[l].Size.Z);
                for (int j = 0; j < lodData[l].UV.Length; j++)
                {
                    uv[j * 2 + 0] = Utils.Utils.PackFloatToUShort(lodData[l].UV[j].X);
                    uv[j * 2 + 1] = Utils.Utils.PackFloatToUShort(lodData[l].UV[j].Y);
                }

                MeshGenerator.Shape[] boxes = new MeshGenerator.Shape[lodData[l].Bricks.Length];

                for (int i = 0; i < boxes.Length; i++)
                {
                    boxes[i] = new MeshGenerator.Shape(
                        System.Numerics.Matrix4x4.CreateScale(lodData[l].Bricks[i].Size) * System.Numerics.Matrix4x4.CreateTranslation(lodData[l].Bricks[i].Position),
                        System.Numerics.Matrix4x4.Identity,
                        MeshGenerator.ShapeType.Cube,
                        MeshGenerator.ShapeFlags.NoNormals,
                        new float[] {
                        lodData[l].Bricks[i].BrickId
                        });

                    boxes[i].SetCubeVertexData(lodData[l].Bricks[i].VertexDistances, lodData[l].Bricks[i].BoneWeights);
                }

                Array3D<byte>[] children = null;

                if (l != lodData.Length - 1)
                {
                    children = new Array3D<byte>[1];
                    {
                        int childrenBlock = 4 * 4 * 4;
                        children[0] = new Array3D<byte>(childrenBlock, lodData[l].ChildrenSize.X, lodData[l].ChildrenSize.Y, lodData[l].ChildrenSize.Z);

                        for (int j = 0; j < lodData[l].Children.Length; j++)
                        {
                            children[0][j * 4 + 0] = (byte)(lodData[l].Children[j]);
                            children[0][j * 4 + 1] = (byte)(lodData[l].Children[j] >> 8);
                            children[0][j * 4 + 2] = (byte)(lodData[l].Children[j] >> 16);
                            children[0][j * 4 + 3] = 0;
                        }
                    }
                }


                Console.WriteLine("[{0}] Saving textures", sw.Elapsed);

#if LOD2_8BIT
                Ktx.SaveKTX(Ktx.KTX_R8, lods, outFile, "_lod_" + lodNumber + ".3d.ktx");
#elif LOD2_16BIT
                Ktx.SaveKTX(Ktx.KTX_R16F, lods, outFile, "_lod_" + lodNumber + ".3d.ktx");
#else
                Ktx.SaveKTX(Ktx.KTX_R32F, lods, outFile, "_lod_" + lodNumber + ".3d.ktx");
#endif

                Ktx.SaveKTX(Ktx.KTX_RG16F, uv, outFile, "_lod_" + lodNumber + "_uv.3d.ktx");

                if (children != null)
                    Ktx.SaveKTX(Ktx.KTX_RGBA8, children, outFile, "_lod_" + lodNumber + "_children.3d.ktx");

                Console.WriteLine("[{0}] KTX saved, saving boundary mesh", sw.Elapsed);
                MeshGenerator.Surface[] boxesSurface = new MeshGenerator.Surface[] { MeshGenerator.CreateBoxesMesh(boxes, "main_box") };
                //Helper.SaveAssimpMesh(boxesSurface, outFile, new[] { bones }, scene.Animations, scene);

                Umesh.SaveUMesh(boxesSurface, outFile + "_lod_" + lodNumber, bones, scene == null ? null : scene.Animations, scene, matrix);

                if (l == 0)
                {
                    int cellSize = data.CellSize;
                    int cellsx = data.Size.X / data.CellSize + ((data.Size.X % data.CellSize != 0) ? 1 : 0);
                    int cellsy = data.Size.Y / data.CellSize + ((data.Size.Y % data.CellSize != 0) ? 1 : 0);
                    int cellsz = data.Size.Z / data.CellSize + ((data.Size.Z % data.CellSize != 0) ? 1 : 0);

                    // TODO: save 4 components: distance, brick id, UV

                    var lod0 = new Array3D<ushort>(2, cellsx, cellsy, cellsz);

                    for (int iz = 0; iz < cellsz; iz++)
                    {
                        for (int iy = 0; iy < cellsy; iy++)
                        {
                            for (int ix = 0; ix < cellsx; ix++)
                            {
                                int index = cellSize * (ix + iy * data.Size.X + iz * data.Size.X * data.Size.Y);

                                lod0[ix, iy, iz, 0] = Utils.Utils.PackFloatToUShort(data.Data[index].DistanceUV.X);
                            }
                        }
                    }

                    foreach (var brick in lodData[l].Bricks)
                    {
                        Vector3i pos = brick.Coord / data.CellSize;
                        float dist = 0.0f;
                        int count = 0;

                        foreach (var cubePoints in brick.VertexDistances)
                            foreach (var distance in cubePoints)
                            {
                                dist += distance;
                                count++;
                                //break;
                            }

                        dist /= count;

                        lod0[pos.X, pos.Y, pos.Z, 0] = Utils.Utils.PackFloatToUShort(dist);
                        lod0[pos.X, pos.Y, pos.Z, 1] = Utils.Utils.PackFloatToUShort(brick.BrickId);
                    }

                    MeshGenerator.Surface[] mainBoxesSurface = new MeshGenerator.Surface[] { MeshGenerator.CreateBoxMesh(data.LowerBound, data.UpperBound, default(Matrix4x4), "box", true) };
                    Umesh.SaveUMesh(mainBoxesSurface, outFile + "_lod_0", null, null, scene, matrix);

                    Ktx.SaveKTX(Ktx.KTX_RG16F, lod0, outFile, "_lod_0.3d.ktx");
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

            int gridSize = args.Length > 2 ? int.Parse(args[2]) : 64;
            int size = args.Length > 3 ? int.Parse(args[3]) : 32;
            int cellSize = args.Length > 4 ? int.Parse(args[4]) : 4;
            float psnr = args.Length > 5 ? float.Parse(args[5]) : 30;

            Stopwatch sw = new Stopwatch();
            sw.Start();

            Assimp.Scene scene;
            ValueTuple<string, Assimp.Bone>[][] bones;
            Assimp.Matrix4x4 matrix;
            DistanceData data;

            Console.WriteLine("[{0}] Loading file {1}", sw.Elapsed, fileName);

            if (Path.GetExtension(fileName).EndsWith(".points"))
            {
                data = new DistanceData(File.OpenRead(fileName));
                scene = null;
                bones = null;
                matrix = Assimp.Matrix4x4.Identity;

                Console.WriteLine("[{0}] Bounding box: {1} - {2}, points {3}", sw.Elapsed, data.LowerBound, data.UpperBound, data.Size);
            }
            else
                data = Utils.Assimp.ProcessAssimpImport(fileName, gridSize, size, cellSize, sw, out scene, out bones, out matrix);

            ProcessPixelData(data, outFileName, psnr, sw, scene, bones, matrix);
            
            Console.WriteLine("[{0}] All done", sw.Elapsed);
            sw.Stop();
        }

    }
}

