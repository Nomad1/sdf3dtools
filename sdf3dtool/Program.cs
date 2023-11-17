//#define LOD0_8BIT
//#define LOD2_8BIT
#define LOD2_16BIT
//#define OLD_MODE

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
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

            CellProcessor.LodData [] lodData;

#if SINGLE_LOD
            lodData = new CellProcessor.LodData[1];
            CellProcessor.ProcessBricks(data, psnr, lodData[0]);
#else
            // find non-empty cells
            CellProcessor.ProcessBricksWithLods(data, 4, out lodData);
#endif

#if LOD2_16BIT
            Array3D<ushort>[] lods = new Array3D<ushort>[1];
            {
                lods[0] = new Array3D<ushort>(1, lodData[0].Size.X, lodData[0].Size.Y, lodData[0].Size.Z);
                for (int j = 0; j < lodData[0].Distances.Length; j++)
                    lods[0][j] = Utils.Utils.PackFloatToUShort(lodData[0].Distances[j]);
            }
#else
            Array3D<float>[] lods = new Array3D<float>[1];
            {
                lods[0] = new Array3D<float>(1, topLodTextureSize.X, topLodTextureSize.Y, topLodTextureSize.Z);
                for (int j = 0; j < alods.Length; j++)
                    lods[0][j] = alods[j];
            }
#endif

            Array3D<ushort> uv = new Array3D<ushort>(2, lodData[0].Size.X, lodData[0].Size.Y, lodData[0].Size.Z);
            for (int j = 0; j < lodData[0].UV.Length; j++)
            {
                uv[j * 2 + 0] = Utils.Utils.PackFloatToUShort(lodData[0].UV[j].X);
                uv[j * 2 + 1] = Utils.Utils.PackFloatToUShort(lodData[0].UV[j].Y);
            }

            MeshGenerator.Shape[] boxes = new MeshGenerator.Shape[lodData[0].Bricks.Length];

            for (int i = 0; i < boxes.Length; i++)
            {
                boxes[i] = new MeshGenerator.Shape(
                    System.Numerics.Matrix4x4.CreateScale(lodData[0].Bricks[i].Size) * System.Numerics.Matrix4x4.CreateTranslation(lodData[0].Bricks[i].Position),
                    System.Numerics.Matrix4x4.Identity,
                    MeshGenerator.ShapeType.Cube,
                    MeshGenerator.ShapeFlags.NoNormals,
                    new float[] {
                        lodData[0].Bricks[i].BrickId
                    });

                boxes[i].SetCubeVertexData(lodData[0].Bricks[i].VertexDistances, lodData[0].Bricks[i].BoneWeights);
            }

            Array3D<byte>[] children = new Array3D<byte>[1];
            {
                int childrenBlock = 4 * 4 * 4;
                children[0] = new Array3D<byte>(childrenBlock, lodData[0].ChildrenSize.X, lodData[0].ChildrenSize.Y, lodData[0].ChildrenSize.Z);

                for (int j = 0; j < lodData[0].Children.Length; j++)
                {   
                    children[0][j * 4 + 0] = (byte)(lodData[0].Children[j]);
                    children[0][j * 4 + 1] = (byte)(lodData[0].Children[j] >> 8);
                    children[0][j * 4 + 2] = (byte)(lodData[0].Children[j] >> 16);
                    children[0][j * 4 + 3] = 0;
                }
            }


            Console.WriteLine("[{0}] Saving LODs", sw.Elapsed);

#if LOD2_8BIT
            Ktx.SaveKTX(Ktx.KTX_R8, lodDistance, outFile, "_lod.3d.ktx");
#elif LOD2_16BIT
            Ktx.SaveKTX(Ktx.KTX_R16F, lods, outFile, "_lod.3d.ktx");
#else
            Ktx.SaveKTX(Ktx.KTX_R32F, lods, outFile, "_lod.3d.ktx");
#endif

            Ktx.SaveKTX(Ktx.KTX_RG16F, uv, outFile, "_lod_2_uv.3d.ktx");

            Ktx.SaveKTX(Ktx.KTX_RGBA8, children, outFile, "_lod_children.3d.ktx");

            Console.WriteLine("[{0}] KTX saved, saving boundary mesh", sw.Elapsed);
            MeshGenerator.Surface[] boxesSurface = new MeshGenerator.Surface[] { MeshGenerator.CreateBoxesMesh(boxes, "main_box") };
            //Helper.SaveAssimpMesh(boxesSurface, outFile, new[] { bones }, scene.Animations, scene);

            Umesh.SaveUMesh(boxesSurface, outFile, bones, scene == null ? null : scene.Animations, scene, matrix);
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
            DistanceData data = Utils.Assimp.ProcessAssimpImport(fileName, gridSize, size, cellSize, sw, out scene, out bones, out matrix);

            ProcessPixelData(data, outFileName, psnr,
                sw, scene, bones, matrix);


            Console.WriteLine("[{0}] All done", sw.Elapsed);
            sw.Stop();
        }
    }
}

