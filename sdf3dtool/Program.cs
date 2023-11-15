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
            // Fill in data structure with SDF data and additional parameters from original model
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


#if OLD_MODE
            List<CellProcessor.BrickData> bricks = new List<CellProcessor.BrickData>();
            float[][] alods;
            System.Numerics.Vector2[] nuv;
            Vector4[] nzeroLod;
            Vector3i topLodTextureSize;

            // find non-empty cells
            int usedCells = CellProcessor.ProcessCells(data, numberOfLods,
                out topLodTextureSize,
                out alods, out nuv, out nzeroLod, bricks);

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

            Array3D<ushort> zeroLod = new Array3D<ushort>(4, data.Size.X / data.CellSize, data.Size.Y / data.CellSize, data.Size.Z / data.CellSize);
            for (int j = 0; j < nzeroLod.Length; j++)
            {
                zeroLod[j * 2 + 0] = Helper.PackFloatToUShort(nzeroLod[j].X);
                zeroLod[j * 2 + 1] = Helper.PackFloatToUShort(nzeroLod[j].Y);
                zeroLod[j * 2 + 2] = Helper.PackFloatToUShort(nzeroLod[j].Z);
                zeroLod[j * 2 + 3] = Helper.PackFloatToUShort(nzeroLod[j].W);
            }
#else
            List<CellProcessor.BrickData> bricks = new List<CellProcessor.BrickData>();
            float[] alods;
            System.Numerics.Vector2[] nuv;
            Vector3i topLodTextureSize;

            // find non-empty cells
            int usedCells = CellProcessor.ProcessBricks(data,
                out topLodTextureSize,
                out alods, out nuv, bricks, psnr);

#if LOD2_16BIT
            Array3D<ushort>[] lods = new Array3D<ushort>[1];
            {
                lods[0] = new Array3D<ushort>(1, topLodTextureSize.X, topLodTextureSize.Y, topLodTextureSize.Z);
                for (int j = 0; j < alods.Length; j++)
                    lods[0][j] = Utils.Utils.PackFloatToUShort(alods[j]);
            }
#else
            Array3D<float>[] lods = new Array3D<float>[1];
            {
                lods[0] = new Array3D<float>(1, topLodTextureSize.X, topLodTextureSize.Y, topLodTextureSize.Z);
                for (int j = 0; j < alods.Length; j++)
                    lods[0][j] = alods[j];
            }
#endif

            Array3D<ushort> uv = new Array3D<ushort>(2, topLodTextureSize.X, topLodTextureSize.Y, topLodTextureSize.Z);
            for (int j = 0; j < nuv.Length; j++)
            {
                uv[j * 2 + 0] = Utils.Utils.PackFloatToUShort(nuv[j].X);
                uv[j * 2 + 1] = Utils.Utils.PackFloatToUShort(nuv[j].Y);
            }
#endif

            MeshGenerator.Shape[] boxes = new MeshGenerator.Shape[bricks.Count];

            for (int i = 0; i < boxes.Length; i++)
            {
                boxes[i] = new MeshGenerator.Shape(
                    System.Numerics.Matrix4x4.CreateScale(bricks[i].Size) * System.Numerics.Matrix4x4.CreateTranslation(bricks[i].Position),
                    System.Numerics.Matrix4x4.Identity,
                    MeshGenerator.ShapeType.Cube,
                    MeshGenerator.ShapeFlags.NoNormals,
                    new float[] {
                        bricks[i].BrickId
                    });

                boxes[i].SetCubeVertexData(bricks[i].VertexDistances, bricks[i].BoneWeights);
            }


            //Console.WriteLine("[{0}] Got {1} empty cells, cell grid size {2}, {3:P}, total {4} of {5}x{5}x{5} cells, size {6} vs {7}, grid {8}x{9}x{10}",
            //sw.Elapsed, totalCells - usedCells, new Vector3i(cellsx, cellsy, cellsz), usedCells / (float)totalCells,
            //usedCells,
            //topLodCellSize,
            //topLodDistance.Bytes,
            //data.Length * Marshal.SizeOf<PixelData>(),
            //packx, packy, packz
            //);

            Console.WriteLine("[{0}] Saving LODs", sw.Elapsed);

#if OLD_MODE
            Helper.SaveKTX(
#if LOD0_8BIT
                Helper.KTX_RGBA8,
#else
                Helper.KTX_RGBA16F,
#endif
                zeroLod, outFile, "_lod_0.3d.ktx");

#endif
#if LOD2_8BIT
            Ktx.SaveKTX(Ktx.KTX_R8, lodDistance, outFile, "_lod.3d.ktx");
#elif LOD2_16BIT
            Ktx.SaveKTX(Ktx.KTX_R16F, lods, outFile, "_lod.3d.ktx");
#else
            Ktx.SaveKTX(Ktx.KTX_R32F, lods, outFile, "_lod.3d.ktx");
#endif

            Ktx.SaveKTX(Ktx.KTX_RG16F, uv, outFile, "_lod_2_uv.3d.ktx");

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

