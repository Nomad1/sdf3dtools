using System;
using System.IO;
using System.Diagnostics;

namespace SDFTool.Utils
{
    /// <summary>
    /// Set of helpers to save data to temporary bitmap or mesh formats
    /// </summary>
    public class Ktx
    {
        public const int KTX_RGBA16F = 0x881A;
        public const int KTX_RGBA16 = 0x805B;
        public const int KTX_RG = 0x8227;
        public const int KTX_RG16 = 0x822C;
        public const int KTX_RG16F = 0x822F;
        public const int KTX_R16F = 0x822D;
        public const int KTX_R32F = 0x822E;
        public const int KTX_R8 = 0x1903;
        public const int KTX_RGBA8 = 0x8058;
        public const int KTX_RGB8 = 0x1907;

        private const int KTX_FLOAT = 0x1406;
        private const int KTX_HALF_FLOAT = 0x140B;
        private const int KTX_KTX_UNSIGNED_BYTE = 0x1401;

        public static void SaveKTX(int format, Array3D<float>[] arrays, string outFile, string extension)
        {
            float[][] data = new float[arrays.Length][];
            for (int i = 0; i < arrays.Length; i++)
                data[i] = arrays[i].Data;

            SaveKTX(format, arrays[0].Width, arrays[0].Height, arrays[0].Depth, data, Path.GetFileNameWithoutExtension(outFile) + extension);
        }

        public static void SaveKTX(int format, Array3D<ushort>[] arrays, string outFile, string extension)
        {
            ushort[][] data = new ushort[arrays.Length][];
            for (int i = 0; i < arrays.Length; i++)
                data[i] = arrays[i].Data;

            SaveKTX(format, arrays[0].Width, arrays[0].Height, arrays[0].Depth, data, Path.GetFileNameWithoutExtension(outFile) + extension);
        }

        public static void SaveKTX(int format, Array3D<byte>[] arrays, string outFile, string extension)
        {
            byte[][] data = new byte[arrays.Length][];
            for (int i = 0; i < arrays.Length; i++)
                data[i] = arrays[i].Data;

            SaveKTX(format, arrays[0].Width, arrays[0].Height, arrays[0].Depth, data, Path.GetFileNameWithoutExtension(outFile) + extension);
        }

        public static void SaveKTX(int format, Array3D<ushort> array3d, string outFile, string extension)
        {
            SaveKTX(format, array3d.Width, array3d.Height, array3d.Depth, new ushort[][] { array3d.Data }, Path.GetFileNameWithoutExtension(outFile) + extension);
        }

        public static void SaveKTX(int format, Array3D<byte> array3d, string outFile, string extension)
        {
            SaveKTX(format, array3d.Width, array3d.Height, array3d.Depth, new byte[][] { array3d.Data }, Path.GetFileNameWithoutExtension(outFile) + extension);
        }

        public static void SaveKTX(int format, Array2D<ushort> array2d, string outFile, string extension)
        {
            SaveKTX(format, array2d.Width, array2d.Height, 0, new ushort[][] { array2d.Data }, Path.GetFileNameWithoutExtension(outFile) + extension);
        }

        public static void SaveKTX(int format, Array2D<byte> array2d, string outFile, string extension)
        {
            SaveKTX(format, array2d.Width, array2d.Height, 0, new byte[][] { array2d.Data }, Path.GetFileNameWithoutExtension(outFile) + extension);
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
        public static void SaveKTX(int format, int width, int height, int depth, float[][] data, string outfile)
        {
            string file = outfile.EndsWith(".ktx") ? outfile : (Path.GetFileNameWithoutExtension(outfile) + ".ktx");

            using (Stream stream = File.Open(file, FileMode.Create, FileAccess.Write, FileShare.None))
            using (BinaryWriter writer = new BinaryWriter(stream))
            {
                writer.Write(new byte[] { 0xAB, 0x4B, 0x54, 0x58, 0x20, 0x31, 0x31, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A });
                writer.Write(0x04030201);
                writer.Write(KTX_FLOAT); // raw type
                writer.Write(4); // raw size
                writer.Write(format); // raw format
                writer.Write(format); // format
                writer.Write(format/* == KTX_RG16 || format == KTX_RG16F ? KTX_RG : format*/); // rgba?
                writer.Write(width);
                writer.Write(height);
                writer.Write(depth);
                writer.Write(0); // elements
                writer.Write(1); // faces
                writer.Write(data.Length); // mipmaps
                writer.Write(0); // metadata

                while (writer.BaseStream.Length < 64 + 0) // header + metadata size
                    writer.Write(0);


                for (int i = 0; i < data.Length; i++)
                {
                    writer.Write(data[i].Length * 4); // current mipmap size

                    for (int j = 0; j < data[i].Length; j++)
                        writer.Write(data[i][j]);
                }
            }
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
        public static void SaveKTX(int format, int width, int height, int depth, ushort[][] data, string outfile)
        {
            string file = outfile.EndsWith(".ktx") ? outfile : (Path.GetFileNameWithoutExtension(outfile) + ".ktx");

            using (Stream stream = File.Open(file, FileMode.Create, FileAccess.Write, FileShare.None))
            using (BinaryWriter writer = new BinaryWriter(stream))
            {
                writer.Write(new byte[] { 0xAB, 0x4B, 0x54, 0x58, 0x20, 0x31, 0x31, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A });
                writer.Write(0x04030201);
                writer.Write(KTX_HALF_FLOAT); // raw type
                writer.Write(2); // raw size
                writer.Write(format); // raw format
                writer.Write(format); // format
                writer.Write(format/* == KTX_RG16 || format == KTX_RG16F ? KTX_RG : format*/); // rgba?
                writer.Write(width);
                writer.Write(height);
                writer.Write(depth);
                writer.Write(0); // elements
                writer.Write(1); // faces
                writer.Write(data.Length); // mipmaps
                writer.Write(0); // metadata

                while (writer.BaseStream.Length < 64 + 0) // header + metadata size
                    writer.Write(0);


                for (int i = 0; i < data.Length; i++)
                {
                    writer.Write(data[i].Length * 2); // current mipmap size

                    for (int j = 0; j < data[i].Length; j++)
                        writer.Write(data[i][j]);
                }
            }
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
        public static void SaveKTX(int format, int width, int height, int depth, byte[][] data, string outfile)
        {
            string file = Path.GetFileNameWithoutExtension(outfile) + ".ktx";

            using (Stream stream = File.Open(file, FileMode.Create, FileAccess.Write, FileShare.None))
            using (BinaryWriter writer = new BinaryWriter(stream))
            {
                writer.Write(new byte[] { 0xAB, 0x4B, 0x54, 0x58, 0x20, 0x31, 0x31, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A });
                writer.Write(0x04030201);
                writer.Write(KTX_KTX_UNSIGNED_BYTE); // raw type. 0x1401 = GL_UNSIGNED_BYTE
                writer.Write(1); // raw size
                writer.Write(format); // raw format
                writer.Write(format); // format
                writer.Write(format == KTX_RG16 || format == KTX_RG16F ? KTX_RG : format == KTX_R8 || format == KTX_R16F ? KTX_R8 : format); // rgba?
                writer.Write(width);
                writer.Write(height);
                writer.Write(depth);
                writer.Write(0); // elements
                writer.Write(1); // faces
                writer.Write(data.Length); // mipmaps
                writer.Write(0); // metadata

                while (writer.BaseStream.Length < 64 + 0) // header + metadata size
                    writer.Write(0);

                int minBlocks = 1;
                int blockSizeX = 1, blockSizeY = 1, blockSizeZ = 1, blockSize = 8;

                for (int i = 0; i < data.Length; i++)
                {
                    int length = (Math.Max((width + blockSizeX - 1) / blockSizeX, minBlocks)
                                        * Math.Max((height + blockSizeY - 1) / blockSizeY, minBlocks)
                                        * Math.Max((depth + blockSizeZ - 1) / blockSizeZ, minBlocks)
                                        * blockSize / 8);

                    writer.Write(length); // current mipmap size

                    for (int j = 0; j < data[i].Length; j++)
                        writer.Write(data[i][j]);

                    for (int j = data[i].Length; j < length; j++)
                        writer.Write((byte)0);

                    width >>= 1;
                    height >>= 1;
                    depth >>= 1;
                }
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
    }
}

