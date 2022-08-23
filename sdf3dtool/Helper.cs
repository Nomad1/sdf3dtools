using System;
using System.IO;

#if USE_SYSTEM_DRAWING
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
#endif

namespace SDFTool
{
    /// <summary>
    /// Set of helpers to save data to temporary bitmap or mesh formats
    /// </summary>
    public static class Helper
    {
        public const int KTX_RGBA16F = 0x881A;
        public const int KTX_RGBA16 = 0x805B;
        public const int KTX_RG16 = 0x822C;
        public const int KTX_RG16F = 0x822F;
        public const int KTX_R16F = 0x822D;
        public const int KTX_R8 = 0x1903;
        public const int KTX_RGBA8 = 0x8058;
        public const int KTX_RGB8 = 0x1907;

        /// <summary>
        /// Saves a 3d texture to KTX format with each pixel being a set of 4 ushort values
        /// </summary>
        /// <param name="format"></param>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <param name="depth"></param>
        /// <param name="data"></param>
        /// <param name="outfile"></param>
        public static void SaveKTX(int format, int width, int height, int depth, ushort[] data, string outfile)
        {
            string file = outfile.EndsWith(".ktx") ? outfile : (Path.GetFileNameWithoutExtension(outfile) + ".ktx");

            using (Stream stream = File.Open(file, FileMode.Create, FileAccess.Write, FileShare.None))
            using (BinaryWriter writer = new BinaryWriter(stream))
            {
                writer.Write(new byte[] { 0xAB, 0x4B, 0x54, 0x58, 0x20, 0x31, 0x31, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A });
                writer.Write(0x04030201);
                writer.Write(0x140B); // raw type. 0x140B = GL_HALF_FLOAT
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
        /// Saves a 3d texture to KTX format with each pixel being a set of 4 ushort values
        /// </summary>
        /// <param name="format"></param>
        /// <param name="width"></param>
        /// <param name="height"></param>
        /// <param name="depth"></param>
        /// <param name="data"></param>
        /// <param name="outfile"></param>
        public static void SaveKTX(int format, int width, int height, int depth, byte[] data, string outfile)
        {
            string file = Path.GetFileNameWithoutExtension(outfile) + ".ktx";

            using (Stream stream = File.Open(file, FileMode.Create, FileAccess.Write, FileShare.None))
            using (BinaryWriter writer = new BinaryWriter(stream))
            {
                writer.Write(new byte[] { 0xAB, 0x4B, 0x54, 0x58, 0x20, 0x31, 0x31, 0xBB, 0x0D, 0x0A, 0x1A, 0x0A });
                writer.Write(0x04030201);
                writer.Write(0x1401); // raw type. 0x1401 = GL_UNSIGNED_BYTE
                writer.Write(1); // raw size
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

        /// <summary>
        /// Save bitmap to 3-component PPM file
        /// </summary>
        /// <param name="data"></param>
        /// <param name="imageWidth"></param>
        /// <param name="imageHeight"></param>
        /// <param name="outFile"></param>
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


        /// <summary>
        /// Saves a mesh to Unigine .mesh format
        /// </summary>
        /// <param name="outFile"></param>
        public static void SaveUnigineMesh(MeshGenerator.Surface[] surfaces, string outFile)
        {
            float minx = float.MaxValue;
            float miny = float.MaxValue;
            float minz = float.MaxValue;
            float maxx = float.MinValue;
            float maxy = float.MaxValue;
            float maxz = float.MaxValue;

            foreach (MeshGenerator.Surface surface in surfaces)
            {
                minx = Math.Min(minx, surface.LowerBound.X);
                miny = Math.Min(miny, surface.LowerBound.Y);
                minz = Math.Min(minz, surface.LowerBound.Z);
                maxx = Math.Min(minx, surface.UpperBound.X);
                maxy = Math.Max(maxy, surface.UpperBound.Y);
                maxz = Math.Max(maxz, surface.UpperBound.Z);
            }

            float centerx = (maxx + minx) / 2;
            float centery = (maxy + miny) / 2;
            float centerz = (maxz + minz) / 2;

            float radius = (float)Math.Sqrt((maxx - minx) * (maxx - minx) + (maxy - miny) * (maxy - miny) + (maxz - minz) * (maxz - minz)) / 2;

            string file = Path.GetFileNameWithoutExtension(outFile) + ".mesh";

            using (Stream stream = File.Open(file, FileMode.Create, FileAccess.Write, FileShare.None))
            using (BinaryWriter writer = new BinaryWriter(stream))
            {
                writer.Write((int)(('m' | ('i' << 8) | ('0' << 16) | ('8' << 24))));
                writer.Write(minx);
                writer.Write(miny);
                writer.Write(minz);
                writer.Write(maxx);
                writer.Write(maxy);
                writer.Write(maxz);
                writer.Write(centerx);
                writer.Write(centery);
                writer.Write(centerz);
                writer.Write(radius);

                writer.Write(surfaces.Length); // surfaces

                foreach (MeshGenerator.Surface surface in surfaces)
                {
                    writer.Write(surface.Name.Length + 1);
                    writer.Write(surface.Name.ToCharArray());
                    writer.Write((byte)0);
                    writer.Write(surface.LowerBound.X);
                    writer.Write(surface.LowerBound.Y);
                    writer.Write(surface.LowerBound.Z);
                    writer.Write(surface.UpperBound.X);
                    writer.Write(surface.UpperBound.Y);
                    writer.Write(surface.UpperBound.Z);
                    writer.Write(surface.Center.X);
                    writer.Write(surface.Center.Y);
                    writer.Write(surface.Center.Z);
                    writer.Write(surface.Radius);
                }

                foreach (MeshGenerator.Surface surface in surfaces)
                {
                    writer.Write(surface.Vertices.Length); // vertices

                    for (int i = 0; i < surface.Vertices.Length; i++)
                    {
                        writer.Write(surface.Vertices[i].X);
                        writer.Write(surface.Vertices[i].Y);
                        writer.Write(surface.Vertices[i].Z);
                        // Unigine normals are stored in 16-bit signed ushort format
                        writer.Write((ushort)((surface.Normals[i].X / 2.0f + 0.5f) * 65535.0f));
                        writer.Write((ushort)((surface.Normals[i].Y / 2.0f + 0.5f) * 65535.0f));
                        writer.Write((ushort)((surface.Normals[i].Z / 2.0f + 0.5f) * 65535.0f));
                    }

                    writer.Write(surface.TexCoords.Length); // texture coordinates

                    for (int i = 0; i < surface.TexCoords.Length; i++)
                    {
                        writer.Write(surface.TexCoords[i].X);
                        writer.Write(surface.TexCoords[i].Y);
                    }

                    if (surface.HasUV2)
                    {
                        writer.Write(surface.TexCoords.Length); //  second texcoords

                        for (int i = 0; i < surface.TexCoords.Length; i++)
                        {
                            writer.Write(surface.TexCoords[i].Z);
                            writer.Write(surface.TexCoords[i].W);
                        }
                    }
                    else
                        writer.Write(0);

                    writer.Write(surface.Faces.Length); // triangles

                    for (int i = 0; i < surface.Faces.Length; i++)
                        for (int j = 0; j < surface.Faces[i].Length; j++)
                            writer.Write((ushort)surface.Faces[i][j]);
                }
            }
        }

        /// <summary>
        /// Saves a mesh to .obj format
        /// </summary>
        /// <param name="outFile"></param>
        public static void SaveObjMesh(MeshGenerator.Surface[] surfaces, string outFile)
        {
            string file = Path.GetFileNameWithoutExtension(outFile) + ".obj";

            using (Stream stream = File.Open(file, FileMode.Create, FileAccess.Write, FileShare.None))
            using (StreamWriter writer = new StreamWriter(stream))
            {
                writer.WriteLine("# OBJ export for {0} surfaces", surfaces.Length);

                foreach (MeshGenerator.Surface surface in surfaces)
                {
                    writer.WriteLine("o {0}", surface.Name);

                    for (int i = 0; i < surface.Vertices.Length; i++)
                        writer.WriteLine("v {0} {1} {2}", surface.Vertices[i].X, surface.Vertices[i].Y, surface.Vertices[i].Z);

                    for (int i = 0; i < surface.Normals.Length; i++)
                        writer.WriteLine("vn {0} {1} {2}", surface.Normals[i].X, surface.Normals[i].Y, surface.Normals[i].Z);

                    for (int i = 0; i < surface.TexCoords.Length; i++)
                        writer.WriteLine("vt {0} {1} {2} {3}", surface.TexCoords[i].X, surface.TexCoords[i].Y, surface.TexCoords[i].Z, surface.TexCoords[i].W);

                    for (int i = 0; i < surface.Faces.Length; i++)
                    {
                        writer.Write("f");
                        for (int j = 0; j < surface.Faces[i].Length; j++)
                            writer.Write(" {0}/{0}/{0}", surface.Faces[i][j] + 1);
                        writer.WriteLine();
                    }
                }
            }
        }

#if USE_SYSTEM_DRAWING
        public static int[] LoadBitmap(string file, out int width, out int height)
        {
            int[] buffer;
            using (Bitmap image = new Bitmap(file))
            {

                width = image.Width;
                height = image.Height;

                BitmapData bitmapdata = image.LockBits(new Rectangle(0, 0, image.Width, image.Height), ImageLockMode.ReadOnly, PixelFormat.Format32bppArgb);

                buffer = new int[bitmapdata.Width * bitmapdata.Height];

                Marshal.Copy(bitmapdata.Scan0, buffer, 0, buffer.Length);

                image.UnlockBits(bitmapdata);

                image.Dispose();
            }

            return buffer;
        }
#else
        public static int[] LoadBitmap(string file, out int width, out int height)
        {
            throw new NotImplementedException();
        }
#endif
}
}

