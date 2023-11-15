using System;
using System.IO;

#if USE_SYSTEM_DRAWING
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
#endif

namespace SDFTool.Utils
{
    /// <summary>
    /// Set of helpers to save data to temporary bitmap or mesh formats
    /// </summary>
    public class Bitmap
    {
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


#if USE_SYSTEM_DRAWING
        public static int[] LoadBitmap(string file, out int width, out int height)
        {
            int[] buffer;
            using (System.Drawing.Bitmap image = new System.Drawing.Bitmap(file))
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

