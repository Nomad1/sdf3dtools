using System;
using RunMobile.Utility;
using System.Threading.Tasks;

namespace SDFTool
{
    /// <summary>
    /// Set of helpers to save data to temporary bitmap or mesh formats
    /// </summary>
    public partial class Helper
    {
        public static float Clamp(float value, float min, float max)
        {
            return Math.Max(min, Math.Min(max, value));
        }

        public static byte PackFloatToSByte(float value)
        {
            value = Clamp(value, -1.0f, 1.0f);
            return (byte)Clamp((value * 0.5f + 0.5f) * 255.0f, 0.0f, 255.0f);
        }

        public static byte PackFloatToByte(float value)
        {
            return (byte)Clamp(value * 255.0f, 0.0f, 255.0f);
        }

        public static ushort PackFloatToUShort(float value)
        {
            return new HalfFloat(value).Data;
        }

        public static void Iterate(int from, int to, Action<int> action)
        {
#if !DEBUG
            int cpus = Environment.ProcessorCount;

            cpus /= 2;
#if !CUSTOM_FOR
            Parallel.For(from, to, new ParallelOptions { MaxDegreeOfParallelism = cpus }, action);
#else
            if (cpus <= 1 || from >= to)
            {
                for (int j = from; j < to; j++)
                    action(j);
                return;
            }

            int split = (to - from) / cpus;
            if (split == 0)
            {
                split = 1;
                cpus = to - from;
            }

            int done = 0;

            for (int i = 0; i < cpus - 1; i++)
            {
                int jfrom = i * split;
                int jto = (i + 1) * split;
                System.Threading.Thread thread = new System.Threading.Thread(delegate ()
                {
                    for (int j = jfrom; j < jto; j++)
                        action(j);

                    System.Threading.Interlocked.Increment(ref done);
                });


                thread.Start();
            }

            int ifrom = (cpus - 1) * split;
            for (int j = ifrom; j < to; j++)
                action(j);

            System.Threading.Interlocked.Increment(ref done);

            while (done < cpus)
                System.Threading.Thread.Sleep(1);
#endif
#else
            for (int i = from; i < to; i++)
                action(i);
#endif
        }


        /// <summary>
        /// Tries to find a best way to divide value by two numbers
        /// </summary>
        /// <param name="value"></param>
        /// <param name="widthCoef"></param>
        /// <param name="x"></param>
        /// <param name="y"></param>
        /// <param name="max"></param>
        public static void FindBestDividers2D(int value, out int x, out int y, int max)
        {
            int root = (int)Math.Ceiling(Math.Pow(value, 1 / 2.0));

            x = root; y = root;
            int closest = x * y;

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

                    int nvalue = lx * ly;

                    if (nvalue < value)
                        continue;

                    if (nvalue < closest)
                    {
                        x = lx;
                        y = ly;

                        closest = nvalue;

                        if (nvalue == value)
                            return;
                    }
                }
            }
        }
    }
}

