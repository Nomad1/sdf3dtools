using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace SDFTool.Utils
{
    public struct Array3D<T> where T:struct
    {
        private readonly int m_components;
        private readonly int m_width;
        private readonly int m_height;
        private readonly int m_depth;

        private readonly T[] m_data;

        public int Bytes
        {
            get { return m_data.Length * Marshal.SizeOf(typeof(T)); }
        }

        public int Width
        {
            get { return m_width; }
        }

        public int Height
        {
            get { return m_height; }
        }

        public int Depth
        {
            get { return m_depth; }
        }

        public int Components
        {
            get { return m_components; }
        }

        public T[] Data
        {
            get { return m_data; }
        }

        public T this[int x, int y, int z, int c = 0]
        {
            get
            {
                return m_data[(x + y * m_width + z * m_width * m_height) * m_components + c];
            }
            set
            {
                m_data[(x + y * m_width + z * m_width * m_height) * m_components + c] = value;
            }
        }

        public T this[int index]
        {
            get
            {
                return m_data[index];
            }
            set
            {
                m_data[index] = value;
            }
        }

        public Array3D(int components, int width, int height, int depth)
        {
            m_width = width;
            m_height = height;
            m_depth = depth;
            m_components = components;
            m_data = new T[components * width * height * depth];
        }

        public Array3D(T [] data, int width, int height, int depth)
        {
            m_width = width;
            m_height = height;
            m_depth = depth;
            m_components = 1;
            m_data = data;
        }

        public bool IsValidCoord(int x, int y, int z)
        {
            return x >= 0 && x < m_width && y >= 0 && y < m_height && z >= 0 && z < m_depth;
        }

        public Array3D<T> GetBlock(int fromx, int fromy, int fromz, int w, int h, int d, params T[] defaultValue)
        {
            Debug.Assert(defaultValue == null || defaultValue.Length >= m_components);

            Array3D<T> result = new Array3D<T>(m_components, w, h, d);

            for (int nz = 0; nz < d; nz++)
            {
                int bz = fromz + nz;
                for (int ny = 0; ny < h; ny++)
                {
                    int by = fromy + ny;
                    for (int nx = 0; nx < w; nx++)
                    {
                        int bx = fromx + nx;

                        if (!IsValidCoord(bx, by, bz))
                        {
                            for (int nc = 0; nc < m_components; nc++)
                                result[nx, ny, nz, nc] = defaultValue == null ? this[0, 0, 0, nc] : defaultValue[nc];
                        }
                        else
                        {
                            for (int nc = 0; nc < m_components; nc++)
                                result[nx, ny, nz, nc] = this[bx, by, bz, nc];
                        }
                    }
                }
            }

            return result;
        }

        public void PutBlock(Array3D<T> block, int tox, int toy, int toz)
        {
            PutBlock(block, tox, toy, toz, (k, i) => k);
        }

        public void PutBlock<K>(Array3D<K> block, int tox, int toy, int toz, Func<K, T> processor) where K : struct
        {
            PutBlock(block, tox, toy, toz, (k, i) => processor(k));
        }

        public void PutBlock<K>(Array3D<K> block, int tox, int toy, int toz, Func<K, int, T> processor) where K : struct
        {
            for (int nz = 0; nz < block.Depth; nz++)
            {
                int bz = toz + nz;
                for (int ny = 0; ny < block.Height; ny++)
                {
                    int by = toy + ny;
                    for (int nx = 0; nx < block.Width; nx++)
                    {
                        int bx = tox + nx;

                        for (int nc = 0; nc < m_components; nc++)
                            this[bx, by, bz, nc] = processor(block[nx, ny, nz, nc], nc);
                    }
                }
            }
        }

        public void PutBlock<K>(K [] block, int blockWidth, int blockHeight, int blockDepth, int components, int tox, int toy, int toz, Func<K, int, T> processor) where K : struct
        {
            for (int nz = 0; nz < blockDepth; nz++)
            {
                int bz = toz + nz;
                for (int ny = 0; ny < blockHeight; ny++)
                {
                    int by = toy + ny;
                    for (int nx = 0; nx < blockWidth; nx++)
                    {
                        int bx = tox + nx;

                        for (int nc = 0; nc < m_components; nc++)
                            this[bx, by, bz, nc] = processor(block[(nx + ny * blockWidth + nz * blockWidth * blockHeight) * components + nc], nc);
                    }
                }
            }
        }
    }
}

