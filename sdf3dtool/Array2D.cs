using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace SDFTool
{
    public struct Array2D<T> where T : struct
    {
        private readonly int m_components;
        private readonly int m_width;
        private readonly int m_height;

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

        public int Components
        {
            get { return m_components; }
        }

        public T[] Data
        {
            get { return m_data; }
        }

        public T this[int x, int y, int c]
        {
            get
            {
                return m_data[(x + y * m_width) * m_components + c];
            }
            set
            {
                m_data[(x + y * m_width) * m_components + c] = value;
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

        public Array2D(int components, int width, int height)
        {
            m_width = width;
            m_height = height;
            m_components = components;
            m_data = new T[components * width * height];
        }

        public bool IsValidCoord(int x, int y)
        {
            return x >= 0 && x < m_width && y >= 0 && y < m_height;
        }

        public Array2D<T> GetBlock(int fromx, int fromy, int w, int h, T[] defaultValue)
        {
            Debug.Assert(defaultValue == null || defaultValue.Length >= m_components);

            Array2D<T> result = new Array2D<T>(m_components, w, h);

            for (int ny = 0; ny < h; ny++)
            {
                int by = fromy + ny;
                for (int nx = 0; nx < w; nx++)
                {
                    int bx = fromx + nx;

                    if (!IsValidCoord(bx, by))
                    {
                        for (int nc = 0; nc < m_components; nc++)
                            result[nx, ny, nc] = defaultValue == null ? this[0, 0, nc] : defaultValue[nc];
                    }
                    else
                    {
                        for (int nc = 0; nc < m_components; nc++)
                            result[nx, ny, nc] = this[bx, by, nc];
                    }
                }
            }

            return result;
        }

        public void PutBlock(Array2D<T> block, int tox, int toy)
        {
            for (int ny = 0; ny < block.Height; ny++)
            {
                int by = toy + ny;
                for (int nx = 0; nx < block.Width; nx++)
                {
                    int bx = tox + nx;

                    for (int nc = 0; nc < m_components; nc++)
                        this[bx, by, nc] = block[nx, ny, nc];
                }
            }
        }

        public void PutBlock<K>(Array2D<K> block, int tox, int toy, Func<K, T> processor) where K : struct
        {
            for (int ny = 0; ny < block.Height; ny++)
            {
                int by = toy + ny;
                for (int nx = 0; nx < block.Width; nx++)
                {
                    int bx = tox + nx;

                    for (int nc = 0; nc < m_components; nc++)
                        this[bx, by, nc] = processor(block[nx, ny, nc]);
                }
            }
        }
    }
}