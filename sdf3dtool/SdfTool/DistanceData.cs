using System;

#if UNITY_EDITOR
using Vector2 = RunServer.SdfTool.Vector2e;
using Vector3 = RunServer.SdfTool.Vector3e;
using Vector4 = RunServer.SdfTool.Vector4e;
#else
using System.Numerics;
#endif

namespace RunServer.SdfTool
{
    public struct PixelData
    {
        public readonly Vector3 DistanceUV;
        public readonly Vector4i Bones;
        public readonly Vector4 BoneWeights;

        public PixelData(float distance, float u, float v, Vector4i bones, Vector4 weights)
        {
            DistanceUV = new Vector3(distance, u, v);
            Bones = bones;
            BoneWeights = weights;
        }
    }

    public struct DistanceData
    {
        [Flags]
        private enum PointFlags : uint
        {
            None = 0,
            Uvs = 1,
            Bones = 2,
            Lods = 4
        }

        private static readonly uint Signature = (('P' | ('T' << 8) | ('0' << 16) | ('3' << 24)));

        public readonly int CellSize;
        public readonly Vector3i Size;
        public readonly Vector3 LowerBound;
        public readonly Vector3 UpperBound;
        public readonly PixelData[][] LodData;

        public PixelData[] Data
        {
            get { return LodData[0]; }
        }
        public DistanceData(int cellSize, PixelData[] data, Vector3i size, Vector3 lowerBound, Vector3 upperBound)
        {
            CellSize = cellSize;
            LodData = new PixelData[][] { data };
            Size = size;
            LowerBound = lowerBound;
            UpperBound = upperBound;
        }

        public DistanceData(System.IO.Stream stream)
        {
            using (System.IO.BinaryReader reader = new System.IO.BinaryReader(stream))
            {
                uint signature = reader.ReadUInt32();

                if (signature != Signature)
                    throw new Exception("Invalid points v2 file signature: " + signature);

                int sx = reader.ReadInt32();
                int sy = reader.ReadInt32();
                int sz = reader.ReadInt32();
                Size = new Vector3i(sx, sy, sz);
                CellSize = reader.ReadInt32();
                float lx = reader.ReadSingle();
                float ly = reader.ReadSingle();
                float lz = reader.ReadSingle();
                float ux = reader.ReadSingle();
                float uy = reader.ReadSingle();
                float uz = reader.ReadSingle();
                LowerBound = new Vector3(lx, ly, lz);
                UpperBound = new Vector3(ux, uy, uz);

                PointFlags flags = (PointFlags)reader.ReadUInt32();

                int nlods = reader.ReadInt32();

                LodData = new PixelData[nlods][];

                for (int l = 0; l < nlods; l++)
                {
                    int length = reader.ReadInt32();

                    LodData[l] = new PixelData[length];

                    for (int i = 0; i < length; i++)
                    {
                        float dx = reader.ReadSingle();

                        float dy = 0.0f;
                        float dz = 0.0f;
                        if ((flags & PointFlags.Uvs) != 0)
                        {
                            dy = reader.ReadSingle();
                            dz = reader.ReadSingle();
                        }

                        int bx = 0;
                        int by = 0;
                        int bz = 0;
                        int bw = 0;
                        float wx = 1.0f;
                        float wy = 0.0f;
                        float wz = 0.0f;
                        float ww = 0.0f;

                        if ((flags & PointFlags.Bones) != 0)
                        {
                            bx = reader.ReadInt32();
                            by = reader.ReadInt32();
                            bz = reader.ReadInt32();
                            bw = reader.ReadInt32();
                            wx = reader.ReadSingle();
                            wy = reader.ReadSingle();
                            wz = reader.ReadSingle();
                            ww = reader.ReadSingle();
                        }

                        LodData[l][i] = new PixelData(dx, dy, dz, new Vector4i(bx, by, bz, bw), new Vector4(wx, wy, wz, ww));
                    }
                }
            }
        }

        public void SaveTo(System.IO.Stream stream)
        {
            using (System.IO.BinaryWriter writer = new System.IO.BinaryWriter(stream))
            {
                writer.Write(Size.X);
                writer.Write(Size.Y);
                writer.Write(Size.Z);
                writer.Write(CellSize);
                writer.Write(LowerBound.X);
                writer.Write(LowerBound.Y);
                writer.Write(LowerBound.Z);
                writer.Write(UpperBound.X);
                writer.Write(UpperBound.Y);
                writer.Write(UpperBound.Z);
                writer.Write(Data.Length);

                for (int i = 0; i < Data.Length; i++)
                {
                    writer.Write(Data[i].DistanceUV.X);
                    writer.Write(Data[i].DistanceUV.Y);
                    writer.Write(Data[i].DistanceUV.Z);
                    writer.Write(Data[i].Bones.X);
                    writer.Write(Data[i].Bones.Y);
                    writer.Write(Data[i].Bones.Z);
                    writer.Write(Data[i].Bones.W);
                    writer.Write(Data[i].BoneWeights.X);
                    writer.Write(Data[i].BoneWeights.Y);
                    writer.Write(Data[i].BoneWeights.Z);
                    writer.Write(Data[i].BoneWeights.W);
                }
            }
        }
    }
}

