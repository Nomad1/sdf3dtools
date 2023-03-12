using System;
using SDFTool;
using System.IO;

namespace SDFTool
{
    public partial class Helper
    {
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

            string file = Path.GetFileNameWithoutExtension(outFile) + ".unigine.mesh";

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
                    writer.Write(surface.Vertices.Count); // vertices

                    for (int i = 0; i < surface.Vertices.Count; i++)
                    {
                        writer.Write(surface.Vertices[i].X);
                        writer.Write(surface.Vertices[i].Y);
                        writer.Write(surface.Vertices[i].Z);
                        // Unigine normals are stored in 16-bit signed ushort format
                        if (surface.Normals == null || surface.Normals.Count != surface.Vertices.Count)
                        {
                            writer.Write((ushort)32768);
                            writer.Write((ushort)32768);
                            writer.Write((ushort)32768);
                        }
                        else
                        {
                            writer.Write((ushort)((surface.Normals[i].X / 2.0f + 0.5f) * 65535.0f));
                            writer.Write((ushort)((surface.Normals[i].Y / 2.0f + 0.5f) * 65535.0f));
                            writer.Write((ushort)((surface.Normals[i].Z / 2.0f + 0.5f) * 65535.0f));
                        }
                        //writer.Write(surface.Normals[i].X);
                        //writer.Write(surface.Normals[i].Y);
                        //writer.Write(surface.Normals[i].Z);
                    }

                    writer.Write(surface.TexCoords.Count); // texture coordinates

                    for (int i = 0; i < surface.TexCoords.Count; i++)
                    {
                        writer.Write(surface.TexCoords[i][0]);
                        writer.Write(surface.TexCoords[i][1]);
                    }

                    if (surface.TCLength > 2)
                    {
                        writer.Write(surface.TexCoords.Count); //  second texcoords

                        /// !!! NON STANDARD !!!
                        writer.Write(surface.TCLength - 2);

                        for (int i = 0; i < surface.TexCoords.Count; i++)
                            for (int j = 2; j < surface.TCLength; j++)
                                writer.Write(surface.TexCoords[i][j]);
                    }
                    else
                        writer.Write(0);

                    writer.Write(surface.Faces.Count); // triangles

                    if (surface.Vertices.Count < 65535)
                    {
                        for (int i = 0; i < surface.Faces.Count; i++)
                            for (int j = 0; j < surface.Faces[i].Length; j++)
                                writer.Write((ushort)surface.Faces[i][j]);
                    }
                    else
                    {
                        for (int i = 0; i < surface.Faces.Count; i++)
                            for (int j = 0; j < surface.Faces[i].Length; j++)
                                writer.Write(surface.Faces[i][j]);
                    }
                }
            }
        }
    }
}

