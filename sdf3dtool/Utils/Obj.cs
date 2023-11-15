using System.IO;

namespace SDFTool.Utils
{
    public class Obj
    {
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

                    for (int i = 0; i < surface.Vertices.Count; i++)
                        writer.WriteLine("v {0} {1} {2}", surface.Vertices[i].X, surface.Vertices[i].Y, surface.Vertices[i].Z);

                    for (int i = 0; i < surface.Normals.Count; i++)
                        writer.WriteLine("vn {0} {1} {2}", surface.Normals[i].X, surface.Normals[i].Y, surface.Normals[i].Z);

                    for (int i = 0; i < surface.TexCoords.Count; i++)
                    {
                        writer.Write("vt ");
                        for (int j = 0; j < surface.TexCoords[i].Length; j++)
                        {
                            writer.Write(surface.TexCoords[i][j]);
                            writer.Write(' ');
                        }
                        writer.WriteLine();
                    }

                    for (int i = 0; i < surface.Faces.Count; i++)
                    {
                        writer.Write("f");
                        for (int j = 0; j < surface.Faces[i].Length; j++)
                            writer.Write(" {0}/{0}/{0}", surface.Faces[i][j] + 1);
                        writer.WriteLine();
                    }
                }
            }
        }
    }
}