using System;
using System.Numerics;
using System.Collections.Generic;

namespace SDFTool
{
    public static class MeshGenerator
    {
        private struct Triangle
        {
            public readonly ValueTuple<int, int, int>[] Points;

            public Triangle(int av, int atc, int an, int bv, int btc, int bn, int cb, int ctc, int cn)
            {
                Points = new[] {
                    new ValueTuple<int, int, int>(av - 1, atc - 1, an - 1),
                    new ValueTuple<int, int, int>(bv - 1, btc - 1, bn - 1),
                    new ValueTuple<int, int, int>(cb - 1, ctc - 1, cn - 1)
                };
            }
        }

        public struct Surface
        {
            public readonly string Name;

            public readonly Vector3[] Vertices;
            public readonly Vector3[] Normals;
            public readonly Vector4[] TexCoords;
            public readonly int[][] Faces;

            public readonly Vector3 LowerBound;
            public readonly Vector3 UpperBound;

            public readonly bool HasUV2;

            public Vector3 Center
            {
                get { return (UpperBound + LowerBound) / 2; }
            }

            public float Radius
            {
                get { return (UpperBound - LowerBound).Length() / 2; }
            }

            public Surface(Vector3[] vertices, Vector3[] normals, Vector4[] texcoords, int[][] faces, string name)
            {
                Name = name;
                Vertices = vertices;
                Normals = normals;
                TexCoords = texcoords;
                Faces = faces;

                float minx = float.MaxValue;
                float miny = float.MaxValue;
                float minz = float.MaxValue;
                float maxx = float.MinValue;
                float maxy = float.MaxValue;
                float maxz = float.MaxValue;

                foreach (Vector3 vertex in vertices)
                {
                    minx = Math.Min(minx, vertex.X);
                    miny = Math.Min(miny, vertex.Y);
                    minz = Math.Min(minz, vertex.Z);
                    maxx = Math.Min(minx, vertex.X);
                    maxy = Math.Max(maxy, vertex.Y);
                    maxz = Math.Max(maxz, vertex.Z);
                }

                LowerBound = new Vector3(minx, miny, minz);
                UpperBound = new Vector3(maxx, maxy, maxz);

                HasUV2 = false;

                foreach (Vector4 tc in texcoords)
                {
                    if (tc.Z != 0 || tc.W != 0)
                    {
                        HasUV2 = true;
                        break;
                    }
                }
            }
        }

        public static Surface CreateBoxMesh(Vector3 center, float width, float height, float depth,
            Matrix4x4 uvwTransform = default(Matrix4x4),
            string surfaceName = "box",
            bool cube = false,
            bool tcAsNormal = false)
        {
            Vector3 lb = center - new Vector3(width, height, depth) / 2;
            Vector3 ub = center + new Vector3(width, height, depth) / 2;

            return CreateBoxesMesh(
                new Tuple<Vector3, Vector3, Matrix4x4>[] { new Tuple<Vector3, Vector3, Matrix4x4>(lb, ub, uvwTransform) },
                surfaceName, cube, tcAsNormal);
        }

        public static Surface CreateBoxMesh(Vector3 lb, Vector3 ub, Matrix4x4 uvwTransform = default(Matrix4x4),
                string surfaceName = "box",
                bool cube = false,
                bool tcAsNormal = false)
        {
            return CreateBoxesMesh(
                new Tuple<Vector3, Vector3, Matrix4x4>[] { new Tuple<Vector3, Vector3, Matrix4x4>(lb, ub, uvwTransform) },
                surfaceName, cube, tcAsNormal);
        }

        public static Surface CreateBoxesMesh(Tuple<Vector3, Vector3, Matrix4x4> [] boxes,
                string surfaceName = "box",
                bool cube = false,
                bool tcAsNormal = false)
        {
            List<Vector3> vertices = new List<Vector3>();
            List<Vector3> normals = new List<Vector3>();
            List<Vector4> texCoords = new List<Vector4>();
            List<int[]> faces = new List<int[]>();

            foreach (var tuple in boxes)
                CreateBox(tuple.Item1, tuple.Item2, tuple.Item3, cube, tcAsNormal, vertices, normals, texCoords, faces);

            return new Surface(vertices.ToArray(), normals.ToArray(), texCoords.ToArray(), faces.ToArray(), surfaceName);
            //return new Surface(vertices, normals, texCoords, faces, surfaceName);
        }

        private static readonly Vector3[] s_boxNormals = {
                new Vector3(0.0f, 0.0f, 1.0f),
                new Vector3(0.0f, 1.0f, 0.0f),
                new Vector3(-1.0f, 0.0f, 0.0f),
                new Vector3(1.0f, 0.0f, 0.0f),
                new Vector3(0.0f, -1.0f, 0.0f),
                new Vector3(0.0f, 0.0f, -1.0f),
            };

        private static readonly Vector3[] s_cubeTexCoords = {
                new Vector3(0.0f, 0.0f, 0.0f),
                new Vector3(1.0f, 0.0f, 0.0f),
                new Vector3(1.0f, 1.0f, 0.0f),
                new Vector3(0.0f, 1.0f, 0.0f),

                new Vector3(0.0f, 1.0f, 1.0f),
                new Vector3(1.0f, 1.0f, 1.0f),
                new Vector3(1.0f, 0.0f, 1.0f),
                new Vector3(0.0f, 0.0f, 1.0f),
            };

        private static readonly Vector3[] s_boxTexCoords = {
                new Vector3(0.0f, 0.0f, 0.0f),
                new Vector3(1.0f, 0.0f, 0.0f),

                new Vector3(0.0f, 1.0f, 0.0f),
                new Vector3(1.0f, 1.0f, 0.0f),

                new Vector3(0.0f, 0.0f, 0.0f),
                new Vector3(1.0f, 0.0f, 0.0f),

                new Vector3(0.0f, 1.0f, 0.0f),
                new Vector3(1.0f, 1.0f, 0.0f),

                new Vector3(0.0f, 0.0f, 0.0f),
                new Vector3(1.0f, 0.0f, 0.0f),

                new Vector3(0.0f, 1.0f, 0.0f),
                new Vector3(1.0f, 1.0f, 0.0f)
        };

        private static readonly Triangle[] s_cubeFaces = {
                // front
                new Triangle(1, 1, 1, 3, 3, 1, 2, 2, 1),
                new Triangle(1, 1, 1, 4, 4, 1, 3, 3, 1),
                // top
                new Triangle(3, 3, 2, 4, 4, 2, 5, 5, 2),
                new Triangle(3, 3, 2, 5, 5, 2, 6, 6, 2),
                // right
                new Triangle(2, 2, 3, 3, 3, 3, 6, 6, 3),
                new Triangle(2, 2, 3, 6, 6, 3, 7, 7, 3),
                // left
                new Triangle(1, 1, 4, 8, 8, 4, 5, 5, 4),
                new Triangle(1, 1, 4, 5, 5, 4, 4, 4, 4),
                // back
                new Triangle(6, 6, 5, 5, 5, 5, 8, 8, 5),
                new Triangle(6, 6, 5, 8, 8, 5, 7, 7, 5),
                // bottom
                new Triangle(1, 1, 6, 7, 7, 6, 8, 8, 6),
                new Triangle(1, 1, 6, 2, 2, 6, 7, 7, 6)
        };

        private static readonly Triangle[] s_boxFaces = {
                new Triangle(1, 10, 1, 3, 12, 1, 4, 11, 1),
                new Triangle(4, 11, 1, 2, 9, 1, 1, 10, 1),

                new Triangle(5, 9, 2, 6, 10, 2, 8, 12, 2),
                new Triangle(8, 12, 2, 7, 11, 2, 5, 9, 2),

                new Triangle(1, 5, 3, 2, 6, 3, 6, 8, 3),
                new Triangle(6, 8, 3, 5, 7, 3, 1, 5, 3),

                new Triangle(2, 1, 4, 4, 2, 4, 8, 4, 4),
                new Triangle(8, 4, 4, 6, 3, 4, 2, 1, 4),

                new Triangle(4, 5, 5, 3, 6, 5, 7, 8, 5),
                new Triangle(7, 8, 5, 8, 7, 5, 4, 5, 5),

                new Triangle(3, 1, 6, 1, 2, 6, 5, 4, 6),
                new Triangle(5, 4, 6, 7, 3, 6, 3, 1, 6)
        };

        private static void CreateBox(Vector3 lb, Vector3 ub,
            Matrix4x4 uvwTransform, bool cube, bool tcAsNormal,
            List<Vector3> vertices, List<Vector3> normals, List<Vector4> texCoords, List<int[]> faces)
        {
            if (uvwTransform.M11 == 0)
                uvwTransform = Matrix4x4.Identity;

            // Here goes ideal box creation - 8 vertices, 6 normals, few texcoords

            // 8 vertices
            Vector3[] tempVertices = {
                lb,
                new Vector3(ub.X, lb.Y, lb.Z),
                new Vector3(ub.X, ub.Y, lb.Z),
                new Vector3(lb.X, ub.Y, lb.Z),

                new Vector3(lb.X, ub.Y, ub.Z),
                ub,
                new Vector3(ub.X, lb.Y, ub.Z),
                new Vector3(lb.X, lb.Y, ub.Z),
            };

            // 6 normals
            Vector3[] tempNormals = s_boxNormals;

            // 8 or 12 tex coords
            Vector3[] tempTexCoords = cube ? s_cubeTexCoords : s_boxTexCoords;

            // 12 triangles, 2 for the each box side
            Triangle[] tempFaces = cube ? s_cubeFaces : s_boxFaces;

            // Here goes deduplication: every vertex should have unique normal and texcoord resulting in 24 unique vertices, normals and tex coords

            List<Vector3> newVertices = new List<Vector3>();
            List<Vector3> newNormals = new List<Vector3>();
            List<Vector4> newTexCoords = new List<Vector4>();
            List<int[]> newFaces = new List<int[]>();

            // temporary collection
            List<ValueTuple<int, int, int>> vertexPairs = new List<ValueTuple<int, int, int>>();

            for (int i = 0; i < tempFaces.Length; i++)
            {
                int[] face = new int[3];

                for (int j = 0; j < tempFaces[i].Points.Length; j++)
                {
                    int index = vertexPairs.IndexOf(tempFaces[i].Points[j]);
                    if (index == -1)
                    {
                        index = vertexPairs.Count;
                        vertexPairs.Add(tempFaces[i].Points[j]);

                        newVertices.Add(tempVertices[tempFaces[i].Points[j].Item1]);

                        Vector3 tc = Vector3.Transform(tempTexCoords[tempFaces[i].Points[j].Item2], uvwTransform);
                        newTexCoords.Add(new Vector4(tc, 0.0f));

                        if (tcAsNormal)
                            newNormals.Add(tc);
                        else
                            newNormals.Add(tempNormals[tempFaces[i].Points[j].Item3]);
                    }
                    face[j] = index + vertices.Count;
                }
                newFaces.Add(face);
            }

            vertices.AddRange(newVertices);
            normals.AddRange(newNormals);
            faces.AddRange(newFaces);
            texCoords.AddRange(newTexCoords);
        }
    }
}

