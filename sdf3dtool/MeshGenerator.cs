using System;
using System.Numerics;
using System.Collections.Generic;

namespace SDFTool
{
    public static class MeshGenerator
    {
        public struct Triangle
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

        public enum ShapeType
        {
            /// <summary>
            /// Box with 2d UV coords
            /// </summary>
            Box = 0,
            /// <summary>
            /// Box with 3d UVW coords
            /// </summary>
            Cube = 1,
            /// <summary>
            /// Box with 3d UVW coords made of 5 tetrahedra
            /// </summary>
            Tetracube = 2
        }

        [Flags]
        public enum ShapeFlags
        {
            None = 0,
            /// <summary>
            /// Stores primitive number in texture coordinate slot: it's added after vertex texture coords
            /// </summary>
            ShapeNumber = 1,
            /// <summary>
            /// Stores sub-mesh number in texture coordinate slot: it's added after PrimitiveNumber
            /// </summary>
            GroupNumber = 2,
            /// <summary>
            /// Stores vertex number in texture coordinates: it's added after GroupNumber
            /// </summary>
            VertexNumber = 4,
            /// <summary>
            /// Ignore normals for deduplication functions
            /// </summary>
            NoNormals = 8,
        }

        public struct Shape
        {
            //public readonly Vector3 LowerBound;
            //public readonly Vector3 UpperBound;
            public readonly Matrix4x4 VertexTransform;

            public readonly Matrix4x4 TextureTransform;

            public readonly ShapeType Type;
            public readonly ShapeFlags Flags;

            public readonly float[] ExtraShapeData;


            public float[][] ExtraVertexData;
            public ValueTuple<int, float>[][] Weights;

            /// <summary>
            /// 
            /// </summary>
            /// <param name="vertexTransform">Model matrix</param>
            /// <param name="textureTransform">UV transform matrix</param>
            /// <param name="type">Cube, Box or something else</param>
            /// <param name="flags"></param>
            /// <param name="extra">Extra data stored to all vertices UV coords</param>
            public Shape(
                            Matrix4x4 vertexTransform,
                            Matrix4x4 textureTransform,
                            ShapeType type,
                            ShapeFlags flags,
                            float[] extra,
                            ValueTuple<int, float>[][] weights = null)
            {
                VertexTransform = vertexTransform;
                TextureTransform = textureTransform;
                Type = type;
                Flags = flags;
                ExtraShapeData = extra;
                Weights = weights;
                ExtraVertexData = null;
                Weights = null;
            }

            /// <summary>
            /// 
            /// </summary>
            /// <param name="extra">Additional UV data for all vertices. It makes sense only when we know the exact amount of vertices and their order, i.e. only for Cube</param>
            /// <param name="weights">Vertex weights for all vertices. It makes sense only when we know the exact amount of vertices and their order, i.e. only for Cube</param>
            public void SetCubeVertexData(
                            float[][] extra,
                            ValueTuple<int, float>[][] weights = null)
            {
                ExtraVertexData = extra;
                Weights = weights;
            }

            public void AddGeometry(Surface surface)
            {
                Vector3[] tempVertices = new Vector3[s_boxVertices.Length];

                for (int i = 0; i < tempVertices.Length; i++)
                    tempVertices[i] = Vector3.Transform(s_boxVertices[i], VertexTransform);

                float[][] texArray = Type == ShapeType.Box ? s_boxTexCoords : s_cubeTexCoords;

                float[][] texCoords = new float[texArray.Length][];

                bool useTexTransform = !TextureTransform.IsIdentity;

                int meshNumberIndex = 0;
                int vertexNumberIndex = -1;

                for (int i = 0; i < texCoords.Length; i++)
                {
                    List<float> tc = new List<float>(texArray[i]);

                    if (useTexTransform && (tc.Count == 2 || tc.Count == 3))
                    {
                        Vector3 tcvector = Vector3.Transform(new Vector3(tc[0], tc[1], tc.Count == 3 ? tc[2] : 0), TextureTransform);
                        tc[0] = tcvector.X;
                        tc[1] = tcvector.Y;
                        if (tc.Count == 3)
                            tc[2] = tcvector.Z;
                    }

                    if ((Flags & ShapeFlags.ShapeNumber) != 0)
                    {
                        tc.Add(surface.PrimitiveNumber);
                    }

                    if ((Flags & ShapeFlags.GroupNumber) != 0)
                    {
                        meshNumberIndex = tc.Count;
                        tc.Add(0);
                    }

                    if ((Flags & ShapeFlags.VertexNumber) != 0)
                    {
                        vertexNumberIndex = tc.Count;
                        tc.Add(0);
                    }

                    tc.AddRange(ExtraShapeData);

                    if (ExtraVertexData != null)
                    {
                        tc.AddRange(ExtraVertexData[i]);
                    }

                    texCoords[i] = tc.ToArray();
                }

                Triangle[][] triangles = Type == ShapeType.Box ? s_boxFaces : Type == ShapeType.Cube ? s_cubeFaces : s_tetraboxFaces;

                int groupNumber = 0;
                foreach (Triangle[] triangleArray in triangles)
                {
                    if ((Flags & ShapeFlags.GroupNumber) != 0)
                    {
                        for (int i = 0; i < texCoords.Length; i++)
                            texCoords[i][meshNumberIndex] = groupNumber;
                    }

                    surface.AddMesh(tempVertices,
                        (Flags & ShapeFlags.NoNormals) == 0 ?
                            Type == ShapeType.Tetracube ? s_tetraboxNormals : s_boxNormals
                            : null,
                        texCoords,
                        triangleArray,
                        Weights,
                        vertexNumberIndex,
                        false);

                    groupNumber++;
                }

                surface.PrimitiveNumber++;
            }
        }

        public class Surface
        {
            private readonly string m_name;

            private readonly IList<Vector3> m_vertices;
            private readonly IList<Vector3> m_normals;
            private readonly IList<float[]> m_texCoords;
            private readonly IList<int[]> m_faces;
            private readonly IList<ValueTuple<int, float>[]> m_vertexWeights;

            private Vector3 m_lowerBound;
            private Vector3 m_upperBound;
            private int m_tcLength;

            private int m_primitiveNumber;

            public string Name
            {
                get { return m_name; }
            }

            public IList<Vector3> Vertices
            {
                get { return m_vertices; }
            }

            public IList<Vector3> Normals
            {
                get { return m_normals; }
            }

            public IList<float[]> TexCoords
            {
                get { return m_texCoords; }
            }

            public IList<int[]> Faces
            {
                get { return m_faces; }
            }

            public IList<ValueTuple<int, float>[]> VertexWeights
            {
                get { return m_vertexWeights; }
            }

            public Vector3 LowerBound
            {
                get { return m_lowerBound; }
            }

            public Vector3 UpperBound
            {
                get { return m_upperBound; }
            }

            public Vector3 Center
            {
                get { return (UpperBound + LowerBound) / 2; }
            }

            public float Radius
            {
                get { return (UpperBound - LowerBound).Length() / 2; }
            }

            public int TCLength
            {
                get { return m_tcLength; }
            }

            public int PrimitiveNumber
            {
                get { return m_primitiveNumber; }
                internal set { m_primitiveNumber = value; }
            }

            //public Surface(Vector3[] vertices, Vector3[] normals, float[][] texcoords, int[][] faces, ValueTuple<int, float>[] vertexWeights, string name)
            //{
            //    m_name = name;
            //    m_vertices = vertices;
            //    m_normals = normals;
            //    m_texCoords = texcoords;
            //    m_faces = faces;
            //    m_vertexWeights = vertexWeights;

            //    UpdateBoundaries();
            //}


            public Surface(string name)
            {
                m_name = name;
                m_vertices = new List<Vector3>();
                m_normals = new List<Vector3>();
                m_texCoords = new List<float[]>();
                m_faces = new List<int[]>();
                m_vertexWeights = new List<ValueTuple<int, float>[]>();

                UpdateBoundaries();
            }

            public void UpdateBoundaries()
            {
                float minx = float.MaxValue;
                float miny = float.MaxValue;
                float minz = float.MaxValue;
                float maxx = float.MinValue;
                float maxy = float.MinValue;
                float maxz = float.MinValue;

                foreach (Vector3 vertex in Vertices)
                {
                    minx = Math.Min(minx, vertex.X);
                    miny = Math.Min(miny, vertex.Y);
                    minz = Math.Min(minz, vertex.Z);
                    maxx = Math.Max(maxx, vertex.X);
                    maxy = Math.Max(maxy, vertex.Y);
                    maxz = Math.Max(maxz, vertex.Z);
                }

                m_lowerBound = new Vector3(minx, miny, minz);
                m_upperBound = new Vector3(maxx, maxy, maxz);

                m_tcLength = 0;

                foreach (float[] tc in TexCoords)
                    if (tc.Length > TCLength)
                    {
                        m_tcLength = tc.Length;
                    }
            }

            public void AddMesh(
                Vector3[] tempVertices,
                Vector3[] tempNormals,
                float[][] tempTexCoords,
                Triangle[] tempFaces,
                ValueTuple<int, float>[][] tempWeights,
                int setVertexIdToTc = -1,
                bool updateBoundaries = true)
            {
                // Here goes deduplication: every vertex should have unique normal and texcoord resulting in 24 unique vertices, normals and tex coords for box

                // temporary collection
                List<ValueTuple<int, int, int>> vertexPairs = new List<ValueTuple<int, int, int>>(tempVertices.Length);

                int startIndex = m_vertices.Count;

                for (int i = 0; i < tempFaces.Length; i++)
                {
                    int[] face = new int[tempFaces[i].Points.Length];

                    for (int j = 0; j < tempFaces[i].Points.Length; j++)
                    {
                        ValueTuple<int, int, int> key = tempFaces[i].Points[j];

                        if (tempNormals == null)
                            key.Item3 = 0; // overide normals

                        int index = vertexPairs.IndexOf(key);
                        if (index == -1)
                        {
                            index = vertexPairs.Count;
                            vertexPairs.Add(key);

                            m_vertices.Add(tempVertices[key.Item1]);
                            if (tempNormals != null)
                                m_normals.Add(Vector3.Normalize(tempNormals[key.Item3]));


                            float[] tc = (float[])tempTexCoords[key.Item2].Clone();
                            if (setVertexIdToTc != -1)
                            {
                                tc[setVertexIdToTc] = index;
                            }

                            m_texCoords.Add(tc);

                            if (tempWeights != null)
                                m_vertexWeights.Add(tempWeights[key.Item1]);
                            else
                                m_vertexWeights.Add(new (int, float)[0]);
                        }
                        face[j] = index + startIndex;
                    }
                    m_faces.Add(face);
                }

                if (updateBoundaries)
                    UpdateBoundaries();
            }
        }

        [Obsolete]
        public static Surface CreateBoxMesh(Vector3 center, float width, float height, float depth,
            Matrix4x4 uvwTransform = default(Matrix4x4),
            string surfaceName = "box",
            bool cube = false)
        {
            Vector3 lb = center - new Vector3(width, height, depth) / 2;
            Vector3 ub = center + new Vector3(width, height, depth) / 2;

            return CreateBoxMesh(lb, ub, uvwTransform, surfaceName, cube);
        }

        [Obsolete]
        public static Surface CreateBoxMesh(
                Vector3 lb, Vector3 ub,
                Matrix4x4 uvwTransform = default(Matrix4x4),
                string surfaceName = "box",
                bool cube = false)
        {
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

            Surface surface = new Surface(surfaceName);
            surface.AddMesh(tempVertices, s_boxNormals, cube ? s_cubeTexCoords : s_boxTexCoords, cube ? s_cubeFaces[0] : s_boxFaces[0], null);
            surface.PrimitiveNumber++;

            return surface;
        }


        public static Surface CreateBoxMesh(Shape box, string surfaceName = "box")
        {
            return CreateBoxesMesh(
                new[] { box },
                surfaceName);
        }

        public static Surface CreateBoxesMesh(Shape[] boxes, string surfaceName = "box")
        {
            Surface surface = new Surface(surfaceName);

            foreach (Shape box in boxes)
            {
                box.AddGeometry(surface);
            }

            surface.UpdateBoundaries();
            return surface;
        }

        #region Static mesh data

        private static readonly Vector3[] s_boxNormals = {
                new Vector3(0.0f, 0.0f, 1.0f),
                new Vector3(0.0f, 1.0f, 0.0f),
                new Vector3(-1.0f, 0.0f, 0.0f),
                new Vector3(1.0f, 0.0f, 0.0f),
                new Vector3(0.0f, -1.0f, 0.0f),
                new Vector3(0.0f, 0.0f, -1.0f),
            };

        private static readonly Vector3[] s_tetraboxNormals = {
                new Vector3(0.0f, 0.0f, 1.0f),
                new Vector3(0.0f, 1.0f, 0.0f),
                new Vector3(-1.0f, 0.0f, 0.0f),
                new Vector3(1.0f, 0.0f, 0.0f),
                new Vector3(0.0f, -1.0f, 0.0f),
                new Vector3(0.0f, 0.0f, -1.0f),

                new Vector3(1.0f, -1.0f, 1.0f), // from 4 to 7
                new Vector3(1.0f, 1.0f, -1.0f), // from 8 to 3
                new Vector3(-1.0f, 1.0f, 1.0f), // from 2 to 5
                new Vector3(-1.0f, -1.0f, -1.0f), // from 6 to 1

                new Vector3(-1.0f, 1.0f, -1.0f), // from 7 to 4
                new Vector3(-1.0f, -1.0f, 1.0f), // from 3 to 8
                new Vector3(1.0f, -1.0f, -1.0f), // from 5 to 2
                new Vector3(1.0f, 1.0f, 1.0f), // from 1 to 6
            };

        private static readonly Vector3[] s_boxVertices = {
                new Vector3(0, 0, 0),
                new Vector3(1, 0, 0),
                new Vector3(1, 1, 0),
                new Vector3(0, 1, 0),

                new Vector3(0, 1, 1),
                new Vector3(1, 1, 1),
                new Vector3(1, 0, 1),
                new Vector3(0, 0, 1),
            };

        private static readonly float[][] s_cubeTexCoords = {
                new []{ 0.0f, 0.0f, 0.0f },
                new []{ 1.0f, 0.0f, 0.0f },
                new []{ 1.0f, 1.0f, 0.0f },
                new []{ 0.0f, 1.0f, 0.0f },

                new []{ 0.0f, 1.0f, 1.0f },
                new []{ 1.0f, 1.0f, 1.0f },
                new []{ 1.0f, 0.0f, 1.0f },
                new []{ 0.0f, 0.0f, 1.0f }
            };

        private static readonly float[][] s_boxTexCoords = {
                new []{ 0.0f, 0.0f },
                new []{ 1.0f, 0.0f },

                new []{ 0.0f, 1.0f },
                new []{ 1.0f, 1.0f },

                new []{ 0.0f, 0.0f },
                new []{ 1.0f, 0.0f },

                new []{ 0.0f, 1.0f },
                new []{ 1.0f, 1.0f },

                new []{ 0.0f, 0.0f },
                new []{ 1.0f, 0.0f },

                new []{ 0.0f, 1.0f },
                new []{ 1.0f, 1.0f }
        };

        private static readonly Triangle[][] s_cubeFaces = {
            new [] {
                // front
                new Triangle(1, 1, 1, 4, 4, 1, 3, 3, 1),
                new Triangle(1, 1, 1, 3, 3, 1, 2, 2, 1),
            //}, new [] {
                // top
                new Triangle(3, 3, 2, 4, 4, 2, 5, 5, 2),
                new Triangle(3, 3, 2, 5, 5, 2, 6, 6, 2),
            //}, new [] {
                // right
                new Triangle(2, 2, 3, 3, 3, 3, 6, 6, 3),
                new Triangle(2, 2, 3, 6, 6, 3, 7, 7, 3),
            //}, new [] {
                // left
                new Triangle(1, 1, 4, 8, 8, 4, 5, 5, 4),
                new Triangle(1, 1, 4, 5, 5, 4, 4, 4, 4),
            //}, new [] {
                // back
                new Triangle(6, 6, 5, 5, 5, 5, 8, 8, 5),
                new Triangle(6, 6, 5, 8, 8, 5, 7, 7, 5),
            //}, new [] {
                // bottom
                new Triangle(1, 1, 6, 2, 2, 6, 7, 7, 6),
                new Triangle(1, 1, 6, 7, 7, 6, 8, 8, 6),
            }
        };

        private static readonly Triangle[][] s_boxFaces = {
            new [] {
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
            }
        };

        private static readonly Triangle[][] s_tetraboxFaces = {
            new [] {
                // p1
                new Triangle(1, 1, 1, 4, 4, 1, 3, 3, 1),
                new Triangle(3, 3, 2, 4, 4, 2, 5, 5, 2),
                new Triangle(1, 1, 4, 5, 5, 4, 4, 4, 4),
                new Triangle(5, 5, 7, 1, 1, 7, 3, 3, 7),
            },
            new [] {
                 //p2
                new Triangle(1, 1, 1, 3, 3, 1, 2, 2, 1),
                new Triangle(1, 1, 6, 2, 2, 6, 7, 7, 6),
                new Triangle(2, 2, 3, 3, 3, 3, 7, 7, 3),
                new Triangle(3, 3, 9, 1, 1, 9, 7, 7, 9),
            },
            new [] {
                 //p3
                new Triangle(1, 1, 6, 7, 7, 6, 8, 8, 6),
                new Triangle(1, 1, 4, 8, 8, 4, 5, 5, 4),
                new Triangle(5, 5, 5, 8, 8, 5, 7, 7, 5),
                new Triangle(5, 5, 8, 7, 7, 8, 1, 1, 8),
            },
            new [] {
                // p4
                new Triangle(3, 3, 3, 6, 6, 3, 7, 7, 3),
                new Triangle(3, 3, 2, 5, 5, 2, 6, 6, 2),
                new Triangle(6, 6, 5, 5, 5, 5, 7, 7, 5),
                new Triangle(5, 5, 10, 3, 3, 10, 7, 7, 10),
            },
            new [] {
                // p5
                new Triangle(5, 5, 11, 3, 3, 11, 1, 1, 11),
                new Triangle(5, 5, 12, 1, 1, 12, 7, 7, 12),
                new Triangle(3, 3, 13, 7, 7, 13, 1, 1, 13),
                new Triangle(5, 5, 14, 7, 7, 14, 3, 3, 14)
            }
        };

        #endregion
    }
}

