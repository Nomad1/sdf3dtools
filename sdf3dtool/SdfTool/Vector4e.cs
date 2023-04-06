#if UNITY_EDITOR
using System;

namespace RunServer.SdfTool
{
    public struct Vector4e
    {
        public float X;
        public float Y;
        public float Z;
        public float W;

        public Vector4e(float x, float y, float z, float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        public Vector4e(UnityEngine.Vector4 vector)
        {
            X = vector.x;
            Y = vector.y;
            Z = vector.z;
            W = vector.w;
        }
    }
}
#endif