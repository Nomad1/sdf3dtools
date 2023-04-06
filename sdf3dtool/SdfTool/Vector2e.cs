#if UNITY_EDITOR
using System;

namespace RunServer.SdfTool
{
    public struct Vector2e
    {
        public float X;
        public float Y;

        public Vector2e(float x, float y)
        {
            X = x;
            Y = y;
        }

        public Vector2e(UnityEngine.Vector2 vector)
        {
            X = vector.x;
            Y = vector.y;
        }
    }
}
#endif