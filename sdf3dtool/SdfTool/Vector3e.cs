#if UNITY_EDITOR
using System;

namespace RunServer.SdfTool
{
    public struct Vector3e
    {
        public float X;
        public float Y;
        public float Z;

        public float Length
        {
            get { return MathF.Sqrt(X * X + Y * Y + Z * Z); }
        }

        public Vector3e(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public Vector3e(UnityEngine.Vector3 vector)
        {
            X = vector.x;
            Y = vector.y;
            Z = vector.z;
        }

        public static Vector3e operator +(Vector3e one, Vector3e another)
        {
            return new Vector3e(one.X + another.X, one.Y + another.Y, one.Z + another.Z);
        }

        public static Vector3e operator -(Vector3e one, Vector3e another)
        {
            return new Vector3e(one.X - another.X, one.Y - another.Y, one.Z - another.Z);
        }

        public static Vector3e operator *(Vector3e one, Vector3e another)
        {
            return new Vector3e(one.X * another.X, one.Y * another.Y, one.Z * another.Z);
        }

        public static Vector3e operator *(Vector3e one, float another)
        {
            return new Vector3e(one.X * another, one.Y * another, one.Z * another);
        }

        public static Vector3e operator /(Vector3e one, Vector3e another)
        {
            return new Vector3e(one.X / another.X, one.Y / another.Y, one.Z / another.Z);
        }

        public static float Distance(Vector3e one, Vector3e another)
        {
            return (one - another).Length;
        }

        public static implicit operator UnityEngine.Vector3(Vector3e one)
        {
            return new UnityEngine.Vector3(one.X, one.Y, one.Z);
        }

        public static implicit operator Vector3e(UnityEngine.Vector3 one)
        {
            return new Vector3e(one);
        }
    }
}
#endif