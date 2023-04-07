using System;

namespace RunServer.SdfTool
{
    public struct Vector3i : IComparable<Vector3i>
    {
        public readonly int X, Y, Z;
        public readonly int LengthSqrd;

        public Vector3i(int x, int y, int z)
        {
            X = x;
            Y = y;
            Z = z;
            LengthSqrd = X * X + Y * Y + Z * Z;
        }

        public override string ToString()
        {
            return string.Format("[{0}, {1}, {2}]", X, Y, Z);
        }

        public int CompareTo(Vector3i other)
        {
            if (X == other.X && Y == other.Y && Z == other.Z)
                return 0;

            return LengthSqrd > other.LengthSqrd ? 1 : -1; // no equality!
            //return this.ToString().CompareTo(other.ToString());
        }

        public static Vector3i operator +(Vector3i one, Vector3i another)
        {
            return new Vector3i(one.X + another.X, one.Y + another.Y, one.Z + another.Z);
        }

        public static Vector3i operator /(Vector3i one, int another)
        {
            return new Vector3i(one.X / another, one.Y / another, one.Z / another);
        }
    }
}