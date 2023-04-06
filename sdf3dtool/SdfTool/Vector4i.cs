using System;

namespace RunServer.SdfTool
{
    public struct Vector4i : IComparable<Vector4i>
    {
        public readonly int X, Y, Z, W;

        public int LengthSqrd
        {
            get { return X * X + Y * Y + Z * Z + W * W; }
        } 

        public int this[int i]
        {
            get
            {
                switch (i)
                {
                    case 0:
                        return X;
                    case 1:
                        return Y;
                    case 2:
                        return Z;
                    case 3:
                        return W;
                    default:
                        throw new IndexOutOfRangeException();
                }
            }
        }

        public Vector4i(int x, int y, int z, int w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        public override string ToString()
        {
            return string.Format("[{0}, {1}, {2}, {3}]", X, Y, Z, W);
        }

        public int CompareTo(Vector4i other)
        {
            if (X == other.X && Y == other.Y && Z == other.Z && W == other.W)
                return 0;

            return LengthSqrd > other.LengthSqrd ? 1 : -1; // no equality!
            //return this.ToString().CompareTo(other.ToString());
        }
    }
}