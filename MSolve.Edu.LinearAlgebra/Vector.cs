using System;

namespace MSolve.Edu.LinearAlgebra
{
    /// <summary>
    /// This class represents one-dimensional vectors and the linear algebra operations that can be applied on them
    /// </summary>
	public class Vector
	{
        /// <summary>
        /// Initializes a vector that has a specific length
        /// </summary>
        /// <param name="length">Length of the vector</param>
        public Vector(int length) => Data = new double[length];

        /// <summary>
        /// Initializes a vector that has specific data
        /// </summary>
        /// <param name="data">The double array that this vector will reference</param>
        public Vector(double[] data) => this.Data = data;

        /// <summary>
        /// The double array that the vector data is stored at
        /// </summary>
        public double[] Data { get; }

        /// <summary>
        /// The length of the array
        /// </summary>
        public int Length { get => Data.Length; }

        public double this[int x]
        {
            get => Data[x];
            set => Data[x] = value;
        }

        /// <summary>
        /// Second order norm of this vector
        /// </summary>
        public double Norm
		{
			get
			{
				double norm = 0;
				double[] v1Data = Data as double[];
				for (int i = 0; i < this.Length; i++)
					norm += v1Data[i] * v1Data[i];
				return Math.Sqrt(norm);
			}
		}

        /// <summary>
        /// Scales vector v with the scalar and generates a new vector with the result of the scaling
        /// </summary>
        /// <param name="scalar">The scalar</param>
        /// <param name="v">The vector to be scaled</param>
        /// <returns>A new vector with the result of the scaling</returns>
		public static Vector operator *(double scalar, Vector v)
		{
			double[] v2Data = v.Data as double[];
			double[] result = new double[v.Length];
			for (int i = 0; i < v.Length; i++) result[i] = scalar * v2Data[i];
			return (new Vector(result)) as Vector;
		}

        /// <summary>
        /// Calculates the dot product of vectors v1 and v2 and returns the result
        /// </summary>
        /// <param name="v1">Left side of the operand</param>
        /// <param name="v2">Right side of the operand</param>
        /// <returns>A double with the result of the dot product</returns>
		public static double operator *(Vector v1, Vector v2)
		{
			double result = 0;
			double[] v1Data = v1.Data;
			double[] v2Data = v2.Data;
			for (int i = 0; i < v1.Length; i++) result += v1Data[i] * v2Data[i];
			return result;
		}

        /// <summary>
        /// Adds vector v1 vector v2 and generates a new vector with the result of the addition
        /// </summary>
        /// <param name="v1">Left side of the operand</param>
        /// <param name="v2">Right side of the operand</param>
        /// <returns>A double array with the result of the addition</returns>
		public static double[] operator +(Vector v1, Vector v2)
		{
			double[] result = new double[v1.Length];
			double[] v1Data = v1.Data;
			double[] v2Data = v2.Data;
			for (int i = 0; i < v1.Length; i++) result[i] = v1Data[i] + v2Data[i];
			return result;
		}

        /// <summary>
        /// Subtracts vector v2 from vector v1 and generates a new vector with the result of the subtraction
        /// </summary>
        /// <param name="v1">Left side of the operand</param>
        /// <param name="v2">Right side of the operand</param>
        /// <returns>A double array with the result of the subtraction</returns>
		public static double[] operator -(Vector v1, Vector v2)
		{
			double[] result = new double[v1.Length];
			double[] v1Data = v1.Data;
			double[] v2Data = v2.Data;
			for (int i = 0; i < v1.Length; i++) result[i] = v1Data[i] - v2Data[i];
			return result;
		}

        /// <summary>
        /// Changes the data of this instance by multiplying them with a scalar
        /// </summary>
        /// <param name="scale">The number that all of the matrix data will be multiplied with</param>
		public void Scale(double scale)
		{
			double[] vData = Data;
			for (int i = 0; i < Data.Length; i++) vData[i] *= scale;
		}

        /// <summary>
        /// Changes the data of this instance by adding another vector to it
        /// </summary>
        /// <param name="v">The vector that will be added to the current matrix</param>
		public void Add(Vector v)
		{
			double[] v1Data = Data;
			double[] v2Data = v.Data;
			for (int i = 0; i < Data.Length; i++) v1Data[i] += v2Data[i];
		}

        /// <summary>
        /// Changes the data of this instance by subtracting another vector from it
        /// </summary>
        /// <param name="v">The vector that will be subracted from the current matrix</param>
		public void Subtract(Vector v)
		{
			double[] v1Data = Data;
			double[] v2Data = v.Data;
			for (int i = 0; i < Data.Length; i++) v1Data[i] -= v2Data[i];
		}

        /// <summary>
        /// Calculates the cross product of vectors v1 and v2 and returns the result
        /// </summary>
        /// <param name="v1">Left side of the operand</param>
        /// <param name="v2">Right side of the operand</param>
        /// <returns>A vector with the result of the cross product</returns>
		public static Vector operator ^(Vector v1, Vector v2)
		{
			if (v1.Length != 3 || v2.Length != 3) throw new InvalidOperationException("Only 3D cross product is supported.");

			double[] v1Data = v1.Data as double[];
			double[] v2Data = v2.Data as double[];
			double[] result = new[] { v1Data[1] * v2Data[2] - v1Data[2] * v2Data[1], v1Data[2] * v2Data[0] - v1Data[0] * v2Data[2], v1Data[0] * v2Data[1] - v1Data[1] * v2Data[0] };
			return new Vector(result);
		}

        /// <summary>
        /// Calculates the dot product of this vector with another vector
        /// </summary>
        /// <param name="y">The vector that will be combined with the current vector for the calculation of the dot product</param>
        /// <returns>The result of the dot product</returns>
		public double DotProduct(Vector y)
		{
			double result = 0;
			for (int i = 0; i < Data.Length; i++)
				result += Data[i] * y[i];
			return result;
		}

        /// <summary>
        /// Clears the data
        /// </summary>
        public void Clear() => Array.Clear(Data, 0, Data.Length);

        /// <summary>
        /// Copies ALL the data from this vector to an array starting from index
        /// </summary>
        /// <param name="array">The array that the vector data will be copied to</param>
        /// <param name="index">Index of the array that the data will start being copied at</param>
        public void CopyTo(Array array, int index) => Data.CopyTo(array, index);

        /// <summary>
        /// Copies data from the fromVector, starting from fromIndex with length size to this vector, starting from startIndex
        /// </summary>
        /// <param name="startIndex">Index of this vector that the data will start getting copied at</param>
        /// <param name="length">Length of data to be copied</param>
        /// <param name="fromVector">The vector that the data will be copied from</param>
        /// <param name="fromStartIndex">The index of the vector to be copied that copying will start from</param>
		public void CopyFrom(int startIndex, int length, Vector fromVector, int fromStartIndex)
		{
			for (int i = 0; i < length; i++)
				Data[i + startIndex] = fromVector[i + fromStartIndex];
		}
	}
}
