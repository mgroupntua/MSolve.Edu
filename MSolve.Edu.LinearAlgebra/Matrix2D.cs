namespace MSolve.Edu.LinearAlgebra;

using System;
using System.Collections.Generic;

/// <summary>
///     This class represents two-dimensional matrices and the linear algebra operations that can be applied on them
/// </summary>
public class Matrix2D
{
	private readonly int rows, columns;
	private bool isTransposed;

	/// <summary>
	///     Initializes an empty matrix with a predefined number of rows and columns
	/// </summary>
	/// <param name="rows">The number of rows of this matrix</param>
	/// <param name="columns">The number of columns of this matrix</param>
	public Matrix2D(int rows, int columns)
	{
		this.rows = rows;
		this.columns = columns;
		Data = new double[rows, columns];
	}

	/// <summary>
	///     Initializes a matrix and assigns the data reference to its internal structure
	/// </summary>
	/// <param name="data">The data that this matrix has reference to</param>
	public Matrix2D(double[,] data)
	{
		rows = data.GetLength(0);
		columns = data.GetLength(1);
		Data = data;
	}

	/// <summary>
	///     Matrix data as it is stored internally
	/// </summary>
	public double[,] Data { get; }

	/// <summary>
	///     Number of rows of this matrix
	/// </summary>
	public int Rows => isTransposed ? columns : rows;

	/// <summary>
	///     Number of columns of this matrix
	/// </summary>
	public int Columns => isTransposed ? rows : columns;

	public double this[int x, int y]
	{
		get => isTransposed ? Data[y, x] : Data[x, y];
		set
		{
			if (isTransposed)
				Data[y, x] = value;
			else
				Data[x, y] = value;
		}
	}

	/// <summary>
	///     Fills the matrix with zeros
	/// </summary>
	public void Clear() => Array.Clear(Data, 0, Data.Length);

	/// <summary>
	///     Generates a new matrix instance with the same data, having juxtaposed the rows with the columns
	/// </summary>
	/// <returns>A new matrix instance referencing the same data matrix</returns>
	public Matrix2D Transpose() => new(Data) { isTransposed = true };

	/// <summary>
	///     Changes the data of this matrix instance by making the addition of another matrix multiplied by a scalar (Axpy)
	/// </summary>
	/// <param name="matrix">The matrix that will be added to the current matrix</param>
	/// <param name="scalar">
	///     The scalar that all the values of the previous parameter will be multiplied before being added to
	///     the current matrix
	/// </param>
	public void AxpyIntoThis(Matrix2D matrix, double scalar)
	{
		if (Rows != matrix.Rows || Columns != matrix.Columns)
			throw new ArgumentException("The two matrices must have the same dimensions.");
		for (var i = 0; i < Rows; ++i)
			for (var j = 0; j < Columns; ++j)
				this[i, j] += scalar * matrix[i, j];
	}

	/// <summary>
	///     Multiplies this matrix with a vector and stores the result to a double array
	/// </summary>
	/// <param name="vIn">The vector that this matrix will be multiplied with</param>
	/// <param name="vOut">The double array that will store the result of the matrix-vector multiplication</param>
	public void Multiply(Vector vIn, double[] vOut)
	{
		if (!isTransposed)
			MultiplyNormal(vIn, vOut);
		else
			MultiplyTranspose(vIn, vOut);
	}

	/// <summary>
	///     Multiplies this matrix with a double array and stores the result to another double array
	/// </summary>
	/// <param name="vIn">The double array that this matrix will be multiplied with</param>
	/// <param name="vOut">The double array that will store the result of the matrix-vector multiplication</param>
	public void Multiply(double[] vIn, double[] vOut)
	{
		var AA = new Matrix2D(Data);
		for (var i = 0; i < rows; i++)
		{
			vOut[i] = 0;
			for (var j = 0; j < columns; j++)
				vOut[i] += AA.Data[i, j] * vIn[j];
		}
	}

	private void MultiplyNormal(Vector vIn, double[] vOut)
	{
		var AA = new Matrix2D(Data);
		for (var i = 0; i < rows; i++)
		{
			vOut[i] = 0;
			for (var j = 0; j < columns; j++)
				vOut[i] += AA.Data[i, j] * vIn[j];
		}
	}

	private void MultiplyTranspose(Vector vIn, double[] vOut)
	{
		var AA = new Matrix2D(Data);
		for (var j = 0; j < columns; j++)
		{
			vOut[j] = 0;
			for (var i = 0; i < rows; i++)
				vOut[j] += AA.Data[i, j] * vIn[i];
		}
	}

	/// <summary>
	///     Changes the data of this instance by multiplying them with a scalar
	/// </summary>
	/// <param name="scale">The number that all of the matrix data will be multiplied with</param>
	public void Scale(double scale)
	{
		var mData = Data;
		for (var i = 0; i < mData.GetLength(0); i++)
			for (var j = 0; j < mData.GetLength(1); j++)
				mData[i, j] *= scale;
	}

	/// <summary>
	///     Performs a matrix-matrix multiplication of matrices A and B and generates a new matrix of the result
	/// </summary>
	/// <param name="A">Left side of the operand</param>
	/// <param name="B">Right side of the operand</param>
	/// <returns>A new matrix instance with the result of the multiplication</returns>
	public static Matrix2D operator *(Matrix2D A, Matrix2D B)
	{
		if (A.Columns != B.Rows) throw new ArgumentException("Matrix sizes mismatch.");

		var c = new double[A.Rows, B.Columns];
		var AA = new Matrix2D(A.Data)
		{
			isTransposed = A.isTransposed
		};

		for (var i = 0; i < A.Rows; i++)
			for (var k = 0; k < B.Columns; k++)
				for (var j = 0; j < B.Rows; j++)
					c[i, k] += AA[i, j] * B[j, k];
		return new Matrix2D(c);
	}

	/// <summary>
	///     Performs a matrix-vector multiplication of matrix A and vector B and generates a new vector of the result
	/// </summary>
	/// <param name="A">The matrix side</param>
	/// <param name="b">The vector side</param>
	/// <returns>A new vector with the result of the multiplication</returns>
	public static Vector operator *(Matrix2D A, Vector b)
	{
		if (A.Columns != b.Length) throw new ArgumentException("Matrix sizes mismatch.");

		var c = new double[A.Rows];
		var AA = new Matrix2D(A.Data)
		{
			isTransposed = A.isTransposed
		};

		for (var i = 0; i < A.Rows; i++)
			for (var j = 0; j < A.Columns; j++)
				c[i] += AA[i, j] * b[j];
		return new Vector(c);
	}

	/// <summary>
	///     Scales matrix A with the scalar and generates a new matrix with the result of the scaling
	/// </summary>
	/// <param name="A">The matrix to be scaled</param>
	/// <param name="scalar">The scalar</param>
	/// <returns>A new matrix with the result of the scaling</returns>
	public static Matrix2D operator *(Matrix2D A, double scalar)
	{
		var AA = new Matrix2D(A.Data);
		for (var i = 0; i < A.Rows; i++)
			for (var j = 0; j < A.Columns; j++)
				AA[i, j] = AA[i, j] * scalar;
		return AA;
	}

	/// <summary>
	///     Changes the data of this instance by adding another matrix to it
	/// </summary>
	/// <param name="B">The matrix that will be added to the current matrix</param>
	public void Add(Matrix2D B)
	{
		var mData = Data;
		if (mData.GetLength(0) != B.rows && mData.GetLength(1) != B.Columns)
			throw new ArgumentException("Matrix sizes mismatch.");
		for (var i = 0; i < mData.GetLength(0); i++)
			for (var j = 0; j < mData.GetLength(1); j++)
				mData[i, j] += B[i, j];
	}

	/// <summary>
	///     Changes the data of this instance by adding a linear combination of matrices
	/// </summary>
	/// <param name="coefficients">The list with scalar coefficients of the linear combination to be performed</param>
	/// <param name="matrices">The list of matrices of the linear combination to be performed</param>
	public void LinearCombination(List<double> coefficients, List<Matrix2D> matrices)
	{
		if (coefficients.Count != matrices.Count)
			throw new ArgumentException(string.Format("Coefficients and matrices count mismatch ({0} <> {1}).",
				coefficients.Count, matrices.Count));
		for (var i = 0; i < matrices.Count; i++)
			if (matrices[i].Rows != rows)
				throw new ArgumentException(string.Format("Matrix at pos {0} has {1} rows instead of {2}.", i,
					matrices[i].Rows, rows));

		var cs = (IList<double>)coefficients;
		var newData = new double[rows, columns];
		for (var k = 0; k < matrices.Count; k++)
		{
			var m = matrices[k];
			for (var i = 0; i < rows; i++)
				for (var j = 0; j < columns; j++)
					newData[i, j] += cs[k] * m[i, j];
		}

		Array.Copy(newData, Data, Data.Length);
	}
}
