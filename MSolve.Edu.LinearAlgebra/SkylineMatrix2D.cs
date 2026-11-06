namespace MSolve.Edu.LinearAlgebra;

using System;
using System.Collections.Generic;

/// <summary>
///     This class represents two-dimensional symmetric sparse matrices which are stored in a skyline format and the linear
///     algebra operations that can be applied on them
/// </summary>
public class SkylineMatrix2D
{
	/// <summary>
	///     Initializes a matrix of skyline format with a specific skyline structure
	/// </summary>
	/// <param name="rowIndex">The skyline structure containing the position of the diagonal elements</param>
	public SkylineMatrix2D(int[] rowIndex)
	{
		RowIndex = rowIndex;
		Data = rowIndex.Length > 0 ? new double[rowIndex[^1]] : [];
	}

	/// <summary>
	///     Generates a skyline equivalent of a double matrix WITHOUT scanning for the skyline
	/// </summary>
	/// <param name="matrix">The matrix from which the skyline matrix will be generated from</param>
	public SkylineMatrix2D(double[,] matrix)
	{
		var rows = matrix.GetLength(0);
		if (matrix.GetLength(1) != rows) throw new ArgumentException("Matrix must be square.");

		RowIndex = new int[rows + 1];
		for (var i = 0; i < rows; i++) RowIndex[i + 1] = RowIndex[i] + i + 1;
		Data = new double[RowIndex[^1]];

		var pos = 0;
		for (var j = 0; j < rows; j++)
			for (var i = j; i >= 0; i--)
			{
				Data[pos] = matrix[i, j];
				pos++;
			}
	}

	/// <summary>
	///     Index of diagonal elements inside the data double array
	/// </summary>
	public int[] RowIndex { get; }

	/// <summary>
	///     Data of the matrix
	/// </summary>
	public double[] Data { get; }

	/// <summary>
	///     Is true if matrix has been factorized and is ready for forward and backward substitutions for solving linear
	///     systems
	/// </summary>
	public bool IsFactorized { get; private set; }

	/// <summary>
	///     The number of rows of this matrix
	/// </summary>
	public int Rows => RowIndex.Length - 1;

	/// <summary>
	///     The number of columns of this matrix
	/// </summary>
	public int Columns => RowIndex.Length - 1;

	public double this[int x, int y]
	{
		get
		{
			var r = x;
			var c = y;
			if (x < y)
			{
				r = y;
				c = x;
			}

			var row1Start = RowIndex[r];
			var row1Stop = RowIndex[r + 1];
			var cols1 = row1Stop - row1Start;
			var minCol1 = r - cols1 + 1;

			if (c >= minCol1)
			{
				var offset = r - c;
				var pos = row1Start + offset;
				return Data[pos];
			}

			return 0;
		}
		set
		{
			var r = x;
			var c = y;
			if (x < y)
			{
				r = y;
				c = x;
			}

			var row1Start = RowIndex[r];
			var row1Stop = RowIndex[r + 1];
			var cols1 = row1Stop - row1Start;
			var minCol1 = r - cols1 + 1;

			if (c >= minCol1)
			{
				var offset = r - c;
				var pos = row1Start + offset;
				Data[pos] = value;
			}
			else
			{
				throw new ArgumentException("Specified position is not indexed in the skyline format.");
			}
		}
	}

	/// <summary>
	///     Generates a new matrix instance with a specific number of rows with empty data
	/// </summary>
	/// <param name="rows">The number of rows (and columns) of the skyline matrix</param>
	/// <returns>A new matrix instance of a specific size with empty data</returns>
	public static SkylineMatrix2D Empty(int rows) => new(new int[rows + 1]);

	/// <summary>
	///     Performs an in-place LDL factorization
	/// </summary>
	/// <param name="tolerance">A number which below that is considered to be zero</param>
	/// <param name="nullSpace">A vector list where any null space vectors found during factorization will be placed</param>
	/// <param name="nullSpaceCols">A number list where the columns of the null space are found</param>
	public void Factorize(double tolerance, List<Vector> nullSpace, List<int> nullSpaceCols)
	{
		if (IsFactorized)
			throw new InvalidOperationException("Matrix is already factorized.");
		var d = Data;

		var kFix = 0;
		for (var n = 0; n < Rows; n++)
		{
			var KN = RowIndex[n];
			var KL = KN + 1;
			var KU = RowIndex[n + 1] - 1;
			var KH = KU - KL;
			if (KH < 0) continue;

			int K;
			if (KH > 0)
			{
				K = n - KH;
				//int IC = 0;
				var KLT = KU;
				for (var j = 0; j < KH; j++)
				{
					//IC++;
					KLT--;
					var KI = RowIndex[K];
					var ND = RowIndex[K + 1] - KI - 1;
					if (ND > 0)
					{
						var KK = Math.Min(j + 1, ND);
						double C = 0;
						for (var l = 1; l <= KK; l++)
							C += d[KI + l] * d[KLT + l];
						d[KLT] -= C;
					}

					K++;
				}
			}

			K = n;
			double B = 0;
			for (var KK = KL; KK <= KU; KK++)
			{
				K--;
				var KI = RowIndex[K];
				if (Math.Abs(d[KI]) < tolerance)
					throw new InvalidOperationException(
						string.Format("Near-zero element in diagonal at index {0}.", KI));
				var C = d[KK] / d[KI];
				B += C * d[KK];
				d[KK] = C;
			}

			d[KN] -= B;

			if (Math.Abs(d[KN]) < tolerance)
			{
				d[KN] = 1;
				nullSpaceCols.Add(n);
				var j1 = n;
				nullSpace.Add(new Vector(Rows));
				nullSpace[kFix][j1] = 1;
				for (var i1 = KN + 1; i1 <= KU; i1++)
				{
					j1--;
					nullSpace[kFix][j1] = -d[i1];
					d[i1] = 0;
				}

				for (var irest = n + 1; irest < Rows; irest++)
				{
					var m1 = RowIndex[irest] + irest - n;
					if (m1 <= RowIndex[irest + 1]) d[m1] = 0;
				}

				kFix++;
			}
		}

		IsFactorized = true;
		if (Rows < 2) return;

		for (var ifl = 0; ifl < kFix; ifl++)
		{
			var n = Rows - 1;
			for (var l = 1; l < Rows; l++)
			{
				var KL = RowIndex[n] + 1;
				var KU = RowIndex[n + 1] - 1;
				if (KU - KL >= 0)
				{
					var k = n;
					for (var KK = KL; KK <= KU; KK++)
					{
						k--;
						nullSpace[ifl][k] -= d[KK] * nullSpace[ifl][n];
					}
				}

				n--;
			}
		}
	}

	/// <summary>
	///     Performs forward and backward substitution for the solution of a linear system with rhs as its right-hand side. The
	///     Factorize method needs to be called prior to using this method
	/// </summary>
	/// <param name="rhs">The right-hand side of the linear system</param>
	/// <param name="result">The solution of the linear system</param>
	public void Solve(Vector rhs, Vector result)
	{
		var K = this;
		if (!K.IsFactorized) throw new InvalidOperationException("Cannot solve if matrix is not factorized.");
		if (K.Rows != rhs.Length) throw new ArgumentException("Matrix and vector size mismatch.");
		var d = K.Data;
		result.CopyFrom(0, rhs.Length, rhs, 0);
		int n;
		for (n = 0; n < K.Rows; n++)
		{
			var KL = K.RowIndex[n] + 1;
			var KU = K.RowIndex[n + 1] - 1;
			if (KU >= KL)
			{
				var k = n;
				double C = 0;
				for (var KK = KL; KK <= KU; KK++)
				{
					k--;
					C += d[KK] * result[k];
				}

				result[n] -= C;
			}
		}

		for (n = 0; n < K.Rows; n++) result[n] /= d[K.RowIndex[n]];

		n = K.Rows - 1;
		for (var l = 1; l < K.Rows; l++)
		{
			var KL = K.RowIndex[n] + 1;
			var KU = K.RowIndex[n + 1] - 1;
			if (KU >= KL)
			{
				var k = n;
				for (var KK = KL; KK <= KU; KK++)
				{
					k--;
					result[k] -= d[KK] * result[n];
				}
			}

			n--;
		}
	}

	/// <summary>
	///     Multiplies this matrix with vIn, starting from vInStartIndex, scaled by scaleFactor and stores it to vOut, starting
	///     from vOutStartIndex and clear vOut if clearvOut flag is true
	/// </summary>
	/// <param name="vIn">The vector that this matrix will be multiplied with</param>
	/// <param name="vOut">The vector that the result will be stored at</param>
	/// <param name="scaleFactor">The number that all data of vIn will be multiplied with before multiplying with the matrix</param>
	/// <param name="vInStartIndex">Starting index of the vIn from which multiplication will start</param>
	/// <param name="vOutStartIndex">Starting index of the vOut where storage of the result will start</param>
	/// <param name="clearvOut">Set to true if before storing data, vOut will be cleared</param>
	public void Multiply(Vector vIn, double[] vOut, double scaleFactor, int vInStartIndex, int vOutStartIndex,
		bool clearvOut)
	{
		if (clearvOut) Array.Clear(vOut, 0, vOut.Length);
		var d = Data;
		var pos = 0;
		for (var i = 0; i < Rows; i++)
		{
			var height = RowIndex[i + 1] - RowIndex[i];
			if (height <= 0) continue;

			vOut[i + vOutStartIndex] += scaleFactor * d[pos] * vIn[i + vInStartIndex];
			pos++;
			for (var j = 0; j < height - 1; j++)
			{
				var row = i - j - 1;
				vOut[row + vOutStartIndex] += scaleFactor * d[pos] * vIn[i + vInStartIndex];
				vOut[i + vOutStartIndex] += scaleFactor * d[pos] * vIn[row + vInStartIndex];
				pos++;
			}
		}
	}

	/// <summary>
	///     Multiplies this matrix with vIn, starting from vInStartIndex, scaled by scaleFactor and stores it to vOut, starting
	///     from vOutStartIndex and clear vOut if clearvOut flag is true
	/// </summary>
	/// <param name="vIn">The vector that this matrix will be multiplied with</param>
	/// <param name="vOut">The vector that the result will be stored at</param>
	public void Multiply(Vector vIn, double[] vOut) => Multiply(vIn, vOut, 1.0, 0, 0, true);

	/// <summary>
	///     Scales all of the matrix data with the number scale
	/// </summary>
	/// <param name="scaleFactor">The number that all data of the matrix will be multiplied with</param>
	public void Scale(double scaleFactor)
	{
		var mData = Data;
		for (var i = 0; i < mData.Length; i++) mData[i] *= scaleFactor;
	}

	/// <summary>
	///     Performs a matrix-vector multiplication of matrix A and vector B and generates a new vector of the result
	/// </summary>
	/// <param name="A">The matrix side</param>
	/// <param name="b">The vector side</param>
	/// <returns>A new vector with the result of the multiplication</returns>
	public static double[] operator *(SkylineMatrix2D A, Vector b)
	{
		if (A.IsFactorized) throw new InvalidOperationException("Cannot multiply if matrix is factorized.");
		if (A.Rows != b.Length) throw new ArgumentException("Matrix and vector size mismatch.");
		var result = new double[A.Rows];
		A.Multiply(b, result);
		return result;
	}

	private void LinearCombinationInternal(IList<double> coefficients, IList<SkylineMatrix2D> matrices)
	{
		foreach (var matrix in matrices)
			if (matrix.Rows != Rows)
				throw new ArgumentException("Matrices do not have the same size.");

		for (var i = 0; i < RowIndex.Length - 1; i++)
		{
			var currentMaxHeight = 0;
			foreach (var matrix in matrices)
				currentMaxHeight = Math.Max(currentMaxHeight, matrix.RowIndex[i + 1] - matrix.RowIndex[i]);
			if (currentMaxHeight > RowIndex[i + 1] - RowIndex[i])
				throw new InvalidOperationException(
					"Current matrix does not have enough storage capacity for the requested linear combination.");
		}

		double temp;
		var d = Data;
		for (var i = 0; i < RowIndex.Length - 1; i++)
			for (var j = RowIndex[i]; j < RowIndex[i + 1]; j++)
			{
				temp = 0;
				for (var k = 0; k < coefficients.Count; k++)
				{
					var pos = j - RowIndex[i];
					if (pos < matrices[k].RowIndex[i + 1] - matrices[k].RowIndex[i])
						temp += coefficients[k] * matrices[k].Data[matrices[k].RowIndex[i] + pos];
				}

				d[j] = temp;
			}
	}

	/// <summary>
	///     Changes the data of this instance by adding a linear combination of matrices
	/// </summary>
	/// <param name="coefficients">The list with scalar coefficients of the linear combination to be performed</param>
	/// <param name="matrices">The list of matrices of the linear combination to be performed</param>
	public void LinearCombination(IList<double> coefficients, IList<SkylineMatrix2D> matrices)
	{
		IsFactorized = false;
		var m = new List<SkylineMatrix2D>(matrices.Count);
		foreach (var matrix in matrices) m.Add(matrix);
		LinearCombinationInternal(coefficients, m);
	}
}
