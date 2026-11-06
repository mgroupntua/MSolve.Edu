namespace MSolve.Edu.Solvers;

using MSolve.Edu.LinearAlgebra;

/// <summary>
///     This class represents a linear system as a collection of a skyline matrix and a right-hand side vector
/// </summary>
/// <remarks>
///     Initializes a linear system of skyline format with a specific right-hand side
/// </remarks>
/// <param name="rhs">The right-hand side of the linear system</param>
public class SkylineLinearSystem(double[] rhs)
{
	private readonly double[] rhs = rhs;

	/// <summary>
	///     The right-hand side of the linear system
	/// </summary>
	public Vector RHS => new(rhs);

	/// <summary>
	///     The solution of the linear system, updated by a SkylineSolver
	/// </summary>
	public Vector Solution { get; set; } = new Vector(rhs.Length);

	/// <summary>
	///     The matrix of the linear system in skyline format
	/// </summary>
	public SkylineMatrix2D Matrix { get; set; }
}
