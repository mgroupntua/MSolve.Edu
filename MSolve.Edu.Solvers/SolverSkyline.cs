namespace MSolve.Edu.Solvers;

using System;
using System.Collections.Generic;

using MSolve.Edu.LinearAlgebra;

/// <summary>
///     This class manipulates a skyline linear system for its solution
/// </summary>
/// <remarks>
///     Initializes a skyline solver that will solve a specific linear system
/// </remarks>
/// <param name="linearSystem">The linear system instance to be solved</param>
public class SolverSkyline(SkylineLinearSystem linearSystem)
{
	private readonly SkylineLinearSystem linearSystem = linearSystem;

	/// <summary>
	///     Solves the linear system of this solver
	/// </summary>
	public void Solve() => linearSystem.Matrix.Solve(linearSystem.RHS, linearSystem.Solution);

	/// <summary>
	///     Makes the proper initialization before the solution of the linear system of equations. This method MUST be called
	///     before the actual solution of the aforementioned system
	/// </summary>
	public void Initialize()
	{
		if (linearSystem.Matrix.IsFactorized) return;

		var zems = new List<Vector>();
		var zemColumns = new List<int>();
		var m = linearSystem.Matrix;

		linearSystem.Matrix.Factorize(1e-5, zems, zemColumns);
		if (zemColumns.Count > 0)
			throw new InvalidOperationException("Skyline solver does not operate on singular matrices.");
	}
}
