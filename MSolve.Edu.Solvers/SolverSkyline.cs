using MSolve.Edu.LinearAlgebra;
using System;
using System.Collections.Generic;

namespace MSolve.Edu.Solvers
{
    /// <summary>
    /// This class manipulates a skyline linear system for its solution
    /// </summary>
	public class SolverSkyline
	{
		private readonly SkylineLinearSystem linearSystem;

        /// <summary>
        /// Initializes a skyline solver that will solve a specific linear system
        /// </summary>
        /// <param name="linearSystem">The linear system instance to be solved</param>
		public SolverSkyline(SkylineLinearSystem linearSystem) => this.linearSystem = linearSystem;

        /// <summary>
        /// Solves the linear system of this solver
        /// </summary>
        public void Solve() => linearSystem.Matrix.Solve(linearSystem.RHS, linearSystem.Solution);

        /// <summary>
        /// Makes the proper initialization before the solution of the linear system of equations. This method MUST be called before the actual solution of the aforementioned system 
        /// </summary>
        public void Initialize()
		{
			if (linearSystem.Matrix.IsFactorized) return;

			List<Vector> zems = new List<Vector>();
			List<int> zemColumns = new List<int>();
			SkylineMatrix2D m = linearSystem.Matrix;

			linearSystem.Matrix.Factorize(1e-5, zems, zemColumns);
			if (zemColumns.Count > 0)
				throw new InvalidOperationException("Skyline solver does not operate on singular matrices.");
		}
	}
}