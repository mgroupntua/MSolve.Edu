using MSolve.Edu.Solvers;

namespace MSolve.Edu.Analyzers
{
    /// <summary>
    /// This class makes the appropriate arrangements for the solution of linear systems of equations
    /// </summary>
	public class LinearAnalyzer
	{
        /// <summary>
        /// The solver instance that will solve the linear system of equations at hand
        /// </summary>
        public SolverSkyline Solver { get; }

        /// <summary>
        /// The parent analyzer that transforms the physical problem to a system of equations
        /// </summary>
        public StaticAnalyzer ParentAnalyzer { get; set; } = null;

        /// <summary>
        /// Creates an instance that uses a specific solver for the solution of a linear system of equations
        /// </summary>
        /// <param name="solver">Instance of the solver that will solve the linear system of equations</param>
        public LinearAnalyzer(SolverSkyline solver) => this.Solver = solver;

        /// <summary>
        /// Makes the proper solver-specific initializations before the solution of the linear system of equations. This method MUST be called before the actual solution of the aforementioned system 
        /// </summary>
        public void Initialize() => Solver.Initialize();

        /// <summary>
        /// Solves the linear system of equations by calling the corresponding method of the specific solver attached during construction of the current instance
        /// </summary>
        public void Solve() => Solver.Solve();
	}
}
