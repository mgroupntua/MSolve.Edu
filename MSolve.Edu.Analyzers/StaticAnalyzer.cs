using System;
using MSolve.Edu.FEM;
using MSolve.Edu.Solvers;

namespace MSolve.Edu.Analyzers
{
    /// <summary>
    /// This class constructs the system of equations to be solved and utilizes a child analyzer for handling the solution of these equations
    /// </summary>
	public class StaticAnalyzer
	{
		private readonly ProblemStructural provider;
        private readonly SkylineLinearSystem linearSystem;

        /// <summary>
        /// The analyzer that will solve the system of equations arising from the actual physical problem
        /// </summary>
        public LinearAnalyzer ChildAnalyzer { get; set; }

        /// <summary>
        /// Creates an instance that uses a specific problem type and an appropriate child analyzer for the construction of the system of equations arising from the actual physical problem
        /// </summary>
        /// <param name="provider">Instance of the problem type to be solver</param>
        /// <param name="childAnalyzer">Instance of the child analyzer that will handle the solution of the system of equations</param>
        /// <param name="linearSystem">Instance of the linear system that will be initialized</param>
        public StaticAnalyzer(ProblemStructural provider, LinearAnalyzer childAnalyzer, SkylineLinearSystem linearSystem)
		{
			this.provider = provider;
			this.ChildAnalyzer = childAnalyzer;
			this.ChildAnalyzer.ParentAnalyzer = this;
			this.linearSystem = linearSystem;
		}

        /// <summary>
        /// Builds the appropriate linear system matrix and updates the linear system instance used in the constructor
        /// </summary>
        public void BuildMatrices() => provider.CalculateMatrix(linearSystem);

        /// <summary>
        /// Makes the proper solver-specific initializations before the solution of the linear system of equations. This method MUST be called before the actual solution of the aforementioned system 
        /// </summary>
        public void Initialize()
		{
			if (ChildAnalyzer == null)
				throw new InvalidOperationException("Static analyzer must contain a linear analyzer.");
			ChildAnalyzer.Initialize();
		}

        /// <summary>
        /// Solves the linear system of equations by calling the corresponding method of the specific solver attached during construction of the current instance
        /// </summary>
		public void Solve()
		{
			if (ChildAnalyzer == null)
				throw new InvalidOperationException("Static analyzer must contain a linear analyzer.");
			ChildAnalyzer.Solve();
		}
	}
}