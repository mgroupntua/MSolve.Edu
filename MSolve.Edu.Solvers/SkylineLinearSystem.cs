using MSolve.Edu.LinearAlgebra;

namespace MSolve.Edu.Solvers
{
    /// <summary>
    /// This class represents a linear system as a collection of a skyline matrix and a right-hand side vector
    /// </summary>
	public class SkylineLinearSystem
	{
        private readonly double[] rhs;
		private SkylineMatrix2D stiffnessMatrix;

        /// <summary>
        /// Initializes a linear system of skyline format with a specific right-hand side
        /// </summary>
        /// <param name="rhs">The right-hand side of the linear system</param>
        public SkylineLinearSystem(double[] rhs)
		{
			this.rhs = rhs;
			Solution = new Vector(rhs.Length);
		}

        /// <summary>
        /// The right-hand side of the linear system
        /// </summary>
		public Vector RHS { get => new Vector(rhs); }

        /// <summary>
        /// The solution of the linear system, updated by a SkylineSolver
        /// </summary>
        public Vector Solution { get; set; }

        /// <summary>
        /// The matrix of the linear system in skyline format
        /// </summary>
        public SkylineMatrix2D Matrix
        {
            get => stiffnessMatrix;
            set => stiffnessMatrix = (SkylineMatrix2D)value;
        }
    }
}
