using MSolve.Edu.FEM.Entities;
using MSolve.Edu.LinearAlgebra;
using MSolve.Edu.Solvers;

namespace MSolve.Edu.FEM
{
	/// <summary>
	/// Responsible for the assembly of the global stiffness matrix.
	/// </summary>
	public class ProblemStructural
	{
		private SkylineMatrix2D ks;
		private readonly Model model;
		private readonly ElementStructuralStiffnessProvider stiffnessProvider = new ElementStructuralStiffnessProvider();

		/// <summary>
		/// Initializes a new instance of the <see cref="ProblemStructural"/> class.
		/// </summary>
		/// <param name="model">The model.</param>
		public ProblemStructural(Model model) => this.model = model;

		private SkylineMatrix2D Ks
		{
			get
			{
				if (ks == null)
					BuildKs();
				else
					RebuildKs();
				return ks;
			}
		}

		/// <summary>
		/// Builds the global stiffness matrix.
		/// </summary>
		public void CalculateMatrix(SkylineLinearSystem linearSystem)
        {
            if (ks == null)
                BuildKs();
            linearSystem.Matrix = ks;
        }

        /// <summary>
        /// Builds the global stiffness matrix.
        /// </summary>
		private void BuildKs()
		{
			ElementStructuralStiffnessProvider s = new ElementStructuralStiffnessProvider();
			ks = GlobalMatrixAssemblerSkyline.CalculateGlobalMatrix(model, s);
		}

		/// <summary>
		/// Rebuilds the global stiffness matrix.
		/// </summary>
		private void RebuildKs() => ks = GlobalMatrixAssemblerSkyline.CalculateGlobalMatrix(model, stiffnessProvider);
	}
}