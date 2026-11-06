namespace MSolve.Edu.FEM;

using MSolve.Edu.FEM.Entities;
using MSolve.Edu.LinearAlgebra;
using MSolve.Edu.Solvers;

/// <summary>
///     Responsible for the assembly of the global stiffness matrix.
/// </summary>
/// <remarks>
///     Initializes a new instance of the <see cref="ProblemStructural" /> class.
/// </remarks>
/// <param name="model">The model.</param>
public class ProblemStructural(Model model)
{
	private readonly Model model = model;
	private readonly ElementStructuralStiffnessProvider stiffnessProvider = new();
	private SkylineMatrix2D ks;

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
	///     Builds the global stiffness matrix.
	/// </summary>
	public void CalculateMatrix(SkylineLinearSystem linearSystem)
	{
		if (ks == null)
			BuildKs();
		linearSystem.Matrix = ks;
	}

	/// <summary>
	///     Builds the global stiffness matrix.
	/// </summary>
	private void BuildKs()
	{
		var s = new ElementStructuralStiffnessProvider();
		ks = GlobalMatrixAssemblerSkyline.CalculateGlobalMatrix(model, s);
	}

	/// <summary>
	///     Rebuilds the global stiffness matrix.
	/// </summary>
	private void RebuildKs() => ks = GlobalMatrixAssemblerSkyline.CalculateGlobalMatrix(model, stiffnessProvider);
}
