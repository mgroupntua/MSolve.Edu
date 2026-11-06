namespace MSolve.Edu.FEM.Output;

using System.IO;

using MSolve.Edu.FEM.Entities;
using MSolve.Edu.Solvers;

/// <summary>
///     Creates a TxtFile Writer that saves basic model results.
/// </summary>
/// <remarks>
///     Initializes a new instance of the <see cref="TxtWriter" /> class.
/// </remarks>
/// <param name="model">The model.</param>
/// <param name="filepath">The filepath.</param>
/// <param name="linearSystem">The linear system.</param>
public class TxtWriter(Model model, string filepath, SkylineLinearSystem linearSystem)
{
	private readonly string _filepath = filepath;
	private readonly SkylineLinearSystem _linearSystem = linearSystem;
	private readonly Model _model = model;

	/// <summary>
	///     Writes to file.
	/// </summary>
	/// <param name="writeSolutionVector">if set to <c>true</c> [writes solution vector].</param>
	/// <param name="writeForceVector">if set to <c>true</c> [writes force vector].</param>
	/// <param name="writeStiffnessMatrix">if set to <c>true</c> [writes stiffness matrix].</param>
	public void WriteToFile(bool writeSolutionVector = true, bool writeForceVector = false,
		bool writeStiffnessMatrix = false)
	{
		using var outputFile = new StreamWriter(_filepath);
		if (writeStiffnessMatrix)
		{
			outputFile.WriteLine("Bounded StiffnessMatrix");
			for (var i = 0; i < _linearSystem.Matrix.Rows; i++)
			{
				for (var j = 0; j < _linearSystem.Matrix.Columns; j++)
					outputFile.Write($"{_linearSystem.Matrix[i, j]}	");
				outputFile.WriteLine(string.Empty);
			}
		}

		outputFile.WriteLine(string.Empty);

		if (writeForceVector)
		{
			outputFile.WriteLine("Force Vector");
			foreach (var node in _model.Nodes)
			{
				outputFile.Write($"{node.ID}	");
				var dofDictionary = _model.NodalDOFsDictionary[node.ID];
				foreach (var dof in dofDictionary.Values)
					outputFile.Write($"{(dof < 0 ? 0.0 : _linearSystem.RHS[dof])}	");
				outputFile.WriteLine("");
			}
		}

		outputFile.WriteLine(string.Empty);

		if (writeSolutionVector)
		{
			outputFile.WriteLine("Solution Vector");
			foreach (var node in _model.Nodes)
			{
				outputFile.Write($"{node.ID}	");
				var dofDictionary = _model.NodalDOFsDictionary[node.ID];
				foreach (var dof in dofDictionary.Values)
					outputFile.Write($"{(dof < 0 ? 0.0 : _linearSystem.Solution[dof])}	");
				outputFile.WriteLine("");
			}
		}
	}
}
