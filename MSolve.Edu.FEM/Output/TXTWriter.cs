using System.IO;
using MSolve.Edu.FEM.Entities;
using MSolve.Edu.Solvers;

namespace MSolve.Edu.FEM.Output
{
	/// <summary>
	/// Creates a TxtFile Writer that saves basic model results.
	/// </summary>
	public class TxtWriter
	{
		private readonly Model _model;
		private readonly string _filepath;
		private readonly SkylineLinearSystem _linearSystem;

		/// <summary>
		/// Initializes a new instance of the <see cref="TxtWriter"/> class.
		/// </summary>
		/// <param name="model">The model.</param>
		/// <param name="filepath">The filepath.</param>
		/// <param name="linearSystem">The linear system.</param>
		public TxtWriter(Model model, string filepath, SkylineLinearSystem linearSystem)
		{
			_model = model;
			_filepath = filepath;
			_linearSystem = linearSystem;
		}

		/// <summary>
		/// Writes to file.
		/// </summary>
		/// <param name="writeSolutionVector">if set to <c>true</c> [writes solution vector].</param>
		/// <param name="writeForceVector">if set to <c>true</c> [writes force vector].</param>
		/// <param name="writeStiffnessMatrix">if set to <c>true</c> [writes stiffness matrix].</param>
		public void WriteToFile(bool writeSolutionVector = true, bool writeForceVector = false, bool writeStiffnessMatrix=false)
		{
			using (StreamWriter outputFile = new StreamWriter(_filepath))
			{
				if (writeStiffnessMatrix)
				{
					outputFile.WriteLine("Bounded StiffnessMatrix");
					for (int i = 0; i < _linearSystem.Matrix.Rows; i++)
					{
						for (int j = 0; j < _linearSystem.Matrix.Columns; j++)
						{
							outputFile.Write($"{_linearSystem.Matrix[i, j]}	");
						}
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
						var dofDictionary=_model.NodalDOFsDictionary[node.ID];
						foreach (var dof in dofDictionary.Values)
							outputFile.Write($"{(dof < 0 ? 0.0 : _linearSystem.Solution[dof])}	");
						outputFile.WriteLine("");
					}
				}
			}
		}
	}
}
