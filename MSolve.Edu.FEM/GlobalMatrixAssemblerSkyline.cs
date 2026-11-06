namespace MSolve.Edu.FEM;

using System;
using System.Collections.Generic;

using MSolve.Edu.FEM.Entities;
using MSolve.Edu.LinearAlgebra;

/// <summary>
///     Assembles the global stiffness matrix.
/// </summary>
public static class GlobalMatrixAssemblerSkyline
{
	/// <summary>
	///     Calculates row indices for the assembly of <see cref="SkylineMatrix2D" /> global stiffness matrix.
	/// </summary>
	/// <param name="model">The model.</param>
	/// <param name="nodalDOFsDictionary">The nodal dofs dictionary.</param>
	/// <returns></returns>
	private static int[] CalculateRowIndex(Model model, Dictionary<int, Dictionary<DOFType, int>> nodalDOFsDictionary)
	{
		var rowHeights = new int[model.TotalDOFs];
		foreach (var element in model.ElementsDictionary.Values)
		{
			var minDOF = int.MaxValue;
			foreach (var node in GenericDOFEnumerator.GetNodesForMatrixAssembly(element))
				foreach (var dof in nodalDOFsDictionary[node.ID].Values)
					if (dof != -1)
						minDOF = Math.Min(dof, minDOF);
			foreach (var node in GenericDOFEnumerator.GetNodesForMatrixAssembly(element))
				foreach (var dof in nodalDOFsDictionary[node.ID].Values)
					if (dof != -1)
						rowHeights[dof] = Math.Max(rowHeights[dof], dof - minDOF);
		}

		var rowIndex = new int[model.TotalDOFs + 1];
		rowIndex[0] = 0;
		rowIndex[1] = 1;
		for (var i = 1; i < model.TotalDOFs; i++)
			rowIndex[i + 1] = rowIndex[i] + rowHeights[i] + 1;
		return rowIndex;
	}

	/// <summary>
	///     Calculates the global stiffness matrix.
	/// </summary>
	/// <param name="model">The model.</param>
	/// <param name="nodalDOFsDictionary">The nodal dofs dictionary.</param>
	/// <param name="elementProvider">The element provider.</param>
	/// <returns></returns>
	public static SkylineMatrix2D CalculateGlobalMatrix(Model model,
		Dictionary<int, Dictionary<DOFType, int>> nodalDOFsDictionary,
		ElementStructuralStiffnessProvider elementProvider)
	{
		var K = new SkylineMatrix2D(CalculateRowIndex(model, nodalDOFsDictionary));
		foreach (var element in model.ElementsDictionary.Values)
		{
			var elStart = DateTime.Now;
			var ElementK = ElementStructuralStiffnessProvider.Matrix(element);

			var elementDOFTypes = GenericDOFEnumerator.GetDOFTypes(element);
			var matrixAssemblyNodes = GenericDOFEnumerator.GetNodesForMatrixAssembly(element);
			var iElementMatrixRow = 0;
			for (var i = 0; i < elementDOFTypes.Count; i++)
			{
				var nodeRow = matrixAssemblyNodes[i];
				foreach (var dofTypeRow in elementDOFTypes[i])
				{
					var dofRow = nodalDOFsDictionary[nodeRow.ID][dofTypeRow];
					if (dofRow != -1)
					{
						var iElementMatrixColumn = 0;
						for (var j = 0; j < elementDOFTypes.Count; j++)
						{
							var nodeColumn = matrixAssemblyNodes[j];
							foreach (var dofTypeColumn in elementDOFTypes[j])
							{
								var dofColumn = nodalDOFsDictionary[nodeColumn.ID][dofTypeColumn];
								if (dofColumn != -1)
								{
									var height = dofRow - dofColumn;
									if (height >= 0)
										K.Data[K.RowIndex[dofRow] + height] +=
											ElementK[iElementMatrixRow, iElementMatrixColumn];
								}

								iElementMatrixColumn++;
							}
						}
					}

					iElementMatrixRow++;
				}
			}
		}

		return K;
	}

	/// <summary>
	///     Calculates the global stiffness matrix.
	/// </summary>
	/// <param name="model">The model.</param>
	/// <param name="elementProvider">The element provider.</param>
	/// <returns></returns>
	public static SkylineMatrix2D CalculateGlobalMatrix(Model model, ElementStructuralStiffnessProvider elementProvider) => CalculateGlobalMatrix(model, model.NodalDOFsDictionary, elementProvider);
}
