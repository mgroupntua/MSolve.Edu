using System;
using System.Collections.Generic;
using MSolve.Edu.FEM.Entities;
using MSolve.Edu.LinearAlgebra;

namespace MSolve.Edu.FEM
{
	/// <summary>
	/// Assembles the global stiffness matrix.
	/// </summary>
	public static class GlobalMatrixAssemblerSkyline
	{
		/// <summary>
		/// Calculates row indices for the assembly of <see cref="SkylineMatrix2D"/> global stiffness matrix.
		/// </summary>
		/// <param name="model">The model.</param>
		/// <param name="nodalDOFsDictionary">The nodal dofs dictionary.</param>
		/// <returns></returns>
		private static int[] CalculateRowIndex(Model model,Dictionary<int, Dictionary<DOFType, int>> nodalDOFsDictionary)
		{
			int[] rowHeights = new int[model.TotalDOFs];
			foreach (Element element in model.ElementsDictionary.Values)
			{
				int minDOF = int.MaxValue;
				foreach (Node node in element.ElementType.DOFEnumerator.GetNodesForMatrixAssembly(element))
				{
					foreach (int dof in nodalDOFsDictionary[node.ID].Values)
						if (dof != -1) minDOF = Math.Min(dof, minDOF);
				}
				foreach (Node node in element.ElementType.DOFEnumerator.GetNodesForMatrixAssembly(element))
				{
					foreach (int dof in nodalDOFsDictionary[node.ID].Values)
						if (dof != -1) rowHeights[dof] = Math.Max(rowHeights[dof], dof - minDOF);
				}
			}

			int[] rowIndex = new int[model.TotalDOFs + 1];
			rowIndex[0] = 0;
			rowIndex[1] = 1;
			for (int i = 1; i < model.TotalDOFs; i++)
				rowIndex[i + 1] = rowIndex[i] + rowHeights[i] + 1;
			return rowIndex;
		}

		/// <summary>
		/// Calculates the global stiffness matrix.
		/// </summary>
		/// <param name="model">The model.</param>
		/// <param name="nodalDOFsDictionary">The nodal dofs dictionary.</param>
		/// <param name="elementProvider">The element provider.</param>
		/// <returns></returns>
		public static SkylineMatrix2D CalculateGlobalMatrix(Model model, Dictionary<int, Dictionary<DOFType, int>> nodalDOFsDictionary, ElementStructuralStiffnessProvider elementProvider)
		{
			SkylineMatrix2D K = new SkylineMatrix2D(GlobalMatrixAssemblerSkyline.CalculateRowIndex(model, nodalDOFsDictionary));
			foreach (Element element in model.ElementsDictionary.Values)
			{
				var elStart = DateTime.Now;
				Matrix2D ElementK = elementProvider.Matrix(element);

				var elementDOFTypes = element.ElementType.DOFEnumerator.GetDOFTypes(element);
				var matrixAssemblyNodes = element.ElementType.DOFEnumerator.GetNodesForMatrixAssembly(element);
				int iElementMatrixRow = 0;
				for (int i = 0; i < elementDOFTypes.Count; i++)
				{
					Node nodeRow = matrixAssemblyNodes[i];
					foreach (DOFType dofTypeRow in elementDOFTypes[i])
					{
						int dofRow = nodalDOFsDictionary[nodeRow.ID][dofTypeRow];
						if (dofRow != -1)
						{
							int iElementMatrixColumn = 0;
							for (int j = 0; j < elementDOFTypes.Count; j++)
							{
								Node nodeColumn = matrixAssemblyNodes[j];
								foreach (DOFType dofTypeColumn in elementDOFTypes[j])
								{
									int dofColumn = nodalDOFsDictionary[nodeColumn.ID][dofTypeColumn];
									if (dofColumn != -1)
									{
										int height = dofRow - dofColumn;
										if (height >= 0)
											K.Data[K.RowIndex[dofRow] + height] += ElementK[iElementMatrixRow, iElementMatrixColumn];
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
		/// Calculates the global stiffness matrix.
		/// </summary>
		/// <param name="model">The model.</param>
		/// <param name="elementProvider">The element provider.</param>
		/// <returns></returns>
		public static SkylineMatrix2D CalculateGlobalMatrix(Model model,ElementStructuralStiffnessProvider elementProvider) =>
			CalculateGlobalMatrix(model, model.NodalDOFsDictionary, elementProvider);
	}
}
