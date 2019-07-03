using MSolve.Edu.FEM.Entities;
using MSolve.Edu.LinearAlgebra;

namespace MSolve.Edu.FEM
{
	/// <summary>
	/// Responsible for providing elemental stiffness matrix for the global matrix assembly.
	/// </summary>
	public class ElementStructuralStiffnessProvider
	{
		/// <summary>
		/// Retrieves the element stiffness matrix.
		/// </summary>
		/// <param name="element">The element.</param>
		/// <returns></returns>
		public Matrix2D Matrix(Element element) => element.ElementType.StiffnessMatrix(element);
	}
}
