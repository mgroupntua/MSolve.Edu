using System.Collections.Generic;
using MSolve.Edu.FEM.Entities;
using MSolve.Edu.LinearAlgebra;

namespace MSolve.Edu.FEM
{
	/// <summary>
	/// Retrieves element connectivity data required for matrix assembly.
	/// </summary>
	public class GenericDOFEnumerator 
	{
		/// <summary>
		/// Retrieves the dof types of each node.
		/// </summary>
		public IList<IList<DOFType>> GetDOFTypes(Element element) => element.ElementType.GetElementDOFTypes(element);

		/// <summary>
		/// Retrieves the dof types of each node.
		/// </summary>
		public IList<IList<DOFType>> GetDOFTypesForDOFEnumeration(Element element) => element.ElementType.GetElementDOFTypes(element);

		/// <summary>
		/// Retrieves the element nodes.
		/// </summary>
		public IList<Node> GetNodesForMatrixAssembly(Element element) => element.INodes;

		/// <summary>
		/// Retrieves matrix transformed from local to global coordinate system.
		/// </summary>
		public Matrix2D GetTransformedMatrix(Matrix2D matrix) => matrix;

		/// <summary>
		///  Retrieves displacements transformed from local to global coordinate system.
		/// </summary>
		public double[] GetTransformedDisplacementsVector(double[] vector) => vector;

		/// <summary>
		///  Retrieves displacements transformed from local to global coordinate system.
		/// </summary>
		public double[] GetTransformedForcesVector(double[] vector) => vector;
	}
}
