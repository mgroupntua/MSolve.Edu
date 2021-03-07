using System;
using System.Collections.Generic;
using System.Text;
using MSolve.Edu.FEM.Entities;
using MSolve.Edu.LinearAlgebra;

namespace MSolve.Edu.FEM.Elements
{
    public interface IElementType
    {
        /// <summary>
		/// Gets the code for this element type.
		/// </summary>
		CellType Code { get; }

        /// <summary>
        /// <inheritdoc cref="GenericDOFEnumerator"/>
        /// </summary>
		GenericDOFEnumerator DOFEnumerator { get; set; }

        /// <summary>
        /// A list with size equal to the number of nodes, each containing a list with the degrees of freedom of each node.
        /// </summary>
        /// <param name="element"></param>
        /// <returns></returns>
		IList<IList<DOFType>> GetElementDOFTypes(Element element);

        /// <summary>
        /// Calculates the element stiffness matrix.
        /// </summary>
        /// <param name="element"></param>
        /// <returns></returns>
        Matrix2D StiffnessMatrix(Element element);
    }
}
