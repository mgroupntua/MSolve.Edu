using System;
using System.Collections.Generic;
using MSolve.Edu.FEM.Entities;
using MSolve.Edu.LinearAlgebra;

namespace MSolve.Edu.FEM.Elements
{
	/// <summary>
	/// Represents a two-dimensional truss element. For further information about the theory used for this element
	/// please refer to <see href="http://www.unm.edu/~bgreen/ME360/Finite%20Element%20Truss.pdf"/>
	/// </summary>
	/// <seealso cref="MSolve.Edu.FEM.Entities.Element" />
	public class Truss2D : IElementType
	{
		private static readonly DOFType[] nodalDOFTypes = new DOFType[2] { DOFType.X, DOFType.Y };
		private static readonly DOFType[][] dofs = new DOFType[][] { nodalDOFTypes, nodalDOFTypes };
		private readonly double youngModulus;
		private GenericDOFEnumerator dofEnumerator = new GenericDOFEnumerator();

		/// <summary>
		/// Gets the code for this element type.
		/// </summary>
		public CellType Code => CellType.Line;

		/// <summary>
		/// Density of the material used in <see cref="Truss2D"/>
		/// </summary>
		public double Density { get; set; } = 1.0;

        /// <summary>
        /// Cross-section area A, of the Euler-Bernoulli beam formulation.
        /// Constant throughout the length of the beam element.
        /// Must be initialized at the instantiation of a <see cref="Truss2D"/> element,
        /// otherwise will result in zero-element of the stiffness matrix.
        /// </summary>
        public double SectionArea { get; set; } = 1.0;

		/// <summary>
		/// Initializes a new instance of the <see cref="Truss2D"/> class.
		/// </summary>
		/// <param name="youngModulus"> Young modulus E used for calculation of the <see cref="Truss2D"/> stiffness matrix.
		/// Constant throughout the length of the material</param>
		/// <example> This example shows how to instantiate a new <see cref="Truss2D"/> element.
		/// <code>
		/// var element1 = new Truss2D(youngModulus) { ID = 1, ElementType = new Truss2D(youngModulus)
		/// {
		///		Density = 1,
		///		SectionArea = sectionArea
		/// } };
		/// </code>
		/// </example>
		public Truss2D(double youngModulus) =>  this.youngModulus = youngModulus;
		
		/// <summary>
		/// Initializes a new instance of the <see cref="Truss2D"/> class.
		/// </summary>
		/// <param name="youngModulus">Young modulus E used for calculation of the <see cref="Truss2D"/> stiffness matrix.
		/// Constant throughout the length of the material</param>
		/// <param name="dofEnumerator"> <inheritdoc cref="GenericDOFEnumerator"/> </param>
		public Truss2D(double youngModulus, GenericDOFEnumerator dofEnumerator)
			: this(youngModulus) => this.dofEnumerator = dofEnumerator;

		/// <summary>
		/// <inheritdoc cref="GenericDOFEnumerator"/>
		/// </summary>
		public GenericDOFEnumerator DOFEnumerator
		{
			get => dofEnumerator;
			set => dofEnumerator = value;
		}

		/// <summary>
		/// A list with size equal to the number of nodes, each containing a list with the degrees of freedom of each node.
		/// </summary>
		/// <param name="element">An element of type <see cref="Truss2D"/></param>
		/// <returns></returns>
		public IList<IList<DOFType>> GetElementDOFTypes(Element element) => dofs;

		/// <summary>
		/// A list with the nodes of the <see cref="Truss2D"/> element.
		/// </summary>
		/// <param name="element">An element of type <see cref="Truss"/></param>
		/// <returns></returns>
		public IList<Node> GetNodesForMatrixAssembly(Element element) => element.Nodes;

		/// <summary>
		/// Calculates the transformation matrix of the <see cref="Truss2D"/> element.
		/// </summary>
		/// <param name="element">An element of type <see cref="Truss2D"/></param>
		/// <returns></returns>
		public Matrix2D TransformationMatrix(Element element)
		{
			double x2 = Math.Pow(element.Nodes[1].X - element.Nodes[0].X, 2);
			double y2 = Math.Pow(element.Nodes[1].Y - element.Nodes[0].Y, 2);
			double L = Math.Sqrt(x2 + y2);
			double c = (element.Nodes[1].X - element.Nodes[0].X) / L;
			double s = (element.Nodes[1].Y - element.Nodes[0].Y) / L;

			double[,] transformation = { {  c,   s, 0.0, 0.0},
										 {0.0, 0.0,   c,   s}};
			return new Matrix2D(transformation);
		}

		/// <summary>
		/// Calculates the axial stress of a <see cref="Truss2D"/> element.
		/// </summary>
		/// <param name="element">An element of type <see cref="Truss2D"/></param>
		/// <param name="localDisplacements"></param>
		/// <param name="local_d_Displacements"></param>
		/// <returns></returns>
		public double CalculateAxialStress(Element element, double[] localDisplacements, double[] local_d_Displacements)
		{
			double[] globalStresses = CalculateStresses(element, localDisplacements, local_d_Displacements).Item2; // item1 = strains
			Matrix2D transformation = TransformationMatrix(element);
			double[] localStresses = new double[2]; 
			transformation.Multiply(new Vector(globalStresses), localStresses);
			return localStresses[1];
		}

		/// <summary>
		/// Method that calculates the stiffness matrix of an Euler-Bernoulli beam element, with constant Young Modulus E and Section Area A.
		/// </summary>
		/// <param name="element">An element of type <see cref="Truss2D"/> </param>
		/// <returns>A (4x4) stiffness matrix of the <see cref="Truss2D"/> </returns>
		public Matrix2D StiffnessMatrix(Element element)
		{
			double x2 = Math.Pow(element.INodes[1].X - element.INodes[0].X, 2);
			double y2 = Math.Pow(element.INodes[1].Y - element.INodes[0].Y, 2);
			double L = Math.Sqrt(x2 + y2);
			double c = (element.INodes[1].X - element.INodes[0].X) / L;
			double c2 = c * c;
			double s = (element.INodes[1].Y - element.INodes[0].Y) / L;
			double s2 = s * s;
			double cs = c * s;
			double E = this.youngModulus;
			double A = SectionArea;

			return dofEnumerator.GetTransformedMatrix(
				new Matrix2D(new double[,]
				{
					{A*E*c2/L, A*E*cs/L, -A*E*c2/L, -A*E*cs/L },
					{A*E*cs/L, A*E*s2/L, -A*E*cs/L, -A*E*s2/L },
					{-A*E*c2/L, -A*E*cs/L, A*E*c2/L, A*E*cs/L },
					{-A*E*cs/L, -A*E*s2/L, A*E*cs/L, A*E*s2/L }
				}));
		}

		/// <summary>
		/// Calculates and returns the strains and stresses of an section of the <see cref="Truss2D"/> element.
		/// </summary>
		/// <param name="element">An element of type <see cref="Quad4"/></param>
		/// <param name="localDisplacements">The displacement of the element nodes. An array with eight values.</param>
		/// <param name="localdDisplacements"></param>
		/// <returns></returns>
		public Tuple<double[], double[]> CalculateStresses(Element element, double[] local_Displacements, double[] local_d_Displacements)
		{
			double[] strains = null;
			double[] forces = CalculateForces(element, local_Displacements, local_d_Displacements);
			double[] stresses = Array.ConvertAll<double, double>(forces, x => x / SectionArea);
			return new Tuple<double[], double[]>(strains, stresses);
		}

		/// <summary>
		/// Calculates the forces at the element nodes.
		/// </summary>
		/// <param name="element">An element of type <see cref="Truss2D"</param>
		/// <param name="localTotalDisplacements">The displacement of the element nodes. An array with four values.</param>
		/// <param name="localdDisplacements"></param>
		/// <returns></returns>
		public double[] CalculateForces(Element element, double[] localDisplacements, double[] localdDisplacements)
		{
			Matrix2D stiffness = StiffnessMatrix(element);
			double[] forces = new double[localDisplacements.Length];
			stiffness.Multiply(new Vector(localDisplacements), forces);
			return forces;
		}
	}
}
