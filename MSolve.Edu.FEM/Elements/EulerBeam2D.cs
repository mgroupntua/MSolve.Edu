namespace MSolve.Edu.FEM.Elements;

using System;
using System.Collections.Generic;

using MSolve.Edu.FEM.Entities;
using MSolve.Edu.LinearAlgebra;

/// <summary>
///     Represents a two-dimensional beam finite element with the typical Euler-Bernoulli formulation.
///     For further information about the Euler-Bernoulli beam theory used for this element
///     please refer to https://en.wikipedia.org/wiki/Euler%E2%80%93Bernoulli_beam_theory
/// </summary>
/// <remarks>
///     Initializes a new instance of the <see cref="EulerBeam2D" /> class.
/// </remarks>
/// <param name="youngModulus">
///     Young modulus E used for calculation of the <see cref="EulerBeam2D" /> stiffness matrix.
///     Constant throughout the length of the material
/// </param>
/// <example>
///     This example shows how to instantiate a new <see cref="EulerBeam2D" /> element.
///     <code>
///  var beam = new EulerBeam2D(youngModulus)
///  {
/// 		Density = 7.85,
/// 		SectionArea = 91.04,
/// 		MomentOfInertia = 8091.00,
///  };  
///  </code>
/// </example>
public class EulerBeam2D(double youngModulus) : IElementType
{
	private static readonly DOFType[] nodalDOFTypes = [DOFType.X, DOFType.Y, DOFType.RotZ];
	private static readonly DOFType[][] dofs = [nodalDOFTypes, nodalDOFTypes];
	private readonly double youngModulus = youngModulus;

	/// <summary>
	///     Initializes a new instance of the <see cref="EulerBeam2D" /> class.
	/// </summary>
	/// <param name="youngModulus">
	///     Young modulus E used for calculation of the <see cref="EulerBeam2D" /> stiffness matrix.
	///     Constant throughout the length of the material
	/// </param>
	/// <param name="dofEnumerator">
	///     <inheritdoc cref="GenericDOFEnumerator" />
	/// </param>
	public EulerBeam2D(double youngModulus, GenericDOFEnumerator dofEnumerator)
		: this(youngModulus) => DOFEnumerator = dofEnumerator;

	/// <summary>
	///     Density of the material used in <see cref="EulerBeam2D" />
	/// </summary>
	public double Density { get; set; }

	/// <summary>
	///     Cross-section area A, of the Euler-Bernoulli beam formulation.
	///     Constant throughout the length of the beam element.
	///     Must be initialized at the instantiation of a <see cref="EulerBeam2D" /> element,
	///     otherwise will result in zero-element of the stiffness matrix.
	/// </summary>
	public double SectionArea { get; set; }

	/// <summary>
	///     Moment of Inertia of a cross-section of the beam element.
	///     Constant throughout the length of the element.
	///     Must be initialized at the instantiation of a <see cref="EulerBeam2D" /> element,
	///     otherwise will result in zero-element of the stiffness matrix.
	/// </summary>
	public double MomentOfInertia { get; set; }

	/// <summary>
	///     Gets the code for this element type.
	/// </summary>
	public CellType Code => CellType.Line;

	/// <summary>
	///     <inheritdoc cref="GenericDOFEnumerator" />
	/// </summary>
	public GenericDOFEnumerator DOFEnumerator { get; set; } = new();

	/// <summary>
	///     A list with size equal to the number of nodes, each containing a list with the degrees of freedom of each node.
	/// </summary>
	/// <param name="element">An element of type <see cref="EulerBeam2D" /></param>
	/// <returns></returns>
	public IList<IList<DOFType>> GetElementDOFTypes(Element element) => dofs;


	/// <summary>
	///     Method that calculates the stiffness matrix of an Euler-Bernoulli beam element, with constant Young Modulus, E,
	///     Section Area A and Moment of Inertia I.
	/// </summary>
	/// <param name="element">An element of type <see cref="EulerBeam2D" /> </param>
	/// <returns>A (6x6) stiffness matrix of the <see cref="EulerBeam2D" /> </returns>
	public Matrix2D StiffnessMatrix(Element element)
	{
		var x2 = Math.Pow(element.INodes[1].X - element.INodes[0].X, 2);
		var y2 = Math.Pow(element.INodes[1].Y - element.INodes[0].Y, 2);
		var L = Math.Sqrt(x2 + y2);
		var c = (element.INodes[1].X - element.INodes[0].X) / L;
		var c2 = c * c;
		var s = (element.INodes[1].Y - element.INodes[0].Y) / L;
		var s2 = s * s;
		var EL = youngModulus / L;
		var EAL = EL * SectionArea;
		var EIL = EL * MomentOfInertia;
		var EIL2 = EIL / L;
		var EIL3 = EIL2 / L;
		return GenericDOFEnumerator.GetTransformedMatrix(new Matrix2D(new[,]
		{
			{
				c2 * EAL + 12 * s2 * EIL3, c * s * EAL - 12 * c * s * EIL3, -6 * s * EIL2,
				-c2 * EAL - 12 * s2 * EIL3, -c * s * EAL + 12 * c * s * EIL3, -6 * s * EIL2
			},
			{
				c * s * EAL - 12 * c * s * EIL3, s2 * EAL + 12 * c2 * EIL3, 6 * c * EIL2,
				-s * c * EAL + 12 * c * s * EIL3, -s2 * EAL - 12 * c2 * EIL3, 6 * c * EIL2
			},
			{ -6 * s * EIL2, 6 * c * EIL2, 4 * EIL, 6 * s * EIL2, -6 * c * EIL2, 2 * EIL },
			{
				-c2 * EAL - 12 * s2 * EIL3, -s * c * EAL + 12 * c * s * EIL3, 6 * s * EIL2,
				c2 * EAL + 12 * s2 * EIL3, s * c * EAL - 12 * c * s * EIL3, 6 * s * EIL2
			},
			{
				-c * s * EAL + 12 * c * s * EIL3, -s2 * EAL - 12 * c2 * EIL3, -6 * c * EIL2,
				s * c * EAL - 12 * c * s * EIL3, s2 * EAL + 12 * c2 * EIL3, -6 * c * EIL2
			},
			{ -6 * s * EIL2, 6 * c * EIL2, 2 * EIL, 6 * s * EIL2, -6 * c * EIL2, 4 * EIL }
		}));
	}

	/// <summary>
	///     A list with the nodes of the <see cref="EulerBeam2D" /> element.
	/// </summary>
	/// <param name="element">An element of type <see cref="EulerBeam2D" /></param>
	/// <returns></returns>
	public static IList<Node> GetNodesForMatrixAssembly(Element element) => element.Nodes;
}
