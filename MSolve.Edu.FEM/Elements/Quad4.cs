using System;
using System.Collections.Generic;
using System.Linq;
using MSolve.Edu.FEM.Entities;
using MSolve.Edu.FEM.Material;
using MSolve.Edu.LinearAlgebra;

namespace MSolve.Edu.FEM.Elements
{
	/// <summary>
	/// Represents a quadrilateral two-dimensional element with four nodes. Can be used with either plane stress or plane strain materials.
	/// For further information on the isoparametric quadrilateral elements please refer to http://kis.tu.kielce.pl/mo/COLORADO_FEM/colorado/IFEM.Ch17.pdf
	/// </summary>
	public class Quad4 : IElementType
	{
		private static double determinantTolerance = 0.00000001;

		private static int iInt = 2;
		private static int iInt2 = iInt * iInt;
		private static int iInt3 = iInt2 * iInt;
		private readonly static DOFType[] nodalDOFTypes = new DOFType[] {DOFType.X, DOFType.Y};

		private readonly static DOFType[][] dofTypes = new DOFType[][]
		{
			nodalDOFTypes, nodalDOFTypes,
			nodalDOFTypes, nodalDOFTypes
		};

		private readonly ElasticMaterial2D[] materialsAtGaussPoints;
		private GenericDOFEnumerator dofEnumerator = new GenericDOFEnumerator();

		/// <summary>Initializes a new instance of the <see cref="Quad4"/> class.</summary>
		/// <param name="material">An elastic two-dimensional plane-strain or plane-stress material.</param>
		/// <example> This example shows how to instantiate a new <see cref="Quad4"/> element.
		/// <code>
		/// var quad = new Quad4(material) { Thickness = thickness };
		/// var element = new Quad4(material)
		/// {
		///		ID = 1,
		/// 	ElementType = quad,
		/// };
		/// </code>
		/// </example>
		public Quad4(ElasticMaterial2D material)
		{
			materialsAtGaussPoints = new ElasticMaterial2D[iInt2];
			for (int i = 0; i < iInt2; i++)
				materialsAtGaussPoints[i] = (ElasticMaterial2D) material.Clone();
		}
		
		/// <summary>Initializes a new instance of the <see cref="Quad4"/> class.</summary>
		/// <param name="material">An elastic two-dimensional plane=strain or plane-stress material.</param>
		/// <param name="dofEnumerator"><inheritdoc cref="GenericDOFEnumerator"/></param>
		public Quad4(ElasticMaterial2D material, GenericDOFEnumerator dofEnumerator)
			: this(material) => this.dofEnumerator = dofEnumerator;

		/// <summary>
		/// Gets the code for this element type.
		/// </summary>
		public CellType Code => CellType.Quad4;

        /// <summary>
        /// Density of the material used in <see cref="Quad4"/>
        /// </summary>
        public double Density { get; set; } = 1.0;

        /// <summary>
        /// Thickness t, of the isoparametric four-noded quadrilateral.
        /// Constant throught the element.
        /// Must be initialized at the instantiation of a <see cref="Quad4"/> element,
        /// otherwise will result in a zero stiffness matrix.
        /// </summary>
        public double Thickness { get; set; } = 1.0;

		/// <summary>
		/// <inheritdoc cref="GenericDOFEnumerator"/>
		/// </summary>
		public GenericDOFEnumerator DOFEnumerator
		{
			get => dofEnumerator;
			set => dofEnumerator = value;
		}

		/// <summary>
		/// Retrieves the element nodal coordinates in (4x2) matrix form.
		/// </summary>
		/// <param name="element"></param>
		/// <returns></returns>
		private double[,] GetCoordinates(Element element)
		{
			double[,] faXY = new double[dofTypes.Length, 2];
			for (int i = 0; i < dofTypes.Length; i++)
			{
				faXY[i, 0] = element.INodes[i].X;
				faXY[i, 1] = element.INodes[i].Y;
			}

			return faXY;
		}

		/// <summary>
		/// Retrieves the element nodal coordinates in (2x4) matrix form.
		/// </summary>
		/// <param name="element"></param>
		/// <returns></returns>
		private double[,] GetCoordinatesTranspose(Element element)
		{
			double[,] faXY = new double[2, dofTypes.Length];
			for (int i = 0; i < dofTypes.Length; i++)
			{
				faXY[0, i] = element.Nodes[i].X;
				faXY[1, i] = element.Nodes[i].Y;
			}

			return faXY;
		}

		/// <summary>
		/// A list with size equal to the number of nodes, each containing a list with the degrees of freedom of each node.
		/// </summary>
		/// <param name="element">An element of type <see cref="Quad4"/></param>
		/// <returns></returns>
		public IList<IList<DOFType>> GetElementDOFTypes(Element element) => dofTypes;

		/// <summary>
		/// A list with the nodes of the <see cref="Quad4"/> element.
		/// </summary>
		/// <param name="element">An element of type <see cref="Quad4"/></param>
		/// <returns></returns>
		public IList<Node> GetNodesForMatrixAssembly(Element element) => element.Nodes;

		/// <summary>
		/// Calculates the shape function values for an integration point
		/// </summary>
		/// <param name="fXi">Xi coordinate of the integration point</param>
		/// <param name="fEta">Eta coordinate of the integration point.</param>
		/// <returns></returns>
		public double[] CalcQ4Shape(double fXi, double fEta)
		{
			// QUAD4 ELEMENT
			//
			//            Eta
			//            ^
			//#4(-1,1)    |     #3(1,1)
			//      ._____|_____.
			//      |     |     |
			//      |     |     |
			//      |     |_____|____>Xi (or psi - the same)
			//      |           |
			//      |           |
			//      .___________.
			// 
			//#1(-1,-1)        #2(1,-1)
			//
			const double fSq050 = 0.50; // 0.50*0.50=0.25
			double fXiP = (1.0 + fXi) * fSq050; //(1+psi) - P means Plus
			double fEtaP = (1.0 + fEta) * fSq050; //(1+eta) - P means Plus
			double fXiM = (1.0 - fXi) * fSq050; //(1-psi) - M means Minus
			double fEtaM = (1.0 - fEta) * fSq050; //(1-eta) - M means Minus
			return new double[]
			{
				fXiM * fEtaM, //N1 = .25*(1-psi).*(1-eta); 
				fXiP * fEtaM, //N2 = .25*(1+psi).*(1-eta);
				fXiP * fEtaP, //N3 = .25*(1+psi).*(1+eta);
				fXiM * fEtaP, //N4 = .25*(1-psi).*(1+eta);
			};
		}

		/// <summary>
		/// Calculates the shape function derivative values for an integration point.
		/// </summary>
		/// <param name="fXi"></param>
		/// <param name="fEta"></param>
		/// <returns></returns>
		private double[] CalcQ4ShapeFunctionDerivatives(double fXi, double fEta)
		{
			const double fN025 = 0.25;
			double fXiP = (1.0 + fXi) * fN025; //(1+psi) - P means Plus
			double fEtaP = (1.0 + fEta) * fN025; //(1+eta) - P means Plus
			double fXiM = (1.0 - fXi) * fN025; //(1-psi) - M means Minus
			double fEtaM = (1.0 - fEta) * fN025; //(1-eta) - M means Minus

			return new double[8]
			{
				-fEtaM, //N1,psi = -.25*(1-eta); 
				fEtaM, //N2,psi =  .25*(1-eta);
				fEtaP, //N3,psi =  .25*(1+eta);
				-fEtaP, //N4,psi = -.25*(1+eta);
				-fXiM, //N1,eta = -.25*(1-psi);
				-fXiP, //N2,eta = -.25*(1+psi);
				fXiP, //N3,eta =  .25*(1+psi);
				fXiM, //N4,eta =  .25*(1-psi);
			};
		}

		/// <summary>
		/// Calculates the jacobian matrix for an integration point.
		/// </summary>
		/// <param name="nodeCoordinates">The nodal coordinate given in matrix form.
		/// Specifically, a (4x2) matrix where the rows represent the local node numbering and the columns, X and Y values accordingly.</param>
		/// <param name="shapeFunctionDerivatives">The derivatives of the shape function in a vector form. The first four values are the Xi derivatives and the next four values the Eta derivatives. </param>
		/// <returns></returns>
		private double[,] CalcQ4J(double[,] nodeCoordinates, double[] shapeFunctionDerivatives) 
		{
			// Jacobian Matrix J
			//                                        [x1  y1]
			//      [ N1,psi  N2,psi  N3,psi  N4,psi] [x2  y2]
			//[J] = [ N1,eta  N2,eta  N3,eta  N4,eta] [x3  y3]
			//                                        [x4  y4]
			//
			//                   [  ]   multiplied  by [  ]     equals [  ]
			//                      2x4                   4x2             2x2

			// REMINDER nodeCoordinates[i, 0] = element.Nodes[i].X;
			//          nodeCoordinates[i, 1] = element.Nodes[i].Y;

			double[,] jacobian = new double[2, 2];
			jacobian[0, 0] = shapeFunctionDerivatives[0] * nodeCoordinates[0, 0] +
			            shapeFunctionDerivatives[1] * nodeCoordinates[1, 0] +
			            shapeFunctionDerivatives[2] * nodeCoordinates[2, 0] +
			            shapeFunctionDerivatives[3] * nodeCoordinates[3, 0];

			jacobian[0, 1] = shapeFunctionDerivatives[0] * nodeCoordinates[0, 1] +
			            shapeFunctionDerivatives[1] * nodeCoordinates[1, 1] +
			            shapeFunctionDerivatives[2] * nodeCoordinates[2, 1] +
			            shapeFunctionDerivatives[3] * nodeCoordinates[3, 1];

			jacobian[1, 0] = shapeFunctionDerivatives[4] * nodeCoordinates[0, 0] +
			            shapeFunctionDerivatives[5] * nodeCoordinates[1, 0] +
			            shapeFunctionDerivatives[6] * nodeCoordinates[2, 0] +
			            shapeFunctionDerivatives[7] * nodeCoordinates[3, 0];

			jacobian[1, 1] = shapeFunctionDerivatives[4] * nodeCoordinates[0, 1] +
			            shapeFunctionDerivatives[5] * nodeCoordinates[1, 1] +
			            shapeFunctionDerivatives[6] * nodeCoordinates[2, 1] +
			            shapeFunctionDerivatives[7] * nodeCoordinates[3, 1];

			return jacobian;
		}

		/// <summary>
		/// Calculates the Jacobian determinant for an integration point.
		/// </summary>
		/// <param name="nodeCoordinates">The nodal coordinate given in matrix form.
		/// Specifically, a (4x2) matrix where the rows represent the local node numbering and the columns, X and Y values accordingly.</param>
		/// <param name="fXi">Xi coordinate of the integration point.</param>
		/// <param name="fEta">Eta coordinate of the integration point.</param>
		/// <returns></returns>
		private double CalcQ4JDetJ(double[,] nodeCoordinates, double fXi, double fEta) //Calculate the determinant of the Jacobian Matrix
		{
			//|J| or det[j] = 1/8 [x1 x2 x3 x4] [   0      1-eta    eta-psi    psi-1 ]  [y1]
			//                                  [ eta-1      0      psi+1    -psi-eta]  [y2]
			//                                  [psi-eta   -psi-1     0        eta+1 ]  [y3]
			//                                  [ 1-psi    psi+eta  -eta-1       0   ]  [y4]

			const double fN0125 = 0.125; //=1/8

			double fDetJ =
				(nodeCoordinates[0, 0] * ((1 - fEta) * nodeCoordinates[1, 1] + (fEta - fXi) * nodeCoordinates[2, 1] + (fXi - 1) * nodeCoordinates[3, 1])) +
				(nodeCoordinates[1, 0] * ((fEta - 1) * nodeCoordinates[0, 1] + (fXi + 1) * nodeCoordinates[2, 1] + (-fXi - fEta) * nodeCoordinates[3, 1])) +
				(nodeCoordinates[2, 0] * ((fXi - fEta) * nodeCoordinates[0, 1] + (-fXi - 1) * nodeCoordinates[1, 1] + (fEta + 1) * nodeCoordinates[3, 1])) +
				(nodeCoordinates[3, 0] * ((1 - fXi) * nodeCoordinates[0, 1] + (fXi + fEta) * nodeCoordinates[1, 1] + (-fEta - 1) * nodeCoordinates[2, 1]));

			fDetJ = fDetJ * fN0125;

			if (fDetJ < determinantTolerance)
				throw new ArgumentException(String.Format(
					"Jacobian determinant is negative or under tolerance ({0} < {1})." +
					"Check the order of nodes or the element geometry.", fDetJ, determinantTolerance));
			return fDetJ;
		}

		/// <summary>
		/// Calculates the inverse jacobian matrix.
		/// </summary>
		/// <param name="jacobianDeterminant"></param>
		/// <param name="jacobianMatrix"></param>
		/// <returns></returns>
		private double[,] CalcQ4JInv(double jacobianDeterminant, double[,] jacobianMatrix) //Calculate the full inverse of the Jacobian Matrix
		{
			//
			//[J]^-1 = 1/det[J]  [ J22    -J12]
			//                   1[-J21     J11]

			double fDetInv = 1.0 / jacobianDeterminant;

			double[,] faJInv = new double[2, 2];
			faJInv[0, 0] = (jacobianMatrix[1, 1]) * fDetInv;
			faJInv[0, 1] = (-jacobianMatrix[0, 1]) * fDetInv;
			faJInv[1, 0] = (-jacobianMatrix[1, 0]) * fDetInv;
			faJInv[1, 1] = (jacobianMatrix[0, 0]) * fDetInv;

			return faJInv;
		}

		/// <summary>
		/// Calculates the deformation matrix for an integration point.
		/// </summary>
		/// <param name="nodeCoordinates"> The nodal coordinate given in matrix form.
		/// Specifically, a (4x2) matrix where the rows represent the local node numbering and the columns, X and Y values accordingly. </param>
		/// <param name="shapeFunctionDerivatives"></param>
		/// <param name="coordinateXi">Xi coordinate of the integration point.</param>
		/// <param name="coordinateEta">Eta coordinate of the integration point.</param>
		/// <param name="jacobianDeterminant"></param>
		/// <returns></returns>
		private double[,] CalculateDeformationMatrix(double[,] nodeCoordinates, double[] shapeFunctionDerivatives, double coordinateXi, double coordinateEta,
			double jacobianDeterminant)
		{
			double fDetInv = 1.0 / jacobianDeterminant;
			double Aparameter;
			double Bparameter;
			double Cparameter;
			double Dparameter;

			Aparameter = 0.25 * (nodeCoordinates[0, 1] * (coordinateXi - 1) + nodeCoordinates[1, 1] * (-1 - coordinateXi) + nodeCoordinates[2, 1] * (1 + coordinateXi) +
			                     nodeCoordinates[3, 1] * (1 - coordinateXi));
			Bparameter = 0.25 * (nodeCoordinates[0, 1] * (coordinateEta - 1) + nodeCoordinates[1, 1] * (1 - coordinateEta) + nodeCoordinates[2, 1] * (1 + coordinateEta) +
			                     nodeCoordinates[3, 1] * (-1 - coordinateEta));
			Cparameter = 0.25 * (nodeCoordinates[0, 0] * (coordinateEta - 1) + nodeCoordinates[1, 0] * (1 - coordinateEta) + nodeCoordinates[2, 0] * (1 + coordinateEta) +
			                     nodeCoordinates[3, 0] * (-1 - coordinateEta));
			Dparameter = 0.25 * (nodeCoordinates[0, 0] * (coordinateXi - 1) + nodeCoordinates[1, 0] * (-1 - coordinateXi) + nodeCoordinates[2, 0] * (1 + coordinateXi) +
			                     nodeCoordinates[3, 0] * (1 - coordinateXi));

			double[,] Bmatrix = new double[3, 8];

			Bmatrix[0, 0] = fDetInv * (Aparameter * shapeFunctionDerivatives[0] - Bparameter * shapeFunctionDerivatives[4]);
			Bmatrix[1, 0] = 0;
			Bmatrix[2, 0] = fDetInv * (Cparameter * shapeFunctionDerivatives[4] - Dparameter * shapeFunctionDerivatives[0]);
			Bmatrix[0, 1] = 0;
			Bmatrix[1, 1] = fDetInv * (Cparameter * shapeFunctionDerivatives[4] - Dparameter * shapeFunctionDerivatives[0]);
			Bmatrix[2, 1] = fDetInv * (Aparameter * shapeFunctionDerivatives[0] - Bparameter * shapeFunctionDerivatives[4]);

			Bmatrix[0, 2] = fDetInv * (Aparameter * shapeFunctionDerivatives[1] - Bparameter * shapeFunctionDerivatives[5]);
			Bmatrix[1, 2] = 0;
			Bmatrix[2, 2] = fDetInv * (Cparameter * shapeFunctionDerivatives[5] - Dparameter * shapeFunctionDerivatives[1]);
			Bmatrix[0, 3] = 0;
			Bmatrix[1, 3] = fDetInv * (Cparameter * shapeFunctionDerivatives[5] - Dparameter * shapeFunctionDerivatives[1]);
			Bmatrix[2, 3] = fDetInv * (Aparameter * shapeFunctionDerivatives[1] - Bparameter * shapeFunctionDerivatives[5]);

			Bmatrix[0, 4] = fDetInv * (Aparameter * shapeFunctionDerivatives[2] - Bparameter * shapeFunctionDerivatives[6]);
			Bmatrix[1, 4] = 0;
			Bmatrix[2, 4] = fDetInv * (Cparameter * shapeFunctionDerivatives[6] - Dparameter * shapeFunctionDerivatives[2]);
			Bmatrix[0, 5] = 0;
			Bmatrix[1, 5] = fDetInv * (Cparameter * shapeFunctionDerivatives[6] - Dparameter * shapeFunctionDerivatives[2]);
			Bmatrix[2, 5] = fDetInv * (Aparameter * shapeFunctionDerivatives[2] - Bparameter * shapeFunctionDerivatives[6]);

			Bmatrix[0, 6] = fDetInv * (Aparameter * shapeFunctionDerivatives[3] - Bparameter * shapeFunctionDerivatives[7]);
			Bmatrix[1, 6] = 0;
			Bmatrix[2, 6] = fDetInv * (Cparameter * shapeFunctionDerivatives[7] - Dparameter * shapeFunctionDerivatives[3]);
			Bmatrix[0, 7] = 0;
			Bmatrix[1, 7] = fDetInv * (Cparameter * shapeFunctionDerivatives[7] - Dparameter * shapeFunctionDerivatives[3]);
			Bmatrix[2, 7] = fDetInv * (Aparameter * shapeFunctionDerivatives[3] - Bparameter * shapeFunctionDerivatives[7]);

			return Bmatrix;
		}

		/// <summary>
		/// Calculates the <see cref="Quad4"/> integration points.
		/// </summary>
		/// <param name="nodeCoordinates">The nodal coordinate given in matrix form.
		/// Specifically, a (4x2) matrix where the rows represent the local node numbering and the columns, X and Y values accordingly.</param>
		/// <returns></returns>
		private GaussLegendrePoint3D[] CalculateGaussMatrices(double[,] nodeCoordinates)
		{
			GaussLegendrePoint1D[] integrationPointsPerAxis = GaussQuadrature.GetGaussLegendrePoints(iInt);
			int totalSamplingPoints = (int) Math.Pow(integrationPointsPerAxis.Length, 2);

			GaussLegendrePoint3D[] integrationPoints = new GaussLegendrePoint3D[totalSamplingPoints];

			int counter = -1;
			foreach (GaussLegendrePoint1D pointXi in integrationPointsPerAxis)
			{
				foreach (GaussLegendrePoint1D pointEta in integrationPointsPerAxis)
				{
					counter += 1;
					double xi = pointXi.Coordinate;
					double eta = pointEta.Coordinate;
					double[] faDS = this.CalcQ4ShapeFunctionDerivatives(xi, eta);
					double[,] faJ = this.CalcQ4J(nodeCoordinates, faDS);
					double fDetJ = this.CalcQ4JDetJ(nodeCoordinates, xi, eta);
					double[,] deformationMatrix =
						this.CalculateDeformationMatrix(nodeCoordinates, faDS, xi, eta, fDetJ);
					double weightFactor =
						pointXi.WeightFactor * pointEta.WeightFactor * fDetJ; // Element thickness is assumed t=1
					integrationPoints[counter] = new GaussLegendrePoint3D(xi, eta, 0, deformationMatrix, weightFactor);
				}
			}

			return integrationPoints;
		}

		/// <summary>
		/// Method that calculates the stiffness matrix of an isoparametric four-noded quadrilateral element, with constant thickness.
		/// </summary>
		/// <param name="element">An element of type <see cref="Quad4"/> </param>
		/// <returns>A (8x8) stiffness matrix of the <see cref="Quad4"/> </returns>
		public Matrix2D StiffnessMatrix(Element element)
		{
			double[,] coordinates = this.GetCoordinates(element);
			GaussLegendrePoint3D[] integrationPoints = this.CalculateGaussMatrices(coordinates);
			Matrix2D stiffnessMatrix = new Matrix2D(8, 8);
			int pointId = -1;
			foreach (GaussLegendrePoint3D intPoint in integrationPoints)
			{
				pointId++;
				Matrix2D constitutiveMatrix = materialsAtGaussPoints[pointId].ConstitutiveMatrix;
				double[,] b = intPoint.DeformationMatrix;
				for (int i = 0; i < 8; i++)
				{
					double[] eb = new double[3];
					for (int iE = 0; iE < 3; iE++)
					{
						eb[iE] = (constitutiveMatrix[iE, 0] * b[0, i]) + (constitutiveMatrix[iE, 1] * b[1, i]) +
						         (constitutiveMatrix[iE, 2] * b[2, i]);
					}

					for (int j = 0; j < 8; j++)
					{
						double stiffness = (b[0, j] * eb[0]) + (b[1, j] * eb[1]) + (b[2, j] * eb[2]);
						stiffnessMatrix[i, j] += stiffness * intPoint.WeightFactor * Thickness;
					}
				}
			}

			return stiffnessMatrix;
		}


		/// <summary>
		/// Calculates and returns the strains and stresses of a gauss point of the <see cref="Quad4"/> element.
		/// </summary>
		/// <param name="element">An element of type <see cref="Quad4"/></param>
		/// <param name="localDisplacements">The displacement of the element nodes. An array with eight values.</param>
		/// <param name="localdDisplacements"></param>
		/// <returns></returns>
		public Tuple<double[], double[]> CalculateStresses(Element element, double[] localDisplacements,
			double[] localdDisplacements)
		{
			double[,] faXY = GetCoordinates(element);
			double[,] faDS = new double[iInt3, 24];
			double[,] faS = new double[iInt3, 8];
			double[,,] faB = new double[iInt3, 24, 6];
			double[] faDetJ = new double[iInt3];
			double[,,] faJ = new double[iInt3, 3, 3];
			double[] faWeight = new double[iInt3];
			double[,] fadStrains = new double[iInt3, 6];
			double[,] faStrains = new double[iInt3, 6];

			double[] dStrains = new double[6];
			double[] strains = new double[6];
			for (int i = 0; i < materialsAtGaussPoints.Length; i++)
			{
				for (int j = 0; j < 6; j++) dStrains[j] = fadStrains[i, j];
				for (int j = 0; j < 6; j++) strains[j] = faStrains[i, j];
				materialsAtGaussPoints[i].UpdateMaterial(new Vector(dStrains));
			}

			return new Tuple<double[], double[]>(strains,
				materialsAtGaussPoints[materialsAtGaussPoints.Length - 1].Stresses.Data);
		}

		/// <summary>
		/// Calculates the forces at the element nodes.
		/// </summary>
		/// <param name="element">An element of type <see cref="Quad4"</param>
		/// <param name="localTotalDisplacements">The displacement of the element nodes. An array with eight values.</param>
		/// <param name="localdDisplacements"></param>
		/// <returns></returns>
		public double[] CalculateForces(Element element, double[] localTotalDisplacements, double[] localdDisplacements)
		{
			double[,] faStresses = new double[iInt3, 6];
			for (int i = 0; i < materialsAtGaussPoints.Length; i++)
			for (int j = 0; j < 6; j++)
				faStresses[i, j] = materialsAtGaussPoints[i].Stresses[j];

			double[,] faXYZ = GetCoordinates(element);
			double[,] faDS = new double[iInt3, 24];
			double[,] faS = new double[iInt3, 8];
			double[,,] faB = new double[iInt3, 24, 6];
			double[] faDetJ = new double[iInt3];
			double[,,] faJ = new double[iInt3, 3, 3];
			double[] faWeight = new double[iInt3];
			double[] faForces = new double[24];

			return faForces;
		}

		public double[] GetNaturalCoordinates(Element element, Node node)
		{
			double[] mins = new double[] {element.Nodes[0].X, element.Nodes[0].Y};
			double[] maxes = new double[] {element.Nodes[0].X, element.Nodes[0].Y};
			for (int i = 0; i < element.Nodes.Count; i++)
			{
				mins[0] = mins[0] > element.Nodes[i].X ? element.Nodes[i].X : mins[0];
				mins[1] = mins[1] > element.Nodes[i].Y ? element.Nodes[i].Y : mins[1];
				maxes[0] = maxes[0] < element.Nodes[i].X ? element.Nodes[i].X : maxes[0];
				maxes[1] = maxes[1] < element.Nodes[i].Y ? element.Nodes[i].Y : maxes[1];
			}

			bool maybeInsideElement = node.X <= maxes[0] && node.X >= mins[0] &&
			                          node.Y <= maxes[1] && node.Y >= mins[1];
			if (maybeInsideElement == false) return new double[0];


			const int jacobianSize = 3;
			const int maxIterations = 1000;
			const double tolerance = 1e-10;
			int iterations = 0;
			double deltaNaturalCoordinatesNormSquare = 100;
			double[] naturalCoordinates = new double[] {0, 0, 0};
			const double toleranceSquare = tolerance * tolerance;

			while (deltaNaturalCoordinatesNormSquare > toleranceSquare && iterations < maxIterations)
			{
				iterations++;
				var shapeFunctions = CalcQ4Shape(naturalCoordinates[0], naturalCoordinates[1]);
				double[] coordinateDifferences = new double[] {0, 0};
				for (int i = 0; i < shapeFunctions.Length; i++)
				{
					coordinateDifferences[0] += shapeFunctions[i] * element.Nodes[i].X;
					coordinateDifferences[1] += shapeFunctions[i] * element.Nodes[i].Y;
				}

				coordinateDifferences[0] = node.X - coordinateDifferences[0];
				coordinateDifferences[1] = node.Y - coordinateDifferences[1];

				double[,] faXY = GetCoordinatesTranspose(element);
				double[] ShapeFunctionDerivatives =
					CalcQ4ShapeFunctionDerivatives(naturalCoordinates[0], naturalCoordinates[1]);

				var inverseJacobian = CalcQ4J(faXY, ShapeFunctionDerivatives);

				double[] deltaNaturalCoordinates = new double[] {0, 0, 0};
				for (int i = 0; i < jacobianSize; i++)
				for (int j = 0; j < jacobianSize; j++)
					deltaNaturalCoordinates[i] += inverseJacobian[j, i] * coordinateDifferences[j];
				for (int i = 0; i < 3; i++)
					naturalCoordinates[i] += deltaNaturalCoordinates[i];

				deltaNaturalCoordinatesNormSquare = 0;
				for (int i = 0; i < 3; i++)
					deltaNaturalCoordinatesNormSquare += deltaNaturalCoordinates[i] * deltaNaturalCoordinates[i];
			}

			return naturalCoordinates.Count(x => Math.Abs(x) - 1.0 > tolerance) > 0
				? new double[0]
				: naturalCoordinates;
		}
	}
}