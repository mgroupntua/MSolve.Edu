using MSolve.Edu.LinearAlgebra;

namespace MSolve.Edu.FEM.Material
{
	/// <summary>
	/// Enum that defines two-dimensional stress state of the <see cref="ElasticMaterial2D"/>
	/// </summary>
	public enum StressState2D
	{
		PlaneStress,
		PlaneStrain
	}

	/// <summary>
	/// Defines a two-dimensional elastic material with plane-strain or plane stress state.
	/// For further information on 2D stress state please refer to <see href="http://www.unife.it/ing/lm.civile/insegnamenti/meccanica-delle-strutture/materiale-didattico/materiale-integrativo/planestates_zienkiewicz.pdf"/>
	/// </summary>
	public class ElasticMaterial2D
	{
		private readonly double[] strains = new double[3];
		private double[] stresses = new double[3];
		private double[,] constitutiveMatrix = null;

		/// <summary>
		/// Gets or sets the coordinates of the material instance. 
		/// </summary>
		public double[] Coordinates { get; set; }
		/// <summary>
		/// Poisson Ration of the material. Must be set during instantiation of the material.
		/// </summary>
		public double PoissonRatio { get; set; }

		/// <summary>
		/// Gets the stress state of the material.
		/// </summary>
		public StressState2D StressState { get; }

		/// <summary>
		/// Gets stresses of the material.
		/// </summary>
		public Vector Stresses { get => new Vector(stresses); }

		/// <summary>
		/// Gets or sets the young modulus of the material. Must be set during instantiation of the material.
		/// </summary>
 		public double YoungModulus { get; set; }

		/// <summary>
		/// Gets the constitutive matrix .
		/// </summary>
		public Matrix2D ConstitutiveMatrix
		{
			get
			{
				if (constitutiveMatrix == null) UpdateMaterial(new Vector(new double[3]));
				return new Matrix2D(constitutiveMatrix);
			}
		}

		/// <summary>
		/// Initializes a new instance of the <see cref="ElasticMaterial2D"/> class.
		/// </summary>
		/// <param name="stressState">Stress state of the 2D material.</param>
		public ElasticMaterial2D(StressState2D stressState) => this.StressState = stressState;

		/// <summary>
		/// Given a strain vector updates the material state.
		/// </summary>
		/// <param name="strains">The strains.</param>
		public void UpdateMaterial(Vector strains)
		{
			strains.CopyTo(this.strains, 0);
			constitutiveMatrix = new double[3, 3];
			if (StressState == StressState2D.PlaneStress)
			{
				double aux = YoungModulus / (1 - PoissonRatio * PoissonRatio);
				constitutiveMatrix[0, 0] = aux;
				constitutiveMatrix[1, 1] = aux;
				constitutiveMatrix[0, 1] = PoissonRatio * aux;
				constitutiveMatrix[1, 0] = PoissonRatio * aux;
				constitutiveMatrix[2, 2] = (1 - PoissonRatio) / 2 * aux;
			}
			else
			{
				double aux = YoungModulus / (1 + PoissonRatio) / (1 - 2 * PoissonRatio);
				constitutiveMatrix[0, 0] = aux * (1 - PoissonRatio);
				constitutiveMatrix[1, 1] = aux * (1 - PoissonRatio);
				constitutiveMatrix[0, 1] = PoissonRatio * aux;
				constitutiveMatrix[1, 0] = PoissonRatio * aux;
				constitutiveMatrix[2, 2] = (1 - 2 * PoissonRatio) / 2 * aux;
			}
			stresses = (new Matrix2D(constitutiveMatrix) * strains).Data;
		}

		/// <summary>
		/// Clones this instance.
		/// </summary>
		/// <returns></returns>
		public ElasticMaterial2D Clone()
		{
			return new ElasticMaterial2D(StressState)
			{
				PoissonRatio = this.PoissonRatio,
				YoungModulus = this.YoungModulus
			};
		}
	}
}
