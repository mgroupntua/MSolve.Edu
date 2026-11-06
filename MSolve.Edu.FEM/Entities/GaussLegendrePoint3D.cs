namespace MSolve.Edu.FEM.Entities;

using System;

using MSolve.Edu.LinearAlgebra;

/// <summary>
///     Defines an one-dimensional Gauss Legendre integration point.
/// </summary>
public class GaussLegendrePoint1D
{
	/// <summary>
	///     Gets or sets the parametric coordinate of the integration point.
	/// </summary>
	public double Coordinate { get; set; }

	/// <summary>
	///     Gets or sets the weight of the integration point.
	/// </summary>
	public double WeightFactor { get; set; }
}

/// <summary>
///     Defines a three-dimensional Gauss Legendre integration point.
/// </summary>
public class GaussLegendrePoint3D
{
	private readonly Matrix2D B;
	private readonly double Heta;
	private readonly double Ksi;

	/// <summary>
	///     Initializes a new instance of the <see cref="GaussLegendrePoint3D" /> class.
	/// </summary>
	/// <param name="ksi">The gauss point parametric coordinate ksi.</param>
	/// <param name="heta">The gauss point parametric coordinate heta.</param>
	/// <param name="zeta">The gauss point parametric coordinate zeta.</param>
	/// <param name="deformationMatrix">The gauss point deformation matrix.</param>
	/// <param name="weightFactor">The gauss point weight factor.</param>
	public GaussLegendrePoint3D(double ksi, double heta, double zeta, Matrix2D deformationMatrix, double weightFactor)
	{
		Ksi = ksi;
		Heta = heta;
		Zeta = zeta;
		B = deformationMatrix;
		WeightFactor = weightFactor;
	}

	/// <summary>
	///     Initializes a new instance of the <see cref="GaussLegendrePoint3D" /> class.
	/// </summary>
	/// <param name="ksi">The gauss point parametric coordinate ksi.</param>
	/// <param name="heta">The gauss point parametric coordinate heta.</param>
	/// <param name="zeta">The gauss point parametric coordinate zeta.</param>
	/// <param name="deformationMatrix">The gauss point deformation matrix.</param>
	/// <param name="weightFactor">The gauss point weight factor.</param>
	public GaussLegendrePoint3D(
		double xi, double eta, double zeta, double[,] deformationMatrix, double weightFactor)
	{
		Xi = xi;
		Eta = eta;
		Zeta = zeta;
		DeformationMatrix = deformationMatrix;
		WeightFactor = weightFactor;
	}

	public double[,] DeformationMatrix { get; private set; }
	public double Eta { get; private set; }
	public double WeightFactor { get; private set; }
	public double Xi { get; private set; }
	public double Zeta { get; private set; }
}

/// <summary>
///     Provides one-dimensional Gauss-Legendre points and weights.
///     For more information on Gauss quadrature please refer to
///     <see href="https://en.wikipedia.org/wiki/Gaussian_quadrature" />
/// </summary>
public class GaussQuadrature
{
	/* For point coordinates, we encounter the following constants:
     * 0.5773502691896 = 1 / Square Root 3
     * 0.7745966692415 = (Square Root 15)/ 5
     * 0.8611363115941 = Square Root( (3 + 2*sqrt(6/5))/7)
     * 0.3399810435849 = Square Root( (3 - 2*sqrt(6/5))/7)
     *
     * For the weights, we encounter the followings constants:
     * 0.5555555555556 = 5/9
     * 0.8888888888889 = 8/9
     * 0.3478548451375 = (18 - sqrt30)/36
     * 0.6521451548625 = (18 + sqrt30)/36
     */
	private static readonly GaussLegendrePoint1D GaussLegendrePoint1 = new()
	{
		Coordinate = 0.0,
		WeightFactor = 2.0
	};

	private static readonly GaussLegendrePoint1D GaussLegendrePoint2A = new()
	{
		Coordinate = -0.5773502691896,
		WeightFactor = 1.0
	};

	private static readonly GaussLegendrePoint1D GaussLegendrePoint2B = new()
	{
		Coordinate = 0.5773502691896,
		WeightFactor = 1.0
	};

	private static readonly GaussLegendrePoint1D GaussLegendrePoint3A = new()
	{
		Coordinate = -0.7745966692415,
		WeightFactor = 0.5555555555556
	};

	private static readonly GaussLegendrePoint1D GaussLegendrePoint3B = new()
	{
		Coordinate = 0.0,
		WeightFactor = 0.8888888888889
	};

	private static readonly GaussLegendrePoint1D GaussLegendrePoint3C = new()
	{
		Coordinate = 0.7745966692415,
		WeightFactor = 0.5555555555556
	};

	private static readonly GaussLegendrePoint1D GaussLegendrePoint4A = new()
	{
		Coordinate = -0.86113631159416,
		WeightFactor = 0.3478548451375
	};

	private static readonly GaussLegendrePoint1D GaussLegendrePoint4B = new()
	{
		Coordinate = -0.3399810435849,
		WeightFactor = 0.6521451548625
	};

	private static readonly GaussLegendrePoint1D GaussLegendrePoint4C = new()
	{
		Coordinate = 0.3399810435849,
		WeightFactor = 0.6521451548625
	};

	private static readonly GaussLegendrePoint1D GaussLegendrePoint4D = new()
	{
		Coordinate = 0.86113631159416,
		WeightFactor = 0.3478548451375
	};

	public static GaussLegendrePoint1D[] GetGaussLegendrePoints(int integrationDegree)
	{
		return integrationDegree < 1
			? throw new InvalidOperationException("Integration Degree must be greater or equal to 1. ")
			: integrationDegree switch
			{
				1 => [GaussLegendrePoint1],
				2 => [GaussLegendrePoint2A, GaussLegendrePoint2B],
				3 => [GaussLegendrePoint3A, GaussLegendrePoint3B, GaussLegendrePoint3C],
				4 => [
									GaussLegendrePoint4A, GaussLegendrePoint4B, GaussLegendrePoint4C, GaussLegendrePoint4D
								],
				_ => throw new NotImplementedException("Integration Degree higher than 4 is not implemented yet. "),
			};
	}
}
