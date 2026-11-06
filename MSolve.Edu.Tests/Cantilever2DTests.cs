namespace MSolve.Edu.Tests;

using System;

using MSolve.Edu.Analyzers;
using MSolve.Edu.FEM;
using MSolve.Edu.FEM.Elements;
using MSolve.Edu.FEM.Entities;
using MSolve.Edu.FEM.Material;
using MSolve.Edu.FEM.Mesh;
using MSolve.Edu.FEM.Output;
using MSolve.Edu.LinearAlgebra;
using MSolve.Edu.Solvers;

using Xunit;

public static class Cantilever2DTests
{
	private const double length = 5.0, height = 1.0, thickness = 1.0;
	private const double youngModulus = 2E7, poissonRatio = 0.3;
	private const double totalLoad = -100;

	[Fact]
	public static void TestQuad4LinearCantileverExample()
	{
		// Create model
		int numElementsX = 20, numElementsY = 4;
		var model = CreateModel(numElementsX, numElementsY);

		// Run linear static analysis
		var displacements = RunAnalysis(model);

		// Test solution
		var momentOfInertia = 1.0 / 12.0 * thickness * Math.Pow(height, 3.0);
		var expectedDisplacement = totalLoad * Math.Pow(length, 3.0) / (3.0 * youngModulus * momentOfInertia);
		var minDisplacement = double.MaxValue;
		for (var i = 0; i < displacements.Length; i++)
			if (displacements[i] < minDisplacement)
				minDisplacement = displacements[i];
		Assert.Equal(expectedDisplacement, minDisplacement, 2);
	}

	[Fact]
	public static void TestDisplacementsAtCantileverFibers()
	{
		// Create model
		int numElementsX = 20, numElementsY = 5; // the middle fiber will intersect some elements
		var model = CreateModel(numElementsX, numElementsY);

		// Run linear static analysis
		var freeDisplacements = RunAnalysis(model);

		// Find displacements at various points along the bottom, middle and top fiber
		var displacementOutput = new Quad4DisplacementOutput(model, freeDisplacements);
		var
			offset = 1E-3; // avoid searching at exactly the edge of elements, since accuracy loss may vause problems.
		double[] samplesX = [0 + offset, 0.25 * length, 0.5 * length, 0.75 * length, length - offset];

		// Bottom fiber
		var bottomY = 0.0 + offset;
		var displacementsBottomFiber = new double[samplesX.Length];
		for (var i = 0; i < samplesX.Length; ++i)
		{
			var u = displacementOutput.FindDisplacementsAt(samplesX[i], bottomY);
			displacementsBottomFiber[i] = u[1];
		}

		// Middle fiber
		var middleY = 0.5 * height;
		var displacementsMiddleFiber = new double[samplesX.Length];
		for (var i = 0; i < samplesX.Length; ++i)
		{
			var u = displacementOutput.FindDisplacementsAt(samplesX[i], middleY);
			displacementsMiddleFiber[i] = u[1];
		}

		// Top fiber
		var topY = height - offset;
		var displacementsTopFiber = new double[samplesX.Length];
		for (var i = 0; i < samplesX.Length; ++i)
		{
			var u = displacementOutput.FindDisplacementsAt(samplesX[i], topY);
			displacementsTopFiber[i] = u[1];
		}

		//Test solution
		for (var i = 0; i < samplesX.Length - 1; ++i)
		{
			Assert.True(Math.Abs(displacementsBottomFiber[i]) < Math.Abs(displacementsBottomFiber[i + 1]));
			Assert.True(Math.Abs(displacementsMiddleFiber[i]) < Math.Abs(displacementsMiddleFiber[i + 1]));
			Assert.True(Math.Abs(displacementsTopFiber[i]) < Math.Abs(displacementsTopFiber[i + 1]));
		}
	}

	private static Model CreateModel(int numElementsX, int numElementsY)
	{
		// Create mesh
		var meshGenerator = new UniformMeshGenerator2D(0, 0, length, height, numElementsX, numElementsY);
		var (nodes, elements) = meshGenerator.CreateMesh();

		// Create model and add the nodes
		var model = new Model();
		foreach (var node in nodes) model.NodesDictionary.Add(node.ID, node);

		// Find nodes on the left side and constrain their displacement
		var leftNodes = model.FindNodesWithX(0);
		foreach (var node in leftNodes) node.Constraints.AddRange([DOFType.X, DOFType.Y]);

		// Find nodes on the right side and apply load
		var rightNodes = model.FindNodesWithX(length);
		var loadPerNode = totalLoad / rightNodes.Count;
		foreach (var node in rightNodes)
			model.Loads.Add(new Load { Amount = loadPerNode, Node = node, DOF = DOFType.Y });

		// Create Quad4 elements and their material
		var material = new ElasticMaterial2D(StressState2D.PlaneStress)
		{
			YoungModulus = youngModulus,
			PoissonRatio = poissonRatio
		};
		foreach (var element in elements)
		{
			element.ElementType = new Quad4(material) { Thickness = thickness };
			model.ElementsDictionary.Add(element.ID, element);
		}

		// Finalize model creation
		model.ConnectDataStructures();
		return model;
	}

	private static Vector RunAnalysis(Model model)
	{
		// Setup analysis
		var linearSystem = new SkylineLinearSystem(model.Forces);
		var solver = new SolverSkyline(linearSystem);
		var provider = new ProblemStructural(model);
		var childAnalyzer = new LinearAnalyzer(solver);
		var parentAnalyzer = new StaticAnalyzer(provider, childAnalyzer, linearSystem);

		parentAnalyzer.BuildMatrices();
		parentAnalyzer.Initialize();
		parentAnalyzer.Solve();

		return linearSystem.Solution;
	}
}
