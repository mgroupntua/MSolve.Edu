using MSolve.Edu.Analyzers;
using MSolve.Edu.FEM;
using MSolve.Edu.FEM.Elements;
using MSolve.Edu.FEM.Entities;
using MSolve.Edu.FEM.Material;
using MSolve.Edu.Solvers;
using Xunit;

namespace MSolve.Edu.Tests
{
	public class Quad4Tests
	{
		[Fact]
		public void TestQuad4LinearCantileverExample()
		{
            // Model and node creation
            var model = new Model();
			model.NodesDictionary.Add(1, new Node { ID = 1, X = 0.0, Y = 0.0, Z = 0.0 });
            model.NodesDictionary.Add(2, new Node { ID = 2, X = 10.0, Y = 0.0, Z = 0.0 });
            model.NodesDictionary.Add(3, new Node { ID = 3, X = 10.0, Y = 10.0, Z = 0.0 });
            model.NodesDictionary.Add(4, new Node { ID = 4, X = 0.0, Y = 10.0, Z = 0.0 });
			// Constrain bottom nodes of the model and add loads
			model.NodesDictionary[1].Constraints.AddRange(new[] { DOFType.X, DOFType.Y });
            model.NodesDictionary[4].Constraints.AddRange(new[] { DOFType.X, DOFType.Y });
            model.Loads.Add(new Load() { Amount = 500, Node = model.NodesDictionary[2], DOF = DOFType.X });
            model.Loads.Add(new Load() { Amount = 500, Node = model.NodesDictionary[3], DOF = DOFType.X });

            // Create Quad4 element and its material
            var material = new ElasticMaterial2D(StressState2D.PlaneStress) { YoungModulus = 3.76, PoissonRatio = 0.3779 };
            var quad = new Quad4(material) { Thickness = 1 };
            // Create the element connectivity
            var element = new Element() { ID = 1, ElementType = quad };
			element.AddNode(model.NodesDictionary[1]);
			element.AddNode(model.NodesDictionary[2]);
			element.AddNode(model.NodesDictionary[3]);
			element.AddNode(model.NodesDictionary[4]);
			// Add quad element to the element and subdomains dictionary of the model
			model.ElementsDictionary.Add(element.ID, element);

            model.ConnectDataStructures();

            // Setup
			var linearSystem = new SkylineLinearSystem(model.Forces);
			var solver = new SolverSkyline(linearSystem);
            var provider = new ProblemStructural(model);
            var childAnalyzer = new LinearAnalyzer(solver);
            var parentAnalyzer = new StaticAnalyzer(provider, childAnalyzer, linearSystem);

			parentAnalyzer.BuildMatrices();
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			var expectedSolution = new double[] { 253.13237596153559, 66.567582057178811, 253.13237596153553, -66.567582057178811 };
			for (int i = 0; i < expectedSolution.Length; i++)
				Assert.Equal(expectedSolution[i], linearSystem.Solution[i], 12);
		}
	}
}
