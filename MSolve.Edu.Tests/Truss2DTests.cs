using MSolve.Edu.Analyzers;
using MSolve.Edu.FEM;
using MSolve.Edu.FEM.Elements;
using MSolve.Edu.FEM.Entities;
using MSolve.Edu.Solvers;
using Xunit;

namespace MSolve.Edu.Tests
{
    public class Truss2DTests
    {
        [Fact]
        private static void TestLinearTrussExample()
        {
            // Model and node creation
            Model model = new Model();
            model.NodesDictionary.Add(1, new Node { ID = 1, X = 0, Y = 0 });
            model.NodesDictionary.Add(2, new Node { ID = 2, X = 0, Y = 40 });
            model.NodesDictionary.Add(3, new Node { ID = 3, X = 40, Y = 40 });
            // Constrain bottom nodes of the model and add loads
            model.NodesDictionary[1].Constraints.AddRange(new[] { DOFType.X, DOFType.Y });
            model.NodesDictionary[2].Constraints.AddRange(new[] { DOFType.X, DOFType.Y });
            model.Loads.Add(new Load() { Amount = 500, Node = model.NodesDictionary[3], DOF = DOFType.X });
            model.Loads.Add(new Load() { Amount = 300, Node = model.NodesDictionary[3], DOF = DOFType.Y });

            var element1 = new Element() { ID = 1 };
            element1.ElementType = new Truss2D(1e7) { Density = 1, SectionArea = 1.5 };
            var element2 = new Element() { ID = 2 };
            element2.ElementType = new Truss2D(1e7) { Density = 1, SectionArea = 1.5 };

            element1.AddNode(model.NodesDictionary[1]);
            element1.AddNode(model.NodesDictionary[3]);

            element2.AddNode(model.NodesDictionary[2]);
            element2.AddNode(model.NodesDictionary[3]);

            model.ElementsDictionary.Add(element1.ID, element1);
            model.ElementsDictionary.Add(element2.ID, element2);

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

            Assert.Equal(0.00053333333333333336, linearSystem.Solution[0], 10);
            Assert.Equal(0.0017294083664636196, linearSystem.Solution[1], 10);
        }
    }
}
