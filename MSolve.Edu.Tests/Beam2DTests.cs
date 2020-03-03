using MSolve.Edu.Analyzers;
using MSolve.Edu.FEM;
using MSolve.Edu.FEM.Elements;
using MSolve.Edu.FEM.Entities;
using MSolve.Edu.Solvers;
using Xunit;

namespace MSolve.Edu.Tests
{
    public class Beam2DTests
    {
        [Fact]
        public void TestEulerBeam2DLinearBendingExample()
        {
            // Model and node creation
            var model = new Model();
            model.NodesDictionary.Add(1, new Node { ID = 1, X = 0.0, Y = 0.0, Z = 0.0 });
            model.NodesDictionary.Add(2, new Node { ID = 2, X = 100.0, Y = 0.0, Z = 0.0 });
            model.NodesDictionary.Add(3, new Node { ID = 3, X = 200.0, Y = 0.0, Z = 0.0 });
            // Constrain bottom node and add nodal load value
            model.NodesDictionary[1].Constraints.AddRange(new[] { DOFType.X, DOFType.Y, DOFType.Z, DOFType.RotZ });
            model.Loads.Add(new Load() { Amount = 2000, Node = model.NodesDictionary[3], DOF = DOFType.Y });

            // Generate elements of the structure
            for (int iElem = 0; iElem < 2; iElem++)
            {
                // Create new Beam2D section and element
                var elementType = new EulerBeam2D(2.1e4)
                {
                    Density = 7.85,
                    SectionArea = 91.04,
                    MomentOfInertia = 8091.00
                };
                // Create the element connectivity
                var element = new Element() { ID = iElem + 1 };
                element.AddNode(model.NodesDictionary[iElem + 1]);
                element.AddNode(model.NodesDictionary[iElem + 2]);
                element.ElementType = elementType;
                // Add beam element to the element and subdomains dictionary of the model
                model.ElementsDictionary.Add(element.ID, element);
            }
            // Needed in order to make all the required data structures
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

            // Tests if calculated solution meets expected
            var expectedSolution = new double[] { 0, 9.80905689841542, 0.17656302417147754, 0, 31.388982074929341, 0.23541736556197002 };
            for (int i = 0; i < expectedSolution.Length; i++)
                Assert.Equal(expectedSolution[i], linearSystem.Solution[i], 12);
        }
    }
}