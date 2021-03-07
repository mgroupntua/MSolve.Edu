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


namespace MSolve.Edu.Tests.Lectures
{
    public static class Fem1Example
    {
        private const double length = 5.0, height = 1.0, thickness = 1.0;
        private const double youngModulus = 2E7, poissonRatio = 0.3;
        private const double totalLoad = -100;

        public static void RunExample()
        {
            int numElementsX = 15, numElementsY = 3;

            // Create model
            Model model = CreateModel(numElementsX, numElementsY);

            // Run linear static analysis
            Vector freeDisplacements = RunAnalysis(model);

            // Output
            // First of all prepare the object that will calculate the displacements at arbitrary points
            var displacementOutput = new Quad4DisplacementOutput(model, freeDisplacements);

            // Plot displacements
            displacementOutput.PlotDisplacementField(@"C:\Users\Serafeim\Desktop\temp\displacements.vtk");

            // *******************************************************************************************
            // *******************************************************************************************
            // Excercise: Refine the mesh until the solutions converge
            // *******************************************************************************************
            // *******************************************************************************************
            Console.WriteLine("Max uy = " + displacementOutput.FindMaxDisplacementY());


            // *******************************************************************************************
            // *******************************************************************************************
            // Excercise: Print the displacements ux & uy along other fibers
            // *******************************************************************************************
            // *******************************************************************************************
            PrintDisplacementsAlongBottomFiber(model, displacementOutput);

        }

        private static void ApplyCantileverBoundaryConditions(Model model)
        {
            // Find nodes on the left side and constrain their displacement
            var leftNodes = model.FindNodesWithX(0);
            foreach (Node node in leftNodes) node.Constraints.AddRange(new[] { DOFType.X, DOFType.Y });

            // Find nodes on the right side and apply load
            var rightNodes = model.FindNodesWithX(length);
            double loadPerNode = totalLoad / rightNodes.Count;
            foreach (Node node in rightNodes)
            {
                model.Loads.Add(new Load() { Amount = loadPerNode, Node = node, DOF = DOFType.Y });
            }
        }

        private static Model CreateModel(int numElementsX, int numElementsY)
        {
            // Create mesh
            var meshGenerator = new UniformMeshGenerator2D(0, 0, length, height, numElementsX, numElementsY);
            (Node[] nodes, Element[] elements) = meshGenerator.CreateMesh();

            // Create model and add the nodes
            var model = new Model();
            foreach (Node node in nodes) model.NodesDictionary.Add(node.ID, node);

            // Create Quad4 elements and their material
            var material = new ElasticMaterial2D(StressState2D.PlaneStress)
            {
                YoungModulus = youngModulus,
                PoissonRatio = poissonRatio
            };
            foreach (Element element in elements)
            {
                element.ElementType = new Quad4(material) { Thickness = thickness };
                model.ElementsDictionary.Add(element.ID, element);
            }

            // Boundary conditions:
            // *******************************************************************************************
            // *******************************************************************************************
            // Excercise: Write your own boundary conditions for simply supported beam 
            // *******************************************************************************************
            // *******************************************************************************************
            ApplyCantileverBoundaryConditions(model);



            // Finalize model creation
            model.ConnectDataStructures();
            return model;
        }

        private static void PrintDisplacementsAlongBottomFiber(Model model, Quad4DisplacementOutput displacementOutput)
        {

            // Define the points where the displacements will be calculated
            double offset = 1E-3; // avoid searching at exactly the edge of the domain, since accuracy loss may cause problems
            double[,] points =
            {
                { 0 + offset, 0 + offset },
                { 0.25 * length, 0 + offset },
                { 0.5 * length, 0 + offset },
                { 0.75 * length, 0 + offset },
                { length - offset, 0 + offset }
            };

            // Calculate the displacements at these points
            int numPoints = points.GetLength(0);
            var displacementsYBottomFiber = new double[numPoints];
            for (int i = 0; i < numPoints; ++i)
            {
                double[] u = displacementOutput.FindDisplacementsAt(points[i, 0], points[i, 1]);
                displacementsYBottomFiber[i] = u[1];
            }

            // Print the displacements
            Console.WriteLine("Displacements y of fiber at y = 0:");
            for (int i = 0; i < numPoints; ++i) Console.Write(" " + displacementsYBottomFiber[i]);
            Console.WriteLine();
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
}
