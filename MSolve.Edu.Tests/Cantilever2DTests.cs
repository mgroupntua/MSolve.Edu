using System;
using System.Collections.Generic;
using MSolve.Edu.Analyzers;
using MSolve.Edu.FEM;
using MSolve.Edu.FEM.Elements;
using MSolve.Edu.FEM.Entities;
using MSolve.Edu.FEM.Material;
using MSolve.Edu.FEM.Mesh;
using MSolve.Edu.LinearAlgebra;
using MSolve.Edu.Solvers;
using Xunit;

namespace MSolve.Edu.Tests
{
    public class Cantilever2DTests
    {
        [Fact]
        public void TestQuad4LinearCantileverExample()
        {
            // Problem definition
            double length = 5.0, height = 1.0, thickness = 1.0;
            double youngModulus = 2E7, poissonRatio = 0.3;
            double totalLoad = -100;

            // Create mesh
            int numElementsX = 20, numElementsY = 4;
            var meshGenerator = new UniformMeshGenerator2D(0, 0, length, height, numElementsX, numElementsY);
            (Node[] nodes, Element[] elements) = meshGenerator.CreateMesh();

            // Create model and add the nodes
            var model = new Model();
            foreach (Node node in nodes) model.NodesDictionary.Add(node.ID, node);

            // Find nodes on the left side and constrain their displacement
            var leftNodes = model.FindNodesWithX(0);
            foreach (Node node in leftNodes) node.Constraints.AddRange(new[] { DOFType.X, DOFType.Y });

            // Find nodes on the right side and apply load
            var rightNodes = model.FindNodesWithY(length);
            double loadPerNode = totalLoad / rightNodes.Count;
            foreach (Node node in rightNodes)
            {
                model.Loads.Add(new Load() { Amount = loadPerNode, Node = node, DOF = DOFType.Y });
            }

            // Create Quad4 elements and their material
            var material = new ElasticMaterial2D(StressState2D.PlaneStress) 
            { 
                YoungModulus = youngModulus, PoissonRatio = poissonRatio 
            };
            foreach (Element element in elements)
            {
                element.ElementType = new Quad4(material) { Thickness = thickness };
                model.ElementsDictionary.Add(element.ID, element);
            }

            // Finalize model creation
            model.ConnectDataStructures();

            // Run linear static analysis
            Vector displacements = RunAnalysis(model);

            // Test solution
            double momentOfInertia = 1.0 / 12.0 * thickness * Math.Pow(height, 3.0);
            double expectedDisplacement = totalLoad * Math.Pow(length, 3.0) / (3.0 * youngModulus * momentOfInertia);
            double minDisplacement = double.MaxValue;
            for (int i = 0; i < displacements.Length; i++)
            {
                if (displacements[i] < minDisplacement) minDisplacement = displacements[i];
            }
            Assert.Equal(expectedDisplacement, minDisplacement, 2);
        }

        private Vector RunAnalysis(Model model)
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
