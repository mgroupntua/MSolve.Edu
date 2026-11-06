namespace MSolve.Edu.FEM.Output;

using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;

using MSolve.Edu.FEM.Elements;
using MSolve.Edu.FEM.Entities;
using MSolve.Edu.FEM.Output.Paraview;
using MSolve.Edu.LinearAlgebra;

public class Quad4DisplacementOutput
{
	private readonly Model model;
	private readonly Dictionary<int, double[]> nodalDisplacements;

	public Quad4DisplacementOutput(Model model, Vector solution)
	{
		this.model = model;

		nodalDisplacements = [];
		foreach (var node in model.NodalDOFsDictionary.Keys) nodalDisplacements[node] = new double[2];

		foreach (var node in model.NodalDOFsDictionary.Keys)
			foreach (var dofIndexPair in model.NodalDOFsDictionary[node])
			{
				var dof = dofIndexPair.Key;
				var index = dofIndexPair.Value;

				double displacement;
				if (index < 0) displacement = 0.0; // constrained dof
				else displacement = solution[dofIndexPair.Value]; // free dof

				if (dofIndexPair.Key == DOFType.X) nodalDisplacements[node][0] = displacement;
				else if (dofIndexPair.Key == DOFType.Y) nodalDisplacements[node][1] = displacement;
				else throw new NotImplementedException();
			}
	}

	public double[] FindDisplacementsAt(double x, double y)
	{
		var element = FindElementThatContains(x, y);
		var quad4 = (Quad4)element.ElementType;
		var nodes = element.Nodes.ToArray();

		// Calculate shape functions
		var inverseInterpolation = new InverseInterpolationQuad4(nodes);
		var naturalCoords = inverseInterpolation.TransformPointCartesianToNatural([x, y]);
		//double[] naturalCoords = quad4.GetNaturalCoordinates(element, new Node() { X = x, Y = y });
		var shapeFunctions = Quad4.CalcQ4Shape(naturalCoords[0], naturalCoords[1]);

		// Interpolate nodal displacements
		double ux = 0.0, uy = 0.0;
		for (var i = 0; i < nodes.Length; i++)
		{
			var nodeID = nodes[i].ID;
			var N = shapeFunctions[i];
			ux += N * nodalDisplacements[nodeID][0];
			uy += N * nodalDisplacements[nodeID][1];
		}

		return [ux, uy];
	}

	public double FindMaxDisplacementX()
	{
		var maxAbs = double.MinValue;
		var result = double.NaN;
		foreach (var u in nodalDisplacements.Values)
			if (Math.Abs(u[0]) > maxAbs)
			{
				result = u[0];
				maxAbs = Math.Abs(u[0]);
			}

		return result;
	}

	public double FindMaxDisplacementY()
	{
		var maxAbs = double.MinValue;
		var result = double.NaN;
		foreach (var u in nodalDisplacements.Values)
			if (Math.Abs(u[1]) > maxAbs)
			{
				result = u[1];
				maxAbs = Math.Abs(u[1]);
			}

		return result;
	}

	public void PlotDisplacementField(string path)
	{
		var allDisplacements = new List<double[]>();
		foreach (var node in model.Nodes) allDisplacements.Add(nodalDisplacements[node.ID]);

		var vtkWriter = new VtkFileWriter();
		vtkWriter.WriteMesh(model.Nodes, model.Elements);
		vtkWriter.WriteVectorField("displacements", allDisplacements, false);

		using var writer = new StreamWriter(path);
		writer.Write(vtkWriter.ToString());
	}

	private Element FindElementThatContains(double x, double y)
	{
		foreach (var element in model.Elements)
		{
			var nodes = element.Nodes;

			// Point-in-polygon
			int i, j;
			var isInside = false;
			for (i = 0, j = nodes.Count - 1; i < nodes.Count; j = i++)
				if (nodes[i].Y > y != nodes[j].Y > y &&
					x < (nodes[j].X - nodes[i].X) * (y - nodes[i].Y)
					/ (nodes[j].Y - nodes[i].Y) + nodes[i].X)
					isInside = !isInside;

			if (isInside) return element;
		}

		throw new ArgumentException("This point is outside all elements");
	}
}
