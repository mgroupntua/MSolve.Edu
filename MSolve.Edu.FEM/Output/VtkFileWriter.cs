using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MSolve.Edu.FEM.Entities;

namespace MSolve.Edu.FEM.Output.Paraview
{
	/// <summary>
	/// Appends meshes, scalars, vectors and tensor fields to .vtk output files. Then these files can be opened in Paraview.
	/// </summary>
	public class VtkFileWriter : IDisposable
	{
		public const string vtkReaderVersion = "4.1";
		private readonly StringBuilder builder;
		private bool writeFieldsNext;

		public VtkFileWriter()
		{
			this.builder = new StringBuilder();
			builder.Append("# vtk DataFile Version ");
			builder.AppendLine(vtkReaderVersion);
			builder.AppendLine("vtk output");
			builder.Append("ASCII\n\n");
			writeFieldsNext = false;
		}

		public void Dispose()
		{
			if (builder != null) builder.Clear();
		}

		public override string ToString() => builder.ToString();

		public void ToFile(string path)
		{
			using (var writer = new StreamWriter(path))
			{
				writer.Write(this.ToString());
			}
		}

		/// <summary>
		/// Outputs a mesh.
		/// </summary>
		/// <param name="points">They must be sorted on their IDs, which start from 0 and are contiguous.</param>
		/// <param name="elements"></param>
		public void WriteMesh(IList<Node> points, IList<Element> elements)
		{
			if (writeFieldsNext) throw new InvalidOperationException("A mesh has already been written.");

			// Vertices 
			builder.AppendLine("DATASET UNSTRUCTURED_GRID");
			builder.AppendLine($"POINTS {points.Count} double");
			foreach (var point in points)
			{
				builder.AppendLine($"{point.X} {point.Y} {point.Z}");
			}

			// Cell connectivity
			int cellDataCount = 0;
			foreach (var cell in elements) cellDataCount += 1 + cell.Nodes.Count;
			builder.AppendLine($"\nCELLS {elements.Count} {cellDataCount}");
			foreach (var cell in elements)
			{
				builder.Append(cell.Nodes.Count);
				foreach (var point in cell.Nodes)
				{
					builder.Append(' ');
					builder.Append(point.ID);
				}
				builder.AppendLine();
			}

			// Element types
			builder.AppendLine("\nCELL_TYPES " + elements.Count);
			foreach (var cell in elements)
			{
				builder.Append((int)(cell.ElementType.Code));
				builder.AppendLine();
			}
		}

		/// <summary>
		/// Outputs a scalar field.
		/// </summary>
		/// <param name="fieldName"></param>
		/// <param name="pointValues">They must be in the exact same order as the nodes.</param>
		public void WriteScalarField(string fieldName, double[] pointValues)
		{
			WriterFieldsHeader(pointValues.Length);
			builder.AppendLine($"SCALARS {fieldName} double 1");
			builder.AppendLine("LOOKUP_TABLE default");
			for (int i = 0; i < pointValues.Length; ++i)
			{
				builder.Append(pointValues[i]);
				builder.AppendLine();
			}
			builder.AppendLine();
		}

		/// <summary>
		/// Tensor components are written as independent scalar fields, as I haven't found any advantage in using tensor datasets 
		/// in Paraview. By exporting each one as a different scalar field, better naming can be enforced: instead of 0, 1, etc. 
		/// indexing for each tensor component.
		/// </summary>
		/// <param name="fieldName">
		/// Each component will be prefixed by it. E.g. fieldName = "S": S11, S22, S33, S12, S23, S31.
		/// </param>
		/// <param name="pointValues">
		/// Each row correspond to a different node. They must be in the exact same order as the nodes.
		/// Columns 0, 1, 2, 3, 4, 5 are the tensor entries T11, T22, T33, T12, T23, T31 respectively.
		/// </param>
		public void WriteTensorField(string fieldName, IReadOnlyList<double[]> pointTensors, bool is3D)
		{
			int numPoints = pointTensors.Count;
			WriterFieldsHeader(numPoints);

			// Component 11
			builder.AppendLine($"SCALARS {fieldName}_11 double 1");
			builder.AppendLine("LOOKUP_TABLE default");
			for (int i = 0; i < numPoints; ++i)
			{
				builder.Append(pointTensors[i][0]);
				builder.AppendLine();
			}
			builder.AppendLine();

			// Component 22
			builder.AppendLine($"SCALARS {fieldName}_22 double 1");
			builder.AppendLine("LOOKUP_TABLE default");
			for (int i = 0; i < numPoints; ++i)
			{
				builder.Append(pointTensors[i][1]);
				builder.AppendLine();
			}
			builder.AppendLine();

			// Component 12
			builder.AppendLine($"SCALARS {fieldName}_12 double 1");
			builder.AppendLine("LOOKUP_TABLE default");
			for (int i = 0; i < numPoints; ++i)
			{
				builder.Append(pointTensors[i][2]);
				builder.AppendLine();
			}
			builder.AppendLine();

			if (is3D)
			{
				// Component 33
				builder.AppendLine($"SCALARS {fieldName}_33 double 1");
				builder.AppendLine("LOOKUP_TABLE default");
				for (int i = 0; i < numPoints; ++i)
				{
					builder.Append(pointTensors[i][3]);
					builder.AppendLine();
				}
				builder.AppendLine();
			
				// Component 23
				builder.AppendLine($"SCALARS {fieldName}_23 double 1");
				builder.AppendLine("LOOKUP_TABLE default");
				for (int i = 0; i < numPoints; ++i)
				{
					builder.Append(pointTensors[i][4]);
					builder.AppendLine();
				}
				builder.AppendLine();

				// Component 31
				builder.AppendLine($"SCALARS {fieldName}_31 double 1");
				builder.AppendLine("LOOKUP_TABLE default");
				for (int i = 0; i < numPoints; ++i)
				{
					builder.Append(pointTensors[i][5]);
					builder.AppendLine();
				}
				builder.AppendLine();
			}
		}

		/// <summary>
		/// Outputs a vector field.
		/// </summary>
		/// <param name="fieldName"></param>
		/// <param name="pointValues">
		/// Each row correspond to a different node. They must be in the exact same order as the nodes. 
		/// Columns 0 and 1 are the vector entries.
		/// </param>
		public void WriteVectorField(string fieldName, IList<double[]> pointVectors, bool is3D)
		{
			WriterFieldsHeader(pointVectors.Count);
			builder.AppendLine($"VECTORS {fieldName} double");
			if (is3D)
			{
				for (int i = 0; i < pointVectors.Count; ++i)
				{
					builder.AppendLine($"{pointVectors[i][0]} {pointVectors[i][1]} {pointVectors[i][2]}");
				}
			}
			else
			{
				for (int i = 0; i < pointVectors.Count; ++i)
				{
					builder.AppendLine($"{pointVectors[i][0]} {pointVectors[i][1]} 0.0");
				}
			}
			builder.AppendLine();
		}

		private void WriterFieldsHeader(int numPoints)
		{
			if (!writeFieldsNext) // Fields header
			{
				builder.Append("\n\n");
				builder.AppendLine("POINT_DATA " + numPoints);
				writeFieldsNext = true;
			}
		}
	}
}
