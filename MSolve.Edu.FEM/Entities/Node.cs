namespace MSolve.Edu.FEM.Entities;

using System.Collections.Generic;

/// <summary>
///     An enum containing all possible degrees of freedom that a node can have in the context of MSolve.
/// </summary>
public enum DOFType
{
	/// <summary>Default value</summary>
	Unknown = 0,

	/// <summary>Displacement X</summary>
	X = 1,

	/// <summary>Displacement Y</summary>
	Y = 2,

	/// <summary>Displacement Z</summary>
	Z = 3,

	/// <summary>Rotation Z</summary>
	RotZ = 4
}

/// <summary>
/// </summary>
public class Node
{
	/// <summary>
	///     A list that contains all constrained <see cref="DOFType" /> of a node.
	/// </summary>
	public List<DOFType> Constraints { get; } = [];

	/// <summary>
	///     A dictionary that contains all elements connected to this node.
	/// </summary>
	public Dictionary<int, Element> ElementsDictionary { get; } = [];

	/// <summary>
	///     Gets or sets the element ID.
	/// </summary>

	public int ID { get; set; }

	/// <summary>
	///     Gets or sets the cartesian X coordinate of the node.
	/// </summary>
	public double X { get; set; }

	/// <summary>
	///     Gets or sets the cartesian Y coordinate of the node.
	/// </summary>
	public double Y { get; set; }

	/// <summary>
	///     Gets or sets the cartesian Z coordinate of the node.
	/// </summary>
	public double Z { get; set; }

	/// <summary>
	///     Converts to string.
	/// </summary>
	/// <returns>
	///     A <see cref="string" /> that represents this instance.
	/// </returns>
	public override string ToString()
	{
		var header = string.Format("{0}: ({1}, {2}, {3})", ID, X, Y, Z);
		var constraintsDescripton = string.Empty;
		foreach (var c in Constraints)
		{
			var con = string.Empty;
			switch (c)
			{
				case DOFType.RotZ:
					con = "rZ";
					break;
				case DOFType.Unknown:
					con = "?";
					break;
				case DOFType.X:
					con = "X";
					break;
				case DOFType.Y:
					con = "Y";
					break;
				case DOFType.Z:
					con = "Z";
					break;
			}

			constraintsDescripton += c + ", ";
		}

		constraintsDescripton = constraintsDescripton.Length > 1
			? constraintsDescripton[..^2]
			: constraintsDescripton;

		return string.Format("{0} - Con ({1})", header, constraintsDescripton);
	}
}
