using System.Collections.Generic;
using System.Linq;
using MSolve.Edu.FEM.Elements;
using MSolve.Edu.LinearAlgebra;

namespace MSolve.Edu.FEM.Entities
{
	/// <summary>
	/// Defines the connectivity of a finite element.
	/// </summary>
	public class Element
	{
		/// <summary>
		/// Gets or sets the element ID.
		/// </summary>
		public int ID { get; set; }

		/// <summary>
		/// Gets or sets element DOFs.
		/// </summary>
		public int[] DOFs { get; set; }

		/// <summary>
		/// Gets or sets a concrete implementation the class.
		/// </summary>
		public IElementType ElementType { get; set; }

		/// <summary>
		/// A dictionary that contains the element nodes by IDs.
		/// </summary>
		public Dictionary<int, Node> NodesDictionary { get; } = new Dictionary<int, Node>();

		/// <summary>
		/// Provides a list with the nodes of the element.
		/// </summary>
		public IList<Node> Nodes { get => NodesDictionary.Values.ToList<Node>(); }

		/// <summary>
		/// Provides a list with the nodes of the element.
		/// </summary>
		public IList<Node> INodes
		{
			get
			{
				IList<Node> a = new List<Node>();
				foreach (var node in NodesDictionary.Values)
					a.Add(node);
				return a;
			}
		}

		/// <summary>
		/// Adds a node the element node dictionary.
		/// </summary>
		/// <param name="node">The node.</param>
		public void AddNode(Node node) => NodesDictionary.Add(node.ID, node);

		/// <summary>
		/// Adds a list of nodes to the element node dictionary.
		/// </summary>
		/// <param name="nodes">The nodes.</param>
		public void AddNodes(IList<Node> nodes)
		{
			foreach (Node node in nodes) AddNode(node);
		}
	}
}
