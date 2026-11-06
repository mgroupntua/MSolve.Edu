namespace MSolve.Edu.FEM.Entities;

using System;
using System.Collections.Generic;
using System.Linq;

public class Model
{
	/// <summary>
	///     Dictionary that contains all dofs of the model.
	/// </summary>
	public Dictionary<int, Node> NodesDictionary { get; } = [];

	/// <summary>
	///     Dictionary that contains all elements of the model.
	/// </summary>
	public Dictionary<int, Element> ElementsDictionary { get; } = [];

	/// <summary>
	///     Return a list with the nodes of the model.
	/// </summary>
	public IList<Node> Nodes => [.. NodesDictionary.Values];

	/// <summary>
	///     Return a list with the elements of the model.
	/// </summary>
	public IList<Element> Elements => [.. ElementsDictionary.Values];

	/// <summary>
	///     Return a list with the Loads of the model.
	/// </summary>
	public IList<Load> Loads { get; } = [];

	/// <summary>
	///     Returns the force vector of the model.
	/// </summary>
	public double[] Forces { get; private set; }

	/// <summary>
	///     Dictionary that links node.ID and DOFType with the equivalent global nodal dof number.
	/// </summary>
	public Dictionary<int, Dictionary<DOFType, int>> NodalDOFsDictionary { get; } = [];

	/// <summary>
	///     Gets the total dofs of the model.
	/// </summary>
	public int TotalDOFs { get; private set; }

	/// <summary>
	///     Builds the element dictionary of each node.
	/// </summary>
	private void BuildElementDictionaryOfEachNode()
	{
		foreach (var element in ElementsDictionary.Values)
			foreach (var node in element.Nodes)
				node.ElementsDictionary.Add(element.ID, element);
	}

	/// <summary>
	///     Enumerates the degrees of freedom of the model.
	/// </summary>
	private void EnumerateGlobalDOFs()
	{
		TotalDOFs = 0;
		var nodalDOFTypesDictionary = new Dictionary<int, List<DOFType>>();
		foreach (var element in ElementsDictionary.Values)
			for (var i = 0; i < element.Nodes.Count; i++)
			{
				if (!nodalDOFTypesDictionary.ContainsKey(element.Nodes[i].ID))
					nodalDOFTypesDictionary.Add(element.Nodes[i].ID, []);
				nodalDOFTypesDictionary[element.Nodes[i].ID]
					.AddRange(GenericDOFEnumerator.GetDOFTypesForDOFEnumeration(element)[i]);
			}

		foreach (var node in NodesDictionary.Values)
		{
			var dofsDictionary = new Dictionary<DOFType, int>();
			foreach (var dofType in nodalDOFTypesDictionary[node.ID].Distinct())
			{
				var dofID = 0;
				foreach (var constraint in node.Constraints)
					if (constraint == dofType)
					{
						dofID = -1;
						break;
					}

				if (dofID == 0)
				{
					dofID = TotalDOFs;
					TotalDOFs++;
				}

				dofsDictionary.Add(dofType, dofID);
			}

			NodalDOFsDictionary.Add(node.ID, dofsDictionary);
		}
	}

	/// <summary>
	///     Enumerates the dofs and initializes the force vector.
	/// </summary>
	private void EnumerateDOFs()
	{
		EnumerateGlobalDOFs();
		Forces = new double[TotalDOFs];
	}

	/// <summary>
	///     Assigns the loads to the force vector.
	/// </summary>
	private void AssignNodalLoads()
	{
		Array.Clear(Forces, 0, Forces.Length);
		foreach (var load in Loads)
		{
			var dof = NodalDOFsDictionary[load.Node.ID][load.DOF];
			if (dof >= 0)
				Forces[dof] = load.Amount;
		}
	}

	/// <summary>
	///     Assigns the loads to the force vector.
	/// </summary>
	public void AssignLoads() => AssignNodalLoads();

	/// <summary>
	///     Connects Elements, Nodes and Assign Loads.
	/// </summary>
	public void ConnectDataStructures()
	{
		BuildElementDictionaryOfEachNode();
		EnumerateDOFs();
		AssignLoads();
	}

	/// <summary>
	///     Clears all model data.
	/// </summary>
	public void Clear()
	{
		Loads.Clear();
		ElementsDictionary.Clear();
		NodesDictionary.Clear();
		NodalDOFsDictionary.Clear();
	}

	/// <summary>
	///     If no such node exists, null will be returned.
	/// </summary>
	public Node FindNode(double x, double y, double meshTolerance = 1E-6)
	{
		foreach (var node in NodesDictionary.Values)
			if (Math.Abs(node.X - x) <= meshTolerance && Math.Abs(node.Y - y) <= meshTolerance)
				return node;
		return null;
	}

	/// <summary>
	///     If no such nodes exists, an empty list will be returned.
	/// </summary>
	public List<Node> FindNodesWithX(double x, double meshTolerance = 1E-6)
	{
		var targetNodes = new List<Node>();
		foreach (var node in NodesDictionary.Values)
			if (Math.Abs(node.X - x) <= meshTolerance)
				targetNodes.Add(node);
		return targetNodes;
	}

	/// <summary>
	///     If no such nodes exists, an empty list will be returned.
	/// </summary>
	public List<Node> FindNodesWithY(double y, double meshTolerance = 1E-6)
	{
		var targetNodes = new List<Node>();
		foreach (var node in NodesDictionary.Values)
			if (Math.Abs(node.Y - y) <= meshTolerance)
				targetNodes.Add(node);
		return targetNodes;
	}
}
