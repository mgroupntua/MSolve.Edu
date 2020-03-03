using System;
using System.Collections.Generic;
using System.Linq;

namespace MSolve.Edu.FEM.Entities
{
	public class Model
	{
		/// <summary>
		/// Dictionary that contains all dofs of the model.
		/// </summary>
		public Dictionary<int, Node> NodesDictionary { get; } = new Dictionary<int, Node>();

		/// <summary>
		/// Dictionary that contains all elements of the model.
		/// </summary>
		public Dictionary<int, Element> ElementsDictionary { get; } = new Dictionary<int, Element>();

		/// <summary>
		/// Return a list with the nodes of the model.
		/// </summary>
		public IList<Node> Nodes { get => NodesDictionary.Values.ToList<Node>(); }

		/// <summary>
		/// Return a list with the elements of the model.
		/// </summary>
		public IList<Element> Elements { get => ElementsDictionary.Values.ToList<Element>(); }

		/// <summary>
		/// Return a list with the Loads of the model.
		/// </summary>
		public IList<Load> Loads { get; } = new List<Load>();

		/// <summary>
		/// Returns the force vector of the model.
		/// </summary>
		public double[] Forces { get; private set; }

		/// <summary>
		/// Dictionary that links node.ID and DOFType with the equivalent global nodal dof number.
		/// </summary>
		public Dictionary<int, Dictionary<DOFType, int>> NodalDOFsDictionary { get; } = new Dictionary<int, Dictionary<DOFType, int>>();

		/// <summary>
		/// Gets the total dofs of the model.
		/// </summary>
		public int TotalDOFs { get; private set; } = 0;

		/// <summary>
		/// Builds the element dictionary of each node.
		/// </summary>
		private void BuildElementDictionaryOfEachNode()
		{
			foreach (Element element in ElementsDictionary.Values)
			foreach (Node node in element.Nodes)
				node.ElementsDictionary.Add(element.ID, element);
		}

		/// <summary>
		/// Enumerates the degrees of freedom of the model.
		/// </summary>
		private void EnumerateGlobalDOFs()
		{
			TotalDOFs = 0;
			Dictionary<int, List<DOFType>> nodalDOFTypesDictionary = new Dictionary<int, List<DOFType>>();
			foreach (Element element in ElementsDictionary.Values)
			{
				for (int i = 0; i < element.Nodes.Count; i++)
				{
					if (!nodalDOFTypesDictionary.ContainsKey(element.Nodes[i].ID))
						nodalDOFTypesDictionary.Add(element.Nodes[i].ID, new List<DOFType>());
					nodalDOFTypesDictionary[element.Nodes[i].ID]
						.AddRange(element.ElementType.DOFEnumerator.GetDOFTypesForDOFEnumeration(element)[i]);
				}
			}

			foreach (Node node in NodesDictionary.Values)
			{
				Dictionary<DOFType, int> dofsDictionary = new Dictionary<DOFType, int>();
				foreach (DOFType dofType in nodalDOFTypesDictionary[node.ID].Distinct<DOFType>())
				{
					int dofID = 0;
					foreach (DOFType constraint in node.Constraints)
					{
						if (constraint == dofType)
						{
							dofID = -1;
							break;
						}
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
		/// Enumerates the dofs and initializes the force vector.
		/// </summary>
		private void EnumerateDOFs()
		{
			EnumerateGlobalDOFs();
			Forces = new double[TotalDOFs];
		}

		/// <summary>
		/// Assigns the loads to the force vector.
		/// </summary>
		private void AssignNodalLoads()
		{
			Array.Clear(this.Forces, 0, this.Forces.Length);
			foreach (Load load in Loads)
			{
				int dof = this.NodalDOFsDictionary[load.Node.ID][load.DOF];
				if (dof >= 0)
					this.Forces[dof] = load.Amount;
			}
		}

		/// <summary>
		/// Assigns the loads to the force vector.
		/// </summary>
		public void AssignLoads()
		{
			AssignNodalLoads();
		}

		/// <summary>
		/// Connects Elements, Nodes and Assign Loads.
		/// </summary>
		public void ConnectDataStructures()
		{
            BuildElementDictionaryOfEachNode();
			EnumerateDOFs();
			AssignLoads();
		}

		/// <summary>
		/// Clears all model data.
		/// </summary>
		public void Clear()
		{
			Loads.Clear();
			ElementsDictionary.Clear();
			NodesDictionary.Clear();
			NodalDOFsDictionary.Clear();
		}

        /// <summary>
        /// If no such node exists, null will be returned.
        /// </summary>
        public Node FindNode(double x, double y, double meshTolerance = 1E-6)
        {
            foreach (Node node in NodesDictionary.Values)
            {
                if ((Math.Abs(node.X - x) <= meshTolerance) && (Math.Abs(node.Y - y) <= meshTolerance)) return node;
            }
            return null;
        }

        /// <summary>
        /// If no such nodes exists, an empty list will be returned.
        /// </summary>
        public List<Node> FindNodesWithX(double x, double meshTolerance = 1E-6)
        {
            var targetNodes = new List<Node>();
            foreach (Node node in NodesDictionary.Values)
            {
                if (Math.Abs(node.X - x) <= meshTolerance) targetNodes.Add(node);
            }
            return targetNodes;
        }

        /// <summary>
        /// If no such nodes exists, an empty list will be returned.
        /// </summary>
        public List<Node> FindNodesWithY(double y, double meshTolerance = 1E-6)
        {
            var targetNodes = new List<Node>();
            foreach (Node node in NodesDictionary.Values)
            {
                if (Math.Abs(node.Y - y) <= meshTolerance) targetNodes.Add(node);
            }
            return targetNodes;
        }
    }
}