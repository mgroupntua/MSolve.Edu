namespace MSolve.Edu.FEM.Entities
{
	/// <summary>
	/// Defines a load for a specific degree of freedom of a node
	/// </summary>
	public class Load
	{
		/// <summary>
		/// Gets or sets the node of the load.
		/// </summary>
		public Node Node { get; set; }
		/// <summary>
		/// Gets or sets the DOFType the load will be applied to.
		/// </summary>
		public DOFType DOF { get; set; }

		/// <summary>
		/// Gets or sets the magnitude of the load.
		/// </summary>
		public double Amount { get; set; }
	}
}
