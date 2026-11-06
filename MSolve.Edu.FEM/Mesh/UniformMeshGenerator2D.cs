namespace MSolve.Edu.FEM.Mesh;

using MSolve.Edu.FEM.Entities;

/// <summary>
///     Creates 2D meshes based on uniform rectilinear grids: the distance between two consecutive vertices for the same
///     axis is
///     constant. This distance may be different for each axis though. For now the cells are quadrilateral with 4 vertices
///     (rectangles in particular).
/// </summary>
public class UniformMeshGenerator2D
{
	private readonly double dx, dy;
	private readonly int elementsPerX, elementsPerY;
	private readonly double minX, minY;
	private readonly int nodesPerX, nodesPerY;

	public UniformMeshGenerator2D(double minX, double minY, double maxX, double maxY, int elementsPerX,
		int elementsPerY)
	{
		this.minX = minX;
		this.minY = minY;
		dx = (maxX - minX) / elementsPerX;
		dy = (maxY - minY) / elementsPerY;
		this.elementsPerX = elementsPerX;
		this.elementsPerY = elementsPerY;
		nodesPerX = this.elementsPerX + 1;
		nodesPerY = this.elementsPerY + 1;
	}

	/// <summary>
	///     Generates a uniform mesh with the dimensions and density defined in the constructor.
	/// </summary>
	public (Node[] nodes, Element[] elements) CreateMesh()
	{
		var nodes = CreateNodes();
		var elements = CreateElements(nodes);
		return (nodes, elements);
	}

	private Node[] CreateNodes()
	{
		var nodes = new Node[nodesPerY * nodesPerX];
		var id = 0;
		for (var j = 0; j < nodesPerY; ++j)
			for (var i = 0; i < nodesPerX; ++i)
			{
				nodes[id] = new Node { ID = id, X = minX + i * dx, Y = minY + j * dy };
				++id;
			}

		return nodes;
	}

	private Element[] CreateElements(Node[] allNodes)
	{
		var elements = new Element[elementsPerY * elementsPerX];
		for (var j = 0; j < elementsPerY; ++j)
			for (var i = 0; i < elementsPerX; ++i)
			{
				var elementID = j * elementsPerX + i;
				var firstNode = j * nodesPerX + i;
				Node[] nodesOfElement =
				[
				allNodes[firstNode], allNodes[firstNode + 1],
				allNodes[firstNode + nodesPerX + 1], allNodes[firstNode + nodesPerX]
			];

				var element = new Element { ID = elementID };
				element.AddNodes(nodesOfElement);
				elements[elementID] = element;
			}

		return elements;
	}
}
