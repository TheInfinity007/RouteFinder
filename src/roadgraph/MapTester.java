// By Infinity

package roadgraph;

import java.util.List;

import geography.GeographicPoint;
import util.GraphLoader;

public class MapTester {

	public static void main(String[] args) {
		System.out.println("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.println("DONE. \nLoading the map...");
		
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
		
		System.out.println("DONE.");
		
		System.out.println("Num nodes: " + theMap.getNumVertices());
		System.out.println("Num edges: " + theMap.getNumEdges());
		
		List<GeographicPoint> route = theMap.bfs(new GeographicPoint(1.0, 1.0), new GeographicPoint(8.0, -1.0));
		System.out.println(route);
	}

}

/*
Class: MapGraph
Modifications made to MapGraph (what and why): I've completed all the given methods and also done the bfs function. Create a helper function reConstructPath() to create a path from the start node to the goal node.
Class: MapNode
Purpose and description of class:  This class is used to store the details of a node. like its location and a set to store its edges. I've made the setters and getters for the class variables. I've also created a function getNeighbours() which returns all the neighbors node of a node.
Class: MapEdge
Purpose and description of class: This class is used to store the details of a edge. It has class variables for the nodes present at both ends of an edge, streetName and streetType and the length of a edge. Made some getters and setters of these. I've created some helper function which give the other end node of an edge when you give one node.
Overall Design Justification: I've created two classes namely mapNode and edgeNode for the vertices and nodes.  mapNode stores the details of the node while the edgeNode stores the details of the edges. In addition to getters and setters, I've added some more helper methods. In the MapGraph Class i've completed the functions and created a helper function. 
*/


  

