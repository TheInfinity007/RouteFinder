/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private HashMap<GeographicPoint, MapNode> pointNodeMap;
	private HashSet<MapEdge> edges;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		pointNodeMap = new HashMap<GeographicPoint, MapNode>();
		edges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return pointNodeMap.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return pointNodeMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return edges.size();
	}

	
	
	public void addVertex(double latitude, double longitude) {
		GeographicPoint pt = new GeographicPoint(latitude, longitude);
		this.addVertex(pt);
	}
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if(location == null) return false;
		MapNode node = pointNodeMap.get(location);
		
		if(node == null) {
			node = new MapNode(location);
			pointNodeMap.put(location, node);
			return true;
		}
		
		System.out.println("Warning: Node at location " + location + " already exists in the graph.");
		return false;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
			
			MapNode n1 = pointNodeMap.get(from);
			MapNode n2 = pointNodeMap.get(to);
			
			if(n1 == null) {
				throw new NullPointerException("addEdge: from:" + from + "is not in graph");
			}
			if(n2 == null) {
				throw new NullPointerException("addEdge: to:" + to + "is not in graph");
			}
			
			MapEdge edge = new MapEdge(roadName, roadType, n1, n2, length);
			edges.add(edge);
			n1.addEdge(edge);
	}
	
	private Set<MapNode> getNeighbors(MapNode node) {
		return node.getNeighbors();
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		if (start == null || goal == null)
			throw new NullPointerException("Cannot find route from or to null node");
		
		
		MapNode n1 = pointNodeMap.get(start);
		MapNode n2 = pointNodeMap.get(goal);
		
		if(n1 == null || n2 == null) {
			return new ArrayList<GeographicPoint>();
		}
		
		// Initialising items
		HashSet<MapNode> visited = new HashSet<MapNode>();
		Queue<MapNode> queue = new LinkedList<MapNode>();
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		boolean found = false;
		
		MapNode curr = n1;
		queue.add(curr);
		
		while(!queue.isEmpty()) {
			curr = queue.remove();
			nodeSearched.accept(curr.getLocation());
			if(curr.equals(n2)) {
				found = true;
				break;
			}
			Set<MapNode> neighbors = curr.getNeighbors();
			
			for(MapNode neighbor: neighbors) {
				if(!visited.contains(neighbor)) {
					visited.add(neighbor);
					queue.add(neighbor);
					parentMap.put(neighbor, curr);
				}
			}
		}
		
		
		if(found == false) {
			return new ArrayList<GeographicPoint>();
		}
		
		LinkedList<GeographicPoint> path = (LinkedList<GeographicPoint>) reconstructPath(parentMap, n1, n2);	

		return path;
	}
	
//	 helper function for the construction of path
	private List<GeographicPoint> reconstructPath(HashMap<MapNode, MapNode> parentMap, MapNode start, MapNode goal) {
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = goal;

		while (!current.equals(start)) {
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}

		// add start
		path.addFirst(start.getLocation());
		return path;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		if(start == null || goal == null)
			return new ArrayList<GeographicPoint>();
		
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		
		if(startNode == null || endNode == null)
			return new ArrayList<GeographicPoint>();
		
		HashSet<MapNode> visited = new HashSet<MapNode>();
		PriorityQueue<MapNode> queue = new PriorityQueue<>();
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();

//		List<MapNode> all =  pointNodeMap.values();
		for(MapNode node: pointNodeMap.values()) {
			node.setDistance(Double.POSITIVE_INFINITY);
			node.setActualDistance(Double.POSITIVE_INFINITY);
		}
		
		startNode.setDistance(0);
		startNode.setActualDistance(0);
		
		MapNode curr = null;
		
		queue.add(startNode);
		while(!queue.isEmpty()) {
			curr = queue.remove();
			nodeSearched.accept(curr.getLocation());
			
			if(!visited.contains(curr)) {
				visited.add(curr);
				if(curr.equals(endNode))
					break;
				Set<MapEdge> edges = curr.getEdges();
				for(MapEdge edge: edges) {
					MapNode neighbor = edge.getOtherNode(curr);
					if(!visited.contains(neighbor)) {
						double dist = edge.getDistance() + curr.getDistance();
						if(dist < neighbor.getDistance()) {
							neighbor.setDistance(dist);
							neighbor.setActualDistance(dist);
							queue.add(neighbor);
							parentMap.put(neighbor, curr);
						}
					}
				}
			}
		}
	
		if(!curr.equals(endNode)) {
			return new ArrayList<GeographicPoint>();
		}
		
		List<GeographicPoint> path = reconstructPath(parentMap, startNode, endNode);
		System.out.println("Nodes Visited: " + path.size());
		return path;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		if(start == null || goal == null)
			return new ArrayList<GeographicPoint>();
		
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		
		if(startNode == null || endNode == null)
			return new ArrayList<GeographicPoint>();
		
		HashSet<MapNode> visited = new HashSet<MapNode>();
		PriorityQueue<MapNode> queue = new PriorityQueue<>();
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();

//		List<MapNode> all =  pointNodeMap.values();
		for(MapNode node: pointNodeMap.values()) {
			node.setDistance(Double.POSITIVE_INFINITY);
			node.setActualDistance(Double.POSITIVE_INFINITY);
		}
		
		startNode.setDistance(0);
		startNode.setActualDistance(0);
		
		MapNode curr = null;
		
		queue.add(startNode);
		while(!queue.isEmpty()) {
			curr = queue.remove();
			nodeSearched.accept(curr.getLocation());
			
			if(!visited.contains(curr)) {
				visited.add(curr);
				if(curr.equals(endNode))
					break;
				Set<MapEdge> edges = curr.getEdges();
				for(MapEdge edge: edges) {
					MapNode neighbor = edge.getOtherNode(curr);
					if(!visited.contains(neighbor)) {
						double dist = edge.getDistance() + curr.getDistance();
						double currToGoalDist = neighbor.getLocation().distance(goal);
						if(dist+currToGoalDist < neighbor.getDistance()) {
							neighbor.setDistance(dist);
							neighbor.setActualDistance(dist+currToGoalDist);
							queue.add(neighbor);
							parentMap.put(neighbor, curr);
						}
					}
				}
			}
		}
	
		if(!curr.equals(endNode)) {
			return new ArrayList<GeographicPoint>();
		}
		
		List<GeographicPoint> path = reconstructPath(parentMap, startNode, endNode);
		System.out.println("Nodes Visited: " + path.size());
		return path;
		
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		if(testroute != null)
			System.out.println("1: dij " + testroute.size());
		if(testroute2 != null)
			System.out.println("1: astar " + testroute.size());
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		if(testroute != null)
			System.out.println("2: dij " + testroute.size());
		if(testroute2 != null)
			System.out.println("2: astar " + testroute.size());
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		if(testroute != null)
			System.out.println("3: dij " + testroute.size());
		if(testroute2 != null)
			System.out.println("3: astar " + testroute.size());
		
		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}
