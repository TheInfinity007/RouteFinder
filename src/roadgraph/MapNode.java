// By Infinity
package roadgraph;


import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

public class MapNode implements Comparable{

	//location of the node
	private GeographicPoint location;
	
	// list of edges corresponding to a node
	private HashSet<MapEdge> edges;	
	
	// distances between node
	private double distance;
	private double actualDistance;
	
	public MapNode() {
		
	}
	
	// Constructor
	public MapNode(GeographicPoint location) {
		this.location = location;
		edges = new HashSet<MapEdge>();
		distance = 0.0;
		actualDistance = 0.0;
	}
	
	//	Add the new edge to the node
	public void addEdge(MapEdge edge) {
		edges.add(edge);
	}

	/** get the location of a node */
	public GeographicPoint getLocation() {
		return location;
	}
	
	/** set the location of a node */
	public void setLocation(GeographicPoint location) {
		this.location = location;
	}

	/** return the edges out of this node */
	public Set<MapEdge> getEdges() {
		return edges;
	}
	
	// get node distance (predicted)
	public double getDistance() {
		return distance;
	}
	
	// set node distance(predicted)
	public void setDistance(double distance) {
		this.distance = distance;
	}
	
	// get node distance (actual)
	public double getActualDistance() {
		return actualDistance;
	}

	// set node distance(actual)
	public void setActualDistance(double actualDistance) {
		this.actualDistance = actualDistance;
	}
	
	// get the list of all the neighbors of a node
	public Set<MapNode> getNeighbors(){
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for(MapEdge edge: edges) {
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}
	
	// equals function which is used to compare the MapNode using the location data
	public boolean equals(Object o) {
		if(!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);
	}
	
	public int HashCode() {
		return location.hashCode();
	}

	@Override
	public int compareTo(Object o) {
		MapNode m = (MapNode)o;
		return ((Double)this.getDistance()).compareTo((Double)m.getDistance());
	}
	
	public String toString()
	{
		String toReturn = "[NODE at location (" + location + ")";
		toReturn += " intersects streets: ";
		for (MapEdge e: edges) {
			toReturn += e.getStreetName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}
}
