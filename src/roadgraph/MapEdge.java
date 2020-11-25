package roadgraph;

import geography.GeographicPoint;

public class MapEdge {

//	Two endpoint of the edge
	private MapNode start;
	private MapNode end;
	
	// name and type of street
	private String streetName;
	private String streetType;
	
	// length of road segment
	private double distance;
	static final double DEFAULT_LENGTH = 0.01;
	
	public MapEdge() {
		
	}
	
	public MapEdge(String streetName, MapNode start, MapNode end) {
		this(streetName, "", start, end, DEFAULT_LENGTH);
	}
	
	public MapEdge(String streetName, String streetType, MapNode start, MapNode end) {
		this(streetName, streetType, start, end, DEFAULT_LENGTH);
	}
	
	public MapEdge(String streetName, String streetType, MapNode start, MapNode end, double distance) {
		this.streetName = streetName;
		this.streetType = streetType;
		this.start = start;
		this.end = end;
		this.distance = distance;
	}

	// return start node
	public MapNode getStart() {
		return start;
	}

	// return the location of starting point
	public GeographicPoint getStartPoint() {
		return start.getLocation();
	}

	public void setStart(MapNode start) {
		this.start = start;
	}

	// return end node
	public MapNode getEnd() {
		return end;
	}
	
	// Give the node on the other side of a edge belong to the node
	public MapNode getOtherNode(MapNode node) {
		if(node.equals(start)) {
			return end;
		}
		if(node.equals(end)) {
			return start;
		}
		throw new IllegalArgumentException("Looking for a point which is not in the edge.");
	}

	// return the location of end point
	public GeographicPoint getEndPoint() {
		return end.getLocation();
	}
	
	public void setEnd(MapNode end) {
		this.end = end;
	}

	// get street name
	public String getStreetName() {
		return streetName;
	}

	// set street name
	public void setStreetName(String streetName) {
		this.streetName = streetName;
	}

	public String getStreetType() {
		return streetType;
	}

	public void setStreetType(String streetType) {
		this.streetType = streetType;
	}

	public double getDistance() {
		return distance;
	}

	public void setDistance(double distance) {
		this.distance = distance;
	}
	
	// return String containing details about the edge
	public String toString()
	{
		String toReturn = "[EDGE between ";
		toReturn += "\n\t" + start.getLocation();
		toReturn += "\n\t" + end.getLocation();
		toReturn += "\nRoad name: " + streetName + " Road type: " + streetType +
				" Segment length: " + String.format("%.3g", distance) + "km";
		
		return toReturn;
	}

}
