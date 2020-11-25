package practice;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class GraphAdjList extends Graph{
	private Map<Integer, ArrayList<Integer>> adjListsMap;
	
	public GraphAdjList() {
//		adjListsMap = new HashMap<Integer, ArrayList<Integer>>();
	}

	@Override
	public void implementAddVertex() {
		int v = getNumVertices();
		ArrayList<Integer> neighbours = new ArrayList<Integer>();
		adjListsMap.put(v, neighbours);
	}
	
	public void implementAddEdge(int v1, int v2) {
		(adjListsMap.get(v1)).add(v2);
	}

	@Override
	public List<Integer> getNeighbours(int v) {
		// TODO Auto-generated method stub
		List<Integer> neighbours = new ArrayList<Integer>();
		
		// calculate Outdegree
//		neighbours = adjListsMap.get(v).clone();
		neighbours = new ArrayList<Integer>(adjListsMap.get(v));
		
//		// calculate indegree
//		for(Integer i: adjListsMap.keySet()) {
//			if(i != v) {
//				if(adjListsMap.get(i).contains(v) && !neighbours.contains(i)){
//					neighbours.add(i);
//				}
//			}
//		}
		return neighbours;
	}
	
	public List<Integer> getDistance2(int v) {
		List<Integer> neighbours = adjListsMap.get(v);
		List<Integer> neighbours2 = new ArrayList<Integer>();
		
		for(Integer n : neighbours) {
			if(!neighbours2.contains(n)) {
				neighbours.add(n);
			}
		}
		
		return neighbours2;
	}
	
	public static void main(String args[]) {
		GraphAdjList matrix = new GraphAdjList();
		matrix.implementAddVertex();
	}

}
