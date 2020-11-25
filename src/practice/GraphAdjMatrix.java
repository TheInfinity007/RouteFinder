package practice;

import java.util.ArrayList;
import java.util.List;

public class GraphAdjMatrix extends Graph{
	private int[][] adjMatrix;
	
	public GraphAdjMatrix() {
		adjMatrix = new int[0][0];
	}

	public void implementAddEdge(int v1, int v2) {
		adjMatrix[v1][v2] = 1;
	}
	
	@Override
	public void implementAddVertex() {
		int v = getNumVertices();
		if(v >= adjMatrix.length) {
			int [][] newAdjMatrix = new int[v*2][v*2];
			for(int i = 0; i < adjMatrix.length; i++) {
				for(int j = 0; j < adjMatrix.length; j++) {
					newAdjMatrix[i][j] = adjMatrix[i][j];
				}
			}
			adjMatrix = newAdjMatrix;
		}
		
		for(int i = 0; i < adjMatrix[v].length; i++) {
			adjMatrix[v][i] = 0;
		}
		
	}

	@Override
	public List<Integer> getNeighbours(int v) {
		List<Integer> neighbours =  new ArrayList<Integer>();
		int len = getNumVertices();
		
		//calculate outdegree
		for(int i = 0; i < len; i++) {
			if(adjMatrix[v][i] > 0) {
				neighbours.add(i);
			}
		}
		
		return neighbours;
	}
	
	public List<Integer> getDistance2(int v) {
		List<Integer> neighbours2 = new ArrayList<Integer>();
		
		int len = getNumVertices();
		for(int i = 0; i < len; i++) {
			if(adjMatrix[v][i] > 0) {
				for(int j = 0; j < len; j++) {
					if(adjMatrix[i][j] > 0 && !neighbours2.contains(j)) {
						neighbours2.add(j);
					}
				}
			}
		}
		
		return neighbours2;
	}
	
	public static void main(String args[]) {
		GraphAdjMatrix matrix = new GraphAdjMatrix();
		matrix.implementAddVertex();
	}

}
