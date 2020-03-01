 package T2;



import java.util.HashSet;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.List;
import java.util.ArrayList;
import java.util.Comparator;

import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;

public class AStar {
    private int hvCost = 10; // Horizontal - Vertical Cost
    private int diagonalCost = 14;
    
    private Map map;
    private PriorityQueue<Node> openList;
    private Set<Node> closedSet;

    public AStar(Map map) {
    	this.map = map;
        this.openList = new PriorityQueue<Node>(200, new Comparator<Node>(){
            @Override
            public int compare(Node node0, Node node1) {
                return Integer.compare(node0.getF(), node1.getF());
            }
        });
        this.closedSet = new HashSet<>();
    }
    
    public AStar(){
    	this(new Map());
    }
    
    public void addBlock(Waypoint addWayPoint){
    	this.map.addBlockedNode(new Node((int)addWayPoint.getY(), (int)addWayPoint.getX()));
    }
    
    public void addBlock(Path path){
    	for(Waypoint point : path){
    		this.map.addBlockedNode(new Node((int)point.getY(), (int)point.getX()));
    	}
    }
    
    public void removeBlock(Waypoint removeWayPoint){
    	this.map.removeBlockedNode(new Node((int)removeWayPoint.getY(), (int)removeWayPoint.getX()));
    }
    
    public void removeBlock(Path path){
    	for(Waypoint point : path){
    		this.map.removeBlockedNode(new Node((int)point.getY(), (int)point.getX()));
    	}
    }

    public Path findPath(Waypoint initialWaypoint, Waypoint finalWaypoint){
    	Node initialNode = new Node((int)initialWaypoint.getY(), (int)initialWaypoint.getX());
    	Node finalNode = new Node((int)finalWaypoint.getY(), (int)finalWaypoint.getX());
    	this.map.setHeuristic(finalNode);
        openList.add(initialNode);
        while (openList.size() != 0) {
            Node currentNode = openList.poll();
            closedSet.add(currentNode);
            if (currentNode.equals(finalNode)) {
                return shortenPath(listToPath(getPath(currentNode)));
            } else {
                addAdjacentNodes(currentNode);
            }
        }
        return new Path();
    }
    
    private Path listToPath(List<Node> list){
    	Path path = new Path();
    	for (Node node : list) {
    		path.add(new Waypoint(node.getCol(), node.getRow()));
    	}
		return path;
    }
    
    private Path shortenPath(Path longPath){
		if (longPath.isEmpty()){
			return longPath;
		}
		
		Path shortPath = new Path();
		float lastAngle = longPath.get(0).angleTo(longPath.get(1));
		shortPath.add(longPath.get(0));
		
		for (int i = 0; i < longPath.size()-1; i++){
			if(longPath.get(i).angleTo(longPath.get(i+1))!=lastAngle){
				lastAngle = longPath.get(i).angleTo(longPath.get(i+1));
				shortPath.add(longPath.get(i));		
			}
	    }
		shortPath.add(longPath.get(longPath.size()-1));
		return shortPath;
	}
    
    
    
    private List<Node> getPath(Node currentNode){
        List<Node> path = new ArrayList<Node>();
        path.add(currentNode);
        Node parent;
        while ((parent = currentNode.getParent()) != null) {
            path.add(0, parent);
            currentNode = parent;
        }
        return path;
    }

    private void addAdjacentNodes(Node currentNode){
        addAdjacentUpperRow(currentNode);
        addAdjacentMiddleRow(currentNode);
        addAdjacentLowerRow(currentNode);
    }
   
    private void addAdjacentLowerRow(Node currentNode){
        int row = currentNode.getRow();
        int col = currentNode.getCol();
        int lowerRow = row + 1;
        if (lowerRow < map.getMap().length) {
            if (col - 1 >= 0) {
                checkNode(currentNode, col - 1, lowerRow, this.diagonalCost); // Comment this line if diagonal movements are not allowed
            }
            if (col + 1 < map.getMap()[0].length) {
                checkNode(currentNode, col + 1, lowerRow, this.diagonalCost); // Comment this line if diagonal movements are not allowed
            }
            checkNode(currentNode, col, lowerRow, this.hvCost);
        }
    }
   
    private void addAdjacentMiddleRow(Node currentNode){
        int row = currentNode.getRow();
        int col = currentNode.getCol();
        int middleRow = row;
        if (col - 1 >= 0) {
            checkNode(currentNode, col - 1, middleRow, this.hvCost);
        }
        if (col + 1 < map.getMap()[0].length) {
            checkNode(currentNode, col + 1, middleRow, this.hvCost);
        }
    }
   
    private void addAdjacentUpperRow(Node currentNode){
        int row = currentNode.getRow();
        int col = currentNode.getCol();
        int upperRow = row - 1;
        if (upperRow >= 0) {
            if (col - 1 >= 0) {
                checkNode(currentNode, col - 1, upperRow, this.diagonalCost); // Comment this if diagonal movements are not allowed
            }
            if (col + 1 < map.getMap()[0].length) {
                checkNode(currentNode, col + 1, upperRow, this.diagonalCost); // Comment this if diagonal movements are not allowed
            }
            checkNode(currentNode, col, upperRow, this.hvCost);
        }
    }
   
    private void checkNode(Node currentNode, int col, int row, int cost){
        Node adjacentNode = map.getMap()[row][col];
        if (!adjacentNode.isBlock() && !this.closedSet.contains(adjacentNode)) {
            if (!this.openList.contains(adjacentNode)) {
                adjacentNode.setNodeData(currentNode, cost);
                this.openList.add(adjacentNode);
            } else {
                boolean changed = adjacentNode.checkBetterPath(currentNode, cost);
                if (changed) {
                    // Remove and Add the changed node, so that the PriorityQueue can sort again its
                    // contents with the modified "finalCost" value of the modified node
                    this.openList.remove(adjacentNode);
                    this.openList.add(adjacentNode);
                }
            }
        }
    }
}

