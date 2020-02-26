package test;

import java.util.*;

public class AStar {
    private int hvCost = 10; // Horizontal - Vertical Cost
    private int diagonalCost = 20;
    
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
    
    public void addBlockNode(Node addNode){
    	this.map.addBlockedNode(addNode);
    }
    
    public void removeBlockNode(Node removeNode){
    	this.map.removeBlockedNode(removeNode);
    }
    
    public void showMap(List<Node> path){
    	this.map.showMap(path);
    }
    
    public List<Node> findPath(Node initialNode, Node finalNode){
    	this.map.setHeuristic(finalNode);
        openList.add(initialNode);
        while (openList.size() != 0) {
            Node currentNode = openList.poll();
            closedSet.add(currentNode);
            if (currentNode.equals(finalNode)) {
                return getPath(currentNode);
            } else {
                addAdjacentNodes(currentNode);
            }
        }
        return new ArrayList<Node>();
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

