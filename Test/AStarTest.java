package test;

import java.util.List;

public class AStarTest {

    public static void main(String[] args) {
    	 
        Node initialNode = new Node(8, 7);
        Node finalNode = new Node(2, 10);
        AStar aStar = new AStar();
        aStar.addBlockNode(new Node(7,12));
        aStar.addBlockNode(new Node(7,13));
        aStar.addBlockNode(new Node(6,12));
        aStar.addBlockNode(new Node(6,13));
        aStar.addBlockNode(new Node(5,12));
        aStar.addBlockNode(new Node(5,13));
        List<Node> path = aStar.findPath(initialNode, finalNode);
        aStar.showMap(path);

    }	
}
