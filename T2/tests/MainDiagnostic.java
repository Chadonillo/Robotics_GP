package tests;
import java.util.Iterator;

import lejos.hardware.Button;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;
import pathfinding.AStar;
import pathfinding.Map;
import pathfinding.Node;
// Author James
public class MainDiagnostic {
	public static void main(String[] args){
		System.out.println();
		System.out.println("Main Diagnostic initiating...");//Construct and display the map constructed of nodes that Wall-Z perceives.
		System.out.println();
		
		System.out.println("This is the input map to be used with Astar encoded within the map class: ");
		Map map = new Map();
		map.showMap();
		AStar aStar = new AStar(map);
		
		Waypoint a = new Waypoint(8,2);
		Waypoint b = new Waypoint(10,0);
		Path c = new Path();
		
		
		System.out.println();
		System.out.println("\n" +"A map is constructed with many nodes");
		System.out.println("Node properties can be set and displayed, e.g: ");
		Node sample = new Node(3,1);
		sample.printNodeProperties();
		
		c = aStar.getPath(a, b);
		Iterator<Waypoint> citerator = c.iterator();
        System.out.println("\n" + "The route calculated by Astar for these two waypoints is: ");
		while (citerator.hasNext())
			System.out.print(citerator.next() + "\n" + " --> ");
		System.out.println("The End");
		//For (8,2) and (8,8) - Point2D.Float[8.0, 2.0] Point2D.Float[8.0, 3.0] Point2D.Float[6.0, 5.0] Point2D.Float[6.0, 7.0] Point2D.Float[7.0, 8.0] Point2D.Float[8.0, 8.0]
		}
	}
