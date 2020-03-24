package term2;
import java.util.Iterator;

import lejos.hardware.Button;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;
import pathfinding.AStar;
import pathfinding.Map;

public class MainDiagnostic {
	public static void main(String[] args){
		//Construct and display the map constructed of nodes that Wall-Z perceives.
		Map map = new Map();
		map.showMap();
		AStar aStar = new AStar(map);
		System.out.println("Main Diagnostic initiating...");
		Waypoint a = new Waypoint(8,2);
		Waypoint b = new Waypoint(8,8);
		Path c = new Path();


		c = aStar.findPath(a, b);
		Iterator<Waypoint> citerator = c.iterator();

		while (citerator.hasNext())
			System.out.print(citerator.next() + " ");
		//For (8,2) and (8,8) - Point2D.Float[8.0, 2.0] Point2D.Float[8.0, 3.0] Point2D.Float[6.0, 5.0] Point2D.Float[6.0, 7.0] Point2D.Float[7.0, 8.0] Point2D.Float[8.0, 8.0]

		}
	}
