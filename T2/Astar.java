package T2;


import java.util.Iterator;

import lejos.hardware.lcd.LCD;
import lejos.robotics.navigation.DestinationUnreachableException;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.pathfinding.ShortestPathFinder;
import lejos.utility.Delay;

public class Astar {
	Environment env = new Environment();
	ShortestPathFinder pathFinder = new ShortestPathFinder(env.getLineMap());
	
	//AstarSearchAlgorithm searchAlgo = new AstarSearchAlgorithm();
	//NodePathFinder pathFinder = new NodePathFinder(searchAlgo, env.getGripMesh());
	
	public Path findPath(Pose start, Waypoint goal){
		try {
			Path path = pathFinder.findRoute(start, goal);
			Iterator <Waypoint> waypointIterator = path.iterator();
			while(waypointIterator.hasNext()) {
				Waypoint cur = waypointIterator.next();
				LCD.drawString("x: "+cur.x+", y: "+cur.y+"            ", 0, 5);
		        Delay.msDelay(2000);
			}
		
			return path;
		} catch (DestinationUnreachableException e) {
			LCD.drawString("Not Found          ", 0, 7);
	        Delay.msDelay(2000);
			return new Path();
		}
	}
}
