package T2;

import lejos.robotics.navigation.Waypoint;

public class Node {
	Waypoint coordinate;
	Waypoint[] connected;

	public Node(float x, float y) {
		coordinate = new Waypoint(x, y);
	}
	
	public Node(float x, float y, Waypoint[] connected){
		coordinate = new Waypoint(x, y);
		this.connected = connected;
	}
	
	public float getX(){
		return coordinate.x;
	}
	
	public float getY(){
		return coordinate.y;
	}
	
	public float getDistanceTo(Node other){
		return (float)Math.sqrt( Math.pow(this.getX() - other.getX(), 2) + Math.pow(this.getY() - other.getY(), 2));
	}
	
}
