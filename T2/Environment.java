package T2;

import lejos.hardware.lcd.LCD;
import lejos.robotics.geometry.Line;
import lejos.robotics.geometry.Rectangle;
import lejos.robotics.mapping.LineMap;
import lejos.robotics.pathfinding.FourWayGridMesh;
import lejos.utility.Delay;

import java.io.FileInputStream;
import java.io.DataInputStream;

public class Environment{
	private LineMap lineMap = new LineMap(getLines(),new Rectangle(0, 0, 120, 120));
	private FourWayGridMesh gridMesh = new FourWayGridMesh(lineMap, 5, 0);
	
	public FourWayGridMesh getGripMesh(){
		return gridMesh;
	}
	
	public LineMap getLineMap(){
		return lineMap;
	}

	private Line[] getLines() {
		Line[] lines = new Line[8];
		lines[0] = new Line(0,0,0,120);
		lines[1] = new Line(0,0,120,0);
		lines[2] = new Line(120,120,0,120);
		lines[3] = new Line(120,12,120,0);
		
		lines[4] = new Line(0,75,70,75);
		lines[5] = new Line(0,60,70,60);
		lines[6] = new Line(50,40,120,40);
		lines[7] = new Line(50,25,120,25);
		return lines;
	}
	
	public LineMap createMap(String svgFileName){
		try{
			LineMap lineMap = new LineMap();
			FileInputStream file = new FileInputStream(getClass().getResource(svgFileName).getPath());
			LCD.drawString("Found          ", 0, 7);
	        Delay.msDelay(2000);
	        DataInputStream data = new DataInputStream(file);
	        lineMap.loadObject(data);
			return lineMap.flip();
	    } 
		catch (Exception ex){
			LCD.drawString("Not Found          ", 0, 7);
	        Delay.msDelay(2000);
			return new LineMap();
	    }
	}
	
}
	
