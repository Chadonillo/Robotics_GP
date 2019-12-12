package a;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;

public class ObstacleAvoidance {
	Helper util;
	Robot wallz;
	
	float currDistance;
	private int obstacleDistance = 10;
	private float maxBlack = 0.3f;
	
	private float derivativeOA = 0; //used to predict next error
	private float errorOA = 0;
	private float lastErrorOA = 0;
	
	int baseSpeed = 100;
	float kpOA = 13;
	float kdOA = 40;
	float pidValueOA = 0;
	
	public ObstacleAvoidance(Robot robot, Helper help){
		wallz = robot;
		util = help;
	}
	
	public void run(){
		LCD.drawString("Distance: "+ wallz.getDistance()+"       ", 0, 7);
		if(wallz.getDistance() <= obstacleDistance){
			wallz.stopParallel();
			
			wallz.turnTillDistance(obstacleDistance+5);
			while(wallz.pollSensorRight() > maxBlack && wallz.pollSensorLeft() > maxBlack && !Button.ENTER.isDown()){ 
				LCD.drawString("Distance: "+ wallz.getDistance()+"       ", 0, 4);
				currDistance = wallz.getDistance();
				if(currDistance>500){
					errorOA = lastErrorOA;
				}
				else{
					errorOA = currDistance - obstacleDistance;
				}
				
				
				derivativeOA = errorOA - lastErrorOA;
				
				pidValueOA = (errorOA * kpOA) + (derivativeOA * kdOA) ;
				wallz.drive(baseSpeed - pidValueOA, baseSpeed + pidValueOA);
				
				lastErrorOA = errorOA;
			}
			wallz.stopParallel();
			wallz.getOnLine();
			
			wallz.turnHeadRight();
		}
	}
	public void setVals(){
		baseSpeed = (int)util.inputLCD("Base Speed", 10, (float) baseSpeed, 0);
		kpOA = util.inputLCD("kpOA", 0.1f, kpOA, 1);
		kdOA = util.inputLCD("kdOA", 0.5f, kdOA, 2);
	}
}
