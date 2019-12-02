package a;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class ObstacleAvoidance {
	Helper util;
	Robot wallz;
	
	private float derivativeOA = 0; //used to predict next error
	private float errorOA = 0;
	private float lastErrorOA = 0;
	
	int baseSpeed = 200;
	float kpOA = 8;
	float kiOA = 0;
	float kdOA = 40;
	
	float pidValueOA = 0;
	
	private int obstacleDistance = 4;
	private float maxBlack = 0.3f;
	
	public ObstacleAvoidance(Robot robot, Helper help){
		wallz = robot;
		util = help;
	}
	
	public void run(){
		LCD.drawString("Distance: "+ wallz.getDistance()+"       ", 0, 7);
		if(wallz.getDistance() <= obstacleDistance+3){
			wallz.stop();
			while(wallz.getDistance() < 4 || wallz.getDistance() > 300) {
				wallz.drive(-100, -100);
			}
			wallz.turnTillDistance(obstacleDistance);
			while(wallz.pollSensorRight() > maxBlack && wallz.pollSensorLeft() > maxBlack && !Button.ENTER.isDown()){ 
				LCD.drawString("Distance: "+ wallz.getDistance()+"       ", 0, 4);
				errorOA = wallz.getDistance() - obstacleDistance;
				
				derivativeOA = errorOA - lastErrorOA;
				
				pidValueOA = (errorOA * kpOA) + (derivativeOA * kdOA) ;
				wallz.drive(baseSpeed - pidValueOA, baseSpeed + pidValueOA);
				
				lastErrorOA = errorOA;
			}
			wallz.stop();
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
