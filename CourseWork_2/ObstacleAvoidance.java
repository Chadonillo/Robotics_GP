package a;

import lejos.utility.Delay;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;

public class ObstacleAvoidance {
	Helper util;
	Robot wallz;
	
	private float derivativeOA = 0; //used to predict next error
	private float errorOA = 0;
	private float lastErrorOA = 0;
	
	int baseSpeed = 150;
	float kpOA = 8;
	float kiOA = 0;
	float kdOA = 50;
	
	float pidValueOA = 0;
	
	private int obstacleDistance = 6;
	
	public ObstacleAvoidance(Robot robot, Helper help){
		wallz = robot;
		util = help;
		//LCD.drawString("Position: "+ Motor.B.getPosition()+"       ", 0, 4);
		//Delay.msDelay(3000);
		LCD.clear();
	}
	
	public void run(){
		LCD.drawString("Distance: "+ wallz.getDistance()+"       ", 0, 7);
		if(wallz.getDistance() <= obstacleDistance){
			wallz.stop();
			wallz.turnTillDistance(obstacleDistance);
			while(wallz.pollSensorLeft() > 0.2 && wallz.pollSensorRight() > 0.2 && !Button.ENTER.isDown()){
				LCD.drawString("Distance: "+ wallz.getDistance()+"       ", 0, 4);
				LCD.drawString("PID: " + pidValueOA +"          ", 0, 7);
				errorOA = wallz.getDistance() - obstacleDistance;
				
				derivativeOA = errorOA - lastErrorOA;
				
				pidValueOA = (errorOA * kpOA) + (derivativeOA * kdOA) ;
				wallz.drive(baseSpeed - pidValueOA, baseSpeed + pidValueOA);
				
				lastErrorOA = errorOA;
			}
			wallz.stop();
			
			//PID

			
			wallz.turnHeadRight();
		}
	}
	public void setVals(){
		baseSpeed = (int)util.inputLCD("Base Speed", 10, (float) baseSpeed, 0);
		kpOA = util.inputLCD("kpOA", 0.1f, kpOA, 1);
		kdOA = util.inputLCD("kdOA", 0.5f, kdOA, 2);
	}
}
