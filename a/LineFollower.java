package a;

import lejos.hardware.lcd.LCD;

public class LineFollower {
	Helper util;
	Robot wallz;
	
	
	//PID setup Line following
	float error;
	float integral = 0; //accumulated error
	float lastError = 0; //store the last error to be used to calculate the derivative
	float pidValue = 0;
	
	int baseSpeed = 200;
	float kp = 2;
	float ki = 0.3f;
	float kd = 30;
	float damping = 0.9f;
	
	DequeMe errors = new DequeMe(200);
	
	public LineFollower(Robot robot, Helper help){
		wallz = robot;
		util = help;
	}
	public void run(){
		LCD.drawString("Left: " + Math.round(wallz.pollSensorLeft()*100) +"          ", 0, 5);
		LCD.drawString("Right: " + Math.round(wallz.pollSensorRight()*100) +"          ", 0, 6);
		//LCD.drawString("PID: " + pidValue +"          ", 0, 7);
		error = (wallz.pollSensorLeft() - wallz.pollSensorRight())*100;
		errors.add(error);
		if (Math.abs(error) <= 5 || util.oppositeSigns((int)error, (int)lastError)){
			integral=0;
		}
		else{
			integral = (damping* integral) + error; //update accumulated error, Dampen by multiplying by 2/3
		}
		
		pidValue = (error * kp) + (integral * ki) + ((error - lastError) * kd);
		//Acceleration
		if(errors.allValsLessThan(5) && Math.abs(pidValue)<150){
			wallz.drive((baseSpeed+200) + pidValue, (baseSpeed+200) - pidValue);
		}
		else if(errors.allValsLessThan(10) && Math.abs(pidValue)<150){
			wallz.drive(baseSpeed+100 + pidValue, baseSpeed+100 - pidValue);
		}
		else{
			wallz.drive(baseSpeed + pidValue, baseSpeed - pidValue);
		}
		
		lastError = error;
	}
	
	public void setVals(){
		baseSpeed = (int)util.inputLCD("Base Speed", 10, (float) baseSpeed, 0);
		kp = util.inputLCD("Kp", 0.1f, kp, 1);
		ki = util.inputLCD("Ki", 0.025f, ki, 2);
		kd = util.inputLCD("Kd", 0.5f, kd, 3);
		damping = util.inputLCD("Damping", 0.01f, damping, 4);
	}
}