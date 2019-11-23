package a;
import lejos.hardware.lcd.LCD;

public class PID {
	Helper util = new Helper();
	Robot wallz = new Robot();
	
	//PID setup Line following
	float error;
	float integral = 0; //accumulated error
	float lastError = 0; //store the last error to be used to calculate the derivative
	float pidValue = 0;
	
	int baseSpeed = 0;
	float kp = 0;
	float ki = 0;
	float kd = 0;
	
	public void run(){
		LCD.drawString("Left: " + Math.round(wallz.pollSensorLeft()*100) +"          ", 0, 5);
		LCD.drawString("Right: " + Math.round(wallz.pollSensorRight()*100) +"          ", 0, 6);
		LCD.drawString("PID: " + pidValue +"          ", 0, 7);
		error = (wallz.pollSensorLeft() - wallz.pollSensorRight())*100;
		// Anti Wind-Up
		if (Math.abs(error) <= 5 || util.oppositeSigns((int)error, (int)lastError)){
			integral=0;
		}
		else{
			integral = ((2/3) * integral) + error; //update accumulated error, Dampen by multiplying by 2/3
		}
		
		pidValue = (error * kp) + (integral * ki) + ((error - lastError) * kd);
		wallz.drive(baseSpeed + pidValue, baseSpeed - pidValue);
		lastError = error;
	}
	
	public void setVals(){
		baseSpeed = (int)util.inputLCD("Base Speed", 10, (float) baseSpeed, 0);
		kp = util.inputLCD("Kp", 0.1f, kp, 1);
		ki = util.inputLCD("Ki", 0.1f, ki, 2);
		kd = util.inputLCD("Kd", 0.1f, kd, 3);
	}
}