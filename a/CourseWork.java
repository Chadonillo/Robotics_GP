package a;

//import java.io.File;

import lejos.hardware.Button;
//import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class CourseWork {
	public static void main(String[] args){
		Helper util = new Helper();
		Robot wallz = new Robot();
		wallz.calibrateSensors();
		LineFollower lFer = new LineFollower(wallz, util);
		ObstacleAvoidance oA = new ObstacleAvoidance(wallz, util);
		double counter = 0;
		boolean colorCheckIsLeft = true;
		while (!Button.ESCAPE.isDown()){
			//lFer.setVals();
			oA.setVals();
			
			while(!Button.ESCAPE.isDown()){
				if(counter%30 == 0 && wallz.isInRedRange()){
					colorCheckIsLeft = !colorCheckIsLeft;
					if(wallz.isRed(colorCheckIsLeft)){
						wallz.drive(-70,-70);
						Delay.msDelay(600);
						wallz.stop();

						
					}
					while(wallz.isRed(colorCheckIsLeft) || wallz.isRed(!colorCheckIsLeft)){
						LCD.drawString("Left: " + Math.round(wallz.pollSensorLeft()*100) +"          ", 0, 5);
						LCD.drawString("Right: " + Math.round(wallz.pollSensorRight()*100) +"          ", 0, 6);
					}
				}
				else{
					lFer.run();
					oA.run();
				}
				++counter;
			}
			wallz.stop();
			util.resetTest();
		}
		util.bye();
	}
}
