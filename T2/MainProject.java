package T2;  

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;

public class MainProject {
	public static void main(String[] args){
		Robot wallz = new Robot();
		while(!Button.ESCAPE.isDown()){
			LCD.clear();
			LCD.drawString("Enter To Start", 0, 1);
			while(!Button.ENTER.isDown()){;} 
			wallz.reset();
			wallz.centralizeOnStripBox();
			int stripPos = wallz.localize();
			wallz.getOnGridFromStrip(stripPos);
			int color = wallz.navigateToBox("left");
			wallz.navigateToBase(color);
			wallz.showPose();
		}
	}
}
// Things To Do
// 1) PilotRotate Test to see if robot offset is okay
// wallz.setDefaultSpeed();
// wallz.testPilotRotation();
// 3) test Navigator Squares