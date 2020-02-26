package T2;  

import lejos.hardware.Button;

public class MainProject {
	public static void main(String[] args){
		while(!Button.ESCAPE.isDown()){
			while(!Button.ENTER.isDown()){;}
			Robot wallz = new Robot();
			wallz.centralizeOnStripBox();
			int stripPos = wallz.localize();
			wallz.getOnGridFromStrip(stripPos);
			int color = wallz.naviagateToBox();
			wallz.naviagateToBase(color);
			wallz.showPose();
		}
	}
}
// Things To Do
// 1) PilotRotate Test to see if robot offset is okay
// wallz.setDefaultSpeed();
// wallz.testPilotRotation();
// 3) test Navigator Square