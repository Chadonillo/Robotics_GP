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
			wallz.naviagate();
			wallz.showPose();
		}
	}
}
// Things To Do
// 1) PilotRotate Test to see if robot offset is okay
// 2) Calibrate light sensor
// 3) test Navigator Square