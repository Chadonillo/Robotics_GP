package T2;  

import lejos.hardware.Button;

public class MainProject {
	public static void main(String[] args){
		Robot wallz = new Robot();
		
		while(!Button.ESCAPE.isDown()){
			wallz.infraRedValues();
			//wallz.centralizeOnStripBox();
			//int stripPos = wallz.localize();
			//wallz.getOnGridFromStrip(stripPos);
			//wallz.showPose();
		}
	}
}
// Things To Do
// 1) PilotRotate Test to see if robot offset is okay
// 2) Calibrate light sensor and save file
// 3) test Navigator Square