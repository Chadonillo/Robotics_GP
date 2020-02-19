package T2;  

import lejos.hardware.Button;

public class MainProject {
	public static void main(String[] args){
		Robot wallz = new Robot();
		
		while(!Button.ESCAPE.isDown()){
			//LCD.drawString("0: " +wallz.pilot.getAngularAcceleration()+"          ", 0, 3);
			//LCD.drawString("dx: " +wallz.pilot.getLinearAcceleration()+"          ", 0, 5);
			wallz.testNavSquare(40);
			wallz.showPose();
			
		}
	}
}
