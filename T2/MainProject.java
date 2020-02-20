package T2;  

import lejos.hardware.Button;

public class MainProject {
	public static void main(String[] args){
		Robot wallz = new Robot();
		
		while(!Button.ESCAPE.isDown()){
			wallz.testZigZagNav(30,4);
			wallz.showPose();
		}
	}
}
