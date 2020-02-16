package T2;  

import lejos.hardware.Button;

public class MainProject {
	public static void main(String[] args){
		Robot wallz = new Robot();
		while(!Button.ESCAPE.isDown()){
			wallz.naviagate();
			//wallz.test();
			wallz.showPose();
			//wallz.infraRedValues();
			//wallz.centralizeOnStripBox();
			//wallz.localize();
			while(!Button.ENTER.isDown()){;}
			
		}
	} 
}
