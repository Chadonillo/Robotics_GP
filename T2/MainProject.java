package T2;  

import lejos.hardware.Button;

public class MainProject {
	public static void main(String[] args){
		Robot wallz = new Robot();
		
		while(!Button.ESCAPE.isDown()){
			wallz.naviagate();
			wallz.showPose();
			//wallz.test();
			//wallz.infraRedValues();
			//wallz.centralizeOnStripBox();
			//wallz.localize();
			while(!Button.ENTER.isDown()){;}
			
		}
	} 
}
