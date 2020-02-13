package T2;

import lejos.hardware.Button;
import lejos.utility.Delay;

public class MainProject {
	public static void main(String[] args){
		Robot wallz = new Robot();
		while(!Button.ESCAPE.isDown()){
			Delay.msDelay(500);
			wallz.infraRedValues();
			wallz.centralizeOnStripBox();
			wallz.localize();
			while(!Button.ENTER.isDown()){;}
		}
	}
}
