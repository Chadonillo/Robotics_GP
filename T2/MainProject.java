package T2;

import lejos.hardware.Button;

public class MainProject {
	public static void main(String[] args){
		Robot wallz = new Robot();
		wallz.rgbValues();
		wallz.localize();
		while(!Button.ESCAPE.isDown()){;}
	}
}
