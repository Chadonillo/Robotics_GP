package a;

import lejos.hardware.Button;

public class LineFollow {
	public static void main(String[] args){
		Helper util = new Helper();
		Robot wallz = new Robot();
		wallz.calibrateSensors();
		PID pid = new PID(wallz, util);
		while (!Button.ESCAPE.isDown()){
			pid.setVals();
			while(!Button.ESCAPE.isDown()){
				pid.run();
			}
			wallz.stop();
			util.resetTest();
		}
		util.bye();
	}
}
