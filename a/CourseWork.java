package a;

import lejos.hardware.Button;

public class CourseWork {
	public static void main(String[] args){
		Helper util = new Helper();
		Robot wallz = new Robot();
		wallz.calibrateSensors();
		LineFollower lFer = new LineFollower(wallz, util);
		ObstacleAvoidance oA = new ObstacleAvoidance(wallz, util);
		while (!Button.ESCAPE.isDown()){
			//lFer.setVals();
			//oA.setVals();
			while(!Button.ESCAPE.isDown()){
				lFer.run();
				oA.run();
			}
			wallz.stop();
			util.resetTest();
		}
		util.bye();
	}
}
