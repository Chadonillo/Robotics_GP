package a;

import lejos.hardware.Button;

public class CourseWork {
	public static void main(String[] args){
		Helper util = new Helper();
		Robot wallz = new Robot();
		wallz.calibrateSensors();
		LineFollower lFer = new LineFollower(wallz, util);
		ObstacleAvoidance oA = new ObstacleAvoidance(wallz, util);
		RedCheck redCheck = new RedCheck(wallz);
		
		while (!Button.ESCAPE.isDown()){
			//lFer.setVals();
			//oA.setVals();
			int counter = 0;
			while(!Button.ESCAPE.isDown()){
				redCheck.run(counter);
				lFer.run();
				oA.run();
				++counter;
			}
			wallz.stopParallel();
			util.resetTest();
		}
		util.bye();
	}
}
