package a;

import java.io.File;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class CourseWork {
	public static void main(String[] args){
		Helper util = new Helper();
		Robot wallz = new Robot();
		wallz.calibrateSensors();
		LineFollower lFer = new LineFollower(wallz, util);
		ObstacleAvoidance oA = new ObstacleAvoidance(wallz, util);
		double counter = 0;
		while (!Button.ESCAPE.isDown()){
			//lFer.setVals();
			oA.setVals();
			while(!Button.ESCAPE.isDown()){
				if(counter%50 == 0 && wallz.isInRedRange()){
					if(wallz.isRed()){
						wallz.drive(-50,-50);
						Delay.msDelay(600);
						wallz.stop();
						
						int aVolume = 100;
						int noteDelay = 250;
						int defaultDuration = 250;
					
						/*Sound.playTone(523, defaultDuration, aVolume);
						Delay.msDelay(noteDelay);
						Sound.playTone(554, defaultDuration, aVolume);
						Delay.msDelay(noteDelay);
						Sound.playTone(523, defaultDuration, aVolume);
						Delay.msDelay(noteDelay);
						Sound.playTone(554, defaultDuration, aVolume);
						Delay.msDelay(noteDelay);
						Sound.playTone(784, defaultDuration, aVolume);
						Delay.msDelay(noteDelay);
						Sound.playTone(784, defaultDuration, aVolume);
						Delay.msDelay(noteDelay);
						Sound.playTone(880, defaultDuration, aVolume);
						Delay.msDelay(noteDelay);
						Sound.playTone(880, defaultDuration, aVolume);
						Delay.msDelay(noteDelay);
						Sound.playTone(784, 750, aVolume);
						Delay.msDelay(750);
						Sound.playTone(698, defaultDuration, aVolume);
						Delay.msDelay(noteDelay);
						Sound.playTone(196, defaultDuration, aVolume);
						Delay.msDelay(noteDelay);
						Sound.playTone(196, defaultDuration, aVolume);
						Delay.msDelay(noteDelay);
						Sound.playTone(110, defaultDuration, aVolume);
						Delay.msDelay(noteDelay);
						Sound.playTone(110, defaultDuration, aVolume);
						Delay.msDelay(noteDelay);*/
						
						//Sound.twoBeeps();
						//Sound.playSample(new File("mono.wav"));
						
					}
					while(wallz.isRed()){
						LCD.drawString("Left: " + Math.round(wallz.pollSensorLeft()*100) +"          ", 0, 5);
						LCD.drawString("Right: " + Math.round(wallz.pollSensorRight()*100) +"          ", 0, 6);
					}
				}
				else{
					lFer.run();
					oA.run();
				}
				++counter;
			}
			wallz.stop();
			util.resetTest();
		}
		util.bye();
	}
}
