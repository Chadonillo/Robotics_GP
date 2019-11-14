package a;
import lejos.hardware.Button;

import lejos.hardware.Sound;

import lejos.hardware.lcd.LCD;

import lejos.hardware.motor.Motor;

import lejos.hardware.port.SensorPort;

import lejos.hardware.sensor.EV3ColorSensor;

import lejos.hardware.sensor.SensorMode;

import lejos.utility.Delay;


public class LineFollow {
	//LCD.drawString(lightLeft[0] + "  " +lightRight[0] ,0,0);
	//LCD.clear();
	public static void main(String[] args) {
		EV3ColorSensor colorSensorLeft = new EV3ColorSensor(SensorPort.S1); 
		EV3ColorSensor colorSensorRight = new EV3ColorSensor(SensorPort.S2);
		SensorMode modeLeft = colorSensorLeft.getRedMode();
		SensorMode modeRight = colorSensorRight.getRedMode();
		double whiteMin = 0.2;
		
		Motor.B.setSpeed(200);
		Motor.C.setSpeed(200);
		

		while(Button.ENTER.isDown()){
			
		}
		while(!Button.ESCAPE.isDown()){
			float[] lightLeft = new float[1]; 
			float[] lightRight = new float[1];
			modeLeft.fetchSample(lightLeft,0); 
			modeRight.fetchSample(lightRight,0);
			
			
		
			if(lightRight[0] < whiteMin){ //Turn Left
				Motor.B.setSpeed(200);
				Motor.C.setSpeed(100);
				
				Motor.C.backward();
				turnright:
				while(lightRight[0]<whiteMin){
					modeLeft.fetchSample(lightLeft,0); 
					modeRight.fetchSample(lightRight,0);
					if(Button.ESCAPE.isDown()){
						break turnright;
					}
				}
				
			}
			
			else if(lightLeft[0] < whiteMin){ // Turn RIght
				Motor.B.setSpeed(100);
				Motor.C.setSpeed(200);
				
				Motor.B.backward();
				turnLeft:
				while(lightLeft[0]<whiteMin){
					modeLeft.fetchSample(lightLeft,0); 
					modeRight.fetchSample(lightRight,0);
					if(Button.ESCAPE.isDown()){
						break turnLeft;
					}
				}
				
			}
			
			Motor.B.setSpeed(200);
			Motor.C.setSpeed(200);
			Motor.B.forward();
			Motor.C.forward();
		}
	}

}

/*EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
SensorMode mode = colorSensor.getRedMode();

float[] color = new float[1];
String colorString = new String();
String currDirection = new String("right");

while (!Button.ESCAPE.isDown()) {
	mode.fetchSample(color,0);
	if (color[0]!=0){
		//colorString = "White";
	}
	else{
		//colorString = "Black";
		Motor.C.backward();
		Motor.B.backward();
	}
	//LCD.drawString(colorString,0,0);
	//Delay.msDelay(10);
	//LCD.clear();

	//Motor.C.forward();
	//Motor.B.forward();
}*/

