package test;
import lejos.hardware.Button;

import lejos.hardware.Sound;

import lejos.hardware.lcd.LCD;

import lejos.hardware.motor.Motor;

import lejos.hardware.port.SensorPort;

import lejos.hardware.sensor.EV3ColorSensor;

import lejos.hardware.sensor.SensorMode;

import lejos.utility.Delay;


public class LineFollow {



	public static void main(String[] args) {

		// TODO Auto-generated method stub

		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);
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
		}



	}

}
