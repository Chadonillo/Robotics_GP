package test;
import lejos.hardware.Button;

import lejos.hardware.Sound;

import lejos.hardware.lcd.LCD;

import lejos.hardware.motor.Motor;

import lejos.hardware.port.SensorPort;

import lejos.hardware.sensor.EV3ColorSensor;

import lejos.hardware.sensor.SensorMode;

import lejos.utility.Delay;



public class EV3LF {



	public static void main(String[] args) {

		// TODO Auto-generated method stub

		EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S4);

		int white = 6;
		int black = 1;
		int color = 0;


		//Motor.C.setSpeed();
		//Motor.B.setSpeed();
		while (!Button.ESCAPE.isDown()) {
			color = colorSensor.getColorID();
			
			Motor.C.forward();
			Motor.B.forward();

		}



	}

}