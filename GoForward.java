package go;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

//import lejos.hardware.Button;

//import lejos.hardware.lcd.LCD;

import lejos.hardware.motor.Motor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.navigation.*;

//import lejos.hardware.port.SensorPort;

//import lejos.hardware.sensor.EV3ColorSensor;

//import lejos.hardware.sensor.EV3UltrasonicSensor;

//import lejos.hardware.sensor.SensorMode;

//import lejos.robotics.SampleProvider;

//import lejos.robotics.filter.LinearCalibrationFilter;

//import lejos.utility.Delay;

import lejos.robotics.Encoder;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.*;
import lejos.utility.Delay;



public class GoForward {



//	public static void drive(float l, float r) {

//		Motor.A.setSpeed(Math.abs(l));

//		Motor.D.setSpeed(Math.abs(r));

//

//		if (l > 0) {

//			Motor.A.forward();

//		} else if (l < 0) {

//			Motor.D.backward();

//		} else {

//			Motor.A.stop(true);

//		}

//

//		if (r > 0) {

//			Motor.D.forward();

//		} else if (r < 0) {

//			Motor.D.backward();

//		} else {

//			Motor.D.stop(true);

//		}

//	}

	

	public static void main(String[] args){

//		drive(50, 50);

//		Delay.msDelay(800);

//		for(int i = 0; i < 10; i++) {
//
//			while(Motor.D.getTachoCount() <= 41 && Motor.A.getTachoCount() <= 41) {
//
//				Motor.A.setSpeed(30);
//
//				Motor.D.setSpeed(30);
//
//				Motor.A.forward();
//
//				Motor.D.forward();
//
//			}
//
//			Motor.A.resetTachoCount();
//
//			Motor.D.resetTachoCount();
//
//			Motor.A.stop();
//
//			Motor.D.stop();
//
//			Delay.msDelay(500);
//
//		}
		
		/*
		EV3GyroSensor gyrosensor = new EV3GyroSensor(SensorPort.S1);
		SampleProvider modeAngle = gyrosensor.getAngleMode();
		float[] sampleAngle = new float[modeAngle.sampleSize()];
		*/
		
		
		EV3LargeRegulatedMotor motorL = new EV3LargeRegulatedMotor(MotorPort.A);
		EV3LargeRegulatedMotor motorR = new EV3LargeRegulatedMotor(MotorPort.D);
		Wheel wheelL = WheeledChassis.modelWheel(motorL, 56.5).offset(-60.5);
		Wheel wheelR = WheeledChassis.modelWheel(motorR, 56.5).offset(60.5);
		Chassis chassis = new WheeledChassis(new Wheel[]{wheelR, wheelL},WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot wallz = new MovePilot(chassis);
		
		/*
		wallz.setLinearSpeed(50);
		wallz.setAngularSpeed(50);
		while(!Button.ESCAPE.isDown()){
			if(Button.ENTER.isDown()){
				gyrosensor.reset();
				for(int i=0; i<4; ++i) {
					wallz.travel(500);
					wallz.rotate(-90);
					modeAngle.fetchSample(sampleAngle, 0);
					LCD.drawString("Angle: " +sampleAngle[0]+"          ", 0, 5);
				}
			}
		}
		*/
		
		//Next Task
		wallz.setLinearSpeed(50);
		wallz.setAngularSpeed(50);
		while(!Button.ESCAPE.isDown()) {
			if(Button.ENTER.isDown()) {
				wallz.travel(80);
				wallz.rotate(-90);
				wallz.travel(50);
				wallz.rotate(135);
				wallz.travel(50);
			}
		}
		
		
		//wallz.rotate(90);

	}

}