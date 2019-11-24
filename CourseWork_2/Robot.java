package a;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.filter.LinearCalibrationFilter;
import lejos.utility.Delay;

public class Robot {
	public EV3ColorSensor sensorLeft = new EV3ColorSensor(SensorPort.S1);
	public EV3ColorSensor sensorRight = new EV3ColorSensor(SensorPort.S4);
	public SensorMode modeLeft = sensorLeft.getRedMode();
	public SensorMode modeRight = sensorRight.getRedMode();
	public float[] sampleLeft = new float[modeLeft.sampleSize()];
	public float[] sampleRight = new float[modeRight.sampleSize()];
	
	LinearCalibrationFilter  calibratorLeft  =  new LinearCalibrationFilter (modeLeft);
	LinearCalibrationFilter  calibratorRight  =  new LinearCalibrationFilter (modeRight);

	public float transform(float val){
		return val;
	}
	public void drive(float l, float r) {
		Motor.B.setSpeed(Math.abs(l));
		Motor.C.setSpeed(Math.abs(r));
		if (l > 0) {
			Motor.B.forward();
		} else if (l < 0) {
			Motor.B.backward();
		} else {
			Motor.B.stop(true);
		}

		if (r > 0) {
			Motor.C.forward();
		} else if (r < 0) {
			Motor.C.backward();
		} else {
			Motor.C.stop(true);
		}
	}

	public float pollSensorLeft() {
		calibratorLeft.fetchSample(sampleLeft, 0);
		return sampleLeft[0];
	}
	
	public float pollSensorRight() {
		calibratorRight.fetchSample(sampleRight, 0);
		return sampleRight[0];
	}
	
	public void calibrateSensors(){
		calibratorLeft.startCalibration();
		calibratorRight.startCalibration();
		LCD.drawString("Press Enter To", 0, 2);
		LCD.drawString("Start Calibration", 0, 4);
		Button.waitForAnyPress();
		LCD.clear();
		
		LCD.drawString("Starting Calibration", 0, 2);
		while(!Button.ENTER.isDown()){
			calibratorLeft.fetchSample (sampleLeft,  0);
			calibratorRight.fetchSample (sampleRight,  0);
		}
		calibratorLeft.stopCalibration();
		calibratorRight.stopCalibration();
		
		Delay.msDelay(500);
		LCD.clear();
	}

	public void stop(){
		Motor.B.stop();
		Motor.C.stop();
	}
	
	/*
	public static void main(String[] args){
		Robot wallz = new Robot();
		wallz.calibrateSensors();
		LCD.clear();
		while(!Button.ESCAPE.isDown()){
			wallz.modeLeft.fetchSample(wallz.sampleLeft, 0);
			wallz.modeRight.fetchSample(wallz.sampleRight, 0);
			
			LCD.drawString("Raw Left: " + Math.round(wallz.sampleLeft[0]*100) +"          ", 0, 3);
			LCD.drawString("Raw Right: " + Math.round(wallz.sampleRight[0]*100) +"          ", 0, 4);
			
			LCD.drawString("Left: " + Math.round(wallz.pollSensorLeft()*100) +"          ", 0, 6);
			LCD.drawString("Right: " + Math.round(wallz.pollSensorRight()*100) +"          ", 0, 7);
		}
	}
	*/
	
}