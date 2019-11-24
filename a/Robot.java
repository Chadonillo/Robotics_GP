package a;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.robotics.filter.LinearCalibrationFilter;
import lejos.utility.Delay;

public class Robot {
	public EV3ColorSensor sensorLeft = new EV3ColorSensor(SensorPort.S1);
	public EV3ColorSensor sensorRight = new EV3ColorSensor(SensorPort.S4);
	public SensorMode modeLeft = sensorLeft.getRedMode();
	public SensorMode modeRight = sensorRight.getRedMode();
	public float[] sampleLeft = new float[modeLeft.sampleSize()];
	public float[] sampleRight = new float[modeRight.sampleSize()];
	
	
	public EV3UltrasonicSensor ultraSonicSensor = new EV3UltrasonicSensor(SensorPort.S3);
	public SampleProvider ussProvider = ultraSonicSensor.getDistanceMode();
	public float[] sampleDistance = new float[ussProvider.sampleSize()];
	
	LinearCalibrationFilter  calibratorLeft  =  new LinearCalibrationFilter (modeLeft);
	LinearCalibrationFilter  calibratorRight  =  new LinearCalibrationFilter (modeRight);

	public float transform (float min, float max, float val){
		float answer = (val-min)/(max-min);
    	if(answer<0){answer = 0;}
    	else if(answer>1){answer = 1;}
    	return answer;
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
	
	public void stop(){
		Motor.B.stop();
		Motor.C.stop();
	}
	
	public void calibrateSensors(){
		LCD.drawString("Press Enter To", 0, 3);
		LCD.drawString("Start Calibration", 0, 5);
		Button.waitForAnyPress();
		LCD.clear();
		Delay.msDelay(500);
		
		calibratorLeft.setScaleCalibration(0, 1);
		calibratorRight.setScaleCalibration(0, 1);
		
		calibratorLeft.startCalibration();
		calibratorRight.startCalibration();
		LCD.drawString("Sensor Calibration", 0, 3);
		this.drive(50, 50);
		while(!Button.ENTER.isDown()){
			calibratorLeft.fetchSample (sampleLeft,  0);
			calibratorRight.fetchSample (sampleRight,  0);
		}
		this.stop();
		calibratorLeft.stopCalibration();
		calibratorRight.stopCalibration();
		Delay.msDelay(1000);
		LCD.clear();
	}

	public float pollSensorLeft() {
		calibratorLeft.fetchSample(sampleLeft, 0);
		return sampleLeft[0];
	}
	
	public float pollSensorRight() {
		calibratorRight.fetchSample(sampleRight, 0);
		return sampleRight[0];
	}
	
	public float getDistance(){
		ussProvider.fetchSample(sampleDistance, 0);
		return sampleDistance[0]*100;
	}
	
	public void turnLeft(){
		this.drive(-300, 300);
		Delay.msDelay(450);
		this.stop();
	}
	
	public void turnRight(){
		this.drive(300, -300);
		Delay.msDelay(450);
		this.stop();
	}
	
	public void turnHeadLeft(){
		Motor.D.rotate(-110);
	}
	
	public void turnHeadRight(){
		Motor.D.rotate(110);
	}
	
	public void turnTillDistance(int distance){
		this.turnHeadLeft();
		this.drive(100, -100);
		while (this.getDistance()>distance+10 && !Button.ENTER.isDown()){
			;
		}
		this.stop();
	}
	

	
	
	/*
	public static void main(String[] args){
		Robot wallz = new Robot();
		wallz.calibrateSensors();
		LCD.clear();
		while(!Button.ESCAPE.isDown()){
			wallz.modeLeft.fetchSample(wallz.sampleLeft, 0);
			wallz.modeRight.fetchSample(wallz.sampleRight, 0);
			LCD.drawString(wallz.calibratorLeft.getScaleCorrection()[0] +"          ", 0, 0);
			LCD.drawString(wallz.calibratorLeft.getOffsetCorrection()[0] +"          ", 0, 1);
			
			LCD.drawString(wallz.calibratorRight.getScaleCorrection()[0] +"          ", 0, 2);
			LCD.drawString(wallz.calibratorRight.getOffsetCorrection()[0] +"          ", 0, 3);
			
			LCD.drawString("Raw Left: " + Math.round(wallz.sampleLeft[0]*100) +"          ", 0, 4);
			LCD.drawString("Raw Right: " + Math.round(wallz.sampleRight[0]*100) +"          ", 0, 5);
			
			LCD.drawString("Left: " + Math.round(wallz.pollSensorLeft()*100) +"          ", 0, 6);
			LCD.drawString("Right: " + Math.round(wallz.pollSensorRight()*100) +"          ", 0, 7);
		}
	}
	*/
}
