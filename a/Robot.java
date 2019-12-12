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
	
	public SensorMode modeLeftColor = sensorLeft.getColorIDMode();
	public SensorMode modeRightColor = sensorRight.getColorIDMode();
	public float[] sampleLeftColor = new float[modeLeftColor.sampleSize()];
	public float[] sampleRightColor = new float[modeRightColor.sampleSize()];
	
	
	public EV3UltrasonicSensor ultraSonicSensor = new EV3UltrasonicSensor(SensorPort.S3);
	public SampleProvider ussProvider = ultraSonicSensor.getDistanceMode();
	public float[] sampleDistance = new float[ussProvider.sampleSize()];
	
	LinearCalibrationFilter  calibratorLeft  =  new LinearCalibrationFilter (modeLeft);
	LinearCalibrationFilter  calibratorRight  =  new LinearCalibrationFilter (modeRight);

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
		Motor.C.stop();
		Motor.B.stop();
	}
	
	public void stopParallel(){
		int stopValB = Motor.B.getTachoCount();
		int stopValC = Motor.C.getTachoCount();
		Motor.C.stop();
		Motor.B.stop();
		Motor.B.setSpeed(100);
		Motor.C.setSpeed(100);
		Motor.B.rotate(stopValB- Motor.B.getTachoCount());
		Motor.C.rotate(stopValC- Motor.C.getTachoCount());
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
		this.stopParallel();
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
	
	public float pollSensorLeftColor() {
		modeLeftColor.fetchSample(sampleLeftColor, 0);
		return sampleLeftColor[0];
	}
	
	public float pollSensorRightColor() {
		modeRightColor.fetchSample(sampleRightColor, 0);
		return sampleRightColor[0];
	}
	
	public float getDistance(){
		ussProvider.fetchSample(sampleDistance, 0);
		return sampleDistance[0]*100;
	}
	
	public void turnHeadLeft(){
		Motor.D.rotate(-100);
	}
	
	public void turnHeadRight(){
		Motor.D.rotate(100);
	}
	
	public void turnTillDistance(int distance){
		this.turnHeadLeft();
		this.drive(50, -50);
		while (this.getDistance()>distance && !Button.ENTER.isDown()){
			;
		}
		this.stop();
	}
	
	public void getOnLine(){
		if(this.pollSensorRight()<0.7){
			while(this.pollSensorLeft()>0.7){
				this.drive(150, 0);
			}
			while(this.pollSensorLeft()<0.7){
				this.drive(150, 0);
			}
			while(this.pollSensorRight()>0.7){
				this.drive(0, -150);
			}
			while(this.pollSensorRight()<0.7){
				this.drive(150, -150);
			}
		} else {
			while(this.pollSensorRight()>0.7){
				this.drive(100, 100);
			}
		}
		this.stopParallel();
		
	}
}
