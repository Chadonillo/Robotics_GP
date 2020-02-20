package T2;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;
import lejos.hardware.Sound;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.navigation.*;
import lejos.robotics.chassis.*;

public class Robot {
	private static double wheelDiameter = 5.6;
	private static double boxLenght = 1.75;
	private static int baseSpeed = 4;
	
	private TheStrip theMainStrip = new TheStrip();
	
	private EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	private EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S2);
	
	private EV3LargeRegulatedMotor motorL = new EV3LargeRegulatedMotor(MotorPort.A);
	private EV3LargeRegulatedMotor motorR = new EV3LargeRegulatedMotor(MotorPort.D);
	private Wheel wheelL = WheeledChassis.modelWheel(motorL, wheelDiameter).offset(6.2);
	private Wheel wheelR = WheeledChassis.modelWheel(motorR, wheelDiameter).offset(-6.2);
	private Chassis chassis = new WheeledChassis(new Wheel[]{wheelR, wheelL},WheeledChassis.TYPE_DIFFERENTIAL);
	MovePilot pilot = new MovePilot(chassis);
	
	GyroPoseProvider poseProvider = new GyroPoseProvider(pilot, gyroSensor);
	CustomNavigator navigator = new CustomNavigator(pilot, poseProvider);
	
	public void showPose(){
		navigator.showPose();
	}
	
	public void naviagate(){
		poseProvider.setPose(new Pose(0,0,0));
		pilot.setAngularSpeed(30);
		pilot.setLinearSpeed(30);
	}
	
	public void testNavSquare(int len){
		pilot.setAngularAcceleration(20);
		pilot.setLinearAcceleration(10);
		
		pilot.setAngularSpeed(20);
		pilot.setLinearSpeed(10);
		
		navigator.addWaypoint(len, 0);
		navigator.addWaypoint(len, len);
		navigator.addWaypoint(0, len);
		navigator.addWaypoint(0, 0, 0);
		navigator.followPath();
	}
	
	public void testPilotSquare(int len){
		pilot.setAngularAcceleration(20);
		pilot.setLinearAcceleration(10);
		
		pilot.setAngularSpeed(20);
		pilot.setLinearSpeed(10);
		for(int i=0; i<4; ++i){
			pilot.travel(len, false);
			pilot.rotate(90, false);
		}
	}
	
	public void testNavRotation(){
		poseProvider.setPose(new Pose(0,0,0));
		pilot.setAngularAcceleration(5);
		pilot.setAngularSpeed(20);
		pilot.setLinearSpeed(10);
		pilot.setLinearAcceleration(2);
		navigator.addWaypoint(0, 0, 180);
		navigator.addWaypoint(0, 0, 359);
		navigator.followPath();
	}
	
	public void testPilotRotation(){
		poseProvider.setPose(new Pose(0,0,0));
		pilot.setAngularSpeed(20);
		pilot.setLinearSpeed(10);
		pilot.rotate(360, true);
	}
	
	public void testNavStraight(int len){
		pilot.setLinearAcceleration(10);
		pilot.setLinearSpeed(10);
		navigator.addWaypoint(len, 0);
		navigator.followPath();
	}
	
	public void testPilotStraight(int len){
		pilot.setLinearAcceleration(10);
		pilot.setLinearSpeed(10);
		pilot.travel(len, true);
	}
	
	public void testZigZagNav(int len, int limit){
		poseProvider.setPose(new Pose(0,0,0));
		pilot.setAngularAcceleration(20);
		pilot.setLinearAcceleration(10);
		
		pilot.setAngularSpeed(20);
		pilot.setLinearSpeed(10);
		
		navigator.addWaypoint(len, 0);
		for (int i=1; i<=limit; i+=2){
			navigator.addWaypoint(len, len*i);
			navigator.addWaypoint(0, len*i);
			navigator.addWaypoint(0, len*(i+1));
			navigator.addWaypoint(len, len*(i+1));
		}
		navigator.addWaypoint(0, 0, 0);
		navigator.followPath();
	}
	
	public void move(double distance, int speed, boolean immediateReturn){
		pilot.setLinearSpeed(speed);
		pilot.travel(distance, immediateReturn);
	}
	
	public void stop(){
		pilot.stop();
	}
	
	public void resetStrip(){
		theMainStrip.resetProbs();
	}
	
	public void rgbValues(){
		SensorMode colorMode = colorSensor.getRGBMode();
        float[] sample = new float[colorMode.sampleSize()];
		while(!Button.ENTER.isDown()){
	        colorMode.fetchSample(sample, 0);
	        LCD.drawString("R: " +sample[0]+"          ", 0, 3);
	        LCD.drawString("G: " +sample[1]+"          ", 0, 5);
	        LCD.drawString("B: " +sample[2]+"          ", 0, 7);
		}
	}
	
	public void infraRedValues(){
		SensorMode colorMode = colorSensor.getRedMode();
        float[] sample = new float[colorMode.sampleSize()];
		while(!Button.ENTER.isDown()){
	        colorMode.fetchSample(sample, 0);
	        LCD.drawString("" +sample[0]+"          ", 0, 3);
	        Delay.msDelay(50);
		}
	}
	
	public void gyroValues(boolean reset){
		if(reset)gyroSensor.reset();
		SampleProvider angleMode = gyroSensor.getAngleMode();
        float[] sample = new float[angleMode.sampleSize()];
		while(!Button.ENTER.isDown()){
			angleMode.fetchSample(sample, 0);
	        LCD.drawString("Angle:" +sample[0]+"          ", 0, 3);
	        Delay.msDelay(50);
		}
	}
	
	public void centralizeOnStripBox(){
		colorSensor.setFloodlight(true);
		double minWhite = 0.55;
		double maxBlue = 0.09;
		double getToColorMin=minWhite;
    	double getToColorMax=1.0;
    	
		SensorMode colorMode = colorSensor.getRedMode();
        float[] sample = new float[colorMode.sampleSize()];
        colorMode.fetchSample(sample, 0);
        double startColor = sample[0];
        
        if(startColor<minWhite && startColor>maxBlue){//if we are in between colours move forward.
        	this.move(boxLenght/2,baseSpeed,false);
        	Delay.msDelay(500);
        	colorMode.fetchSample(sample, 0);
        	startColor = sample[0];	
        }
        if(startColor>minWhite){ //if we are on white. go to blue
        	getToColorMin = 0.0;
        	getToColorMax = maxBlue;
        }
        this.stop();
        this.move(10,baseSpeed,true);
        while((sample[0]<getToColorMin || sample[0]>getToColorMax) && !Button.ESCAPE.isDown()){
        	colorMode.fetchSample(sample, 0);
        }
        float dist = pilot.getMovement().getDistanceTraveled();
        this.stop();
        int moveBackSteps = (int) Math.round((double)dist/(double)boxLenght);
        this.move(-boxLenght*moveBackSteps, baseSpeed, false);
	}
	
	public double localize() {
		colorSensor.setFloodlight(true);
		this.resetStrip();
        double sensorProbability = 0.95;
        double threshold = 0.1;

        SensorMode colorMode = colorSensor.getRedMode();
        float[] sample = new float[colorMode.sampleSize()];
        boolean movingForward = true;
        while(theMainStrip.getHighestProbability() < 0.85 && !Button.ESCAPE.isDown()) {
            colorMode.fetchSample(sample, 0);
            boolean isBlue = false;
            if (sample[0] < threshold){isBlue = true;}
            if(isBlue){LCD.drawString("Blue             ", 0, 7);}
            else{LCD.drawString("white             ", 0, 7);}
            if(theMainStrip.getLocation()+1==37 && theMainStrip.getHighestProbability()>= 0.4){movingForward=false;}
            if(theMainStrip.getLocation()+1==10 && theMainStrip.getHighestProbability()>= 0.4){movingForward=true;}
            if(movingForward){this.move(boxLenght, baseSpeed, false);}
            else{this.move(-boxLenght, baseSpeed, false);}
            theMainStrip.setBayesianProbabilities(movingForward, isBlue, sensorProbability, 1);
        }
        LCD.drawString("Location: " +(theMainStrip.getLocation()+1)+"          ", 0, 5);
        Sound.beep();
        return theMainStrip.getHighestProbability();
    }
	
}
