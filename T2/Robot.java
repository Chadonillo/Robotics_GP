package T2;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;
import lejos.hardware.Sound;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.Color;
import lejos.robotics.SampleProvider;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.navigation.*;
import lejos.robotics.chassis.*;
import lejos.robotics.filter.LinearCalibrationFilter;

public class Robot {
	private static double wheelDiameter = 5.6;
	private static double boxLenght = 1.75;
	private static int baseSpeed = 4;
	private static double robotOffset = 5.1;
	private static float gridXlen = 15;
	private static float gridYlen = 19;
	
	private TheStrip theMainStrip = new TheStrip();
	
	private EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	private SensorMode lightMode = colorSensor.getRedMode();
	private LinearCalibrationFilter  calibratorLight  =  new LinearCalibrationFilter (lightMode);
	
	private EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
	
	private EV3LargeRegulatedMotor motorL = new EV3LargeRegulatedMotor(MotorPort.A);
	private EV3LargeRegulatedMotor motorR = new EV3LargeRegulatedMotor(MotorPort.D);
	private Wheel wheelL = WheeledChassis.modelWheel(motorL, wheelDiameter).offset(robotOffset);
	private Wheel wheelR = WheeledChassis.modelWheel(motorR, wheelDiameter).offset(-robotOffset);
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
	
	public void test(){
		pilot.setAngularAcceleration(20);
		pilot.setLinearAcceleration(10);
		
		pilot.setAngularSpeed(20);
		pilot.setLinearSpeed(10);
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
	
	public void infraRedValues(){
        float[] sample = new float[calibratorLight.sampleSize()];
        colorSensor.setFloodlight(Color.WHITE);
        colorSensor.setFloodlight(true);
        if(colorSensor.isFloodlightOn())LCD.drawString("It is On          ", 0, 1);
        else LCD.drawString("It is Off          ", 0, 1);
		while(!Button.ENTER.isDown()){
			calibratorLight.fetchSample(sample, 0);
	        LCD.drawString(sample[0]+"          ", 0, 3);
	        Delay.msDelay(10);
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
	
	public void calibrateLightSensor(){
		float[] sample = new float[lightMode.sampleSize()];
		LCD.drawString("Press Enter To", 0, 3);
		LCD.drawString("Start Calibration", 0, 5);
		Button.waitForAnyPress();
		LCD.clear();
		Delay.msDelay(500);
		
		calibratorLight.setScaleCalibration(0, 1);
		
		calibratorLight.startCalibration();
		LCD.drawString("Sensor Calibration", 0, 3);
		this.move(10, 1, true);
		while(pilot.isMoving()){
			calibratorLight.fetchSample(sample,  0);
		}
		calibratorLight.stopCalibration();
		LCD.drawString("Calibration Complete", 0, 3);
		Delay.msDelay(1000);
		calibratorLight.save("ligthSensorCalibration");
		LCD.clear();
		
		while(!Button.ENTER.isDown()){
			calibratorLight.fetchSample(sample, 0);
	        LCD.drawString("" +sample[0]+"          ", 0, 3);
	        Delay.msDelay(50);
		}
	}
	
	private int[] closest(int of, int[][] in) {
		int min = Integer.MAX_VALUE;
	    int[] closest = new int[1];
	    for (int i = 0; i<in.length; ++i) {
	        final int diff = Math.abs(in[i][0] - of);
	        if (diff < min) {
	            min = diff;
	            closest = in[i];
	        }
	    }
	    return closest;
	}
    
	public void getOnGridFromStrip(int gridPosition){
		int[][] stripPositionsOnGrid = {{16,3,0},{27,3,1}}; //stripNumber, GridX, GridY
		int[] closestStrip = closest(gridPosition, stripPositionsOnGrid);
		double distanceToTravel = (closestStrip[0]-gridPosition)*boxLenght;
		this.move(distanceToTravel, 4, false);
		
		pilot.setAngularAcceleration(20);
		pilot.setLinearAcceleration(10);
		pilot.setAngularSpeed(20);
		pilot.setLinearSpeed(10);
		
		poseProvider.setPose(new Pose(0,0,90));
		navigator.addWaypoint(-gridXlen, 0,180);
		navigator.followPath();
		navigator.waitForStop();
		poseProvider.setPose(new Pose(closestStrip[1]*gridXlen,closestStrip[2]*gridYlen,180));
	}
	
	public void centralizeOnStripBox(){
		colorSensor.setFloodlight(true);
		double minWhite = 0.55;
		double maxBlue = 0.09;
		double getToColorMin=minWhite;
    	double getToColorMax=1.0;
    	
        float[] sample = new float[calibratorLight.sampleSize()];
        calibratorLight.fetchSample(sample, 0);
        double startColor = sample[0];
        
        if(startColor<minWhite && startColor>maxBlue){//if we are in between colours move forward.
        	this.move(boxLenght/2,baseSpeed,false);
        	Delay.msDelay(500);
        	calibratorLight.fetchSample(sample, 0);
        	startColor = sample[0];	
        }
        if(startColor>minWhite){ //if we are on white. go to blue
        	getToColorMin = 0.0;
        	getToColorMax = maxBlue;
        }
        this.stop();
        this.move(10,baseSpeed,true);
        while((sample[0]<getToColorMin || sample[0]>getToColorMax) && !Button.ESCAPE.isDown()){
        	calibratorLight.fetchSample(sample, 0);
        }
        float dist = pilot.getMovement().getDistanceTraveled();
        this.stop();
        int moveBackSteps = (int) Math.round((double)dist/(double)boxLenght);
        this.move(-boxLenght*moveBackSteps, baseSpeed, false);
	}
	
	public int localize(){
		this.resetStrip();
        double sensorProbability = 0.95;
        double threshold = 0.1;
        
        float[] sample = new float[calibratorLight.sampleSize()];
        boolean movingForward = true;
        while(theMainStrip.getHighestProbability() < 0.85 && !Button.ESCAPE.isDown()) {
        	calibratorLight.fetchSample(sample, 0);
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
        LCD.drawString("Location: " +(theMainStrip.getLocation()+1)+"          ", 0, 0);
        Sound.beep();
        return (theMainStrip.getLocation()+1-2);
    }
	
}
