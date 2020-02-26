package T2;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;
import lejos.hardware.Sound;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.SensorMode;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.robotics.navigation.*;
import lejos.robotics.pathfinding.Path;
import lejos.robotics.chassis.*;

public class Robot {
	private static double wheelDiameter = 5.6;
	private static double boxLenght = 1.75;
	private static double robotOffset = 5.15;
	
	private Waypoint currentPos;
	
	private static int baseSpeed = 10;
	private static int baseAcceleration = 5;
	private static int baseAngleSpeed = 20;
	private static int baseAngleAcceleration = 10;
	
	private double minWhite = 0.4;
	private double maxBlue = 0.15;
	
	private static float gridXlen = 10.83f;
	private static float gridYlen = 9.5f;
	
	private TheStrip theMainStrip = new TheStrip();
	
	private EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	private SensorMode lightMode = colorSensor.getRedMode();
	private SensorMode colorMode = colorSensor.getColorIDMode();
	
	private EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S3);
	private EV3TouchSensor touchSensor = new EV3TouchSensor(SensorPort.S2);
	private SensorMode touchMode = touchSensor.getTouchMode();
	
	private EV3LargeRegulatedMotor motorL = new EV3LargeRegulatedMotor(MotorPort.A);
	private EV3LargeRegulatedMotor motorR = new EV3LargeRegulatedMotor(MotorPort.D);
	private Wheel wheelL = WheeledChassis.modelWheel(motorL, wheelDiameter).offset(robotOffset);
	private Wheel wheelR = WheeledChassis.modelWheel(motorR, wheelDiameter).offset(-robotOffset);
	private Chassis chassis = new WheeledChassis(new Wheel[]{wheelR, wheelL},WheeledChassis.TYPE_DIFFERENTIAL);
	MovePilot pilot = new MovePilot(chassis);
	
	private GyroPoseProvider poseProvider = new GyroPoseProvider(pilot, gyroSensor);
	private CustomNavigator navigator = new CustomNavigator(pilot, poseProvider);
	private AStar aStar = new AStar(); 

	private Path obstacle_1 = new Path();
	private Path obstacle_2 = new Path();
	private Path obstacle_red = new Path();
	private Path obstacle_green = new Path();
	private Path startWall = new Path();
	
	public Robot(){
		obstacle_1.add(new Waypoint(5,7));
		obstacle_1.add(new Waypoint(6,7));
		obstacle_1.add(new Waypoint(5,6));
		obstacle_1.add(new Waypoint(6,6));
		obstacle_1.add(new Waypoint(5,5));
		obstacle_1.add(new Waypoint(6,5));
		
		obstacle_2.add(new Waypoint(4,7));
		obstacle_2.add(new Waypoint(3,7));
		obstacle_2.add(new Waypoint(4,6));
		obstacle_2.add(new Waypoint(3,6));
		obstacle_2.add(new Waypoint(4,5));
		obstacle_2.add(new Waypoint(3,5));
		
		obstacle_green.add(new Waypoint(12,7));
		obstacle_green.add(new Waypoint(13,7));
		obstacle_green.add(new Waypoint(12,6));
		obstacle_green.add(new Waypoint(13,6));
		obstacle_green.add(new Waypoint(12,5));
		obstacle_green.add(new Waypoint(13,5));
		
		obstacle_red.add(new Waypoint(14,7));
		obstacle_red.add(new Waypoint(15,7));
		obstacle_red.add(new Waypoint(14,6));
		obstacle_red.add(new Waypoint(15,6));
		obstacle_red.add(new Waypoint(14,5));
		obstacle_red.add(new Waypoint(15,5));
		
		startWall.add(new Waypoint(10,5));
		startWall.add(new Waypoint(10,4));
		startWall.add(new Waypoint(10,3));
		startWall.add(new Waypoint(10,2));
		startWall.add(new Waypoint(10,1));
		startWall.add(new Waypoint(10,0));
	}

	public void setDefaultSpeed(){
		pilot.setAngularAcceleration(baseAngleAcceleration);
		pilot.setLinearAcceleration(baseAcceleration);
		pilot.setAngularSpeed(baseAngleSpeed);
		pilot.setLinearSpeed(baseSpeed);
	}
	
	public void showPose(){
		navigator.showPose();
	}
	
	public int naviagateToBox(){
		Waypoint goal = new Waypoint(7,8);
		aStar.addBlock(obstacle_1);
		aStar.addBlock(startWall);
		
		Path path = aStar.findPath(currentPos, goal);
		aStar.removeBlock(startWall);
		navigator.setPath(gridToReal(path));
		navigator.followPath();
		navigator.waitForStop();
		currentPos = goal;
		
		int color = getInAndOutBox();
		
		poseProvider.setPose(new Pose((float)currentPos.getX()*gridXlen,(float)currentPos.getY()*gridYlen,0));
		return color;
	}
	
	public void naviagateToBase(int color){
		Waypoint goal = new Waypoint(10,2);
		if(color==0){aStar.addBlock(obstacle_red);}
		else if(color==1){aStar.addBlock(obstacle_green);}
		
		Path path = aStar.findPath(currentPos, goal);
		navigator.setPath(gridToReal(path));
		navigator.followPath();
		navigator.waitForStop();
	}
	
	public void testNavSquare(int len){
		navigator.addWaypoint(len, 0);
		navigator.addWaypoint(len, len);
		navigator.addWaypoint(0, len);
		navigator.addWaypoint(0, 0, 0);
		navigator.followPath();
	}
	
	public void testPilotSquare(int len){
		for(int i=0; i<4; ++i){
			pilot.travel(len, false);
			pilot.rotate(90, false);
		}
	}

	public void testPilotRotation(){
		poseProvider.setPose(new Pose(0,0,0));
		pilot.rotate(360, true);
	}
	
	public void resetStrip(){
		theMainStrip.resetProbs();
	}
	
	public void infraRedValues(){
        float[] sample = new float[lightMode.sampleSize()];
		while(!Button.ENTER.isDown()){
			lightMode.fetchSample(sample, 0);
	        LCD.drawString(sample[0]+"          ", 0, 3);
	        Delay.msDelay(10);
		}
	}
	
	public void touchValues(){
        float[] sample = new float[touchMode.sampleSize()];
		while(!Button.ENTER.isDown()){
			touchMode.fetchSample(sample, 0);
	        LCD.drawString(sample[0]+"          ", 0, 3);
	        Delay.msDelay(10);
		}
	}
	
	public void centralizeOnStripBox(){
		pilot.setLinearSpeed(4);
		double getToColorMin=minWhite;
    	double getToColorMax=1.0;
    	
        float[] sample = new float[lightMode.sampleSize()];
        lightMode.fetchSample(sample, 0);
        double startColor = sample[0];
        
        if(startColor<minWhite && startColor>maxBlue){//if we are in between colours move forward.
        	pilot.travel(boxLenght/2, false);
        	Delay.msDelay(500);
        	lightMode.fetchSample(sample, 0);
        	startColor = sample[0];	
        }
        if(startColor>minWhite){ //if we are on white. go to blue
        	getToColorMin = 0.0;
        	getToColorMax = maxBlue;
        }
        pilot.stop();
        pilot.travel(10,true);
        while((sample[0]<getToColorMin || sample[0]>getToColorMax) && !Button.ESCAPE.isDown()){
        	lightMode.fetchSample(sample, 0);
        }
        float dist = pilot.getMovement().getDistanceTraveled();
        pilot.stop();
        int moveBackSteps = (int) Math.round((double)dist/(double)boxLenght);
        pilot.travel(-boxLenght*moveBackSteps, false);
	}
	
	public int localize(){
		pilot.setLinearSpeed(4);
		this.resetStrip();
        double sensorProbability = 0.95;
        
        float[] sample = new float[lightMode.sampleSize()];
        boolean movingForward = true;
        while(theMainStrip.getHighestProbability() < 0.85 && !Button.ESCAPE.isDown()) {
        	lightMode.fetchSample(sample, 0);
            boolean isBlue = false;
            if (sample[0] < maxBlue){isBlue = true;}
            if(isBlue){LCD.drawString("Blue             ", 0, 7);}
            else{LCD.drawString("white             ", 0, 7);}
            if(theMainStrip.getLocation()+1==37 && theMainStrip.getHighestProbability()>= 0.4){movingForward=false;}
            if(theMainStrip.getLocation()+1==10 && theMainStrip.getHighestProbability()>= 0.4){movingForward=true;}
            if(movingForward){pilot.travel(boxLenght, false);}
            else{pilot.travel(-boxLenght, false);}
            theMainStrip.setBayesianProbabilities(movingForward, isBlue, sensorProbability, 1);
        }
        LCD.drawString("Location: " +(theMainStrip.getLocation()+1)+"          ", 0, 0);
        Sound.beep();
        return (theMainStrip.getLocation()+1-2);
    }
	
	public void getOnGridFromStrip(int gridPosition){
		int[][] stripPositionsOnGrid = {{11,9,2},{18,9,3},{24,9,4},{31,9,5}}; //stripNumber, GridX, GridY
		int[] closestStrip = closest(gridPosition, stripPositionsOnGrid);
		double distanceToTravel = (closestStrip[0]-gridPosition)*boxLenght;
		setDefaultSpeed();
		pilot.travel(distanceToTravel, false);
		pilot.rotate(-90);
		
		currentPos = new Waypoint(closestStrip[1], closestStrip[2]);
		poseProvider.setPose(new Pose(closestStrip[1]*gridXlen,closestStrip[2]*gridYlen,0));
	}

	private int getInAndOutBox(){
		float distToBoxCenter = 5;
		int color = 2 ;
		pilot.travel(distToBoxCenter, false);
		pilot.rotate(90, false);
		
		pilot.setLinearAcceleration(2);
		pilot.setLinearSpeed(2);
		pilot.travel(100, true);
		
		float[] sample = new float[touchMode.sampleSize()];
		touchMode.fetchSample(sample, 0);
		while(sample[0]==0){
			touchMode.fetchSample(sample, 0);
		}
		float distBackUp = pilot.getMovement().getDistanceTraveled();
		pilot.stop();
		setDefaultSpeed();
		
		Delay.msDelay(50);
		colorMode.fetchSample(sample, 0);
		while(sample[0]!=0 && sample[0]!=1){
			if(sample[0]==0){color=0;}//Red
			else if(sample[0]==1){color=1;}//Green
			colorMode.fetchSample(sample, 0);
		}
		
		pilot.travel(-distBackUp, false);
		pilot.rotate(-90, false);
		pilot.travel(-distToBoxCenter, false);
		
        return color;
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
    
	private Path gridToReal(Path gridPath){
    	for (Waypoint waypoint : gridPath) {
			waypoint.setLocation(waypoint.getX()*gridXlen, waypoint.getY()*gridYlen);
		}
		return gridPath;
    }
	
	public Waypoint gridToReal(Waypoint gridwaypoint){
		Waypoint temp = new Waypoint(gridwaypoint.getX()*gridXlen, gridwaypoint.getY()*gridYlen);
		return temp;
    }
	
}
