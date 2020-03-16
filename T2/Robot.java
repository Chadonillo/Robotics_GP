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
	
	private static int baseSpeed = 20;
	private static int baseAcceleration = 20;
	private static int baseAngleSpeed = 40;
	private static int baseAngleAcceleration = 40;
	
	private double minWhite = 0.4;
	private double maxBlue = 0.15;
	
	private static float gridXlen = 5.2f; //10.83f
	private static float gridYlen = 4.6216f; //9.5f
	
	private TheStrip theMainStrip = new TheStrip();
	
	private EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	private SensorMode lightMode = colorSensor.getRedMode();
	private SensorMode colorMode = colorSensor.getColorIDMode();
	
	private EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S4);
	private EV3TouchSensor touchSensorLeft = new EV3TouchSensor(SensorPort.S2);
	private SensorMode touchModeLeft = touchSensorLeft.getTouchMode();
	private EV3TouchSensor touchSensorRight = new EV3TouchSensor(SensorPort.S3);
	private SensorMode touchModeRight = touchSensorRight.getTouchMode();
	
	private EV3LargeRegulatedMotor motorL = new EV3LargeRegulatedMotor(MotorPort.A);
	private EV3LargeRegulatedMotor motorR = new EV3LargeRegulatedMotor(MotorPort.D);
	private Wheel wheelL = WheeledChassis.modelWheel(motorL, wheelDiameter).offset(robotOffset);
	private Wheel wheelR = WheeledChassis.modelWheel(motorR, wheelDiameter).offset(-robotOffset);
	private Chassis chassis = new WheeledChassis(new Wheel[]{wheelR, wheelL},WheeledChassis.TYPE_DIFFERENTIAL);
	MovePilot pilot = new MovePilot(chassis);
	
	private GyroPoseProvider poseProvider = new GyroPoseProvider(pilot, gyroSensor);
	private CustomNavigator navigator = new CustomNavigator(pilot, poseProvider);
	private AStar aStar = new AStar(); 

	private Path obstacle_right = new Path();
	private Path obstacle_left = new Path();
	private Path obstacle_red = new Path();
	private Path obstacle_green = new Path();
	private Path startWall = new Path();
	
	public Robot(){
		obstacle_right.add(new Waypoint(8,10));
		obstacle_right.add(new Waypoint(8,11));
		obstacle_right.add(new Waypoint(8,12));
		obstacle_right.add(new Waypoint(8,13));
		obstacle_right.add(new Waypoint(8,14));
		obstacle_right.add(new Waypoint(9,14));
		obstacle_right.add(new Waypoint(10,14));
		obstacle_right.add(new Waypoint(11,14));
		obstacle_right.add(new Waypoint(12,14));
		obstacle_right.add(new Waypoint(13,14));
		obstacle_right.add(new Waypoint(13,13));
		obstacle_right.add(new Waypoint(13,12));
		obstacle_right.add(new Waypoint(13,11));
		obstacle_right.add(new Waypoint(13,10));
		obstacle_right.add(new Waypoint(12,10));
		obstacle_right.add(new Waypoint(11,10));
		obstacle_right.add(new Waypoint(10,10));
		obstacle_right.add(new Waypoint(9,10));
		
		obstacle_left.add(new Waypoint(5,10));
		obstacle_left.add(new Waypoint(5,11));
		obstacle_left.add(new Waypoint(5,12));
		obstacle_left.add(new Waypoint(5,13));
		obstacle_left.add(new Waypoint(5,14));
		obstacle_left.add(new Waypoint(6,14));
		obstacle_left.add(new Waypoint(7,14));
		obstacle_left.add(new Waypoint(8,14));
		obstacle_left.add(new Waypoint(8,13));
		obstacle_left.add(new Waypoint(8,12));
		obstacle_left.add(new Waypoint(8,11));
		obstacle_left.add(new Waypoint(8,10));
		obstacle_left.add(new Waypoint(7,10));
		obstacle_left.add(new Waypoint(6,10));
		
		obstacle_green.add(new Waypoint(22,10));
		obstacle_green.add(new Waypoint(22,11));
		obstacle_green.add(new Waypoint(22,12));
		obstacle_green.add(new Waypoint(22,13));
		obstacle_green.add(new Waypoint(22,14));
		obstacle_green.add(new Waypoint(23,14));
		obstacle_green.add(new Waypoint(24,14));
		obstacle_green.add(new Waypoint(25,14));
		obstacle_green.add(new Waypoint(26,14));
		obstacle_green.add(new Waypoint(27,14));
		obstacle_green.add(new Waypoint(28,14));
		obstacle_green.add(new Waypoint(28,13));
		obstacle_green.add(new Waypoint(28,12));
		obstacle_green.add(new Waypoint(28,11));
		obstacle_green.add(new Waypoint(28,10));
		obstacle_green.add(new Waypoint(27,10));
		obstacle_green.add(new Waypoint(26,10));
		obstacle_green.add(new Waypoint(25,10));
		obstacle_green.add(new Waypoint(24,10));
		obstacle_green.add(new Waypoint(23,10));
		
		obstacle_red.add(new Waypoint(28,10));
		obstacle_red.add(new Waypoint(28,11));
		obstacle_red.add(new Waypoint(28,12));
		obstacle_red.add(new Waypoint(28,13));
		obstacle_red.add(new Waypoint(28,14));
		obstacle_red.add(new Waypoint(29,14));
		obstacle_red.add(new Waypoint(30,14));
		obstacle_red.add(new Waypoint(31,14));
		obstacle_red.add(new Waypoint(31,13));
		obstacle_red.add(new Waypoint(31,12));
		obstacle_red.add(new Waypoint(31,11));
		obstacle_red.add(new Waypoint(31,10));
		obstacle_red.add(new Waypoint(30,10));
		obstacle_red.add(new Waypoint(29,10));
		
		startWall.add(new Waypoint(19,11));
		startWall.add(new Waypoint(19,10));
		startWall.add(new Waypoint(19,9));
		startWall.add(new Waypoint(19,8));
		startWall.add(new Waypoint(19,7));
		startWall.add(new Waypoint(19,6));
		startWall.add(new Waypoint(19,5));
		startWall.add(new Waypoint(19,4));
		startWall.add(new Waypoint(19,3));
		startWall.add(new Waypoint(19,2));
		startWall.add(new Waypoint(19,1));
		startWall.add(new Waypoint(19,0));
	}
	
	public void reset(){
		LCD.clear();
		gyroSensor.reset();
		pilot = new MovePilot(chassis);
		poseProvider = new GyroPoseProvider(pilot, gyroSensor);
		navigator = new CustomNavigator(pilot, poseProvider);
		aStar = new AStar(); 
	}
	
	public String setFirstObstacle(){
		String obstacle = null;
		LCD.clear();
		LCD.drawString("  Left Obstacle", 0, 3);
		LCD.drawString("        Or       ", 0, 4);
		LCD.drawString("  Right Obstacle", 0, 5);
		while(obstacle==null){
			if(Button.LEFT.isDown()){obstacle="left";}
			else if(Button.RIGHT.isDown()){obstacle="right";}
		}
		LCD.clear();
		LCD.drawString("  Place Wallz On ", 0, 3);
		LCD.drawString("   Strip. Press  ", 0, 4);
		LCD.drawString("  Enter To Start ", 0, 5);
		return obstacle;
	}
	
	public void setDefaultSpeed(){
		pilot.setAngularAcceleration(baseAngleAcceleration);
		pilot.setLinearAcceleration(baseAcceleration);
		pilot.setAngularSpeed(baseAngleSpeed);
		pilot.setLinearSpeed(baseSpeed);
	}
	
	public void showPose(){
		LCD.clear();
		navigator.showPose();
	}
	
	public int navigateToBox(String obstaclePos){
		Waypoint goal = new Waypoint(10,19);
		if(obstaclePos.equals("left")){aStar.addBlock(obstacle_left);}
		else if(obstaclePos.equals("right")){aStar.addBlock(obstacle_right);}
		
		aStar.addBlock(startWall);
		
		Path path = aStar.findPath(currentPos, goal);
		aStar.removeBlock(startWall);
		navigator.setPath(gridToReal(path));
		navigator.followPath();
		navigator.waitForStop();
		currentPos = goal;
		
		int color = getInAndOutBox();
		
		//poseProvider.setPose(new Pose((float)currentPos.getX()*gridXlen,(float)currentPos.getY()*gridYlen,0));
		return color;
	}
	
	public void navigateToBase(int color){
		Waypoint goal = new Waypoint(20,2);
		if(color==0){aStar.addBlock(obstacle_red);}
		else if(color==1){aStar.addBlock(obstacle_green);}
		
		Path path = aStar.findPath(currentPos, goal);
		navigator.setPath(gridToReal(path));
		navigator.followPath();
		navigator.waitForStop();
		
		navigator.rotateTo(270);
		float[] sampleLeft = new float[touchModeLeft.sampleSize()];
		float[] sampleRight = new float[touchModeRight.sampleSize()];
		touchModeLeft.fetchSample(sampleLeft, 0);
		touchModeRight.fetchSample(sampleRight, 0);
		pilot.travel(50, true);
		while(sampleLeft[0]==0 && sampleRight[0]==0){
			touchModeLeft.fetchSample(sampleLeft, 0);
			touchModeRight.fetchSample(sampleRight, 0);
		}
		pilot.stop();
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
	
	public void testNavRotation(){
		poseProvider.setPose(new Pose(0,0,0));
		navigator.rotateTo(270);
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
        float[] sample = new float[touchModeLeft.sampleSize()];
		while(!Button.ENTER.isDown()){
			touchModeLeft.fetchSample(sample, 0);
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
            //if(isBlue){LCD.drawString("Blue             ", 0, 7);}
            //else{LCD.drawString("white             ", 0, 7);}
            if(theMainStrip.getLocation()+1==37 && theMainStrip.getHighestProbability()>= 0.4){movingForward=false;}
            if(theMainStrip.getLocation()+1==10 && theMainStrip.getHighestProbability()>= 0.4){movingForward=true;}
            if(movingForward){pilot.travel(boxLenght, false);}
            else{pilot.travel(-boxLenght, false);}
            theMainStrip.setBayesianProbabilities(movingForward, isBlue, sensorProbability, 1);
        }
        LCD.drawString("  Location: " +(theMainStrip.getLocation()+1)+"          ", 0, 4);
        Sound.beep();
        return (theMainStrip.getLocation()+1-2);
    }
	
	public void getOnGridFromStrip(int gridPosition){
		int[][] stripPositionsOnGrid = {{28,18,9},{15,18,5}}; //stripNumber, GridX, GridY
		int[] closestStrip = closest(gridPosition, stripPositionsOnGrid);
		double distanceToTravel = (closestStrip[0]-gridPosition)*boxLenght;
		setDefaultSpeed();
		pilot.travel(distanceToTravel, false);
		navigator.rotateTo(270);
		currentPos = new Waypoint(closestStrip[1], closestStrip[2]);
		poseProvider.setPose(new Pose(closestStrip[1]*gridXlen,closestStrip[2]*gridYlen,0));
	}

	/*private int getInAndOutBox(){
		float distToBoxCenter = 0;
		int color = 2 ;
		navigator.rotateTo(0);
		pilot.travel(distToBoxCenter, false);
		navigator.rotateTo(90);
		
		pilot.setLinearAcceleration(5);
		pilot.setLinearSpeed(5);
		pilot.travel(100, true);
		
		float[] sampleLeft = new float[touchModeLeft.sampleSize()];
		float[] sampleRight = new float[touchModeRight.sampleSize()];
		touchModeLeft.fetchSample(sampleLeft, 0);
		touchModeRight.fetchSample(sampleRight, 0);
		while(sampleLeft[0]==0 && sampleRight[0]==0){
			touchModeLeft.fetchSample(sampleLeft, 0);
			touchModeRight.fetchSample(sampleRight, 0);
		}
		float distBackUp = pilot.getMovement().getDistanceTraveled();
		pilot.stop();
		
		Delay.msDelay(50);
		
		float[] sample = new float[colorMode.sampleSize()];
		colorMode.fetchSample(sample, 0);
		while(color==2){
			if(sample[0]==0){//Red
				LCD.drawString("   Strip: Red", 0, 7);
				color=0;
			}
			else if(sample[0]==1){//Green
				LCD.drawString("   Strip: Green", 0, 7);
				color=1;
			}
			colorMode.fetchSample(sample, 0);
		}
		Sound.beep();
		pilot.travel(-distBackUp, false);
		setDefaultSpeed();
		navigator.rotateTo(0);
		pilot.travel(-distToBoxCenter, false);
        return color;
	}*/
	
	private int getInAndOutBox(){
		int color = 2 ;
		//navigator.rotateTo(45);
		
		pilot.setLinearAcceleration(5);
		pilot.setLinearSpeed(5);
		pilot.travel(100, true);
		
		float[] sampleLeft = new float[touchModeLeft.sampleSize()];
		float[] sampleRight = new float[touchModeRight.sampleSize()];
		touchModeLeft.fetchSample(sampleLeft, 0);
		touchModeRight.fetchSample(sampleRight, 0);
		while(sampleLeft[0]==0 && sampleRight[0]==0){
			touchModeLeft.fetchSample(sampleLeft, 0);
			touchModeRight.fetchSample(sampleRight, 0);
		}
		float distBackUp = pilot.getMovement().getDistanceTraveled();
		pilot.stop();
		
		Delay.msDelay(50);
		
		float[] sample = new float[colorMode.sampleSize()];
		colorMode.fetchSample(sample, 0);
		while(color==2){
			if(sample[0]==0){//Red
				LCD.drawString("   Strip: Red", 0, 7);
				color=0;
			}
			else if(sample[0]==1){//Green
				LCD.drawString("   Strip: Green", 0, 7);
				color=1;
			}
			colorMode.fetchSample(sample, 0);
		}
		Sound.beep();
		pilot.travel(-distBackUp, false);
		setDefaultSpeed();
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
	
}
