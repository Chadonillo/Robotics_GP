package term2;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;
import localization.TheStrip;
import navigation.CustomNavigator;
import navigation.GyroPoseProvider;
import pathfinding.AStar;
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
/**
 * This class brings everything in this project together into one location.
 * This class has three main purposes:
 * 		- To initialise and manage the different components of the EV3 robot for example;
 * 		  motors, touch sensors, gyroscope sensor,light sensor etc. Also fixed parameters
 *  	  like the robots width, wheel diameter and default linear and angular speeds of the robot.
 *  
 *  	- To initialise, manage and edit the environment that the EV3 is going to be traversing.
 *  	  This includes things like the localisation strip, the map and the obstacles. 
 *  	  Another interesting point is that this class manages parameters that reflect the true world
 *  	  for example; the length of each box in the strip, the real size of the map, current position
 *  	  of the EV3 and light intensities of different colours such as white and blue.
 *  
 *  	- To tie this all up this class has several methods that will be executed in main to perform
 *  	  the given tasks. It uses other classes functionality to achieve its goals, classes such as;
 *  	  MovePilot, GyroPoseProvider, CustomNavigator, AStar and TheStrip.
 *  	  (Read the method annotations below for accurate descriptions of functionality.) 
 * @author Brandon Cardillo
 */
public class Robot{
	/**
	 * The wheels diameter in centimetres
	 */
	private static double wheelDiameter = 5.6;
	/**
	 * The distance between the middle of the
	 * robot and the middle of the wheels.
	 */
	private static double robotOffset = 5.15;
	/**
	 * The length of the localisation strip box
	 */
	private static double boxLenght = 1.75;
	/**
	 * The current Position of the robot.
	 */
	private Waypoint currentPos;
	/**
	 * The default speed for the robot to travel at.
	 */
	private static int baseSpeed = 20;
	/**
	 * The default acceleration for the robot to travel at. 
	 */
	private static int baseAcceleration = 20;
	/**
	 * The default speed for the robot to rotate at.
	 */
	private static int baseAngleSpeed = 40;
	/**
	 * The default acceleration for the robot to rotate at.
	 */
	private static int baseAngleAcceleration = 40;
	/**
	 * The Minimum value for the light sensor for it to read white.
	 */
	private double minWhite = 0.4;
	/**
	 * The Maximum value for the light sensor for it to read blue.
	 */
	private double maxBlue = 0.15;
	/**
	 *  Real world length of each X axis unit in centimetres.
	 */
	private static float gridXlen = 10.83f;
	/**
	 *  Real world length of each Y axis unit in centimetres.
	 */
	private static float gridYlen = 9.5f;
	/**
	 * The localisation strip. 
	 */
	private TheStrip theMainStrip = new TheStrip();
	/**
	 * The EV3 colour/light sensor.
	 */
	private EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	/**
	 * EV3 light level mode between 0 and 1.
	 */
	private SensorMode lightMode = colorSensor.getRedMode();
	/**
	 * EV3 colour mode.
	 */
	private SensorMode colorMode = colorSensor.getColorIDMode();
	/**
	 * EV3 Gyroscope sensor.
	 */
	private EV3GyroSensor gyroSensor = new EV3GyroSensor(SensorPort.S4);
	/**
	 * EV3 Touch sensor on the left.
	 */
	private EV3TouchSensor touchSensorLeft = new EV3TouchSensor(SensorPort.S2);
	/**
	 * Touch mode for left Touch Sensor.
	 */
	private SensorMode touchModeLeft = touchSensorLeft.getTouchMode();
	/**
	 * EV3 Touch sensor on the right.
	 */
	private EV3TouchSensor touchSensorRight = new EV3TouchSensor(SensorPort.S3);
	/**
	 * Touch mode for right Touch Sensor.
	 */
	private SensorMode touchModeRight = touchSensorRight.getTouchMode();
	/**
	 * Initialise left motor.
	 */
	private EV3LargeRegulatedMotor motorL = new EV3LargeRegulatedMotor(MotorPort.A);
	/**
	 * Initialise right motor.
	 */
	private EV3LargeRegulatedMotor motorR = new EV3LargeRegulatedMotor(MotorPort.D);
	/**
	 * Initialise left wheel with left motor.
	 */
	private Wheel wheelL = WheeledChassis.modelWheel(motorL, wheelDiameter).offset(robotOffset);
	/**
	 * Initialise right wheel with right motor.
	 */
	private Wheel wheelR = WheeledChassis.modelWheel(motorR, wheelDiameter).offset(-robotOffset);
	/**
	 * Initialise robot chassis.
	 */
	private Chassis chassis = new WheeledChassis(new Wheel[]{wheelR, wheelL},WheeledChassis.TYPE_DIFFERENTIAL);
	/**
	 * Pilot that moves the robot. 
	 * Does so by using the chassis dimensions provided.
	 */
	MovePilot pilot = new MovePilot(chassis);
	/**
	 * Initialise the custom GyroPoseProvider class to get 
	 * the most accurate odometry + gyroscope sensing possible on an EV3.
	 */
	private GyroPoseProvider poseProvider = new GyroPoseProvider(pilot, gyroSensor);
	/**
	 * Custom Navigator that will take a path to follow and execute that path perfectly.
	 */
	private CustomNavigator navigator = new CustomNavigator(pilot, poseProvider);
	/**
	 * AStar class which is used to find the optimal path in the map.
	 */
	private AStar aStar = new AStar(); 
	
	//All the obstacle that can be added or removed from the map.
	private Path obstacle_right = new Path();
	private Path obstacle_left = new Path();
	private Path obstacle_red = new Path();
	private Path obstacle_green = new Path();
	private Path startWall = new Path();
	
	/**
	 * This is the initialise method of the robot class
	 * All it does is construct obstacles that are possible to show
	 * up in the map during execution.
	 * These obstacles are made by stating the map points that make
	 * up each obstacle. Obstacle are just a collection of points which is 
	 * called a path in Lejos.
	 */
	public Robot(){
		obstacle_right.add(new Waypoint(5,7));
		obstacle_right.add(new Waypoint(6,7));
		obstacle_right.add(new Waypoint(5,6));
		obstacle_right.add(new Waypoint(6,6));
		obstacle_right.add(new Waypoint(5,5));
		obstacle_right.add(new Waypoint(6,5));
		
		obstacle_left.add(new Waypoint(4,7));
		obstacle_left.add(new Waypoint(3,7));
		obstacle_left.add(new Waypoint(4,6));
		obstacle_left.add(new Waypoint(3,6));
		obstacle_left.add(new Waypoint(4,5));
		obstacle_left.add(new Waypoint(3,5));
		
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
	
	/**
	 * This method is used to reset the EV3 after a complete run of 
	 * the track so that it can go again without wait time.
	 * This is done by re-initialising the key components of the
	 * robots functionality. Even thought this seems inefficient it was
	 * done like so because simple soft resets were giving issues with
	 * gyroscope localisation.
	 */
	public void reset(){
		LCD.clear();
		gyroSensor.reset();
		pilot = new MovePilot(chassis);
		poseProvider = new GyroPoseProvider(pilot, gyroSensor);
		navigator = new CustomNavigator(pilot, poseProvider);
		aStar = new AStar(); 
	}
	
	/**
	 * This is a simple utility function that allows us to 
	 * choose during execution of the program whether the 
	 * first obstacle is on the right or left. We can also edit 
	 * this and add a third obstacle with centre position.
	 * @return String "left" or "right" depending on obstacle position.
	 */
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
	
	/**
	 * This simply does just as the method name says and sets the 
	 * linear and angular speed of the robot also the acceleration 
	 * to their default parameters.
	 */
	public void setDefaultSpeed(){
		pilot.setAngularAcceleration(baseAngleAcceleration);
		pilot.setLinearAcceleration(baseAcceleration);
		pilot.setAngularSpeed(baseAngleSpeed);
		pilot.setLinearSpeed(baseSpeed);
	}
	
	/**
	 * This method is another utility method that just shows
	 * the pose (location) of the EV3 according to the CustomNavigator
	 * which in turn gets that information from the GyroPoseProvider.
	 * This is then printed on the LCD and constantly updated.
	 */
	public void showPose(){
		LCD.clear();
		navigator.showPose();
	}
	
	/**
	 * Before localising on the strip, the EV3 has to be in the centre of 
	 * each "Strip Box". This in most part is done with a few logical 
	 * statements that use the value provided by the light sensor.
	 * To do this Sean came up with the idea of moving forward till the 
	 * colour detected changed for example white to blue or blue to white.
	 * This puts the robot in the centre of a box. The EV3 then needs to move
	 * back onto the middle of the box it was placed in to give it the best 
	 * chance to localise; for example if it was placed close to the end of the 
	 * strip and we move forward then we have lost valuable localisation data.
	 * When the ev3 moves back it only does so in integer step units of the 
	 * boxes length. That is done to make sure it is still in the centre of
	 * a "Strip Box".
	 */
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
	
	/**
	 * 
	 * 
	 * 
	 * @return
	 */
	public int localize(){
		pilot.setLinearSpeed(4);
		theMainStrip.resetProbs();
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
		int[][] stripPositionsOnGrid = {{11,9,2},{18,9,3},{24,9,4},{31,9,5}}; //stripNumber, GridX, GridY
		int[] closestStrip = closest(gridPosition, stripPositionsOnGrid);
		double distanceToTravel = (closestStrip[0]-gridPosition)*boxLenght;
		setDefaultSpeed();
		pilot.travel(distanceToTravel, false);
		navigator.rotateTo(270);
		currentPos = new Waypoint(closestStrip[1], closestStrip[2]);
		poseProvider.setPose(new Pose(closestStrip[1]*gridXlen,closestStrip[2]*gridYlen,0));
	}

	public int navigateToBox(String obstaclePos){
		Waypoint goal = new Waypoint(7,8);
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
		
		poseProvider.setPose(new Pose((float)currentPos.getX()*gridXlen,(float)currentPos.getY()*gridYlen,0));
		return color;
	}
	
	public void navigateToBase(int color){
		Waypoint goal = new Waypoint(10,1);
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
	
	private int getInAndOutBox(){
		float distToBoxCenter = 7;
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
