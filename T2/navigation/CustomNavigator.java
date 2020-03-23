package navigation;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.localization.PoseProvider;
import lejos.robotics.navigation.MoveController;
import lejos.robotics.navigation.Pose;
import lejos.robotics.navigation.RotateMoveController;
import lejos.robotics.navigation.Waypoint;
import lejos.robotics.navigation.WaypointListener;
import lejos.robotics.pathfinding.Path;

/**
 * CustomNavigator is my version of the lejos Navigator Class.
 * The Lejos Navigator class had a lot of bugs and did not execute the
 * asked navigations as accurately was necessary for this task.
 * 
 * @author Brandon Cardillo
 */
public class CustomNavigator implements WaypointListener{
	/**
	 * Nav class used to operate the navigation as 
	 * a thread and concurrent to other processes.
	 */
	private Nav _nav ;
	/**
	 * The current path that this class is trying 
	 * to follow.
	 */
	private Path _path = new Path();
	/**
	 * Boolean to check if robot can keep going.
	 */
	private boolean _keepGoing = false;
	/**
	 * Boolean to check if robot has been interrupted.
	 */
	private boolean _interrupted = false;
	/**
	 * The pilot that will perform required movements.
	 */
	private MoveController _pilot;
	/**
	 * The PoseProvider that will give us robot's current Position.
	 */
	private PoseProvider poseProvider;
	/**
	 * Pose value to store robot's current Pose
	 */
	private Pose _pose = new Pose();
	/**
	 * A Waypoint value that stores our next desired position
	 */
	private Waypoint _destination;
	
	/**
	 * This is the initialise method for CustomNavigator that
	 * requires a pilot and a PoseProvider.
	 * This also starts the new thread that will be running the
	 * navigation in the background.
	 * @param pilot Pilot class to control the robots movements.
	 * @param poseProvider PoseProvider to tell us robot's current Pose.
	 */
	public CustomNavigator(MoveController pilot, PoseProvider poseProvider){
        _pilot = pilot;
        if (poseProvider == null){this.poseProvider = new OdometryPoseProvider(_pilot);}
        else{this.poseProvider = poseProvider;}
        _nav = new Nav();
        _nav.setDaemon(true);
        _nav.start();
    }
	
	/**
	 * This is a second initialise method that only requires a pilot
	 * and uses OdometryPoseProvider as the default PoseProvider.
	 * @param pilot Pilot class to control the robots movements.
	 */
	public CustomNavigator(MoveController pilot){
		this(pilot, null);
	}
	
	/**
	 * This method changes the PoseProvider.
	 * @param aProvider New PoseProvider for CustomNavigator.
	 */
	public void setPoseProvider(PoseProvider aProvider){
        poseProvider = aProvider;
    }
	
	/**
	 * This method returns the current PoseProvider.
	 * @return The current PoseProvider.
	 */
	public PoseProvider getPoseProvider(){
        return poseProvider;
    }
	
	/**
	 * This method returns the current MoveController.
	 * @return The current MoveController.
	 */
	public MoveController getMoveController(){
        return _pilot;
    }
	
	/**
	 * This method stops the robot's movements
	 * and sets the thread to interrupted.
	 */
	public void stop(){
        _keepGoing = false;
        _pilot.stop();
        _interrupted = true;
    }
	
	/**
	 * This method sets the path for the robot to follow.
	 * @param path Desired path.
	 */
	public void setPath(Path path){
        if (_keepGoing){stop();}  
        _path = path;
    }
	
	/**
	 * This method clears the current path and 
	 * if the robot is moving it will call the stop method.
	 */
	public void clearPath(){
	    if (_keepGoing){stop();}
	    _path.clear();
    }

	/**
	 * This method return the current Path.
	 * @return The current Path.
	 */
	public Path getPath(){
		return _path;
    }
	
	/**
	 * This method takes a path and immediately 
	 * sets the robot to follow that path.
	 * @param path Desired Path
	 */
	public void followPath(Path path){
		_path = path;
		followPath();
    }
	
	/**
	 * This method sets the robot to follow the 
	 * path that is saved in the variable: _path.
	 */
	public void followPath(){
	    if (_path.isEmpty()){return;}
	    _interrupted = false;
	    _keepGoing = true;
    }
	
	/**
	 * This method takes a Waypoint and sets the 
	 * robot to go there instantly.
	 * @param destination Desired destination.
	 */
	public void goTo(Waypoint destination){
		addWaypoint(destination);
		followPath();
	}
	
	/**
	 * This method takes an x and a y value
	 * and sets the robot to go there instantly.
	 * @param x X coordinate of desired destination.
	 * @param y Y coordinate of desired destination.
	 */
	public void goTo(float x, float y){
		goTo(new Waypoint(x, y));
	}
	
	/**
	 * This method takes an x, a y, and a heading value
	 * and sets the robot to go there instantly.
	 * @param x X coordinate of desired destination.
	 * @param y Y coordinate of desired destination.
	 * @param heading The heading the robot should face at the desired destination.
	 */
	public void goTo(float x, float y, float heading){
		goTo(new Waypoint(x, y, heading));
	}
	
	/**
	 * This method is a key to the accurate navigation of 
	 * the robot. It takes a desired heading and uses it to calculate the
	 * shortest rotation required to get there. Either clockwise or anti-clockwise.
	 * It then keeps on adjusting the robots heading until the error is less than 0.5 degrees
	 * or it has tried to fix its heading 10 times.
	 * These two values have been fined tuned to give accurate rotations without 
	 * wasting too much time each rotation.
	 * @param angle Desired angle.
	 * @return boolean False if robot is moving, True if robot is stopped.
	 */
	public boolean rotateTo(double angle){
		int maxBearingFix = 10;
		int bearingFixCounter = 0;
		float maxRotationError = 0.5f;
		Pose currPose = poseProvider.getPose();
		
	    double diff = angle - currPose.getHeading()%360;
	    while(diff > 180) diff = diff - 360;
	    while(diff < -180) diff = diff + 360;
	    if(isMoving()) return false;
	    
	    while(!nearllyEqual(angle,currPose.getHeading()%360,maxRotationError) && bearingFixCounter<maxBearingFix){
        	bearingFixCounter+=1;
        	((RotateMoveController) _pilot).rotate(diff,false);
        	currPose = poseProvider.getPose();
    	    diff = angle - currPose.getHeading()%360;
    	    while(diff > 180) diff = diff - 360;
    	    while(diff < -180) diff = diff + 360;
        }
	    return true;   
    }
	
	/**
	 * This method adds a new Waypoint which is the lejos 
	 * representation of a coordinate to the path that the 
	 * robot is intended to follow.
	 * @param aWaypoint The new Waypoint in the path
	 */
	public void addWaypoint(Waypoint aWaypoint){
	    _path.add(aWaypoint);
	}
 
	/**
	 * This method adds a new Waypoint which is the lejos 
	 * representation of a coordinate to the path that the 
	 * robot is intended to follow.
	 * @param x The x Coordinate of new Waypoint in path
	 * @param y The y Coordinate of new Waypoint in path
	 */
    public void addWaypoint(float x, float y){
    	addWaypoint(new Waypoint(x, y));
    }

    /**
     * This method adds a new Waypoint which is the lejos 
	 * representation of a coordinate to the path that the 
	 * robot is intended to follow.
	 * @param x The x Coordinate of new Waypoint in path
	 * @param y The y Coordinate of new Waypoint in path
     * @param heading The heading of new Waypoint in path
     */
    public void addWaypoint(float x, float y, float heading){
    	addWaypoint(new Waypoint(x, y, heading));
    }

    /**
     * This method gets the next waypoint in the robots path.
     * @return Waypoint The next Waypoint or null if path is empty.
     */
    public Waypoint getWaypoint(){
    	if (_path.size() <= 0)return null;
        return _path.get(0);
    }

    /**
     * This method simply checks if the path has been completed by 
     * checking if the path is empty.
     * @return boolean True if path is empty.
     */
    public boolean pathCompleted(){
    	return _path.size() == 0;
    }

    /**
     * This method calls Thread.yield() to make the Navigation thread
     * always the main runnable thread as long as the robot is moving.
     * This is done so that we can wait for the robot to stop before 
     * continuing with the rest of the program essentially removing 
     * the concurrency aspect of this class when called.
     * @return boolean True if path is empty after the robot has stopped.
     */
    public boolean waitForStop(){
    	while (_keepGoing)Thread.yield();
        return _path.isEmpty();
    }

    /**
     * This method simply returns a boolean that lets us know 
     * whether the robot is currently moving.
     * @return boolean True if robot is moving.
     */
    public boolean isMoving(){
    	return _keepGoing;
    }
    
    /**
     * This is a utility method that just prints the current
     * pose on the LCD of the EV3.
     */
    public void showPose(){
		while(!Button.ENTER.isDown()){
			LCD.drawString("x: " +poseProvider.getPose().getX()+"          ", 0, 3);
	        LCD.drawString("y: " +poseProvider.getPose().getY()+"          ", 0, 5);
	        LCD.drawString("0: " +poseProvider.getPose().getHeading()+"          ", 0, 7);
		}
    }	
	
    /**
     * This is an incomplete method of the WaypointListener class 
     * that this class implements. It is not required for this project
     * but needed to be added for java reasons.
     */
	@Override
	public void pathGenerated() {
		// TODO Auto-generated method stub
	}
	
	/**
	 * This method is used to find the average between 
	 * two degrees and normalise them between 0-360.
	 * @param a This can be any double value represented in degree.
	 * @param b This can be any double value represented in degree.
	 * @return The average of a and b between 0 and 360.
	 */
	private float averageDegree(double a, double b){
		a = Math.toRadians(a%360);
		b = Math.toRadians(b%360);
		double arct; //Math.atan()
		double s = (0.5*(Math.sin(a)+Math.sin(b)));
		double c = (0.5*(Math.cos(a)+Math.cos(b)));
		
		if(s>0 && c>0){arct = Math.atan(s/c);}
		else if(c<0){arct = Math.atan(s/c)+Math.PI;}
		else{arct = Math.atan(s/c)+(Math.PI*2);}
		return (float) Math.toDegrees(arct);
	}
	
	/**
	 * This method is crucial for accurate navigation. With experimentation I 
	 * discovered that the Odometry localisation would be off by about 0-3 centimetres.
	 * and that would change the bearing of the turn taken by the robot. The error was minimal but 
	 * after 3-5 turns it would start to accumulate and could cause the robot the be off the intended trajectory.
	 * To fix it I just use the intended current position of the robot and not the 
	 * pose provided by Odometry. It massively improved the angle of rotation of the robot.
	 * @param intendedCurrentPoint The intended current Pose of the robot.
	 * @param currentPose The current Pose of the robot provided by Odometry and the Gyroscope.
	 * @param finalDestination The intended Waypoint that the robot is trying to get to.
	 * @return A normalised bearing of intended angle of rotation between -180 and 180.
	 */
	private float relativeBearing(Waypoint intendedCurrentPoint, Pose currentPose, Waypoint finalDestination){
		Pose intendedCurrentPose = new Pose((float)intendedCurrentPoint.getX(), (float)intendedCurrentPoint.getY(), currentPose.getHeading());
		float intendedBearing = intendedCurrentPose.relativeBearing(finalDestination);
		float acctualBearing = currentPose.relativeBearing(finalDestination);
		if(intendedCurrentPoint.getX()>1000 && intendedCurrentPoint.getY()>1000){return acctualBearing;}
		return normalize(averageDegree(intendedBearing,acctualBearing));
	}

	/**
	 * THis method simply normalises the any angle 
	 * it is given to be between -180 and 180.
	 * @param angle input angle.
	 * @return normalised angle.
	 */
	private float normalize(float angle){
	    float a = angle;
	    while (a > 180)a -= 360;
	    while (a < -180) a += 360;
	    return a;
	  }
	
	/**
	 * This is a simple utility method I made to take to values 
	 * and if they are nearly equal to +/- the given error
	 * then the method returns true.
	 * Example: 
	 *     nearllyEqual(10.5, 12, 1) False.
	 *     nearllyEqual(10.5, 11, 1) True.
	 * @param val_1 First value to check.
	 * @param val_2 Second value to check.
	 * @param error the max +/- error for val_1 and val_2.
	 * @return boolean True if the two values are within +/- error of each other.
	 */
	private <T extends Number> boolean nearllyEqual(T val_1, T val_2, T error){
	    if(val_1.doubleValue() < val_2.doubleValue()-error.doubleValue() || 
	        val_1.doubleValue() > val_2.doubleValue()+error.doubleValue()) return false;
	    return true;
	  }
	
	/**
	 * This a the backbone of all the navigation. This class carries out all of the required
	 * navigations and tells the robot exactly what to do. This class extends Thread to be as 
	 * efficient as possible. Read run() method for more detail.
	 * @author Brandon Cardillo
	 */
	private class Nav extends Thread{
		/**
		 * This is the waypoint that represent where the robot is actually
		 * supposed the be. It is set to a default value.
		 */
		Waypoint intendedCurrWayPoint = new Waypoint(1001,1001);
		/**
		 * This is the relative bearing that the robot needs to turn 
		 * to get to the next destination in the path.
		 */
		float destinationRelativeBearing;
		/**
		 * The distance from current Pose to next destination in the path.
		 */
		float distance;
		/**
		 * The bearing that we are tying to accurately achieve 
		 * to reach next destination in path.
		 */
		float achiveBearing;
		/**
		 * The robot never actually turns the amount we tell it to.
		 * To fix this in the run method we keep fixing the angle of the robot
		 * until it is nearly equal with and error of maxRotationError.
		 * To not waste too much time fixing the bearing there is a fixed value of times
		 * that the robot can be adjusted for the perfect bearing.
		 */
		int maxBearingFix = 10;
		/**
		 * The counter of how many times we have fixed the bearing.
		 */
		int bearingFixCounter = 0;
		/**
		 * The maximum bearing error that I allow the robot to get away with.
		 */
		float maxRotationError = 0.5f;
		
		/**
		 * This is the main run method that we see in all java threads.
		 * I will break this down into three sections:
		 * 1) Bearing Adjustment (The Perfect Rotation):
		 * 			First we have to face the right direction of travel. To make it extremely accurate 
		 * 			I first use the relativeBearing() method that I confirmed worked better through experimentation
		 * 			than just taking bearing using the current Pose provided by Odometry.
		 * 			Then I use the relative bearing to rotate but the robot does not usually turn exactly how much its told.
		 * 			To fix that I simply put a loop that keeps on correcting the heading of the robot until it is nearly equal 
		 * 			to the relative bearing value.
		 * 
		 * 2) Move Towards Next Waypoint:
		 * 			This step is a lot easier because the inbuilt pilot function works quite well at moving in a straight line.
		 * 			All that had to be done was to calculate the distance from current point to goal. Then ask to pilot to 
		 * 			move that amount. 
		 * 
		 * 3) Rotate If Waypoint Has Heading:
		 * 			Then finally any Waypoint in the path can have a desired heading as well and if it does then we have to
		 * 			simply rotate toward the desired angle and to do this we do it the exact same way we did in step one to
		 * 			get the most accurate rotation possible from the robot
		 * 
		 * Last point to add is that there are some lines of code to simple check if the robot has been interrupted so 
		 * that we can stop the robot if necessary. 
		 * 
		 */
	    @Override
		public void run(){ 
		    while (true){
		        while (_keepGoing && _path != null && ! _path.isEmpty()){
		        	//Face the correct direction to go to next way point
		            _destination = _path.get(0);
		            _pose = poseProvider.getPose();
		            destinationRelativeBearing = relativeBearing(intendedCurrWayPoint,_pose, _destination);
		            achiveBearing = (destinationRelativeBearing+_pose.getHeading())%360;
		            while(!nearllyEqual(achiveBearing,_pose.getHeading()%360,maxRotationError) && bearingFixCounter<maxBearingFix){
		            	bearingFixCounter+=1;
		            	if(!_keepGoing) break;
		            	((RotateMoveController) _pilot).rotate(destinationRelativeBearing,false);
		            	_pose = poseProvider.getPose();
		            	destinationRelativeBearing = relativeBearing(intendedCurrWayPoint,_pose, _destination);
		            }
		            bearingFixCounter = 0;
		            if(!_keepGoing) break;

		            
		            //Move distance necessary to go to next way point
		            while (_pilot.isMoving() && _keepGoing)Thread.yield(); 
		            if(!_keepGoing) break;
		            _pose = poseProvider.getPose();
		            if(!_keepGoing) break;
		            distance = _pose.distanceTo(_destination);
		            _pilot.travel(distance, true);
		            
		            
		           //If way point has a heading then DO THIS:
		            while (_pilot.isMoving() && _keepGoing)Thread.yield(); 
		            _pose = poseProvider.getPose();
		            if(!_keepGoing) break;
		            if (_destination.isHeadingRequired()){
		            	_pose = poseProvider.getPose();
		          	    while(!nearllyEqual(_destination.getHeading(),_pose.getHeading(),maxRotationError) && bearingFixCounter<maxBearingFix){
		          	    	bearingFixCounter+=1;
		          	    	((RotateMoveController) _pilot).rotate(normalize((float)(_destination.getHeading()-_pose.getHeading())),false);
		          	    	_pose = poseProvider.getPose();
		            	}
		          	    bearingFixCounter = 0; 
		            }//
		                
		        
		            if (_keepGoing && ! _path.isEmpty()){
		            	if(!_interrupted){ 
		            		intendedCurrWayPoint =_path.remove(0);
		            	}
		            	_keepGoing = ! _path.isEmpty();
	            	}
		            Thread.yield();
		        }
		        Thread.yield();
	        }
	    }
    }
}

