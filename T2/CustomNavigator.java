package T2;

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

public class CustomNavigator implements WaypointListener{
	  private Nav _nav ;
	  private Path _path = new Path();
	  private boolean _keepGoing = false;
	  private boolean _singleStep = false;
	  private boolean _interrupted = false;
	  private MoveController _pilot;
	  private PoseProvider poseProvider;
	  private Pose _pose = new Pose();
	  private Waypoint _destination;
	
	public CustomNavigator(MoveController pilot){
		this(pilot, null);
	}
	
	public CustomNavigator(MoveController pilot, PoseProvider poseProvider){
        _pilot = pilot;
        if (poseProvider == null){this.poseProvider = new OdometryPoseProvider(_pilot);}
        else{this.poseProvider = poseProvider;}
        _nav = new Nav();
        _nav.setDaemon(true);
        _nav.start();
    }
	
	public void setPoseProvider(PoseProvider aProvider){
        poseProvider = aProvider;
    }
	
	public PoseProvider getPoseProvider(){
        return poseProvider;
    }
	
	public MoveController getMoveController(){
        return _pilot;
    }
	
	public void stop(){
        _keepGoing = false;
        _pilot.stop();
        _interrupted = true;
    }
	
	public void setPath(Path path){
        if (_keepGoing){stop();}  
        _path = path;
        _singleStep = false;
    }
	
	public void clearPath(){
	    if (_keepGoing){stop();}
	    _path.clear();
    }

	public Path getPath(){
		return _path;
    }
	
	public void followPath(Path path){
		_path = path;
		followPath();
    }
	
	public void followPath(){
	    if (_path.isEmpty()){return;}
	    _interrupted = false;
	    _keepGoing = true;
    }
	
	public void singleStep(boolean yes){
		_singleStep = yes;
	}
	
	public void goTo(Waypoint destination){
		addWaypoint(destination);
		followPath();
	}
		
	public void goTo(float x, float y){
		goTo(new Waypoint(x, y));
	}
	
	public void goTo(float x, float y, float heading){
		goTo(new Waypoint(x, y, heading));
	}
	
	public boolean rotateTo(double angle){
	    float head = getPoseProvider().getPose().getHeading();
	    double diff = angle - head;
	    while(diff > 180) diff = diff - 360;
	    while(diff < -180) diff = diff + 360;
	    if(isMoving()) return false;
	    if(_pilot instanceof RotateMoveController){
	    	((RotateMoveController) _pilot).rotate(diff,false);
	    }
	    return true;   
    }
	
	public void addWaypoint(Waypoint aWaypoint){
		if(_path.isEmpty()){
			_singleStep = false;
		}
	    _path.add(aWaypoint);
	}
 
    public void addWaypoint(float x, float y){
    	addWaypoint(new Waypoint(x, y));
    }

    public void addWaypoint(float x, float y, float heading){
    	addWaypoint(new Waypoint(x, y, heading));
    }

    public Waypoint getWaypoint(){
    	if (_path.size() <= 0)return null;
        return _path.get(0);
    }

    public boolean pathCompleted(){
    	return _path.size() == 0;
    }

    public boolean waitForStop(){
    	while (_keepGoing)Thread.yield();
        return _path.isEmpty();
    }

    public boolean isMoving(){
    	return _keepGoing;
    }
    
    public void showPose(){
		while(!Button.ENTER.isDown()){
			LCD.drawString("x: " +poseProvider.getPose().getX()+"          ", 0, 3);
	        LCD.drawString("y: " +poseProvider.getPose().getY()+"          ", 0, 5);
	        LCD.drawString("0: " +poseProvider.getPose().getHeading()+"          ", 0, 7);
		}
    }	
		
	@Override
	public void pathGenerated() {
		// TODO Auto-generated method stub
	}
	
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

	private float relativeBearing(Waypoint intendedCurrentPoint, Pose currentPose, Waypoint finalDestination){
		Pose intendedCurrentPose = new Pose((float)intendedCurrentPoint.getX(), (float)intendedCurrentPoint.getY(), currentPose.getHeading());
		float intendedBearing = intendedCurrentPose.relativeBearing(finalDestination);
		float acctualBearing = currentPose.relativeBearing(finalDestination);
		LCD.drawString("IB: " +intendedBearing+"          ", 0, 0);
		LCD.drawString("AB: " +acctualBearing+"          ", 0, 1);
		if(intendedCurrentPoint.getX()>1000 && intendedCurrentPoint.getY()>1000){return acctualBearing;}
		return normalize(averageDegree(intendedBearing,acctualBearing));
	}

	private float normalize(float angle){
	    float a = angle;
	    while (a > 180)a -= 360;
	    while (a < -180) a += 360;
	    return a;
	  }
	
	private <T extends Number> boolean nearllyEqual(T val_1, T val_2, T error){
	    if(val_1.doubleValue() < val_2.doubleValue()-error.doubleValue() || 
	        val_1.doubleValue() > val_2.doubleValue()+error.doubleValue()) return false;
	    return true;
	  }
	
	private class Nav extends Thread{
		boolean more = true;
		Waypoint intendedCurrWayPoint = new Waypoint(1001,1001);
		float destinationRelativeBearing;
		float distance;
		float achiveBearing;
		int maxBearingFix = 4;
		int bearingFixCounter = 0;
		
	    @Override
		public void run(){ 
		    while (more){
		        while (_keepGoing && _path != null && ! _path.isEmpty()){
		        	//Face the correct direction to go to next way point
		            _destination = _path.get(0);
		            _pose = poseProvider.getPose();
		            destinationRelativeBearing = relativeBearing(intendedCurrWayPoint,_pose, _destination);
		            achiveBearing = (destinationRelativeBearing+_pose.getHeading())%360;
		            while(!nearllyEqual(achiveBearing,_pose.getHeading()%360,0.5) && bearingFixCounter<maxBearingFix){
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
		          	    while(!nearllyEqual(_destination.getHeading(),_pose.getHeading(),0.5) && bearingFixCounter<maxBearingFix){
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
		            	if(_singleStep)_keepGoing = false;
	            	}
		            Thread.yield();
		        }
		        Thread.yield();
	        }
	    }
    }
}

