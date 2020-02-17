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

	
	private class Nav extends Thread{
		boolean more = true;
		float startAngle;
		float currAngle;
		float destinationRelativeBearing;
		float distance;
		
	    @Override
		public void run(){ 
		    while (more){
		        while (_keepGoing && _path != null && ! _path.isEmpty()){ 
		            _destination = _path.get(0);
		            _pose = poseProvider.getPose();
		            destinationRelativeBearing = _pose.relativeBearing(_destination);
		            
		            
		            if(!_keepGoing) break;
		            startAngle = _pose.getHeading();
		            ((RotateMoveController) _pilot).rotate(destinationRelativeBearing,false); 
		            while (_pilot.isMoving() && _keepGoing)Thread.yield(); 
		            if(!_keepGoing) break;
		            _pose = poseProvider.getPose();
		            if(!_keepGoing) break;
		            
		            
		            distance = _pose.distanceTo(_destination);
		            _pilot.travel(distance, true);
		            
		            while ((int)Math.round(poseProvider.getPose().getLocation().x) !=(int)Math.round(_destination.getPose().getLocation().x) 
		            		|| (int)Math.round(poseProvider.getPose().getLocation().y) !=(int)Math.round(_destination.getPose().getLocation().y)){
		            	_pose = poseProvider.getPose();
		            	currAngle = _pose.getHeading();
		            	if(currAngle>startAngle+destinationRelativeBearing+2 || currAngle<startAngle+destinationRelativeBearing-2 || !_pilot.isMoving() ){
		            		_pilot.stop();
		            		startAngle = _pose.getHeading();
		            		destinationRelativeBearing = _pose.relativeBearing(_destination);
		            		((RotateMoveController) _pilot).rotate(destinationRelativeBearing,false);
		            		distance = _pose.distanceTo(_destination);
				            _pilot.travel(distance, true);
		            	}
		            }
		            _pose = poseProvider.getPose();
		            if(!_keepGoing) break;
		          
		            if (_destination.isHeadingRequired()){
		            	
		            	_pose = poseProvider.getPose();
		          	    while(_pose.getHeading() != _destination.getHeading()){
		          	    	((RotateMoveController) _pilot).rotate(_destination.getHeading()-_pose.getHeading(),false);
		          	    	_pose = poseProvider.getPose();
		            	}
		          	    
		            }
		                
		        
		            if (_keepGoing && ! _path.isEmpty()){
		            	if(!_interrupted){ 
		            		_path.remove(0);
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

