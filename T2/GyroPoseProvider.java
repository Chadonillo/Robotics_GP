package T2;

import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.MoveProvider;
import lejos.robotics.navigation.Pose;

public class GyroPoseProvider extends OdometryPoseProvider {
	
	private EV3GyroSensor gyroSensor;
	private SampleProvider angleProvider;
	private float[] sample;
	private float gyroOffset = 0;
	
	public GyroPoseProvider(MoveProvider mp, EV3GyroSensor gyroSensor) {
		super(mp);
		this.gyroSensor = gyroSensor;
		this.angleProvider = this.gyroSensor.getAngleMode();
		this.sample = new float[angleProvider.sampleSize()];
		this.gyroSensor.reset();
	}
	
	public float averageDegree(double a, double b){
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
	
	private float unnormalize(float angle){
		float a = angle;
	    while (a < 0) a += 360;
	    return a;
	  }
	
	private float getGyroVal(){
		angleProvider.fetchSample(sample, 0);
		float a = (sample[0]+gyroOffset)%360;
		while (a < 0) a += 360;
		return a;
	}
	
	@Override
	public Pose getPose() {
		Pose temp = super.getPose();
		//temp.setHeading(getGyroVal());
		temp.setHeading(averageDegree((double)getGyroVal(),(double)unnormalize(temp.getHeading())));
		return temp;
	}
	
	@Override
	public synchronized void setPose(Pose aPose){
		super.setPose(aPose);
		gyroSensor.reset();
		gyroOffset = aPose.getHeading();
	}
}
