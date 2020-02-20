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
	
	public Pose getPose() {
		angleProvider.fetchSample(sample, 0);
		Pose temp = super.getPose();
		temp.setHeading(averageDegree((double)sample[0],(double)unnormalize(temp.getHeading())));
		return temp;
	}
}
