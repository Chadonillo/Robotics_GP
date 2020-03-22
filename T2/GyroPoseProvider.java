package T2;

import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.MoveProvider;
import lejos.robotics.navigation.Pose;

/**
 * This class is being made to replace OdometryPoseProvider.
 * It will be used to manage to position of the robot,
 * this is done by using Odometry and an external sensor (GryoScope)
 * to aid the accuracy of passive localisation.
 * @author Brandon Cardillo
 */
public class GyroPoseProvider extends OdometryPoseProvider {
	/**
	 * The external gyroscope sensor used to aid Odometry.
	 */
	private EV3GyroSensor gyroSensor;
	/**
	 * The SampleProvider that will give us the data from the gyroscope.
	 */
	private SampleProvider angleProvider;
	/**
	 * The array where the gyroscope's values are stored.
	 */
	private float[] sample;
	
	/**
	 * This is the initialise method that will take a MoveProvider
	 * and an EV3GyroSensor to use for Odometry.
	 * 
	 * @param mp The MoveProvider that will tell us how the robot is moving.
	 * @param gyroSensor The gyroscope sensor to aid with Odometry heading value
	 */
	public GyroPoseProvider(MoveProvider mp, EV3GyroSensor gyroSensor) {
		super(mp);
		this.gyroSensor = gyroSensor;
		this.angleProvider = this.gyroSensor.getAngleMode();
		this.sample = new float[angleProvider.sampleSize()];
		this.gyroSensor.reset();
	}
	
	/**
	 * This method is used to find the average between 
	 * two degrees and normalise them between 0-360.
	 * @param a This can be any double value represented in degree.
	 * @param b This can be any double value represented in degree.
	 * @return The average of a and b between 0 and 360.
	 */
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
	
	/**
	 * This method takes an angle and if the angle is
	 * less than 0 then it changes it to be between 0 - 360.
	 * @param angle
	 * @return the angle between 0 - 360.
	 */
	private float absAngle(float angle){
		float a = angle;
	    while (a < 0) a += 360;
	    return a;
	  }
	
	/**
	 * This method takes the value of the gyroscope which
	 * can be between -infinity and +infinity and normalises 
	 * it to be between 0 - 360.
	 * @return the gyroscopes value between 0-360.
	 */
	private float getGyroVal(){
		angleProvider.fetchSample(sample, 0);
		float a = sample[0]%360;
		while (a < 0) a += 360;
		return a;
	}
	
	/**
	 * This method is an override of the super class
	 * OdometryPoseProvider. It takes the Pose provided by
	 * its parent class and sets the new heading as
	 * the average of the gyroscope value and the 
	 * OdometryPoseProvider heading value.
	 * This is done to give a more accurate representation
	 * of the robots current heading.
	 * @return the current Pose. 
	 */
	@Override
	public Pose getPose() {
		Pose temp = super.getPose();
		temp.setHeading(averageDegree((double)getGyroVal(),(double)absAngle(temp.getHeading())));
		return temp;
	}
	
	/**
	 * This method is an override of the super class
	 * OdometryPoseProvider. We first set the Pose with 
	 * the super class method and then reset the gyroscope
	 * value to 0.
	 * 
	 * FLAW: Gyroscope value can only be reset to 0 and not set to a specific 
	 * value of choice so set Pose has to have a heading of 0 if not robots 
	 * navigation is incorrect.
	 * IMPROVEMENT: Find a way to set gyroscope to a specific value.
	 */
	@Override
	public synchronized void setPose(Pose aPose){
		super.setPose(aPose);
		gyroSensor.reset();
	}
}
