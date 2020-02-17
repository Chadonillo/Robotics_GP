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

	public Pose getPose() {
		angleProvider.fetchSample(sample, 0);
		Pose temp = super.getPose();
		temp.setHeading(sample[0]);
		return temp;
	}
}
