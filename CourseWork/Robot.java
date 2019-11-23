package a;
import lejos.hardware.motor.Motor;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

public class Robot {
	public static EV3ColorSensor sensorLeft = new EV3ColorSensor(SensorPort.S1);
	public static EV3ColorSensor sensorRight = new EV3ColorSensor(SensorPort.S4);
	float[] sample = new float[1];

	public void drive(float l, float r) {
		Motor.B.setSpeed(Math.abs(l));
		Motor.C.setSpeed(Math.abs(r));
		if (l > 0) {
			Motor.B.forward();
		} else if (l < 0) {
			Motor.B.backward();
		} else {
			Motor.B.stop(true);
		}

		if (r > 0) {
			Motor.C.forward();
		} else if (r < 0) {
			Motor.C.backward();
		} else {
			Motor.C.stop(true);
		}
	}

	public float pollSensorLeft() {
		sensorLeft.getRedMode().fetchSample(sample, 0);
		return sample[0];
	}
	
	public float pollSensorRight() {
		sensorRight.getRedMode().fetchSample(sample, 0);
		return sample[0];
	}
	
	public void stop(){
		Motor.B.stop();
		Motor.C.stop();
	}
}
