package go;
//import lejos.hardware.Button;
//import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
//import lejos.hardware.port.SensorPort;
//import lejos.hardware.sensor.EV3ColorSensor;
//import lejos.hardware.sensor.EV3UltrasonicSensor;
//import lejos.hardware.sensor.SensorMode;
//import lejos.robotics.SampleProvider;
//import lejos.robotics.filter.LinearCalibrationFilter;
//import lejos.utility.Delay;
import lejos.robotics.Encoder;
import lejos.utility.Delay;

public class Forward {

//	public static void drive(float l, float r) {
//		Motor.A.setSpeed(Math.abs(l));
//		Motor.D.setSpeed(Math.abs(r));
//
//		if (l > 0) {
//			Motor.A.forward();
//		} else if (l < 0) {
//			Motor.D.backward();
//		} else {
//			Motor.A.stop(true);
//		}
//
//		if (r > 0) {
//			Motor.D.forward();
//		} else if (r < 0) {
//			Motor.D.backward();
//		} else {
//			Motor.D.stop(true);
//		}
//	}
	
	public static void main(String[] args){
//		drive(50, 50);
//		Delay.msDelay(800);
		for(int i = 0; i < 10; i++) {
			while(Motor.D.getTachoCount() <= 41 && Motor.A.getTachoCount() <= 41) {
				Motor.A.setSpeed(30);
				Motor.D.setSpeed(30);
				Motor.A.forward();
				Motor.D.forward();
			}
			Motor.A.resetTachoCount();
			Motor.D.resetTachoCount();
			Motor.A.stop();
			Motor.D.stop();
			Delay.msDelay(500);
		}
		
//		Motor.A.rotate(41);
//		Motor.D.rotate(41);
	}
}
