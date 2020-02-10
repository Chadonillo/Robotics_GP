package T2;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.navigation.*;

import lejos.robotics.Encoder;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.*;
import lejos.utility.Delay;

public class Robot {
	EV3LargeRegulatedMotor motorL = new EV3LargeRegulatedMotor(MotorPort.A);
	EV3LargeRegulatedMotor motorR = new EV3LargeRegulatedMotor(MotorPort.D);
	Wheel wheelL = WheeledChassis.modelWheel(motorL, 56.5).offset(-60.5);
	Wheel wheelR = WheeledChassis.modelWheel(motorR, 56.5).offset(60.5);
	Chassis chassis = new WheeledChassis(new Wheel[]{wheelR, wheelL},WheeledChassis.TYPE_DIFFERENTIAL);
	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);

	MovePilot wallz = new MovePilot(chassis);

	/*
	wallz.setLinearSpeed(50);
	wallz.setAngularSpeed(50);
	wallz.travel(500);
	wallz.rotate(-90);
	*/
	public void move(double distance, int speed, boolean immediateReturn){
		wallz.setLinearSpeed(speed);
		wallz.travel(distance, immediateReturn);
	}
	public double localize() {
        TheStrip theMainStrip = new TheStrip();

        double sensorProbability = 0.95;
        double threshold = 0.1;

        SensorMode colorMode = colorSensor.getRGBMode();
        float[] sample = new float[colorMode.sampleSize()];
        colorMode.fetchSample(sample, 0);
        double startColor = sample[2];
        //minWhite = 0.25
        //maxBlue = 0.061
        while(theMainStrip.getHighestProbability() < 0.85 && !Button.ESCAPE.isDown()) {
            colorMode.fetchSample(sample, 0);
            boolean isBlue = false;

            if (sample[2] < threshold) // if robot senses blue
                isBlue = true;
            if(isBlue){LCD.drawString("Blue             ", 0, 7);}
            else{LCD.drawString("white             ", 0, 7);}
            Delay.msDelay(500);
            this.move(17, 60, false);
            theMainStrip.setBayesianProbabilities(true, isBlue, sensorProbability, 1);

        }
        
        LCD.drawString("Probaility: " +theMainStrip.getHighestProbability()+"          ", 0, 3);
        LCD.drawString("Location: " +theMainStrip.getLocation()+"          ", 0, 5);
        return theMainStrip.getHighestProbability();

    }
	
	public void rgbValues(){
		SensorMode colorMode = colorSensor.getRGBMode();
        float[] sample = new float[colorMode.sampleSize()];
		while(!Button.ENTER.isDown()){
	        colorMode.fetchSample(sample, 0);
	        LCD.drawString("R: " +sample[0]+"          ", 0, 3);
	        LCD.drawString("G: " +sample[1]+"          ", 0, 5);
	        LCD.drawString("B: " +sample[2]+"          ", 0, 7);
		}
		
	}
}
