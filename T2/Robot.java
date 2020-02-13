package T2;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.navigation.*;

import lejos.robotics.chassis.*;
import lejos.utility.Delay;
import lejos.hardware.Sound;

public class Robot {
	private static double wheelDiameter = 56.5;
	private static double boxLenght = 17.5;
	
	TheStrip theMainStrip = new TheStrip();
	
	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
	
	EV3LargeRegulatedMotor motorL = new EV3LargeRegulatedMotor(MotorPort.A);
	EV3LargeRegulatedMotor motorR = new EV3LargeRegulatedMotor(MotorPort.D);
	Wheel wheelL = WheeledChassis.modelWheel(motorL, wheelDiameter).offset(-60.5);
	Wheel wheelR = WheeledChassis.modelWheel(motorR, wheelDiameter).offset(60.5);
	Chassis chassis = new WheeledChassis(new Wheel[]{wheelR, wheelL},WheeledChassis.TYPE_DIFFERENTIAL);
	MovePilot pilot = new MovePilot(chassis);

	public void move(double distance, int speed, boolean immediateReturn){
		pilot.setLinearSpeed(speed);
		pilot.travel(distance, immediateReturn);
	}
	
	public void stop(){
		pilot.stop();
	}
	
	public void resetStrip(){
		theMainStrip.resetProbs();
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
	
	public void centralizeOnStripBox(){
		double minWhite = 0.3;
		double maxBlue = 0.1;
		double getToColorMin=minWhite;
    	double getToColorMax=1.0;
    	
		SensorMode colorMode = colorSensor.getRGBMode();
        float[] sample = new float[colorMode.sampleSize()];
        colorMode.fetchSample(sample, 0);
        double startColor = sample[2];
        
        if(startColor<minWhite && startColor>maxBlue){//if we are in between colours move forward.
        	this.move(boxLenght/2,60,false);
        	Delay.msDelay(500);
        	colorMode.fetchSample(sample, 0);
        	startColor = sample[2];	
        }
        if(startColor>minWhite){ //if we are on white. go to blue
        	getToColorMin = 0.0;
        	getToColorMax = maxBlue;
        }
        
        motorL.resetTachoCount();
        this.move(100,60,true);
        while((sample[2]<getToColorMin || sample[2]>getToColorMax) && !Button.ESCAPE.isDown()){
        	colorMode.fetchSample(sample, 0);
        }
        this.stop();
        int moveBackSteps = (int) Math.ceil((double)motorL.getTachoCount()/(double)35.0);
        this.move(-boxLenght*moveBackSteps, 60, false);
	}
	
	public double localize() {
		this.resetStrip();
        double sensorProbability = 0.95;
        double threshold = 0.1;

        SensorMode colorMode = colorSensor.getRGBMode();
        float[] sample = new float[colorMode.sampleSize()];
        boolean movingForward = true;
        while(theMainStrip.getHighestProbability() < 0.85 && !Button.ESCAPE.isDown()) {
            colorMode.fetchSample(sample, 0);
            boolean isBlue = false;
            if (sample[2] < threshold){isBlue = true;}
            if(isBlue){LCD.drawString("Blue             ", 0, 7);}
            else{LCD.drawString("white             ", 0, 7);}
            if(theMainStrip.getLocation()+1==37 && theMainStrip.getHighestProbability()>= 0.4){movingForward=false;}
            if(theMainStrip.getLocation()+1==10 && theMainStrip.getHighestProbability()>= 0.4){movingForward=true;}
            if(movingForward){this.move(boxLenght, 60, false);}
            else{this.move(-boxLenght, 60, false);}
            theMainStrip.setBayesianProbabilities(movingForward, isBlue, sensorProbability, 1);
        }
        LCD.drawString("Location: " +(theMainStrip.getLocation()+1)+"          ", 0, 5);
        Sound.beep();
        return theMainStrip.getHighestProbability();
    }
	
	
}
