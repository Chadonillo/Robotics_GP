package a;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.utility.Delay;

public class test {
	public static double calibrate (double min, double max, float val){
    	double answer = (val-min)/(max-min);
    	if(answer<0){answer = 0;}
    	else if(answer>1){answer = 1;}
    	return answer;
    }
	public static void main(String[] args){
		// Set up the two light sensors
		EV3ColorSensor colorSensorLeft = new EV3ColorSensor(SensorPort.S1); //left colour sensor name
		EV3ColorSensor colorSensorRight = new EV3ColorSensor(SensorPort.S2);//right colour sensor name
		SensorMode modeLeft = colorSensorLeft.getRedMode();  //set the sensor mode to detect light intensity 
		SensorMode modeRight = colorSensorRight.getRedMode();//0 means no light reflected 1 means 100% light reflected
		float[] lightLeft = new float[1]; //where value of left sensor is saved
		float[] lightRight = new float[1];//where value of right sensor is saved
		
		Motor.B.setSpeed(-100);
		Motor.C.setSpeed(100);
		
		Motor.B.forward();
		Motor.C.forward();
		while(!Button.ESCAPE.isDown()){
			modeLeft.fetchSample(lightLeft,0);          // Update sensor with new data
			modeRight.fetchSample(lightRight,0);        // Update sensor with new data
			
			LCD.drawString("Left: "+calibrate(0.05, 0.75, lightLeft[0]), 0, 4);
			LCD.drawString("Right: "+calibrate(0.05, 0.6, lightRight[0]), 0, 6);
			Delay.msDelay(10);
			LCD.clear();
		}
		colorSensorLeft.close();
		colorSensorRight.close();
	}
}
