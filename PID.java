package a;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

import lejos.robotics.chassis.*;  // https://lejos.sourceforge.io/ev3/docs/lejos/robotics/chassis/WheeledChassis.html
import lejos.robotics.navigation.MovePilot; // http://www.lejos.org/ev3/docs/lejos/robotics/navigation/MovePilot.html
import lejos.utility.Delay;

public class PID {
    static boolean oppositeSigns(int x, int y) { 
        return ((x ^ y) < 0); 
    }
    
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
	
		//Set up ultrasonic sensor
		EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S3);//ultrasonic sensor name
		SampleProvider ussProvider = ultrasonicSensor.getDistanceMode();  //set the sensor mode to detect distance 
		float [] sampleDistance = new float[ussProvider.sampleSize()]; //where the value of distance will be saved

		//Set up the higher level API to control the robots movement more accurately 
		Wheel leftWheel = WheeledChassis.modelWheel(Motor.B, 2.7).offset(-5.8); // motor, wheel diameter. offset is wheel distance/2 (negative for left)
		Wheel rightWheel = WheeledChassis.modelWheel(Motor.C, 2.7).offset(5.8);
		Chassis chassis = new WheeledChassis(new Wheel[]{leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot pilot = new MovePilot(chassis);

		//PID setup Line following
		double integral = 0; //accumulated error
		double derivative = 0; //used to predict next error
		double error = 0;
		double lastError = 0; //store the last error to be used to calculate the derivative

		double kp = 3; //constant to determine how sharp to adjust back to line
		double ki = 5; //
		double kd = 400; //

		//PID setup obstacle avoidance
		double integralOA = 0; //accumulated error
		double derivativeOA = 0; //used to predict next error
		double errorOA = 0;
		double lastErrorOA = 0; //store the last error to be used to calculate the derivative

		double kpOA = 3; //constant to determine how sharp to adjust back to line
		double kiOA = 0; //
		double kdOA = 0; //

		
		double maxBlack = 0.5;
		double obstacleDistance = 15;

		double pidValue = 0;
		double pidValueOA = 0;
		int baseSpeed = 200;

		while(!Button.ESCAPE.isDown()){
			Motor.B.forward();
			Motor.C.forward();
			//PID line follower
			//Update sensors
			modeLeft.fetchSample(lightLeft,0);          // Update sensor with new data
			modeRight.fetchSample(lightRight,0);        // Update sensor with new data
			ussProvider.fetchSample(sampleDistance, 0); // Update sensor with new data
			/*
			if(calibrate(0.05, 0.75, lightLeft[0]) == 1.1 && calibrate(0.05, 0.6, lightRight[0]) == 1.1){
				while(calibrate(0.05f, 0.75f, lightLeft[0]) > maxBlack && calibrate(0.05f, 0.6f, lightRight[0]) > maxBlack && !Button.ESCAPE.isDown()){
					LCD.drawString("Im Turning", 0, 6);
				}
				LCD.clear();
			}*/
			
			error = (calibrate(0.05, 0.75, lightLeft[0]) - calibrate(0.05, 0.6, lightRight[0]))*100;

			/* Anti Wind-Up
			 * Zero the integral, set the variable integral equal to zero,
			 * every time the error is zero or the error changes sign
			*/
			if (Math.abs(error) <= 1 || oppositeSigns((int)error, (int)lastError)){
				integral=0;
			}
			else{
				integral = ((2/3) * integral) + error; //update accumulated error, Dampen by multiplying by 2/3
			}
			derivative = error - lastError;

			// sample k values 1.2, 0.012, 200
			pidValue = (error * kp) + (integral * ki) + (derivative * kd);
			
			if (Math.abs(pidValue)>baseSpeed){
				if(pidValue>=0){
					Motor.B.forward();
					Motor.C.backward();
					if(pidValue >100){
						Motor.B.setSpeed((int)(baseSpeed + 100)); // use this value for steering
						Motor.C.setSpeed((int)(100 - baseSpeed));
					}
					else {
						Motor.B.setSpeed((int)(baseSpeed + pidValue)); // use this value for steering
						Motor.C.setSpeed((int)(pidValue - baseSpeed));
					}
					
				}
				else{
					Motor.B.backward();
					Motor.C.forward();
					if(pidValue >100){
						Motor.B.setSpeed((int)(100 + baseSpeed)); // use this value for steering
						Motor.C.setSpeed((int)(baseSpeed - 100));
					}
					else {
						Motor.B.setSpeed((int)(pidValue + baseSpeed)); // use this value for steering
						Motor.C.setSpeed((int)(baseSpeed - pidValue));
					}
				}
			}
			else{
				Motor.B.forward();
				Motor.C.forward();
				Motor.B.setSpeed((int)(baseSpeed + pidValue)); // use this value for steering 
				Motor.C.setSpeed((int)(baseSpeed - pidValue));
			}
			
			
			
			lastError = error;
			/*
			//Obstacle Avoidance
			//Do something when ultrasonic sensor is less than obstacleDistance
			if(sampleDistance[0]*100 < obstacleDistance){
				Motor.B.stop();
				Motor.C.stop();
				
				pilot.rotate(35);// turn robot right
				
				Motor.D.rotate(-90);//turn sensor to look at obstacle

				integralOA = 0; //accumulated error
				derivativeOA = 0; //used to predict next error
				errorOA = 0;
				lastErrorOA = 0; //store the last error to be used to calculate the derivative

				//Update sensors
				modeLeft.fetchSample(lightLeft,0);   // Update sensor with new data
				modeRight.fetchSample(lightRight,0); // Update sensor with new data
				ussProvider.fetchSample(sampleDistance, 0); // Update sensor with new data
				
				Motor.B.setSpeed(baseSpeed); // use this value for steering 
				Motor.C.setSpeed(baseSpeed);
				Motor.B.forward();
				Motor.C.forward();
				
				Delay.msDelay(100);
				while (!(lightLeft[0] < maxBlack) && !(lightRight[0] < maxBlack) && !Button.ESCAPE.isDown()){
					errorOA = sampleDistance[0]*100 - obstacleDistance;

					//Anti Wind-Up
					//Zero the integral, set the variable integral equal to zero,
					//every time the error is zero or the error changes sign
					if (Math.abs(errorOA) <= 1 || oppositeSigns((int)errorOA, (int)lastErrorOA)){
						integralOA=0;
					}
					else{
						integralOA = ((2/3) * integralOA) + errorOA; //update accumulated error, Dampen by multiplying by 2/3
					}
					derivativeOA = errorOA - lastErrorOA;
					
					pidValueOA = (errorOA * kpOA) + (integralOA * kiOA) + (derivativeOA * kdOA);
					Motor.B.setSpeed((int)(baseSpeed + pidValueOA)); // use this value for steering 
					Motor.C.setSpeed((int)(baseSpeed - pidValueOA));

					lastErrorOA = errorOA;

					//Update sensors
					modeLeft.fetchSample(lightLeft,0);   // Update sensor with new data
					modeRight.fetchSample(lightRight,0); // Update sensor with new data
					ussProvider.fetchSample(sampleDistance, 0); // Update sensor with new data

				}
				Motor.D.rotate(90, true);//turn sensor to look straight again, do not wait for motor to rotate
			}
			*/
		}


		//Close all sensors and exit program
		colorSensorLeft.close();
		colorSensorRight.close();
		ultrasonicSensor.close();

	}
}