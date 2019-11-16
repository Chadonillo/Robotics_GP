package a;

import lejos.hardware.Button;
import lejos.hardware.motor.Motor;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

import lejos.robotics.chassis.*;  // https://lejos.sourceforge.io/ev3/docs/lejos/robotics/chassis/WheeledChassis.html
import lejos.robotics.navigation.MovePilot; // http://www.lejos.org/ev3/docs/lejos/robotics/navigation/MovePilot.html

public class newThings {
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
		Wheel leftWheel = WheeledChassis.modelWheel(Motor.B, 1.5).offset(-5); // motor, wheel diameter. offset is wheel distance/2 (negative for left)
		Wheel rightWheel = WheeledChassis.modelWheel(Motor.C, 1.5).offset(5);
		Chassis chassis = new WheeledChassis(new Wheel[]{leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot pilot = new MovePilot(chassis);
		
	
		//PID setup
		double integral = 0; //accumulated error
		double derivative = 0; //used to predict next error
		double error = 0;
		double lastError = 0; //store the last error to be used to calculate the derivative
		
		double kp = 1; //constant to determine how sharp to adjust back to line
		double ki = 0; //
		double kd = 0; //
		
		double maxBlack = 0.2;
		double obstacleDistance = 10;
		
		
		pilot.forward();
		while(!Button.ESCAPE.isDown()){
			//PID line follower
			//Update sensors
			modeLeft.fetchSample(lightLeft,0);   // Update sensor with new data
			modeRight.fetchSample(lightRight,0); // Update sensor with new data
			ussProvider.fetchSample(sampleDistance, 0); // Update sensor with new data
			
			error = (lightRight[0] - lightLeft[0])*100 ;
			integral += error; //update accumulated error 
			derivative = error - lastError;
			
			// sample k values 1.2, 0.012, 200
			pilot.rotate((error * kp) + (integral * ki) + (derivative * kd)); // use this value for steering 
			
			lastError = error;
			
			//Obstacle Avoidance
			//Do something when ultrasonic sensor is less than obstacleDistance
			if(sampleDistance[0]*100 < obstacleDistance){
				pilot.stop();
				pilot.travel(sampleDistance[0]*100 - obstacleDistance); // move backwards to make sure we are obstacle distance away from object
				pilot.rotateRight();// turn robot right
				Motor.A.rotate(-90);//turn sensor to look at obstacle
				pilot.forward(); //start robot moving forward
				
				//Update sensors
				modeLeft.fetchSample(lightLeft,0);   // Update sensor with new data
				modeRight.fetchSample(lightRight,0); // Update sensor with new data
				ussProvider.fetchSample(sampleDistance, 0); // Update sensor with new data
				
				while (!(lightLeft[0] < maxBlack) && !(lightRight[0] < maxBlack) && !Button.ESCAPE.isDown()){
					//robot is too far from obstacle
					if (sampleDistance[0]*100 > obstacleDistance){
						//robot turns left
						pilot.rotate(1);
					}
					//robot is too close from obstacle
					else if (sampleDistance[0]*100 < obstacleDistance){
						//robot turns right
						pilot.rotate(-1);
					}
					
					//Update sensors
					modeLeft.fetchSample(lightLeft,0);   // Update sensor with new data
					modeRight.fetchSample(lightRight,0); // Update sensor with new data
					ussProvider.fetchSample(sampleDistance, 0); // Update sensor with new data
				}
				
				Motor.A.rotate(90, true);//turn sensor to look straight again, do not wait for motor to rotate
			}
		}
		//Close all sensors and exit program
		colorSensorLeft.close();
		colorSensorRight.close();
		ultrasonicSensor.close();
	}
}

