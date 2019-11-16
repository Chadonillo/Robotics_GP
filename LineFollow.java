package a;

//import lejos.hardware.Sound;

import lejos.hardware.lcd.LCD;

import lejos.utility.Delay;



import lejos.hardware.Button;

import lejos.hardware.motor.Motor;

import lejos.hardware.port.SensorPort;

import lejos.hardware.sensor.EV3ColorSensor;

import lejos.hardware.sensor.SensorMode;

import lejos.hardware.sensor.EV3UltrasonicSensor;

import lejos.robotics.SampleProvider;





public class LineFollow {

	public static void main(String[] args) {

		//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

		// Set up the two light sensors

		EV3ColorSensor colorSensorLeft = new EV3ColorSensor(SensorPort.S1); //left colour sensor name

		EV3ColorSensor colorSensorRight = new EV3ColorSensor(SensorPort.S2);//right colour sensor name

		SensorMode modeLeft = colorSensorLeft.getRedMode();  //set the sensor mode to detect light intensity 

		SensorMode modeRight = colorSensorRight.getRedMode();//0 means no light reflected 1 means 100% light reflected

		float[] lightLeft = new float[1]; //where value of left sensor is saved

		float[] lightRight = new float[1];//where value of right sensor is saved

		//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

		//Set up ultrasonic sensor

		EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S3);//ultrasonic sensor name

		SampleProvider ussProvider = ultrasonicSensor.getDistanceMode();  //set the sensor mode to detect distance 

		float [] sampleDistance = new float[ussProvider.sampleSize()]; //where the value of distance will be saved

		//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

		// declare other useful variables 

		double  maxBlack = 0.2; //the max value that black can be

		int defualtSpeed = 300; //robots default straight line speed

		

		//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

		//Main Loop

		while(!Button.ESCAPE.isDown()){ //loop until you press escape

			//Update sensors

			modeLeft.fetchSample(lightLeft,0);   // Update sensor with new data

			modeRight.fetchSample(lightRight,0); // Update sensor with new data

			ussProvider.fetchSample(sampleDistance, 0); // Update sensor with new data

			

			

			//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

			//Turn Right

			if(lightRight[0] < maxBlack){ //if right sensor sees black: ....

				// set speed of wheels so that the robot turns right

				Motor.B.setSpeed(300);

				Motor.C.setSpeed(100);

				

				// set inner wheel to rotate backwards for sharper turn

				Motor.C.backward();

				

				// while the right sensor does not read white 

				// continue turning and updating sensor values

				while(lightRight[0]<maxBlack){
					if (lightRight[0]<maxBlack && lightLeft[0]<maxBlack){
						Delay.msDelay(300);
					}
					modeLeft.fetchSample(lightLeft,0); 

					modeRight.fetchSample(lightRight,0);

				}

			}

			

			// Turn Left

			else if(lightLeft[0] < maxBlack){ //if left sensor sees black: ....

				// set speed of wheels so that the robot turns left

				Motor.B.setSpeed(100);

				Motor.C.setSpeed(300);

				

				// set inner wheel to rotate backwards for sharper turn

				Motor.B.backward();

				

				// while the left sensor does not read white 

				// continue turning and updating sensor values

				while(lightLeft[0]<maxBlack){
					if (lightRight[0]<maxBlack && lightLeft[0]<maxBlack){
						Delay.msDelay(300);
					}
					modeLeft.fetchSample(lightLeft,0);
					modeRight.fetchSample(lightRight,0);
				}

			}

			//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

			

			if(sampleDistance[0]*100 < 10){

				//Do something when ultrasonic sensor is less than 10
				// set speed of wheels so that the robot turns right

				// stop motors with brakes on. 
	            Motor.C.stop();
	            Motor.B.stop();
				// set motors to 50% power.
	            Motor.C.setSpeed(200);
	            Motor.B.setSpeed(200);

	            // turn right by reversing the right motor.
	            Motor.C.backward();
	            Motor.B.forward();
	            
	         // adjust time to get a 90% turn.
	            Delay.msDelay(400);

	            Motor.C.stop();
	            Motor.B.stop();

				

				// set inner wheel to rotate backwards for sharper turn

				Motor.C.backward();

				LCD.drawString(sampleDistance[0]*100+"", 0, 4);

				Delay.msDelay(10);
				
				LCD.clear();

			}

			

			//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

			//set to default speed and continue going forward

			Motor.B.setSpeed(defualtSpeed);

			Motor.C.setSpeed(defualtSpeed);

			Motor.B.forward();

			Motor.C.forward();

		}// Close Main Loop
		
		colorSensorLeft.close();
		colorSensorRight.close();
		ultrasonicSensor.close();
		//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

	}// Close Java Main 

	//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

}// Close LineFollow Class

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------