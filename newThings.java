package a;

public class newThings {
	/*
	 * New way to move our robot is to use a higher level leJOS API
	 * it is called MovePilot. http://www.lejos.org/ev3/docs/lejos/robotics/navigation/MovePilot.html
	 * This is how to declare a MovePilot Object:
	 * 
	import lejos.robotics.chassis.*;  // https://lejos.sourceforge.io/ev3/docs/lejos/robotics/chassis/WheeledChassis.html
	import lejos.robotics.navigation.MovePilot; // http://www.lejos.org/ev3/docs/lejos/robotics/navigation/MovePilot.html
	
	Wheel leftWheel = WheeledChassis.modelWheel(Motor.B, wheelDiameter(mm)).offset(-5); // motor, wheel diameter. offset is wheel distance/2 (negative for left)
		Wheel rightWheel = WheeledChassis.modelWheel(Motor.C, wheelDiameter(mm)).offset(5);
		Chassis chassis = new WheeledChassis(new Wheel[]{leftWheel, rightWheel}, WheeledChassis.TYPE_DIFFERENTIAL);
		MovePilot pilot = new MovePilot(chassis);
	*/
	
	
	
	/*PID setup
	double integral = 0; //accumulated error
	double derivative = 0; //used to predict next error
	double error = 0;
	double lastError = 0; //store the last error to be used to calculate the derivative
	
	double kp = 1; //constant to determine how sharp to adjust back to line
	double ki = 1; //
	double kd = 1; //
	
	pilot.forawrd();
	while(){
		error = ((left light value) - (right light value))*100;
		integral += error; //update accumulated error 
		derivative = error - lastError;
		
		// sample k values 1.2, 0.012, 200
		pilot.rotate((error * kp) + (integral * ki) + (derivative * kd)); // use this value for steering 
		
		lastError = error;
	}
	*/
}
