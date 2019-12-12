package a;

import lejos.hardware.lcd.LCD;

public class RedCheck {
	Robot wallz;
	boolean colorCheckIsLeft = true;
	public RedCheck(Robot robot){
		wallz = robot;
	}
	
	public boolean isRed(boolean isLeft){
		if(isLeft){
			if(wallz.pollSensorLeftColor()==0){
				wallz.stopParallel();
				return true;
				
			}
		}
		else{
			if(wallz.pollSensorRightColor()==0){
				wallz.stopParallel();
				return true;
			}
		}
		
		return false;
	}
	
	public boolean isInRedRange(){
		if(wallz.pollSensorLeft()>0.5f && wallz.pollSensorRight()>0.5f){
			return true;
		}
		return false;
	}
	
	public void run(int counter){
		if(counter%30 == 0 && this.isInRedRange()){
			colorCheckIsLeft = !colorCheckIsLeft;
			if(this.isRed(colorCheckIsLeft)){
				wallz.stopParallel();
			}
			while(this.isRed(colorCheckIsLeft) || this.isRed(!colorCheckIsLeft)){
				LCD.drawString("Left: " + Math.round(wallz.pollSensorLeft()*100) +"          ", 0, 5);
				LCD.drawString("Right: " + Math.round(wallz.pollSensorRight()*100) +"          ", 0, 6);
			}
		}
	}
}
