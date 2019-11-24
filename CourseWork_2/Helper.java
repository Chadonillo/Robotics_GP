package a;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class Helper {
	public void bye(){
		LCD.clear();
		LCD.drawString("WALL Z", 5, 3);
		Delay.msDelay(2000);
	}
	public boolean oppositeSigns(int x, int y) { 
        return ((x ^ y) < 0); 
    }
	
	public float calibrate (float min, float max, float val){
		float answer = (val-min)/(max-min);
    	if(answer<0){answer = 0;}
    	else if(answer>1){answer = 1;}
    	return answer;
    }
    
    public float increment (String name, float resolution, int screenPos){
    	float input = 0;
    	LCD.drawString(name + ": " + input+"      ", 0, screenPos);
		LCD.clear(screenPos);
		input += resolution;
		LCD.drawString(name + ": " +input+"      ", 0, screenPos);
    	return input;
    }
    
    public float decrement (String name, float resolution, int screenPos){
    	float input = 0;
    	LCD.drawString(name + ": " + input+"      ", 0, screenPos);
		LCD.clear(screenPos);
		input -= resolution;
		LCD.drawString(name + ": " +input+"      ", 0, screenPos);
    	return input;
    }
    
    public float inputLCD (String name, float resolution, float preVal, int screenPos){
    	float input = preVal;
    	LCD.drawString(name + ": " + input, 0, screenPos);
		while(!Button.ENTER.isDown()){
			if(Button.UP.isDown()){
				LCD.clear(screenPos);
				input += resolution;
				input = (float) (Math.round(input * 100.0) / 100.0);
				LCD.drawString(name + ": " +input, 0, screenPos);
				Delay.msDelay(100);
			}
			if(Button.DOWN.isDown()){
				LCD.clear(screenPos);
				input -= resolution;
				input = (float) (Math.round(input * 100.0) / 100.0);
				LCD.drawString(name + ": " +input, 0, screenPos); 
				Delay.msDelay(100);
			}
		}
		Delay.msDelay(500);
    	return input;
    }
    
    public void resetTest (){
    	LCD.clear();
		LCD.drawString("To Quit", 3, 3);
		LCD.drawString("Hold escape",3, 5);
		Delay.msDelay(2000);
		LCD.clear();
    }
}
