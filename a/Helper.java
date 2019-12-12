package a;

import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class Helper {
	public boolean oppositeSigns(int x, int y) { 
        return ((x ^ y) < 0); 
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
			if(Button.LEFT.isDown()){
				LCD.clear(screenPos);
				input = 0;
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
    

	public void bye(){
		LCD.clear();
		LCD.drawString("WALL Z", 5, 3);
		Delay.msDelay(2000);
	}
}
