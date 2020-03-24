package term2;  

import lejos.hardware.Button;

public class MainProject{
	public static void main(String[] args){
		Robot wallz = new Robot();
		while(!Button.ESCAPE.isDown()){
			String obstacle = wallz.setFirstObstacle();;
			while(Button.ENTER.isUp()){;} 
			wallz.reset();
			wallz.centralizeOnStripBox();
			int stripPos = wallz.localize();
			wallz.getOnGridFromStrip(stripPos);
			int color = wallz.navigateToBox(obstacle);
			wallz.navigateToBase(color);
			wallz.showPose();
		}
	}
}