
package org.usfirst.frc.team4669.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ButtonBoard {

	private Joystick buttonBoard;
	
	public ButtonBoard() {
		buttonBoard = new Joystick(RobotMap.buttonBoard);
	}

	public boolean getButton(int buttonPort) {
		return buttonBoard.getRawButton(buttonPort);
	}

	public double getX() {
		return -buttonBoard.getRawAxis(0);
	}
	public double getY() {
		return buttonBoard.getRawAxis(1);
	}
	
	public boolean isRight() {
		return (getX() == 1);
	}
	
	public boolean isLeft() {
		return (getX() == -1);
	}
	
	public boolean isUp() {
		return (getY() == 1);
	}
	
	public boolean isDown() {
		return (getY() == -1);
	}
	
	public int getAngle() {
		if (isRight()) {
			return 0;
		}
		else if (isRight() && isUp()) {
			return 45;
		}
		else if (isUp()) {
			return 90;
		}
		else if (isLeft() && isUp()) {
			return 135;
		}
		else if (isLeft()) {
			return 180;
		}
		else if (isLeft() && isDown()) {
			return 225;
		}
		else if (isDown()) {
			return 270;
		}
		else if (isRight() && isDown()) {
			return 315;
		}
		else {
			return -1;
		}
	}
}
