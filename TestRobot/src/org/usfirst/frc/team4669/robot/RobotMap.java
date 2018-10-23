package org.usfirst.frc.team4669.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	
	//main Motor control
	public static final int topLeftMotor = 5;
	public static final int topRightMotor = 8;
	public static final int bottomRightMotor = 7;
	public static final int bottomLeftMotor = 10;
	public static final int intakeRightMotor = 2;
	public static final int intakeLeftMotor = 1;

	//sprocket Motor control
	public static final int sprocketMotor = 17;
	
	//leftJoystick controls
	public static final int leftJoystick = 0;
	
	//rightJoystick controls
	public static final int rightJoystick = 1;
	
	//gamePad controls
	public static final int f310 = 2;
	
	//Constants
	public static final int timeout = 10; //units in ms
	public static final double kCollisionThresholdDeltaG = 5;
	
	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
}
