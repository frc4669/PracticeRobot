package org.usfirst.frc.team4669.robot;

public class Constants {
	public static final double kF = 0.385; //Feed forward constant
	public static final double kP = 4; //Proportional constant
	public static final double kI = 0.0; //Integral constant
	public static final double kD = 0; //Differential constant
	public static final int kPIDLoopIdx = 0;
	public static final int kSlotIdx = 0;
	public static final int kTimeoutMs = 10;
	
	public static final int cruiseVel = 2300;
	public static final int accel = 2300;
	
	public static final double wheelDiameter = 4;
	public static final double wheelCircumference = Math.PI*wheelDiameter;
	public static final double encoderUnitsPerRotation = 4096;
	
	public static final int inchToEncoderUnits = (int) (encoderUnitsPerRotation/wheelCircumference); //Multiply inches by this to covert to encoder units
	public static final double encoderToInchUnits = wheelCircumference/encoderUnitsPerRotation;
	
	public static final int elevatorVel = 0; //change "0" 
	public static final int elevatorAcc = 0; //change "0" 
	
	public static final int elevatorHeightMiddle = 0;
	
	public static final double elevatorkF = 0.385; //Feed forward constant
	public static final double elevatorkP = 4; //Proportional constant
	public static final double elevatorkI = 0.0; //Integral constant
	public static final double elevatorkD = 0; //Differential constant
	public static final int elevatorkPIDLoopIdx = 0;
	public static final int elevatorkSlotIdx = 0;
	public static final int elevatorkTimeoutMs = 10;
	public static final int elevateMEwatermelon = 0;
	
}
