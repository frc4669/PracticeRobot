/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;

	// CAN IDs for motor controllers
	public static final int driveTopLeft = 5;
	public static final int driveBottomLeft = 20; // 10
	public static final int driveTopRight = 8;
	public static final int driveBottomRight = 7;

	public static final int leftMotorElevator = 10;
	public static final int rightMotorElevator = 22; // 2
	public static final int wheelMotorElevator = 23;

	// USB IDs for gamepad and joysticks
	public static final int leftJoystick = 0;
	public static final int rightJoystick = 1;
	public static final int f310 = 2;
	public static final int extremeJoystick = 3;
	public static final int buttonBoard = 4;

	// Slot for Talon SRX encoder?
	public static final int slotIdx = 0;
	public static final int pidIdx = 0;

	public static final int shoulderMotor = 11;
	public static final int wristMotor = 13;
	public static final int elbowMotor = 2; // 2

}
