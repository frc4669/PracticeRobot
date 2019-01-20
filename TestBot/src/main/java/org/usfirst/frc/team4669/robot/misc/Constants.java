/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package org.usfirst.frc.team4669.robot.misc;

/**
 * Include field measurements, robot length measurements, and other numbered
 * constants here to reduce magic numbers floating around.
 */
public class Constants {
	// Robot Constants
	public static final double wheelDiameter = 4; // in inches
	public static final double wheelBase = 22.25; // figure out real distance later

	public static final int encoderTicksPerRotation = 4096;

	// Constants for Pathfinder
	public static final double maxVel = 88.9; // units in inches

	/**
	 * Array for accessing PID constants for the drive train turning
	 * {kF,kP,kI,kD,Integral Zone}
	 */
	public static final double[] driveTrainPID = { 0.3343, 0.4, 0.0003, 20, 50 };

	/**
	 * Array for accessing PID constants for the gyro turning {kP,kI,kD}
	 */
	public static final double[] gyroPID = { 0.25, 0, 0.045 };

	/**
	 * Array for accessing PID constants for vision {kP,kI,kD}
	 */
	public static final double[] cameraPID = { 0.8, 0, 0 };

	/**
	 * Array for accessing PID constants for elevator {kF,kP,kI,kD,Integral zone}
	 */
	public static final double[] elevatorPID = { 0.977, 1.056, 0.006, 21.12, 50 };

	public static final int timeout = 20;
	public static final int baseTrajPeriodMs = 0;

	// Velocities & Acceleration for Motion Magic
	public static final int elevatorVel = 1100;
	public static final int elevatorAccel = 1300;

	public static final int elevatorDownVel = 1300;
	public static final int elevatorDownAccel = 2900;

	// Encoder Heights for Elevator
	public static final int elevatorSwitch = -10000;
	public static final int elevatorExchange = -2450;
	public static final int elevatorLift = -1200;
	public static final int elevatorScaleMid = -23000;
	public static final int elevatorMax = -27295;

	// Conversion factors and quick maffs
	public static final double wheelCircumference = Math.PI * wheelDiameter;

	// Multiply encoder ticks by this to convert to inches
	public static final double encoderToInch = wheelCircumference / encoderTicksPerRotation;

	// Multiply inches by this to convert to encoder ticks
	public static final double inchToEncoder = encoderTicksPerRotation / wheelCircumference;

	public static final double distancePerRotation = wheelBase * Math.PI / 4;

	public static final int angleTolerance = 2;
	public static final double kPStraightGyro = 0.025;

	// Drive Train current limits
	public static final int continuousCurrentLimit = 20;
	public static final int peakCurrentLimit = 22;
	public static final int currentDuration = 50;

	// Drive Train current limits
	public static final int continuousCurrentLimitArm = 3;
	public static final int peakCurrentLimitArm = 5;

}
