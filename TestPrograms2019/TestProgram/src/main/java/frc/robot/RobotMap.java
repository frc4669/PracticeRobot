/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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


  public static final int DRIVETRAIN_LEFT_TALON = 8;
  public static final int DRIVETRAIN_RIGHT_TALON = 5;

  public static final int F310 = 2; 
  public static final int F310_DPAD_UP = 0;
  public static final int F310_DPAD_DOWN = 180;

  public static final int DRIVER_CONTROLLER_MOVE_AXIS = 1;
  public static final int DRIVER_CONTROLLER_ROTATE_AXIS = 0;

  public static final int DRIVER_CONTROLLER_MOVE_AXIS_R = 5;
  public static final int DRIVER_CONTROLLER_ROTATE_AXIS_R = 4;

  public static final int SHOOTER_SOLENOID_FORWARD = 0;
  public static final int SHOOTER_SOLENOID_BACKWARD = 1;
  
  public static final int JAW_MOTOR = 2;
  public static final int INTAKE_MOTOR = 13;

  public static final int ELEVATOR_MOTOR = 9;

  // Sensors
  public static final int elevatorVel = 1100;
	public static final int elevatorAccel = 1300;
	
	public static final int elevatorDownVel = 1300;
  public static final int elevatorDownAccel = 2900;


}

