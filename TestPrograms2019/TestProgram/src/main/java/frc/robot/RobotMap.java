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


  public static final int DRIVETRAIN_LEFT_FRONT_TALON = 5;
  public static final int DRIVETRAIN_LEFT_BACK_TALON = 12;
  public static final int DRIVETRAIN_RIGHT_FRONT_TALON = 11;
  public static final int DRIVETRAIN_RIGHT_BACK_TALON = 9;

  public static final int F310 = 2; 

  public static final int DRIVER_CONTROLLER_MOVE_AXIS = 5;
  public static final int DRIVER_CONTROLLER_ROTATE_AXIS = 4;

  public static final int DRIVER_CONTROLLER_MOVE_AXIS_R = 1;
  public static final int DRIVER_CONTROLLER_ROTATE_AXIS_R = 0;

  public static final int SHOOTER_SOLENOID_FORWARD = 0;
	public static final int SHOOTER_SOLENOID_BACKWARD = 1;

}

