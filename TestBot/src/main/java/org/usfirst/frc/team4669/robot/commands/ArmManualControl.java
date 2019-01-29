/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4669.robot.commands;

import org.usfirst.frc.team4669.robot.OI;
import org.usfirst.frc.team4669.robot.Robot;
import org.usfirst.frc.team4669.robot.misc.Constants;

import edu.wpi.first.wpilibj.command.Command;

public class ArmManualControl extends Command {

  private double targetY = 0;
  private double targetX = 0;
  private boolean armRunning = false;

  public ArmManualControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.arm.stop();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // double shoulderPower = Robot.oi.extremeY();
    // double elbowPower = Robot.oi.extremeX();
    // double wristPower = Robot.oi.extremeZ();

    double yAxis = Robot.oi.extremeY();
    double xAxis = Robot.oi.extremeZ();

    // Code below is for manual control with joystick

    // if (Math.abs(yAxis) > Math.abs(xAxis)) {
    // xAxis = 0;
    // } else if (Math.abs(yAxis) < Math.abs(xAxis)) {
    // yAxis = 0;
    // }

    // // Robot.arm.motorControl(yAxis, xAxis, 0);
    // if (xAxis != 0 || yAxis != 0) {
    // if (!armRunning) {
    // targetY = Robot.arm.getY();
    // targetX = Robot.arm.getX();
    // armRunning = true;
    // }
    // double tY = targetY + yAxis * 0.05;
    // double tX = targetX + xAxis * 0.05;
    // if (Robot.arm.calculateAngles(tX, tY, tX < 0) == null) {
    // System.out.println("Impossible position");
    // Robot.arm.stop();
    // } else {
    // // if position is posssible move robot arm
    // targetY = tY;
    // targetX = tX;

    // double shoulderAngle = Robot.arm.calculateAngles(targetX, targetY, targetX <
    // 0)[0];
    // double elbowAngle = Robot.arm.calculateAngles(targetX, targetY, targetX <
    // 0)[1];
    // System.out.print("Target X: " + targetX);
    // System.out.print("Target Y: " + targetY);
    // System.out.print(" Shoulder Angle: " + shoulderAngle);
    // System.out.println(" Elbow Angle: " + elbowAngle);

    // Robot.arm.setToAngle(Robot.arm.getShoulderMotor(), shoulderAngle, true);
    // Robot.arm.setToAngle(Robot.arm.getElbowMotor(), elbowAngle, true);
    // }

    // } else if (xAxis == 0 && yAxis == 0) {
    // // armRunning = false;
    // Robot.arm.stop();
    // }

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.arm.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }

  /**
   * @param targetX the targetX to set
   */
  public void setTargetX(double targetX) {
    this.targetX = targetX;
  }

  /**
   * @param targetX the targetX to set
   */
  public void setTargetY(double targetY) {
    this.targetY = targetY;
  }

}
