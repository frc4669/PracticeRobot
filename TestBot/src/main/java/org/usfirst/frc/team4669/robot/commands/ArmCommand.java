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

public class ArmCommand extends Command {

  double initialY = 30;
  double initialX = 30;
  boolean yRunning = false, xRunning = false;

  public ArmCommand() {
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

    if (Math.abs(yAxis) > Math.abs(xAxis)) {
      xAxis = 0;
    } else if (Math.abs(yAxis) < Math.abs(xAxis)) {
      yAxis = 0;
    }

    // Robot.arm.motorControl(yAxis, xAxis, 0);

    if (yAxis != 0 && xAxis == 0) {
      if (!yRunning) {
        initialY = Robot.arm.getY();
        yRunning = true;
      }
      double targetY = initialY;
      targetY += yAxis * 0.001;
      double shoulderAngle = Robot.arm.targetToAngleShoulder(Robot.arm.getX(), targetY);
      double elbowAngle = Robot.arm.targetToAngleElbow(Robot.arm.getX(), targetY);
      // System.out.println("Shoulder Angle: " + shoulderAngle);
      // System.out.println("Elbow Angle: " + elbowAngle);
      Robot.arm.setToAngle(Robot.arm.getShoulderMotor(), shoulderAngle, true);
      Robot.arm.setToAngle(Robot.arm.getElbowMotor(), elbowAngle, true);
    } else
      yRunning = false;

    if (xAxis != 0 && yAxis == 0) {
      if (!xRunning) {
        initialX = Robot.arm.getX();
        xRunning = true;
      }
      double targetX = initialX;
      targetX += xAxis * 0.001;
      double shoulderAngle = Robot.arm.targetToAngleShoulder(targetX, Robot.arm.getY());
      double elbowAngle = Robot.arm.targetToAngleElbow(targetX, Robot.arm.getY());
      // System.out.println("Shoulder Angle: " + shoulderAngle);
      // System.out.println("Elbow Angle: " + elbowAngle);
      Robot.arm.setToAngle(Robot.arm.getShoulderMotor(), shoulderAngle, true);
      Robot.arm.setToAngle(Robot.arm.getElbowMotor(), elbowAngle, true);

    } else
      xRunning = false;

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
}
